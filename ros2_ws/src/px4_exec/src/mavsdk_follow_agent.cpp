// Include the C++ libaries;
#include <chrono>
#include <cstring>
#include <future>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

// MAVSDK libraries for the MAVSDK + Follow me plugin:
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/follow_me/follow_me.h>

// UDP port to receive messages from the command window;
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace std::chrono_literals;



// Function to identify teh PX4 drone and return it:
static std::shared_ptr<mavsdk::System> wait_for_autopilot(mavsdk::Mavsdk& mavsdk, std::chrono::seconds timeout)
{
    // Wait for the drone pointer
    std::promise<std::shared_ptr<mavsdk::System>> prom;
    auto fut = prom.get_future();
    // Thread-safe boolean that if it already finds teh drone, the system would not try to find it again
    std::atomic<bool> done{false};

    // Function that triggers every time that the MAVSDK detects a new drone:
    mavsdk.subscribe_on_new_system([&]() {
        // Check if it already found a drone
        if (done.load()) return;
        // List of the current systems
        auto systems = mavsdk.systems();
        // Deifne if there are not current systems
        if (systems.empty()) return;

        // Get the last detected system:
        auto sys = systems.back();
        // Send the drone pointer to the thread
        if (sys && sys->is_connected() && sys->has_autopilot()) {
            done = true;
            prom.set_value(sys);
        }
    });

    // Pauses teh mavsdk thread until promise recieves a drone value
    if (fut.wait_for(timeout) != std::future_status::ready) {
        mavsdk.subscribe_on_new_system(nullptr);
        return {};
    }

    // Return the found droe system
    mavsdk.subscribe_on_new_system(nullptr);
    return fut.get();
}



// Main logic of the Code
int main(int argc, char** argv)
{
    // Arguments of the connection URL of the drone nd the command input (14560)   
    const std::string conn_url = (argc > 1) ? argv[1] : "udp://127.0.0.1:18570";
    const int udp_port = (argc > 2) ? std::stoi(argv[2]) : 14560;

    // Initalizaion of mavsdk so it works as a GroudnStation (GCS)
    mavsdk::Mavsdk mavsdk{mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}};
    std::cout << "[agent] MAVSDK version: " << mavsdk.version() << "\n";

    // It establish the connection between the MAVSDK with the url drone:
    const auto conn_res = mavsdk.add_any_connection(conn_url);
    if (conn_res != mavsdk::ConnectionResult::Success) {
        std::cerr << "[agent] Connection failed: " << static_cast<int>(conn_res) << "\n";
        return 1;
    }
    std::cout << "[agent] Listening MAVLink on: " << conn_url << "\n";

    // Hold the programm until the drone is discovered
    auto autopilot = wait_for_autopilot(mavsdk, 15s);
    if (!autopilot) {
        std::cerr << "[agent] No autopilot found within timeout.\n";
        return 1;
    }
    std::cout << "[agent] PX4 autopilot discovered.\n";

    // Start and set up the Follow ME:
    mavsdk::FollowMe follow_me{autopilot};
    // Default FollowMe config
    mavsdk::FollowMe::Config cfg{};
    cfg.follow_distance_m = 8.0f;
    cfg.follow_height_m = 8.0f;
    cfg.responsiveness = 0.1f;
    
    // Set teh cofigruation to the follow me system:
    if (auto r = follow_me.set_config(cfg); r != mavsdk::FollowMe::Result::Success) {
        std::cerr << "[agent] set_config failed: " << static_cast<int>(r) << "\n";
    } else {
        std::cout << "[agent] FollowMe config set.\n";
    }

    // State shared between threads
    std::atomic<bool> running{true};
    std::atomic<bool> follow_active{false};
    std::mutex mtx;
    std::optional<mavsdk::FollowMe::TargetLocation> last_target;

    // Stream thread (sends target regularly when active)
    std::thread streamer([&]() {
        while (running.load()) {
            if (follow_active.load()) {
                std::optional<mavsdk::FollowMe::TargetLocation> target_copy;
                {
                    std::lock_guard<std::mutex> lk(mtx);
                    target_copy = last_target;
                }
                if (target_copy) {
                    (void)follow_me.set_target_location(*target_copy);
                }
            }
            std::this_thread::sleep_for(200ms); // 5 Hz
        }
    });

    // UDP server to listen to the standard UDP socket:
    int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::perror("[agent] socket");
        running = false;
        streamer.join();
        return 1;
    }

    // Netwprk setup
    sockaddr_in addr{};
    // The system that we are using (IPv4)
    addr.sin_family = AF_INET;
    // bIND THE SOCKET TO THE DEFINED UDP port for security
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // 127.0.0.1
    addr.sin_port = htons(static_cast<uint16_t>(udp_port));
    // If the socket can not be find return
    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("[agent] bind");
        ::close(sock);
        running = false;
        streamer.join();
        return 1;
    }
    // Print the UPD port and the commands that the follow me system can read:
    std::cout << "[agent] UDP command port: 127.0.0.1:" << udp_port << "\n";
    std::cout << "[agent] Commands: START | STOP | TARGET <lat> <lon> <abs_alt>\n";

    // FOrloop unitl the running is stropped by teh user
    char buf[1024];
    while (running.load()) {
        // Socket varible to save the socket address adn teh size
        sockaddr_in src{};
        socklen_t srclen = sizeof(src);
        // Pause until UDP package arrives (buf saves hte recieverd data)
        const ssize_t n = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                                  reinterpret_cast<sockaddr*>(&src), &srclen);
        // Cgeck if the number of bytes recieed by buf are more than 0
        if (n <= 0) continue;
        buf[n] = '\0';

        // String conversion of the buf
        std::string line(buf);
        // REmobe the white spaces
        while (!line.empty() && (line.back() == '\n' || line.back() == '\r' || line.back() == ' ' || line.back() == '\t'))
            line.pop_back();
        
        // DO an input stream 
        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;
        
        // Define the logic between each command
        // Start the FOLLOW ME behavior
        if (cmd == "START") {
            auto r = follow_me.start();
            if (r == mavsdk::FollowMe::Result::Success) {
                follow_active = true;
                std::cout << "[agent] FollowMe START\n";
            } else {
                std::cout << "[agent] FollowMe START failed: " << static_cast<int>(r) << "\n";
            }
        // Stop the FOllow ME behaviour
        } else if (cmd == "STOP") {
            auto r = follow_me.stop();
            follow_active = false;
            std::cout << "[agent] FollowMe STOP (" << static_cast<int>(r) << ")\n";
        // Listen the command from the buff command send from the command window
        } else if (cmd == "TARGET") {
            double lat{}, lon{}, alt{};
            if (!(iss >> lat >> lon >> alt)) {
                std::cout << "[agent] Bad TARGET. Use: TARGET <lat> <lon> <abs_alt>\n";
                continue;
            }
            mavsdk::FollowMe::TargetLocation t{};
            t.latitude_deg = lat;
            t.longitude_deg = lon;
            t.absolute_altitude_m = static_cast<float>(alt);
            t.velocity_x_m_s = 0.0f;
            t.velocity_y_m_s = 0.0f;
            t.velocity_z_m_s = 0.0f;

            {
                std::lock_guard<std::mutex> lk(mtx);
                last_target = t;
            }
            std::cout << "[agent] TARGET set: " << lat << " " << lon << " " << alt << "\n";
        // Close the MAVSDK and finish the server.
        } else if (cmd == "QUIT") {
            std::cout << "[agent] QUIT\n";
            running = false;
        } else {
            std::cout << "[agent] Unknown cmd: " << cmd << "\n";
        }
    }

    // Close the stramer thread but waits until it finisht eh current loop.
    ::close(sock);
    running = false;
    if (streamer.joinable()) streamer.join();
    return 0;
}

