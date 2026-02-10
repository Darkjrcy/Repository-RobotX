// Include C++ Libraries:
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <optional>
#include <thread>
#include <atomic>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Include ROS2 Libraries and messages:
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Include PX4 messages:
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

// Include your custom messages:
#include "px4_exec/msg/april_tag_detection.hpp"



// Add time unit parameter:
using namespace std::chrono_literals;



// Start the class node executable:
class UavLandingNode : public rclcpp::Node {
    public:

        UavLandingNode() : rclcpp::Node("uav_landing_node") {
            // Declare topic paramters:
            usv_gps_topic_ = this->declare_parameter<std::string>("usv_gps_topic", "/wamv/sensors/gps/gps/fix");
            uav_gps_topic_ = this->declare_parameter<std::string>("uav_gps_topic", "/x500_mono_cam/sensors/gps/gps/fix");
            uav_apriltag_info_topic_ = this->declare_parameter<std::string>("uav_apriltag_info_topic", "uav/apriltag_detect");

            // Declare the camera position offset (In teh NED referece frame of the UAV):
            cam_x_offset_ = this->declare_parameter<double>("cam_x_offset", 0.0);
            cam_y_offset_ = this->declare_parameter<double>("cam_y_offset", 0.0);
            cam_z_offset_ = -1*this->declare_parameter<double>("cam_z_offset", -0.1);
            // Decalre teh roation of the camera from teh UAV NED refernce frame the declared parameters should be from the mono_cam
            // x500 model.sdf
            double cam_roll_gz  = this->declare_parameter<double>("cam_roll", 0.0);
            double cam_pitch_gz = this->declare_parameter<double>("cam_pitch", -1.5707);
            double cam_yaw_gz   = this->declare_parameter<double>("cam_yaw", 1.5707);

            // Get the real rotation in terms of teh UAV NED referene frame:
            tf2::Quaternion q_nwu;
            q_nwu.setRPY(cam_roll_gz, cam_pitch_gz, cam_yaw_gz);
            // Define the rotation:
            tf2::Quaternion q_rot_nwu_to_ned;
            q_rot_nwu_to_ned.setRPY(M_PI, 0, 0);
            // Get teh final rotation in teh NED frmae:
            tf2::Quaternion q_ned = q_rot_nwu_to_ned * q_nwu;
            // Obtain the rotation angles from teh global frame:
            double cam_roll_, cam_pitch_, cam_yaw_;
            rot_matrix_ned.getRPY(cam_roll_, cam_pitch_, cam_yaw_);

            // Temperarlly define initial position of the UAV:
            uav_pos_init_  = this->declare_parameter<std::string>("uav_init_pos", "-540,140,1.76,0,0,0");
            parse_uav_init_pos(uav_pos_init_);

            // Define the Subscription Nodes to the GPS of the UAV and the USV:
            // USV:
            usv_gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(usv_gps_topic_, 10,
                            [this](const sensor_msgs::msg::NavSatFix &msg) {std::lock_guard<std::mutex> lock(mutex_);last_usv_gps_ = msg;});
            // UAV:
            uav_gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(uav_gps_topic_, 10,
                            [this](const sensor_msgs::msg::NavSatFix &msg) {std::lock_guard<std::mutex> lock(mutex_);last_uav_gps_ = msg;});

            // Subscribe to the AprilTag Detection infromation:
            apriltag_info_sub_ = this->create_subscription<px4_exec::msg::AprilTagDetection>(uav_apriltag_info_topic_, 10,
                            [this](const px4_exec::msg::AprilTagDetection &msg) {std::lock_guard<std::mutex> lock(mutex_);last_apriltag_info_ = msg;});

            // Temporarlly position of the wamv:
            usv_pose_topic_ = this->declare_parameter<std::string>("wamv_pose_topic", "/wamv/pose");
            usv_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(usv_pose_topic_, 10,
                            [this](const geometry_msgs::msg::PoseStamped &msg) {std::lock_guard<std::mutex> lock(mutex_);last_usv_pose_enu_ = msg;});

            // Define the QoS for PX4 nodes:
            auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

            // PX4-ROS2 Subscribers:
            // Vehicle status subscription:
            vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", px4_qos,
                [this](const px4_msgs::msg::VehicleStatus &msg){std::lock_guard<std::mutex> lock(mutex_);vehicle_status_ = msg;});
            // Local position of the UAV subscriber:
            local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", px4_qos,
                [this](const px4_msgs::msg::VehicleLocalPosition &msg){std::lock_guard<std::mutex> lock(mutex_);local_pos_ = msg;});

            // PX4-ROS2 Mode publishers:
            // Offboard controller publisher:
            offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
            // Trajectory setpoint:
            trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
            // Vehicle command:
            vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

            // Create a timer to use the landing strategy:
            timer_ = this->create_wall_timer(100ms, [this]() { this->thick(); });
            
        }

    
    private:

        // Start defining the private variables:
        // USV GPS topic:
        std::string usv_gps_topic_;
        // UAV GPS topic:
        std::string uav_gps_topic_;
        // USV intiial poisiton to use the wamv pose topic for hte moment:
        std::string uav_pos_init_;
        // Topic that has the APiltag inforamtion:
        std::string uav_apriltag_info_topic_;
        // UAV local position:
        std::optional<px4_msgs::msg::VehicleLocalPosition> local_pos_;
        // UAV status:
        std::optional<px4_msgs::msg::VehicleStatus> vehicle_status_;

        // Parsed initial position of the UAV:
        float uav_init_x_{0.0f};
        float uav_init_y_{0.0f};
        float uav_init_z_{0.0f};

        // Camera offset from the UAV:
        // Posiiton
        float cam_x_offset_;
        float cam_y_offset_;
        float cam_z_offset_;
        // Orientation:
        float cam_roll_;
        float cam_pitch_;
        float cam_yaw_;

        // Temporary position subscriber from the USV position:
        std::string usv_pose_topic_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr usv_pose_sub_;
        std::optional<geometry_msgs::msg::PoseStamped> last_usv_pose_enu_;

        // Last received messages (optional until first message arrives)
        std::mutex mutex_;
        std::optional<sensor_msgs::msg::NavSatFix> last_usv_gps_;
        std::optional<sensor_msgs::msg::NavSatFix> last_uav_gps_;
        std::optional<px4_exec::msg::AprilTagDetection> last_apriltag_info_;

        // Supscripions of the GPS signals:
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr usv_gps_sub_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr uav_gps_sub_;
        // Sucription to the AprilTag detection info:
        rclcpp::Subscription<px4_exec::msg::AprilTagDetection>::SharedPtr apriltag_info_sub_;
        // Subscriptions to the vehicle status adn the position:
        rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;    

        // PUblishers to the PX4 commands and controls:
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;

        // Timer to applyt he control logic:
        rclcpp::TimerBase::SharedPtr timer_;

        // Varible to detect when the offbaord straming is acting:
        bool offboard_streaming_{false};
        // Define how many times hte offboard has the setpoint:
        int offboard_setpoint_counter_{0};
        // Varibale to see if the UAV is armed:
        bool arm_sent_{false};
        // Variable to see if teh offboard is requested:
        bool offboard_requested_{false};

        // Landing tunnables:
        // Takeoff altitde
        double takeoff_alt_m_{5.0};
        // Follow USV horziontal distance:
        double follow_distance_m_{2.0};
        // FOllow altitude:
        double follow_height_m_{5.0};
        // Distance to change from follow to OFFBOARD the USV position
        double handoff_radius_m_{8.0};
        // FollowMe target altitude (absolute) where the USV probably is:
        double target_abs_alt_m_{0.0};
        // Current North position of USV:
        float current_sp_x_{0.f};
        // Current Eat position of USV:
        float current_sp_y_{0.f};
        // Current position on the down axis
        float current_sp_z_{-2.f};
        // Current yaw angle of the USV
        float current_sp_yaw_{0.f};
        // Altitude where its going to start to hover:
        float hover_high_m_{0.75f};
        // Altitude where its going to end hovering before landing:
        float hover_low_m_{0.5f};

        // NUmber of iterations the timer hovers:
        // Maxmium altitude:
        int hover_ticks_{0};
        int low_hover_ticks_{0};

        // Define Initial condiitons for the UAV GPS:
        bool origin_set_{false};
        double origin_lat_deg_{0.0};
        double origin_lon_deg_{0.0};
        double origin_cos_lat_{1.0};
        
        // Following increase percentage ue to segmentate the following commands:
        float alpha_{0.1f};


        // Define the private functions that are going to be used during the landing strategy:
        // Class that define the phase of the landing manneuver:
        enum class Phase {
            WAIT_INPUTS,          // wait for GPS/pose and PX4 status
            ARM,                  // send arm command
            START_SETPOINT_STREAM,// stream setpoints for ~1 sec
            SWITCH_TO_OFFBOARD,   // request offboard mode
            WAIT_OFFBOARD,        // Wait for the Offboard 
            CLIMB,                // go to takeoff_alt_m_
            GOTOUSV,              // Go tot eh USV horizontal positon while maintaning the takeoff alt.
            TRACK_USV_HIGH,       // follow USV x/y at hover_high_m_
            DESCEND_LOW,          // descend to hover_low_m_
            LAND,                 // land command
            DONE  // Finish teh landing
        };

        // Define the pahse class that the contol is work on:
        Phase phase_{Phase::WAIT_INPUTS};



        // Function to change the gps psotiions to position in the refernce frame:
        static void latlon_to_northeats(double lat_deg, double lon_deg, double lat0_deg, double lon0_deg, double cos_lat0,
            double &north_m, double &east_m){
            constexpr double kDegToRad = M_PI / 180.0;
            constexpr double R = 6378137.0;  // Earth radius (WGS84 approx)

            const double dLat = (lat_deg - lat0_deg) * kDegToRad;
            const double dLon = (lon_deg - lon0_deg) * kDegToRad;

            north_m = dLat * R;
            east_m  = dLon * R * cos_lat0;
        }



        // Temporally change the string initial posito of the UAV in numebrs:
        void parse_uav_init_pos(const std::string &pos_str){
            std::stringstream ss(pos_str);
            std::string segment;
            std::vector<float> values;
            while (std::getline(ss, segment, ',')) {
                values.push_back(std::stof(segment));
            }
            // Save the inital values:
            if (values.size() >= 3) {
                uav_init_x_ = values[0];
                uav_init_y_ = values[1];
                uav_init_z_ = values[2];
                // Print to console to verify
                RCLCPP_INFO(this->get_logger(), 
                    "\033[1;32m[UAV Init] Parsed Initial Position -> X: %.2f, Y: %.2f, Z: %.2f\033[0m", 
                    uav_init_x_, uav_init_y_, uav_init_z_);
            }
        }



        // Set origin fromthe UAV Gps:
        void origin_from_uav_gps (const std::optional<sensor_msgs::msg::NavSatFix> &uav_gps){
            // Check if the origin was already set up:
            if (origin_set_ || !uav_gps) return;

            // Save teh original values:
            origin_lat_deg_ = uav_gps->latitude;
            origin_lon_deg_ = uav_gps->longitude;
            origin_cos_lat_ = std::cos(origin_lat_deg_ * M_PI / 180.0);
            origin_set_ = true;
        }



        // Function to detect if the UAV has reached a target:
        bool has_reached_target(double target_x, double target_y, double target_z, double tolerance_m) {
            // Don't do it if there is no stream of the UAV position:
            if (!local_pos_) return false;

            // Cacualt the sitance between teh UAV and the targte 
            double dx = local_pos_->x - target_x;
            double dy = local_pos_->y - target_y;
            double dz = local_pos_->z - target_z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            // Return truw if the distance is lower than a threshold:
            return dist < tolerance_m;
        }



        // Function used in the ROS2 timer that contains the landing logic:
        void thick(){
            // If we are in OFFBOARD phases, continuously stream these (PX4 will drop out if stream stops).
            if (offboard_streaming_) {
                publish_offboard_control_mode();
                publish_trajectory_setpoint(current_sp_x_, current_sp_y_, current_sp_z_, current_sp_yaw_);
                if (offboard_setpoint_counter_ < 11) {
                    offboard_setpoint_counter_++;
                }
            }

            // Create a copy of the actual position inputs:
            std::optional<sensor_msgs::msg::NavSatFix> usv_gps;
            std::optional<sensor_msgs::msg::NavSatFix> uav_gps;
            std::optional<geometry_msgs::msg::PoseStamped> usv_pose_enu;
            std::optional<px4_msgs::msg::VehicleStatus> uav_status;
            std::optional<px4_msgs::msg::VehicleLocalPosition> uav_pose;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (vehicle_status_) uav_status = *vehicle_status_;
                if (local_pos_) uav_pose    = *local_pos_;
                if (last_usv_gps_) usv_gps = *last_usv_gps_;
                if (last_uav_gps_) uav_gps = *last_uav_gps_;
                if (last_usv_pose_enu_) usv_pose_enu = *last_usv_pose_enu_;
            }

            // Detect the east and north position of the USV from the UAV:
            double north_m = 0.0, east_m = 0.0;
            latlon_to_northeats(usv_gps->latitude, usv_gps->longitude, origin_lat_deg_, origin_lon_deg_, origin_cos_lat_, north_m, east_m);

            // After the message from gps are working change to  takeoff:
            switch(phase_){
                // Define the logic to wait for the inputs:
                case Phase::WAIT_INPUTS: {
                    // You need the PX4 status, the USV position and the UAV position to make it work:
                    if (!uav_status || !uav_pose) return;
                    if (!usv_pose_enu) return;

                    // Wait until PX4 can be armed:
                    if (uav_status->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
                        phase_ = Phase::ARM;
                    }
                    return;
                }

                // After the system is armed use the next logic:
                case Phase::ARM: {
                    // If arm is required for irst time run the command:
                    if (!arm_sent_) {
                        publish_vehicle_command(
                            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                            1.0f, 0.0f
                        );
                        arm_sent_ = true;
                    }

                    // Wait until PX4 confirms ARMED
                    if (!uav_status) return;


                    if (uav_status->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
                        // Save the orgin gps position of the uav:
                        origin_from_uav_gps(uav_gps);
                        if (!origin_set_) {
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                                "Waiting for valid UAV GPS to set origin...");
                            return;
                        }

                        // Start setpoint streaming now that we're armed
                        offboard_setpoint_counter_ = 0;
                        offboard_streaming_ = true;

                        // initialize setpoint at CURRENT UAV position (good)
                        current_sp_x_ = static_cast<float>(uav_pose->x);
                        current_sp_y_ = static_cast<float>(uav_pose->y);
                        current_sp_z_ = -static_cast<float>(hover_high_m_);
                        current_sp_yaw_ = 0.0f;

                        phase_ = Phase::START_SETPOINT_STREAM;
                    }
                    return;
                }

                // Phase to star teh USV position straming:
                case Phase::START_SETPOINT_STREAM: {
                    // Stream setpoints for PX4 requirement (~10 msgs)
                    current_sp_x_ = static_cast<float>(uav_pose->x);
                    current_sp_y_ = static_cast<float>(uav_pose->y);
                    current_sp_z_ = -static_cast<float>(hover_high_m_);

                    if (offboard_setpoint_counter_ >= 10) {
                        phase_ = Phase::SWITCH_TO_OFFBOARD;
                    }
                    return;
                }

                // After the postion is streamed change to offboard:
                case Phase::SWITCH_TO_OFFBOARD: {
                    // If offboard mode is requested the first the first time change:
                    if (!offboard_requested_){
                        // Change to offboard mode:
                        publish_vehicle_command(
                            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                            1.0f, 6.0f   
                        );
                        offboard_requested_ = true;
                    }
                    // Change to CLIMB:
                    phase_ = Phase::WAIT_OFFBOARD;
                    return;
                }

                case Phase::WAIT_OFFBOARD: {
                    // Keep streaming setpoints while we wait
                    // Confirm by checking nav_state
                    if (!uav_status) return;

                    if (uav_status->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                        phase_ = Phase::CLIMB;
                        return;
                    }

                    // If it didn’t switch yet, keep requesting occasionally (optional)
                    static int retry = 0;
                    if (++retry % 20 == 0) { // every 2 seconds at 10Hz
                        publish_vehicle_command(
                            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                            1.0f, 6.0f
                        );
                    }
                    return;
                }

                // Climb to a fixed altitude so it can simulate take-off:
                case Phase::CLIMB : {
                    // Hold position above USV at takeoff_alt_m_
                    current_sp_x_ = static_cast<float>(uav_pose->x);
                    current_sp_y_ = static_cast<float>(uav_pose->y);
                    current_sp_z_ = -static_cast<float>(takeoff_alt_m_);

                    // you can use the local altitude to decide when altitude is reqched:
                    if (uav_pose && std::fabs(uav_pose->z-current_sp_z_) < 0.5f) {
                        // Change to the hovering phase:
                        phase_ = Phase::GOTOUSV;
                        hover_ticks_ = 0;
                    }
                    return;
                }

                //Go to the USV in the same climb altitue:
                case  Phase::GOTOUSV: {                    
                    // Update the reference position 
                    current_sp_x_ = current_sp_x_ + (alpha_ * (north_m - current_sp_x_));
                    current_sp_y_ = current_sp_y_ + (alpha_ * (east_m - current_sp_y_));
                    current_sp_z_ = -static_cast<float>(takeoff_alt_m_);

                    // Define if it ghet to the goal:
                    if (has_reached_target(north_m, east_m, -static_cast<float>(takeoff_alt_m_), 0.5)) {
                        phase_ = Phase::TRACK_USV_HIGH;
                    }
                    return;
                }

                // Hover at a hogher altitude fromt eh USV:
                case Phase::TRACK_USV_HIGH: {
                    // SEnd the USV position with the Takeoff altitude:
                    current_sp_x_ = current_sp_x_ + (alpha_ * (north_m - current_sp_x_));
                    current_sp_y_ = current_sp_y_ + (alpha_ * (east_m - current_sp_y_));
                    current_sp_z_ = -hover_high_m_-usv_gps->altitude;

                    // If it hovers for 3 secods change to the phase where it hovers at a lower altitude
                    if (last_apriltag_info_->apriltag_detected) {
                        phase_ = Phase::DESCEND_LOW;
                    }
                    return;
                }
                
                // Phase where it hovers at a lower altitude:
                case Phase::DESCEND_LOW: {
                    // SEnd the USV position with the Takeoff altitude:
                    current_sp_x_ = current_sp_x_ + (alpha_ * (north_m - current_sp_x_));
                    current_sp_y_ = current_sp_y_ + (alpha_ * (east_m - current_sp_y_));
                    current_sp_z_ = -hover_low_m_;

                    // If it hovers at that lower altitude for 3 seconds start landing:
                    if (++low_hover_ticks_ > 50) {
                        phase_ = Phase::LAND;
                    }
                    return;
                }

                // Send the landing command:
                case Phase::LAND: {
                    publish_vehicle_command(
                        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND,
                        0.0f, 0.0f
                    );
                    offboard_streaming_ = false;
                    phase_ = Phase::DONE;
                    return;
                }

                // Tell the system that the landing is already done
                case Phase::DONE:
                default:
                    return;
            }

        }
        



        // Function to publish teh offboard control mode to PX4:
        void publish_offboard_control_mode()
        {
            px4_msgs::msg::OffboardControlMode msg{};
            msg.position = true;
            msg.velocity = false;
            msg.acceleration = false;
            msg.attitude = false;
            msg.body_rate = false;
            msg.thrust_and_torque = false;
            msg.direct_actuator = false;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            offboard_control_mode_pub_->publish(msg);
        }

        

        // Function to publish the setpoint in the offboard:
        void publish_trajectory_setpoint(float x, float y, float z, float yaw)
        {
            px4_msgs::msg::TrajectorySetpoint msg{};
            msg.position = {x, y, z};
            msg.yaw = yaw;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            trajectory_setpoint_pub_->publish(msg);
        }



        // Function to send the evhicle command in the PX4 configuration:
        void publish_vehicle_command(uint16_t command, float param1, float param2)
        {
            px4_msgs::msg::VehicleCommand msg{};
            msg.param1 = param1;
            msg.param2 = param2;
            msg.command = command;
            msg.confirmation = 0;
            msg.target_system = 1;
            msg.target_component = 1;
            msg.source_system = 1;
            msg.source_component = 1;
            msg.from_external = true;
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            vehicle_command_pub_->publish(msg);
        }    
};



// Start the executable:
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UavLandingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



