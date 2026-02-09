// Include the C++ libraries:
#include <cstdio>
#include <memory>
#include <string>

// Include ROS2 libraries:
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

// Include the OpenCV library:
#include <opencv2/opencv.hpp>

// Include AprilTag libraies:
#include "apriltag/apriltag.h"
#include "apriltag/common/image_u8.h"
// Remember to add the family of the respective apriltag you are using (tag36h11):
#include "apriltag/tag36h11.h"



// Define a placehodler for the first detection:
using std::placeholders::_1;

// Start the NOde lass:
class AprilTagDetector : public rclcpp::Node{
    public:
        AprilTagDetector() : Node("apriltag_detector"){
            // Define the caera topic:
            uav_camera_topic_ = this->declare_parameter<std::string>("uav_cam_topic", "/uav/camera");

            // Define the APrilTag family::
            april_family_ = tag36h11_create();
            // Define the Detector:
            april_detector_ = apriltag_detector_create();
            // Add the family to the detector:
            apriltag_detector_add_family(april_detector_, april_family_);
            // Configure the detector:
            april_detector_->quad_decimate = resulution_resize_; 
            april_detector_->quad_sigma = gaussian_blur_;   
            april_detector_->nthreads = num_cpu_cores_;        
            april_detector_->debug = detect_debug_;
            april_detector_->refine_edges = refiner_edges_;

            // Subscribe to the image:
            uav_cam_sub_ = this->create_subscription<sensor_msgs::msg::Image>(uav_camera_topic_,10,
                std::bind(&AprilTagDetector::cam_callback, this, _1));

            //////////////// Temporarlly Publish and image to show the AprilTag detection:
            april_tag_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("uav/apriltag_detect", 10);
        }

        // Destroy the detection and ht efamily system:
        ~AprilTagDetector(){
            apriltag_detector_destroy(april_detector_);
            tag36h11_destroy(april_family_);
        }

    
    
    private:
        // AprilTag family:
        apriltag_family_t *april_family_;
        // AprilTag detector:
        apriltag_detector_t *april_detector_;

        // Define the characteristcs of teh AprilTag detector:
        // The speed booster to shrinks hte imahe before looking for the Tags
        float resulution_resize_{4.f};
        // Noise filter:
        float gaussian_blur_{0.0f};
        // Number of CPU cores it acn use:
        int num_cpu_cores_{2};      
        // Boolean variable to save the images:
        int detect_debug_{0};
        // Boolean to use theSharpner of the square segments identified:
        int refiner_edges_{1};

        // UAV camera topic:
        std::string uav_camera_topic_;

        // Camera Subscriber:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr uav_cam_sub_;

        // AprilTag detection image publisher:
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr april_tag_img_pub_;




        // Functions used inside the class:
        // Function to detect the AprilTags every time an image comes:
        void cam_callback(const sensor_msgs::msg::Image::SharedPtr msg){
            // Change teh Image from ROS2 msg to OpenCV Mat:
            cv_bridge::CvImagePtr cv_ptr;
            // Copy the ROS2 image using a pointer:
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }

            // Convert the image to gray scale:
            cv::Mat gray;
            cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
            // Wrap the gray cale image in a image_u8_t struct required by APrilTags:
            image_u8_t img = { gray.cols, gray.rows, gray.cols, gray.data };

            // Use the detector to try and detect the AprilTags of the defined family:
            zarray_t *detections = apriltag_detector_detect(april_detector_, &img);
            // NUmber of apriltags detected:
            int size = zarray_size(detections);
            // Process the AprilTag to highlight them in an image:
            for (int i = 0; i < size; i++){
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                // Dwat the squares that connects the corners:
                cv::line(cv_ptr->image, cv::Point(det->p[0][0], det->p[0][1]),
                     cv::Point(det->p[1][0], det->p[1][1]), cv::Scalar(0, 255, 0), 2);
                cv::line(cv_ptr->image, cv::Point(det->p[1][0], det->p[1][1]),
                        cv::Point(det->p[2][0], det->p[2][1]), cv::Scalar(0, 255, 0), 2);
                cv::line(cv_ptr->image, cv::Point(det->p[2][0], det->p[2][1]),
                        cv::Point(det->p[3][0], det->p[3][1]), cv::Scalar(0, 255, 0), 2);
                cv::line(cv_ptr->image, cv::Point(det->p[3][0], det->p[3][1]),
                        cv::Point(det->p[0][0], det->p[0][1]), cv::Scalar(0, 255, 0), 2);

                // Draw the center:
                cv::circle(cv_ptr->image, cv::Point(det->c[0], det->c[1]), 5, cv::Scalar(0,0,255), -1);
            }

            // Cleanup the memory:
            apriltag_detections_destroy(detections);

            // Publish the apriltag segmented image:
            april_tag_img_pub_->publish(*cv_ptr->toImageMsg());
        }
};


// MAIN CODE LOGIC:
int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagDetector>());
  rclcpp::shutdown();
  return 0;
}