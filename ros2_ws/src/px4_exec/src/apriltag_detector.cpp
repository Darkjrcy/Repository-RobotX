// Include the C++ libraries:
#include <cstdio>
#include <memory>
#include <string>

// Include ROS2 libraries:
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "cv_bridge/cv_bridge.h"
// Include your custom messages:
#include "px4_exec/msg/april_tag_detection.hpp"

// Include the OpenCV library:
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Include AprilTag libraies:
#include "apriltag/apriltag.h"
#include "apriltag/common/image_u8.h"
#include "apriltag/apriltag_pose.h"
#include "apriltag/common/matd.h"
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
            uav_camera_info_topic_ = this->declare_parameter<std::string>("uav_cam_info_topic", "/uav/camera_info");

            // Declare the apriltag size in mm:
            apriltag_size_ = this->declare_parameter<double>("apriltag_size", 0.2);

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
            // Subscribe to the image info topic:
            uav_cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(uav_camera_info_topic_, 10,
                std::bind(&AprilTagDetector::cam_info_callback, this, _1));

            // Publish the AprilTag detection info:
            apriltag_detect_info_pub_ = this->create_publisher<px4_exec::msg::AprilTagDetection>("uav/apriltag_info", 10);

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
        // UAV camera info topic:
        std::string uav_camera_info_topic_;

        // Camera Subscriber:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr uav_cam_sub_;
        // Camera Infor Subscriber:
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr uav_cam_info_sub_;
        
        // Boolean to see if the camera information is already obtained:
        bool camera_info_collected{false};
        // Varibles to save the distortion coefficients:
        double fx_, fy_, cx_, cy_;
        // save teh apriltag size in mm
        double apriltag_size_;

        //Define the AprilTag info publisher:
        rclcpp::Publisher<px4_exec::msg::AprilTagDetection>::SharedPtr apriltag_detect_info_pub_;

        // AprilTag detection image publisher:
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr april_tag_img_pub_;


        
        // Functions used inside the class:
        // Function to change from a Rotation matrix to teh quaternions:
        geometry_msgs::msg::Quaternion matd_to_quat(matd_t *R){
            // Create teh quaternion messgae:
            geometry_msgs::msg::Quaternion q;

            // Define each element of the matrix:
            double r11 = MATD_EL(R, 0, 0); double r12 = MATD_EL(R, 0, 1); double r13 = MATD_EL(R, 0, 2);
            double r21 = MATD_EL(R, 1, 0); double r22 = MATD_EL(R, 1, 1); double r23 = MATD_EL(R, 1, 2);
            double r31 = MATD_EL(R, 2, 0); double r32 = MATD_EL(R, 2, 1); double r33 = MATD_EL(R, 2, 2);

            // Define the trace:
            double trace = r11 + r22 + r33;
            // Define a varible that saves the square root of the trace:
            double S;

            // Depending onteh values of teh matrix the method to get the quaternion changes:
            if (trace > 0){
                S = sqrt(trace + 1.0) * 2;
                q.w = 0.25 * S;
                q.x = (r32 - r23) / S;
                q.y = (r13 - r31) / S;
                q.z = (r21 - r12) / S;
            } else if ((r11 > r22) && (r11 > r33)){
                S = sqrt(1.0 + r11 - r22 - r33) * 2;
                q.w = (r32 - r23) / S;
                q.x = 0.25 * S;
                q.y = (r12 + r21) / S;
                q.z = (r13 + r31) / S;
            } else if (r22 > r33) {
                S = sqrt(1.0 + r22 - r11 - r33) * 2;
                q.w = (r13 - r31) / S;
                q.x = (r12 + r21) / S;
                q.y = 0.25 * S;
                q.z = (r23 + r32) / S;
            } else {
                S = sqrt(1.0 + r33 - r11 - r22) * 2;
                q.w = (r21 - r12) / S;
                q.x = (r13 + r31) / S;
                q.y = (r23 + r32) / S;
                q.z = 0.25 * S;
            }
            // return the queaternion
            return q;
        }



        // Function to pass form a 3D point to a 2D point in the camera plane:
        cv::Point2d project_3d_to_pixel(double x, double y, double z, const apriltag_pose_t& pose) {
            // Transform point from Tag Frame to Camera Frame: P_cam = R * P_tag + t
            double P_cam_x = MATD_EL(pose.R, 0, 0)*x + MATD_EL(pose.R, 0, 1)*y + MATD_EL(pose.R, 0, 2)*z + MATD_EL(pose.t, 0, 0);
            double P_cam_y = MATD_EL(pose.R, 1, 0)*x + MATD_EL(pose.R, 1, 1)*y + MATD_EL(pose.R, 1, 2)*z + MATD_EL(pose.t, 1, 0);
            double P_cam_z = MATD_EL(pose.R, 2, 0)*x + MATD_EL(pose.R, 2, 1)*y + MATD_EL(pose.R, 2, 2)*z + MATD_EL(pose.t, 2, 0);

            // Project to Pixel: u = (fx * x / z) + cx
            double u = (fx_ * P_cam_x / P_cam_z) + cx_;
            double v = (fy_ * P_cam_y / P_cam_z) + cy_;

            return cv::Point2d(u, v);
        }



        // Function to obtain the cmaera info inlcuiding the distortion matrix, coefficient etc:
        void cam_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
            // Obatin the distortion coefficients:
            if (camera_info_collected){
                return;
            }
            // The distorsion coefficientsa re saved as:
            fx_ = msg->k[0];
            cx_ = msg->k[2];
            fy_ = msg->k[4];
            cy_ = msg->k[5];
            camera_info_collected = true;
        }



        // Function to detect the AprilTags every time an image comes:
        void cam_callback(const sensor_msgs::msg::Image::SharedPtr msg){
            // Firs identify if the camera info was collected:
            if (!camera_info_collected){
                return;
            }

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
            
            // Send an empty message in case no apriltags were detected:
            if (size == 0){
                px4_exec::msg::AprilTagDetection msg_out;
                msg_out.header = msg->header;
                msg_out.apriltag_detected = false;
                apriltag_detect_info_pub_->publish(msg_out);
            }

            // Process the AprilTag to highlight them in an image:
            for (int i = 0; i < size; i++){
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                // Define the characteristics of the AprilTag:
                apriltag_detection_info_t info;
                info.det = det;
                info.tagsize = apriltag_size_;
                info.fx = fx_;
                info.fy = fy_;
                info.cx = cx_;
                info.cy = cy_;

                // Determine the relative position of the AprilTag:
                apriltag_pose_t pose;
                estimate_tag_pose(&info, &pose);

                // Craete a message to send the APrilTag info:
                px4_exec::msg::AprilTagDetection msg_out;
                msg_out.header = msg->header;
                msg_out.apriltag_detected = true;
                // Publsih position:
                msg_out.position.x = MATD_EL(pose.t, 0, 0);
                msg_out.position.y = MATD_EL(pose.t, 1, 0);
                msg_out.position.z = MATD_EL(pose.t, 2, 0);
                // Publish orientation
                msg_out.orientation = matd_to_quat(pose.R);
                // Publish the message:
                apriltag_detect_info_pub_->publish(msg_out);

                // Draw the squares that connects the corners:
                cv::line(cv_ptr->image, cv::Point(det->p[0][0], det->p[0][1]),
                     cv::Point(det->p[1][0], det->p[1][1]), cv::Scalar(0, 255, 0), 2);
                cv::line(cv_ptr->image, cv::Point(det->p[1][0], det->p[1][1]),
                        cv::Point(det->p[2][0], det->p[2][1]), cv::Scalar(0, 255, 0), 2);
                cv::line(cv_ptr->image, cv::Point(det->p[2][0], det->p[2][1]),
                        cv::Point(det->p[3][0], det->p[3][1]), cv::Scalar(0, 255, 0), 2);
                cv::line(cv_ptr->image, cv::Point(det->p[3][0], det->p[3][1]),
                        cv::Point(det->p[0][0], det->p[0][1]), cv::Scalar(0, 255, 0), 2);

                // Draw the axis of the AprilTag:
                // Define the length of the axis witht he length:
                double len = apriltag_size_ / 2.0;
                // Draw the origin:
                cv::Point2d p_origin = project_3d_to_pixel(0, 0, 0, pose);
                // ThE X-axis point:
                cv::Point2d p_x = project_3d_to_pixel(len, 0, 0, pose);
                // Y-Axis point:
                cv::Point2d p_y = project_3d_to_pixel(0, len, 0, pose);
                // Z-Axis point:
                cv::Point2d p_z = project_3d_to_pixel(0, 0, -len, pose);
                // Draw it using cv2:
                cv::line(cv_ptr->image, p_origin, p_x, cv::Scalar(0, 0, 255), 2); 
                cv::line(cv_ptr->image, p_origin, p_y, cv::Scalar(0, 255, 0), 2);
                cv::line(cv_ptr->image, p_origin, p_z, cv::Scalar(255, 0, 0), 2);
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