// Libraries from ROS2;
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Include the gz libraries to subscripe to teh node:
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

// Start the Node:
class ModelPosePub : public rclcpp::Node{
    public:
        ModelPosePub() : rclcpp::Node("model_pose_pub"){
            // Define the parameters:
            world_name_ = this->declare_parameter<std::string>("world_name", "sydney_regatta");
            model_name_ = this->declare_parameter<std::string>("model_name", "wamv");
            frame_id_ = this->declare_parameter<std::string>("frame_id", world_name_);

            // Define the ROS2 out topic
            out_topic_ = "/" + model_name_ + "/pose";
            // Gz topic fro where the world positions are publsihed:
            gz_topic_ = this->declare_parameter<std::string>("gz_topic", "/world/" + world_name_ + "/pose/info");

            // Define the publisher ROS2 for the model position:
            pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(out_topic_, 10);

            // SUbscribe to teh gazebo topic of the world position:
            const bool ok = gz_node_.Subscribe(gz_topic_, &ModelPosePub::OnPoseInfo, this);
            // If it can't subscribve send a warning:
            if (!ok) {
                RCLCPP_FATAL(get_logger(), "Failed to subscribe to Gazebo topic: %s", gz_topic_.c_str());
                throw std::runtime_error("gz subscribe failed");
            }
            
            // Print the topic transformation information:
            RCLCPP_INFO(get_logger(),"Subscribed to %s (Gazebo). Filtering name='%s'. Publishing PoseStamped on %s (frame_id='%s')",
                gz_topic_.c_str(), model_name_.c_str(), out_topic_.c_str(), frame_id_.c_str());
        }
    
    private:
        // Parameters:
        std::string world_name_;
        std::string model_name_;
        std::string frame_id_;
        std::string gz_topic_;
        std::string out_topic_;

        // ROS2 publisher:
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

        // GZ transport:
        gz::transport::Node gz_node_;

        

        // Function to publish in teh ROS2 node from teh GZ subscription:
        void OnPoseInfo(const gz::msgs::Pose_V &msg){
            // Go inside of the list that the world pose is publishing:
            for (int i = 0; i < msg.pose_size(); i++){
                const auto &p = msg.pose(i);
                // Only pass if the name of the child id is the one of hte model:
                if (p.name() == model_name_){
                    // Start the ROS2 message that is going to be published:
                    geometry_msgs::msg::PoseStamped out;
                    out.header.stamp = this->now(); 
                    out.header.frame_id = frame_id_;

                    // Define the position:
                    out.pose.position.x = p.position().x();
                    out.pose.position.y = p.position().y();
                    out.pose.position.z = p.position().z();

                    // Define the rotation in quaternion:
                    out.pose.orientation.x = p.orientation().x();
                    out.pose.orientation.y = p.orientation().y();
                    out.pose.orientation.z = p.orientation().z();
                    out.pose.orientation.w = p.orientation().w();

                    // Publish the message
                    pose_pub_->publish(out);
                    return;

                }
            }
        }
};

// Start the node:
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ModelPosePub>());
  rclcpp::shutdown();
  return 0;
}

