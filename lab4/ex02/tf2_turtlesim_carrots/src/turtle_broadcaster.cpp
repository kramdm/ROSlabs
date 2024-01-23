#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"

class DynamicTurtleBroadcater : public  rclcpp::Node {
public:
    DynamicTurtleBroadcater() : Node("turtle_tf2_broadcater"){
        turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle");
        tf_trans_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        std::ostringstream pose_topic_stream;
        pose_topic_stream << "/" << turtlename_.c_str() << "/pose";
        
        msg_pos_sub_ = this->create_subscription<turtlesim::msg::Pose>(pose_topic_stream.str(), 10, std::bind(&DynamicTurtleBroadcater::broadcaster_callback, this, std::placeholders::_1));

    }

private:
    std::string turtlename_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr msg_pos_sub_{nullptr};
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_trans_broadcaster_{nullptr};

    void broadcaster_callback(const std::shared_ptr<turtlesim::msg::Pose> msg){
        geometry_msgs::msg::TransformStamped trans_msg;

        trans_msg.header.stamp = this->get_clock()->now();
        trans_msg.header.frame_id = "world";
        trans_msg.child_frame_id = turtlename_.c_str();

        trans_msg.transform.translation.x = msg->x;
        trans_msg.transform.translation.y = msg->y;
        trans_msg.transform.translation.z = 0.;

        tf2::Quaternion quat {};
        quat.setRPY(0, 0, msg->theta);
        trans_msg.transform.rotation.x = quat.x();
        trans_msg.transform.rotation.y = quat.y();
        trans_msg.transform.rotation.z = quat.z();
        trans_msg.transform.rotation.w = quat.w();

        tf_trans_broadcaster_->sendTransform(trans_msg);

    }
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicTurtleBroadcater>());
    rclcpp::shutdown();

    return 0;
}