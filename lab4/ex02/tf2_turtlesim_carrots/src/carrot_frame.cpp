#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class CarrotBroadcaster : public rclcpp::Node {
public:
    CarrotBroadcaster() : Node("carry_frame_tf2_broadcaster"), point_index(0) {
        
        radius_                = this->declare_parameter<double>("radius", 1);
        direction_of_rotation_ = this->declare_parameter<double>("direction_of_rotation", 1);
        RCLCPP_INFO(this->get_logger(), "Set radius %f", radius_);

        tf_trans_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_                = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CarrotBroadcaster::timer_callback, this));

        for(double m = 0; m < 2 * M_PI; m+=0.1){
            theta_.push_back(m);
        }
        
    }


private:

    void timer_callback(){
        point_index %= theta_.size();
        point_index += 1;

        rclcpp::Time now = this->get_clock()->now();
        double x = radius_ * sin(theta_[point_index]);
        double y = radius_ * cos(theta_[point_index]);

        geometry_msgs::msg::TransformStamped trans_msg;

        trans_msg.header.stamp = now;
        trans_msg.header.frame_id = "turtle1";
        trans_msg.child_frame_id = "carrot1";

        trans_msg.transform.translation.x = x;
        trans_msg.transform.translation.y = y;
        trans_msg.transform.translation.z = 0.0;
        trans_msg.transform.rotation.x = 0.0;
        trans_msg.transform.rotation.y = 0.0;
        trans_msg.transform.rotation.z = 0.0;
        trans_msg.transform.rotation.w = direction_of_rotation_;

        tf_trans_broadcaster_->sendTransform(trans_msg);
    }

    double direction_of_rotation_;
    double radius_;
    std::vector<double> theta_;
    size_t point_index;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_trans_broadcaster_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarrotBroadcaster>());
  rclcpp::shutdown();

  return 0;
}
