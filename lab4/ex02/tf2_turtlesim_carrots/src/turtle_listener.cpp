#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "turtlesim/srv/spawn.hpp"


class FrameTurtleListener : public rclcpp::Node {
public:
    
    FrameTurtleListener() : Node("tf2_frame_turtle_listener"),
        turtle_spawning_service_ready_(false), turtle_spawned_(false) 
    {
        target_frame_     = this->declare_parameter<std::string>("target_frame", "turtle1");
        turtle_name_      = this->declare_parameter<std::string>("turtle_name", "turtle2");
        RCLCPP_INFO(this->get_logger(), "Set target frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Set turtle name: %s", turtle_name_.c_str());


        tf_buffer_        = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_trans_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        std::ostringstream cmd_vel_sstream;
        cmd_vel_sstream << "/" << turtle_name_ << "/cmd_vel";

        spawner_srv_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        vel_pub_     = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_sstream.str(), 1);
        timer_       = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FrameTurtleListener::timer_callback, this)); 

    }

private:
    static constexpr double scaleRotationRate = 1.0;
    static constexpr double scaleForwardSpeed = 0.8;

    bool turtle_spawning_service_ready_;
    bool turtle_spawned_;

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawner_srv_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_trans_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::string target_frame_;
    std::string turtle_name_;

    void timer_callback(){
        std::string targetframe_name = target_frame_.c_str();
        std::string workingframe_name = turtle_name_.c_str();

        if(!turtle_spawning_service_ready_){ // Проверяем готовность сервиса для создания черепах
            if(spawner_srv_->service_is_ready()){
                auto req = std::make_shared<turtlesim::srv::Spawn::Request>();
                req->x = 3;
                req->y = 3;
                req->theta = 0;
                req->name = turtle_name_;

                auto response_rec_callback = [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future){
                    auto res = future.get();
                    if(res->name == turtle_name_){
                        turtle_spawning_service_ready_ = true;
                    }
                    else{
                        RCLCPP_ERROR(this->get_logger(), "Service return smt bad");
                    }
                };
                auto res = spawner_srv_->async_send_request(req, response_rec_callback);
            }
            return;
        }
        if(!turtle_spawned_){
            RCLCPP_INFO(this->get_logger(), "Spawned turtle(?)");
            turtle_spawned_ = true;
            return;
        }

        geometry_msgs::msg::TransformStamped trans_msg;

        try { // Проверяем есть ли возможность трансформации между целевым кадром и рабочим
            trans_msg = tf_buffer_->lookupTransform(workingframe_name, targetframe_name, tf2::TimePointZero);
            // tf2::timePointZero - благодаря ему вернется последнее доступное преобразование
        } catch (...){
            RCLCPP_WARN(this->get_logger(), "Couldn't transform %s to %s", targetframe_name.c_str(), workingframe_name.c_str());
            return;
        }

        geometry_msgs::msg::Twist vel_msg;

        vel_msg.angular.z = scaleRotationRate * atan2(trans_msg.transform.translation.y, trans_msg.transform.translation.x);
        vel_msg.linear.x = scaleForwardSpeed * sqrt(pow(trans_msg.transform.translation.x, 2) + pow(trans_msg.transform.translation.y, 2));

        vel_pub_->publish(vel_msg);
    }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameTurtleListener>());
  rclcpp::shutdown();

  return 0;
}