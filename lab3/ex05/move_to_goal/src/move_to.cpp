
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

#define DISTANCE_ACCURACY  0.1
#define ANGLE_ACCURACY    0.1

class MoveToGoal : public rclcpp::Node {
public:

    MoveToGoal(const rclcpp::NodeOptions& options, int argc, char* argv[]) : Node("move_to", options){
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pose_sub_= this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&MoveToGoal::callback_cmd_pose, this, std::placeholders::_1));        
        if(argc != 4){
            RCLCPP_ERROR(this->get_logger(), "Wrong amount of arguments. Shutdown..\nUsage: X Y theta");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(this->get_logger(), "Succesfully create Move_to_goal node");
        move_to_goal(argv);
    }

private:

    void callback_cmd_pose(const turtlesim::msg::Pose& msg) {(&msg);};

    /*
        Sends a turtlesim Twist message and waits for its execution (Only then returns control)
    */
    void send_message_sync(float linear_speed = 0, float angle_speed = 0){
        geometry_msgs::msg::Twist vel_msg;

        vel_msg.linear.x = linear_speed;
        vel_msg.angular.z = angle_speed;
        cmd_vel_pub_->publish(vel_msg);

        is_moving();
    }

    void send_message_async(float linear_speed = 0, float angle_speed = 0){
        geometry_msgs::msg::Twist vel_msg;

        vel_msg.linear.x = linear_speed;
        vel_msg.angular.z = angle_speed;
        cmd_vel_pub_->publish(vel_msg);
    }

    /*
        Sets turtle position to msg pointer refered as arg
    */
    void get_turtle_pos(turtlesim::msg::Pose* msg){
        rclcpp::MessageInfo minfo;
        while(!pose_sub_->take(*msg, minfo));
    }

    /*
        Returns control to the program further as soon as the turtle finishes moving
    */
    void is_moving(){
        turtlesim::msg::Pose c_pos;
        rclcpp::MessageInfo minfo;
        while(true){
            while(!pose_sub_->take(c_pos, minfo));
            if(!(c_pos.linear_velocity || c_pos.angular_velocity)) break;
        }
    }

    /*
        Calculates distance to target from turtle
    */
    float calc_distance_from_turtle(const float x_target, const float y_target){
        turtlesim::msg::Pose c_pos;
        get_turtle_pos(&c_pos);

        return calc_distance_btw(c_pos.x, c_pos.y, x_target, y_target);
    }


    /*
        Calc distance between two points
    */
    float calc_distance_btw(const float x_1, const float y_1, const float x_2, const float y_2){
        return sqrtf(powf(x_1 - x_2, 2) + powf(y_1 - y_2, 2));
    }

    /*
        Calcs angle to rotate to target coords
    */
    float steer_angle(const float x_del, const float y_del){
        return atan2f(y_del, x_del);
    }

    /*
        Calcs angle for turtle to rotate towards coords
    */
    float calculate_angle_from_turtle(const float x_target, const float y_target){
        turtlesim::msg::Pose c_pos;
        get_turtle_pos(&c_pos);

        float x_delta = (x_target-c_pos.x);
        float y_delta = (y_target-c_pos.y);

        return steer_angle(x_delta, y_delta) - c_pos.theta;
    }

    void move_to_goal(char* argv[]){
        float x_target = atof(argv[1]);
        float y_target = atof(argv[2]);
        float theta = atof(argv[3]);

        rclcpp::Rate message_rate(std::chrono::milliseconds(1000));

        turtlesim::msg::Pose c_pos;
        get_turtle_pos(&c_pos);

        RCLCPP_INFO(this->get_logger(), "Current position %f %f %f", c_pos.x, c_pos.y, c_pos.theta);
        RCLCPP_INFO(this->get_logger(), "Goal set to %f %f %f", x_target, y_target, theta);

        while(calc_distance_from_turtle(x_target, y_target) > DISTANCE_ACCURACY){
            float ang = calculate_angle_from_turtle(x_target, y_target);
            if(fabsf(ang) < ANGLE_ACCURACY){
                RCLCPP_WARN(this->get_logger(), "Ignore angle %f", ang);
                ang = 0;
            }
            float dis = calc_distance_from_turtle(x_target, y_target) * 0.5;
            send_message_async(dis, ang);
            message_rate.sleep();
        }

        get_turtle_pos(&c_pos);

        float target_angle = theta - c_pos.theta;
        send_message_sync(0, target_angle); // Rotate to target angle


        get_turtle_pos(&c_pos);
        RCLCPP_INFO(this->get_logger(), "End position %f %f %f", c_pos.x, c_pos.y, c_pos.theta);
    }

    turtlesim::msg::Pose::SharedPtr turtle_position;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;

};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    std::make_shared<MoveToGoal>(rclcpp::NodeOptions(), argc, argv);
    // rclcpp::spin(std::make_shared<MoveToGoal>(rclcpp::NodeOptions(), argc, argv));
    rclcpp::shutdown();
    return 0;
    

}