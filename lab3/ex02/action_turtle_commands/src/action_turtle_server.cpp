#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "turtlesim/msg/pose.hpp"

#include "action_turtle_commands/action/execute_turtle_commands.hpp"

namespace action_turtle_executer
{

  const int FORWARD_STEP = 1;

class TurtleActionServer : public rclcpp::Node
{
public:
  using ExecuteAction = action_turtle_commands::action::ExecuteTurtleCommands;
  using GoalHandleExecute = rclcpp_action::ServerGoalHandle<ExecuteAction>;

   TurtleActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("execute_action_server", options) {
    action_server_ = rclcpp_action::create_server<ExecuteAction>(this, "execute_action",
      std::bind(&TurtleActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TurtleActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&TurtleActionServer::handle_accepted, this, std::placeholders::_1));
    
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
    pose_sub_= this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&TurtleActionServer::callback_cmd_pose, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<ExecuteAction>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;

  int global_odometer = 0;

  void callback_cmd_pose(const turtlesim::msg::Pose& msg) {(&msg);};

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

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Get goal request (%s %d %d)", goal->command.c_str(), goal->s, goal->angle);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecute> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleExecute> goal_handle){
    std::thread{std::bind(&TurtleActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleExecute> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();

    geometry_msgs::msg::Twist cmd_vel_msg;
    auto feedback = std::make_shared<ExecuteAction::Feedback>();
    auto result = std::make_shared<ExecuteAction::Result>();

    result->result = false;
    feedback->odom = global_odometer;

    is_moving();

    if(goal->command == "forward"){ // Move forward
      RCLCPP_INFO(this->get_logger(), "Moving forward on %d meters", goal->s);
      int step_size = action_turtle_executer::FORWARD_STEP;
      cmd_vel_msg.linear.x = (float)step_size;

        for(int moved = 0; moved < goal->s; moved+= step_size){
          global_odometer += step_size;
          cmd_vel_pub_->publish(cmd_vel_msg);
          feedback->odom = global_odometer;
          goal_handle->publish_feedback(feedback);
          is_moving();
      }
    }
    else if (goal->command == "turn_right") { // Rotate right
      RCLCPP_INFO(this->get_logger(), "Rotating right on %d angle", goal->angle);
      cmd_vel_msg.angular.z = -M_PI/2;
      cmd_vel_pub_->publish(cmd_vel_msg);
      goal_handle->publish_feedback(feedback);
    }
    else if (goal->command == "turn_left") { // Rotate left
      RCLCPP_INFO(this->get_logger(), "Rotating left on %d angle", goal->angle);
      cmd_vel_msg.angular.z = M_PI/2;
      cmd_vel_pub_->publish(cmd_vel_msg);
      goal_handle->publish_feedback(feedback);
    }
    
    is_moving();

    if (rclcpp::ok()) {
      result->result = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(action_turtle_executer::TurtleActionServer)