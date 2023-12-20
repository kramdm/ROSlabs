#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_turtle_commands/action/execute_turtle_commands.hpp"

namespace action_turtle_executer
{
    class TurtleActionClient : public rclcpp::Node
    {
    public:
        using ExecuteAction = action_turtle_commands::action::ExecuteTurtleCommands;
        using GoalHandleExecute = rclcpp_action::ClientGoalHandle<ExecuteAction>;

        TurtleActionClient(const rclcpp::NodeOptions& options): Node("execute_action", options){
            this->client_ptr_ = rclcpp_action::create_client<ExecuteAction>(this, "execute_action");
            this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TurtleActionClient::send_goal, this));
        }

        void send_goal(){

            this->timer_->cancel();

            if(!this->client_ptr_->wait_for_action_server()){
                RCLCPP_ERROR(this->get_logger(), "Couldn't connect to server. Shutdown..");
                rclcpp::shutdown();
            }

            send_goals();
        }

    private:

        void send_goals(){
            auto send_goal_options = rclcpp_action::Client<ExecuteAction>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&TurtleActionClient::goal_response_callback, this, std::placeholders::_1);
            send_goal_options.feedback_callback      = std::bind(&TurtleActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback        = std::bind(&TurtleActionClient::result_callback, this, std::placeholders::_1);

            auto goal_msg = ExecuteAction::Goal();
            if (curr_goal_number_ == 0) {
                goal_msg.command = "forward";
                goal_msg.angle = 0;
                goal_msg.s = 2;
                RCLCPP_INFO(this->get_logger(), "Sending goal 1");
            } else if (curr_goal_number_ == 1) {
                goal_msg.command = "turn_right";
                goal_msg.angle = 90;
                goal_msg.s = 0;
                RCLCPP_INFO(this->get_logger(), "Sending goal 2");
            } else if (curr_goal_number_ == 2) {
                goal_msg.command = "forward";
                goal_msg.angle = 0;
                goal_msg.s = 1;
                RCLCPP_INFO(this->get_logger(), "Sending goal 3");
            }
            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
            curr_goal_number_ += 1;
        }

        int curr_goal_number_ = 0;

        void goal_response_callback(const GoalHandleExecute::SharedPtr& goal_handle){
            if(!goal_handle){
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
            }
        }
        void feedback_callback(GoalHandleExecute::SharedPtr, const std::shared_ptr<const ExecuteAction::Feedback> feedback){
            RCLCPP_INFO(this->get_logger(), "Odometer: %d", feedback->odom);
        }

        void result_callback(const GoalHandleExecute::WrappedResult& result){
            switch(result.code){
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded"); break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal aborted"); break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal canceled");
                default:
                    return;
            }
            if(curr_goal_number_ < 3) send_goals();

        }

        rclcpp_action::Client<ExecuteAction>::SharedPtr client_ptr_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

} // namespace action_tutrle_executor

RCLCPP_COMPONENTS_REGISTER_NODE(action_turtle_executer::TurtleActionClient)