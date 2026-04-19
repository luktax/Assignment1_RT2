#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include "action_tutorials_interfaces/action/set_target.hpp"

#include <thread>

class Action_Server : public rclcpp::Node
{
public:
    Action_Server(const rclcpp::NodeOptions & options) : Node("Action_Server", options)
    {
        using namespace std::placeholders;
        this->action_server_ = rclcpp_action::create_server<action_tutorials_interfaces::action::SetTarget>(
            this,
            "set_target",
            std::bind(&Action_Server::handle_goal, this, _1, _2),
            std::bind(&Action_Server::handle_cancel, this, _1),
            std::bind(&Action_Server::handle_accepted, this, _1)
        );
        RCLCPP_INFO(this->get_logger(), "Action_Server has been started.");
    }
private:
    rclcpp_action::Server<action_tutorials_interfaces::action::SetTarget>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const action_tutorials_interfaces::action::SetTarget::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with target position (x: %.2f, y: %.2f, theta: %.2f)", goal->target_x, goal->target_y, goal->target_theta);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_tutorials_interfaces::action::SetTarget>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_tutorials_interfaces::action::SetTarget>> goal_handle)
    {
        std::thread{std::bind(&Action_Server::execute, this, std::placeholders::_1), goal_handle}.detach();
    }
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_tutorials_interfaces::action::SetTarget>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        //rclcpp::Rate loop_rate(1);
        //const auto goal = goal_handle->get_goal();
        //auto feedback = std::make_shared<action_tutorials_interfaces::action::SetTarget::Feedback>();
        //auto result = std::make_shared<action_tutorials_interfaces::action::SetTarget::Result>();

        auto result = std::make_shared<action_tutorials_interfaces::action::SetTarget::Result>();
        result->endposition_x = goal_handle->get_goal()->target_x;
        result->endposition_y = goal_handle->get_goal()->target_y;
        result->endangle_theta = goal_handle->get_goal()->target_theta;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(Action_Server)
        