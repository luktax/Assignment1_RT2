#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include "action_msgs/srv/cancel_goal.hpp"
#include <iostream>
#include <thread>
#include <atomic>
#include <limits>

#include "action_tutorials_interfaces/action/set_target.hpp"

class UI_Node : public rclcpp::Node
{
public:
    explicit UI_Node(const rclcpp::NodeOptions & options) : Node("UI_Node", options), running_(true)
    {   
        RCLCPP_INFO(this->get_logger(), "UI_Node has been started.");
        this->client_ = rclcpp_action::create_client<action_tutorials_interfaces::action::SetTarget>(this, "set_target");
        input_thread_ = std::thread(&UI_Node::input_loop, this);
    }
    ~UI_Node()
    {
        running_ = false;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }
private:
    void input_loop()
    {
        while(running_ && rclcpp::ok()) {
            std::string input;
            std::cout << "Enter '1' to set a new target position (x y theta), enter '2' to cancel the current goal or '3' to quit: ";
            std::getline(std::cin, input);
            if (input == "3") {
                running_ = false;
                rclcpp::shutdown();
                break;
            }
            else if (input == "2") {
                // Handle canceling current goal
                cancel_goal();
            }
            else if (input == "1") {
                // Handle setting new target position
                request_new_goal();
            }
            else if(input != "1" && input != "2" && input != "3") {
                std::cout << "Invalid input. Please enter '1', '2', or '3'." << std::endl;
            } 
        }
    }
    void request_new_goal()
    {
        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available!");
            return;
        }
        auto goal_msg = action_tutorials_interfaces::action::SetTarget::Goal();

        float x, y, theta;
        std::cout << "Enter target position x: ";
        while (!(std::cin >> x) || x < -10 || x > 10) {
            std::cout << "Invalid input. Please enter a number between -10 and 10 for x: ";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        std::cout << "Enter target position y: ";
        while (!(std::cin >> y) || y < -10 || y > 10) {
            std::cout << "Invalid input. Please enter a number between -10 and 10 for y: ";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
        std::cout << "Enter target position theta: ";
        while (!(std::cin >> theta) || theta < -90 || theta > 90) {
            std::cout << "Invalid input. Please enter a number between -90 and 90 for theta: ";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }

        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        goal_msg.target_x = x;
        goal_msg.target_y = y;
        goal_msg.target_theta = theta;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<action_tutorials_interfaces::action::SetTarget>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&UI_Node::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&UI_Node::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&UI_Node::result_callback, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, send_goal_options);

    }
    void goal_response_callback(const rclcpp_action::ClientGoalHandle<action_tutorials_interfaces::action::SetTarget>::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            current_goal_handle_ = goal_handle;
        }
    }
    void feedback_callback(rclcpp_action::ClientGoalHandle<action_tutorials_interfaces::action::SetTarget>::SharedPtr, const std::shared_ptr<const action_tutorials_interfaces::action::SetTarget::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback: current position (x: %.2f, y: %.2f, theta: %.2f)", feedback->current_x, feedback->current_y, feedback->current_theta);
    }
    void result_callback(const rclcpp_action::ClientGoalHandle<action_tutorials_interfaces::action::SetTarget>::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded! Final position (x: %.2f, y: %.2f, theta: %.2f)", result.result->endposition_x, result.result->endposition_y, result.result->endangle_theta);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        current_goal_handle_.reset(); 
    }
    void cancel_goal()
    {
        if (!current_goal_handle_) {
            RCLCPP_WARN(this->get_logger(), "No active goal to cancel!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Requesting goal cancellation...");
        auto cancel_future = client_->async_cancel_goal(
            current_goal_handle_,
            std::bind(&UI_Node::cancel_response_callback, this, std::placeholders::_1)
        );
    }
    void cancel_response_callback(const std::shared_ptr<action_msgs::srv::CancelGoal::Response> response)
    {
        if (response->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
            RCLCPP_INFO(this->get_logger(), "Goal cancellation successful");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Goal cancellation failed with return code: %d", response->return_code);
        }
    }
    rclcpp_action::Client<action_tutorials_interfaces::action::SetTarget>::SharedPtr client_;
    rclcpp_action::ClientGoalHandle<action_tutorials_interfaces::action::SetTarget>::SharedPtr current_goal_handle_;
    std::thread input_thread_;
    std::atomic<bool> running_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(UI_Node)
