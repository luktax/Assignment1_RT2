#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include "action_tutorials_interfaces/action/set_target.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "tf2/utils.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <thread>
#include <algorithm>

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
        
        // subscribe to /odom -> get robot pose
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&Action_Server::odom_callback, this, _1)
        );

        // Initialize cmd_vel publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Action_Server has been started.");
    }
private:
    // variables
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp_action::Server<action_tutorials_interfaces::action::SetTarget>::SharedPtr action_server_;
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;

    const double ANGLE_TOLERANCE = 0.01;    // gradi
    const double POSITION_TOLERANCE = 0.05; // metri

    // FUNCTIONS
    // Action server callbacks
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
        stop_robot();
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_tutorials_interfaces::action::SetTarget>> goal_handle)
    {
        std::thread{std::bind(&Action_Server::execute, this, std::placeholders::_1), goal_handle}.detach();
    }
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_tutorials_interfaces::action::SetTarget>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        rclcpp::Rate loop_rate(10);
        const auto goal = goal_handle->get_goal();
        //auto feedback = std::make_shared<action_tutorials_interfaces::action::SetTarget::Feedback>();
        //auto result = std::make_shared<action_tutorials_interfaces::action::SetTarget::Result>();
        auto result = std::make_shared<action_tutorials_interfaces::action::SetTarget::Result>();


        double target_x = goal->target_x;
        double target_y = goal->target_y;  
        double target_theta = goal->target_theta;

        geometry_msgs::msg::Twist cmd_vel_msg;

        // Navigation algorithm (rotation + straight-line motion + rotation)
        RCLCPP_INFO(this->get_logger(), "Phase 1: Rotating towards target...");

        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                goal_handle->canceled(result);
                return;
            }
            double target_robot_x, target_robot_y;
            transform_to_robot_frame(target_x, target_y, target_robot_x, target_robot_y);


            double angle_to_target = atan2(target_robot_y, target_robot_x)* 180.0 / M_PI; // Convert to degrees

            if (std::abs(angle_to_target) < ANGLE_TOLERANCE) {
                stop_robot();
                break;
            }
            //rotation to the target angle with proportional control
            double angular_vel = 0.02 * angle_to_target; // Proportional control
            angular_vel = std::clamp(angular_vel, -0.5, 0.5);

            if (std::abs(angular_vel) < 0.1) {
                angular_vel = (angular_vel > 0) ? 0.1 : -0.1; // Minimum angular velocity
            }

            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = angular_vel;
            cmd_vel_pub_->publish(cmd_vel_msg);

            loop_rate.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "Phase 1 completed. Position: (x: %.2f, y: %.2f, theta: %.2f°)", 
            current_x_, current_y_, current_theta_);
        RCLCPP_INFO(this->get_logger(), "Phase 2: Moving towards target...");
        //straight-line motion to the target position
        while (rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                goal_handle->canceled(result);
                return;
            }

            double target_robot_x, target_robot_y;
            transform_to_robot_frame(target_x, target_y, target_robot_x, target_robot_y);

            double distance = std::sqrt(target_robot_x * target_robot_x + target_robot_y * target_robot_y);
            if (distance < POSITION_TOLERANCE) {
                stop_robot();
                break;
            } 

            // proportional control for linear velocity
            double linear_vel = 0.5 * distance; // Proportional control
            linear_vel = std::clamp(linear_vel, 0.0, 0.5); // Limit maximum

            cmd_vel_msg.angular.z = 0.0;
            cmd_vel_msg.linear.x = linear_vel;
            cmd_vel_pub_->publish(cmd_vel_msg);

            loop_rate.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "Phase 2 completed. Position: (x: %.2f, y: %.2f, theta: %.2f°)", 
            current_x_, current_y_, current_theta_);
        RCLCPP_INFO(this->get_logger(), "Phase 3: Final rotation...");
        while(rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                goal_handle->canceled(result);
                return;
            }

            //final rotation to the target angle
            double angle_diff = target_theta - current_theta_;

            if (angle_diff > 180) {
                angle_diff -= 360;
            } else if (angle_diff < -180) {
                angle_diff += 360;
            }

            if (std::abs(angle_diff) < ANGLE_TOLERANCE) {
                stop_robot();
                break;
            }

            //rotation to the target angle with proportional control
            double angular_vel = 0.02 * angle_diff; // Proportional control
            angular_vel = std::clamp(angular_vel, -0.5, 0.5);

            if (std::abs(angular_vel) < 0.1) {
                angular_vel = (angular_vel > 0) ? 0.1 : -0.1; // Minimum angular velocity
            }

            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = angular_vel;
            cmd_vel_pub_->publish(cmd_vel_msg);

            loop_rate.sleep();
        }

        RCLCPP_INFO(this->get_logger(), "Phase 3 completed. Position: (x: %.2f, y: %.2f, theta: %.2f°)", 
            current_x_, current_y_, current_theta_);
        stop_robot();
        
        result->endposition_x = current_x_;
        result->endposition_y = current_y_;
        result->endangle_theta = current_theta_;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded! Final: (%.2f, %.2f, %.2f°)",
            current_x_, current_y_, current_theta_ );
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_theta_ = tf2::getYaw(msg->pose.pose.orientation);
        current_theta_ = current_theta_ * 180.0 / M_PI; // Convert to degrees
    }
    void transform_to_robot_frame(double target_x, double target_y, double & transformed_x, double & transformed_y)
    {
        double dx = target_x - current_x_;
        double dy = target_y - current_y_;
        double theta_rad = current_theta_ * M_PI / 180.0; 

        transformed_x = dx * cos(theta_rad) + dy * sin(theta_rad);
        transformed_y = -dx * sin(theta_rad) + dy * cos(theta_rad);
    }
    void stop_robot()
    {
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel_msg);
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(Action_Server)
        