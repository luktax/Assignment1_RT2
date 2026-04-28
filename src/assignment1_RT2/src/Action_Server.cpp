#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include "action_tutorials_interfaces/action/set_target.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "tf2/utils.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

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

        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Action_Server has been started.");
    }
private:
    // VARIABLES
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp_action::Server<action_tutorials_interfaces::action::SetTarget>::SharedPtr action_server_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // robot pose
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_theta_ = 0.0;
    // thresholds 
    const double ANGLE_TOLERANCE = 0.01;    // rad
    const double POSITION_TOLERANCE = 0.05; // metri
    // control gains
    double k_linear = 0.5;  // Proportional control gain for linear velocity
    double k_angular = 0.5; // Proportional control gain for angular velocity
    // Cartesian error
    struct CartesianError {
        double distance;
        double angle_to_goal;
        double orientation_err;
        bool valid;
    };

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
        auto feedback = std::make_shared<action_tutorials_interfaces::action::SetTarget::Feedback>();
        auto result = std::make_shared<action_tutorials_interfaces::action::SetTarget::Result>();

        double target_x = goal->target_x;
        double target_y = goal->target_y;  
        double target_theta = goal->target_theta * M_PI / 180.0;

        bool goal_reached = false;

        geometry_msgs::msg::Twist cmd_vel_msg;

        publishGoalFrame(target_x, target_y, target_theta);


        /* 
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
        */

        // Control loop
        while (!goal_reached && rclcpp::ok()) {            
            if (goal_handle->is_canceling()){
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return; 
            }
            CartesianError error = computeCartesiaError();
            if (!error.valid) {
                RCLCPP_WARN(this->get_logger(), "Invalid Cartesian error, stopping robot");
                stop_robot();
                loop_rate.sleep();
                continue;
            }
            cmd_vel_msg.linear.x = 0;
            cmd_vel_msg.angular.z = 0;

            if (error.distance > POSITION_TOLERANCE) {
                // angle factor to reduce linear velocity when the robot is not facing the target
                double angle_factor = std::cos(error.angle_to_goal);
                angle_factor = std::max(0.0, angle_factor); // Ensure non-negative

                cmd_vel_msg.linear.x = k_linear * error.distance * angle_factor;

                cmd_vel_msg.angular.z = k_angular * error.angle_to_goal;

                RCLCPP_DEBUG(this->get_logger(), 
                    "MOVING: dist=%.3f, angle_to_goal=%.3f, factor=%.2f",
                    error.distance, error.angle_to_goal * 180.0 / M_PI, angle_factor);
            }
            else {
                // If we are close enough to the target position, focus on orientation
                if (std::abs(error.orientation_err) > ANGLE_TOLERANCE) {
                    cmd_vel_msg.linear.x = 0.0;
                    cmd_vel_msg.angular.z = k_angular * error.orientation_err;
                    RCLCPP_DEBUG(this->get_logger(), 
                        "ORIENTING: orientation_err=%.3f",
                        error.orientation_err * 180.0 / M_PI);
                }
                else {
                    // GOAL REACHED
                    goal_reached = true;
                    RCLCPP_INFO(this->get_logger(), "Goal reached! Position: (x: %.2f, y: %.2f, theta: %.2f°)", 
                        current_x_, current_y_, current_theta_ * 180.0 / M_PI);
                }
            }

            cmd_vel_msg.linear.x = std::clamp(cmd_vel_msg.linear.x, 0.0, 1.0);
            cmd_vel_msg.angular.z = std::clamp(cmd_vel_msg.angular.z, -1.0, 1.0);
            cmd_vel_pub_->publish(cmd_vel_msg);

            feedback->current_x = current_x_;
            feedback->current_y = current_y_;
            feedback->current_theta = current_theta_ * 180.0 / M_PI;
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }
        
        stop_robot();
        result->endposition_x = current_x_;
        result->endposition_y = current_y_;
        result->endangle_theta = current_theta_ * 180.0 / M_PI;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded! Final: (%.2f, %.2f, %.2f°)",
            current_x_, current_y_, current_theta_ * 180.0 / M_PI);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_theta_ = yaw;


        // publish the frame transformation from "world" to "base_footprint"
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = msg->header.stamp;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "base_footprint";

        transformStamped.transform.translation.x = current_x_;
        transformStamped.transform.translation.y = current_y_;
        transformStamped.transform.translation.z = 0.0;

        transformStamped.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(transformStamped);
    }

    /*void transform_to_robot_frame(double target_x, double target_y, double & transformed_x, double & transformed_y)
    {
        double dx = target_x - current_x_;
        double dy = target_y - current_y_;
        double theta_rad = current_theta_ * M_PI / 180.0; 

        transformed_x = dx * cos(theta_rad) + dy * sin(theta_rad);
        transformed_y = -dx * sin(theta_rad) + dy * cos(theta_rad);
    }*/

    void stop_robot()
    {
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel_msg);
    }

    void publishGoalFrame(double target_x, double target_y, double target_theta)
    {
       // publish the goal frame transformation from "world" to "goal"
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "goal";

        //traslation
        transformStamped.transform.translation.x = target_x;
        transformStamped.transform.translation.y = target_y;
        transformStamped.transform.translation.z = 0.0;

        //rotation
        tf2::Quaternion q;
        q.setRPY(0, 0, target_theta); 

        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        tf_static_broadcaster_->sendTransform(transformStamped);
    }

    CartesianError computeCartesiaError()
    {
        CartesianError error;
        error.valid = false;

        geometry_msgs::msg::TransformStamped base_to_goal;
        try {
            base_to_goal = tf_buffer_->lookupTransform("base_footprint", "goal", tf2::TimePointZero);
        } catch (tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform from 'base_footprint' to 'goal': %s", ex.what());
            return error;
        }

        double error_x = base_to_goal.transform.translation.x;
        double error_y = base_to_goal.transform.translation.y;
        error.distance = std::sqrt(error_x * error_x + error_y * error_y);

        error.angle_to_goal = std::atan2(error_y, error_x);

        tf2::Quaternion q(
            base_to_goal.transform.rotation.x,
            base_to_goal.transform.rotation.y,
            base_to_goal.transform.rotation.z,
            base_to_goal.transform.rotation.w);
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        error.orientation_err = normalizeAngle(yaw);

        error.valid = true;
        return error;
    }
    double normalizeAngle(double angle)
    {
        // Normalizza l'angolo tra -180 e 180 gradi
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(Action_Server)
        