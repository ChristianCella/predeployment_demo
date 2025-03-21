#include <memory>
#include <vector>
#include <cmath>
#include <chrono>
#include <cstdlib>  // Required for std::system()
#include <thread>   // For sleep
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <abb_robot_msgs/srv/set_io_signal.hpp>  // Correct ABB IO service include

// The service is defined here: /home/kriscell/Projects/mics_ws/src/abb_libs/abb_ros2_msgs/abb_robot_msgs/srv/SetIOSignal.srv

class PickAndPlace
{
public:
    // Constructor to initialize the PickAndPlace class
    explicit PickAndPlace(
        const rclcpp::Node::SharedPtr &node, 
        bool use_cartesian, 
        const std::vector<double> &home_pose,
        const std::vector<double> &pre_pick_pose,
        const std::vector<double> &pick_pose,
        const std::vector<double> &pre_place_pose,
        const std::vector<double> &place_pose
    )
        : node_(node),
          move_group_(node, "manipulator"),
          cartesian_poses_(use_cartesian),
          home_pose_(home_pose),
          pre_pick_pose_(pre_pick_pose),
          pick_pose_(pick_pose),
          pre_place_pose_(pre_place_pose),
          place_pose_(place_pose),
          z_offset_(0.2) // Default Z-offset
    {
        // Initialize service client for ABB IO control
        io_signal_client_ = node_->create_client<abb_robot_msgs::srv::SetIOSignal>("/rws_client/set_io_signal");

        // Execute Pick & Place operation
        if (cartesian_poses_)
            execute_cartesian_pick_and_place();
        else
            execute_joint_pick_and_place();
    }

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    
    bool cartesian_poses_;
    std::vector<double> home_pose_, pre_pick_pose_, pick_pose_, pre_place_pose_, place_pose_;
    double z_offset_;

    // ABB IO Service Client
    rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedPtr io_signal_client_;

    // Function to send IO signal
    void set_io_signal(const std::string &signal_name, const std::string &value)
    {
        // Wait for service to be available, but timeout if it takes too long
        if (!io_signal_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_->get_logger(), "IO signal service is not available! Check if ABB RWS Client is running.");
            return;
        }

        // Create the request
        auto request = std::make_shared<abb_robot_msgs::srv::SetIOSignal::Request>();
        request->signal = signal_name;
        request->value = value;

        // Call the service asynchronously
        auto future = io_signal_client_->async_send_request(request);

        // Wait for the result (timeout after 5 seconds)
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
        {
            try {
                auto response = future.get();
                if (response->result_code == 1) { // Success if result_code == 1, verified from terminal
                    RCLCPP_INFO(node_->get_logger(), "Successfully set IO signal: %s to %s", signal_name.c_str(), value.c_str());
                } else {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to set IO signal: %s. Error Code: %d, Message: %s", 
                                signal_name.c_str(), response->result_code, response->message.c_str());
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(node_->get_logger(), "Exception while calling IO signal service: %s", e.what());
            }
        }
        else {
            RCLCPP_ERROR(node_->get_logger(), "Timeout waiting for IO signal service response!");
        }

        // Small delay to ensure IO is processed before continuing
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }


    void move_to_joint_position(const std::vector<double> &joint_positions)
    {
        move_group_.setJointValueTarget(joint_positions);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success)
        {
            move_group_.execute(plan);
            RCLCPP_INFO(node_->get_logger(), "Reached joint position.");
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to plan to joint position.");
        }
    }

    void execute_joint_pick_and_place()
    {
        move_to_joint_position(home_pose_);
        move_to_joint_position(pre_pick_pose_);

        // Activate gripper (open)
        set_io_signal("ABB_Scalable_IO_0_DO1", "1");

        move_to_joint_position(pick_pose_);

        move_to_joint_position(pre_pick_pose_);
        move_to_joint_position(pre_place_pose_);
        move_to_joint_position(place_pose_);

        // Deactivate gripper at place
        set_io_signal("ABB_Scalable_IO_0_DO1", "0");

        move_to_joint_position(pre_place_pose_);
        move_to_joint_position(home_pose_);
    }

    void execute_cartesian_pick_and_place()
    {
        move_to_joint_position(home_pose_);

        std::vector<double> pre_pick_pose = pick_pose_;
        pre_pick_pose[2] += z_offset_;
        move_to_joint_position(pre_pick_pose);

        move_to_joint_position(pick_pose_);

        std::vector<double> post_pick_pose = pick_pose_;
        post_pick_pose[2] += z_offset_;
        move_to_joint_position(post_pick_pose);

        std::vector<double> pre_place_pose = place_pose_;
        pre_place_pose[2] += z_offset_;
        move_to_joint_position(pre_place_pose);

        move_to_joint_position(place_pose_);

        std::vector<double> post_place_pose = place_pose_;
        post_place_pose[2] += z_offset_;
        move_to_joint_position(post_place_pose);

        move_to_joint_position(home_pose_);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("move_robot_pose");

    // Launch the ABB RWS Client
    std::string launch_command = "ros2 launch abb_bringup abb_rws_client.launch.py robot_ip:=192.168.125.1 &";
    std::cout << "Launching ABB RWS Client..." << std::endl;
    
    int result = std::system(launch_command.c_str());  // Launch process in background
    if (result != 0) {
        std::cerr << "Failed to launch ABB RWS Client!" << std::endl;
        return -1; // Exit if launch fails
    }

    // Wait a few seconds to allow the launch process to initialize
    std::this_thread::sleep_for(std::chrono::seconds(5));

    double correct_rotation = -90.0; // degrees

    // Define pick and place positions
    std::vector<double> home_pose1 = {(-90.31 + correct_rotation) * M_PI / 180.0, 11.78 * M_PI / 180.0, 0.21 * M_PI / 180.0, 0.28 * M_PI / 180.0, 76.70 * M_PI / 180.0, -20.20 * M_PI / 180.0};
    std::vector<double> pre_pick_pose1 = {(-15.50 + correct_rotation) * M_PI / 180.0, 53.11 * M_PI / 180.0, -16.97 * M_PI / 180.0, 0.00 * M_PI / 180.0, 53.85 * M_PI / 180.0, -195.50 * M_PI / 180.0};
    std::vector<double> pick_pose1 = {(-15.50 + correct_rotation) * M_PI / 180.0, 62.44 * M_PI / 180.0, -13.69 * M_PI / 180.0, 0.00 * M_PI / 180.0, 41.25 * M_PI / 180.0, -195.50 * M_PI / 180.0};
    std::vector<double> pre_place_pose1 = {(-143.41 + correct_rotation) * M_PI / 180.0, 29.99 * M_PI / 180.0, 34.40 * M_PI / 180.0, 0.00 * M_PI / 180.0, 25.60 * M_PI / 180.0, 36.59 * M_PI / 180.0};
    std::vector<double> place_pose1 = {(-143.41 + correct_rotation) * M_PI / 180.0, 45.69 * M_PI / 180.0, 36.96 * M_PI / 180.0, 0.00 * M_PI / 180.0, 7.36 * M_PI / 180.0, 36.59 * M_PI / 180.0};

    auto pick_and_place1 = std::make_shared<PickAndPlace>(node, false, home_pose1, pre_pick_pose1, pick_pose1, pre_place_pose1, place_pose1);

    rclcpp::shutdown();
    return 0;
}
