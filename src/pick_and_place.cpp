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

// Definition of the Pcik&Place class for the robot
class PickAndPlace
{
public:
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
        // Set planner and movement parameters
        move_group_.setPlannerId("RRTConnectkConfigDefault");
        move_group_.setMaxVelocityScalingFactor(0.1);
        move_group_.setMaxAccelerationScalingFactor(0.1);
        move_group_.setPoseReferenceFrame("base_link");

        // Define Cartesian planning parameters
        cartesian_max_step_ = 0.0025;
        cartesian_fraction_threshold_ = 0.95;
        cartesian_jump_threshold_ = 0.00;
        cartesian_avoid_collisions_ = false;

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
    
    // Declare variables
    bool cartesian_poses_; 
    std::vector<double> home_pose_, pre_pick_pose_, pick_pose_, pre_place_pose_, place_pose_;
    double z_offset_;

    // ABB IO Service Client
    rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedPtr io_signal_client_;

    // Cartesian planning parameters
    double cartesian_max_step_, cartesian_fraction_threshold_, cartesian_jump_threshold_;
    bool cartesian_avoid_collisions_;

    // Function to send IO signal
    void set_io_signal(const std::string &signal_name, const std::string &value)
    {
        // Wait for service to be available, but timeout if it takes too long
        if (!io_signal_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(node_->get_logger(), "IO signal service is not available! Check if ABB RWS Client is running.");
            return;
        }

        // Create the request
        auto request = std::make_shared<abb_robot_msgs::srv::SetIOSignal::Request>();
        request->signal = signal_name;
        request->value = value;

        auto result = io_signal_client_->async_send_request(request);

        RCLCPP_INFO(node_->get_logger(), "SENT REQUEST");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

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

    void move_to_cartesian_pose(const std::vector<double> &pose)
    {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = pose[0];
        target_pose.position.y = pose[1];
        target_pose.position.z = pose[2];
        target_pose.orientation.x = pose[3];
        target_pose.orientation.y = pose[4];
        target_pose.orientation.z = pose[5];
        target_pose.orientation.w = pose[6];

        // Create vector of waypoints
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target_pose);

        // Compute the cartesian path
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_.computeCartesianPath(
            waypoints, cartesian_max_step_, cartesian_jump_threshold_, trajectory, cartesian_avoid_collisions_);

        if (fraction >= cartesian_fraction_threshold_)
        {
            move_group_.execute(trajectory);
            RCLCPP_INFO(node_->get_logger(), "Reached Cartesian pose.");
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to compute full Cartesian path! Executed fraction: %.2f%%", fraction * 100);
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
        move_to_cartesian_pose(pre_pick_pose);

        move_to_cartesian_pose(pick_pose_);

        std::vector<double> post_pick_pose = pick_pose_;
        post_pick_pose[2] += z_offset_;
        move_to_cartesian_pose(post_pick_pose);

        std::vector<double> pre_place_pose = place_pose_;
        pre_place_pose[2] += z_offset_;
        move_to_cartesian_pose(pre_place_pose);

        move_to_cartesian_pose(place_pose_);

        std::vector<double> post_place_pose = place_pose_;
        post_place_pose[2] += z_offset_;
        move_to_cartesian_pose(post_place_pose);

        move_to_joint_position(home_pose_);
    }
};

// Definition of the class to move the line
class MoveLine
{
public:
    explicit MoveLine(
        const rclcpp::Node::SharedPtr &node, 
        const std::vector<double> &target_pose

    )
        : node_(node),
          move_group_(node, "linear_guide"),
          target_pose_(target_pose)
    {
        // Set planner and movement parameters
        move_group_.setPlannerId("RRTConnectkConfigDefault");
        move_group_.setMaxVelocityScalingFactor(0.5);
        move_group_.setMaxAccelerationScalingFactor(0.5);
        move_group_.setPoseReferenceFrame("linear_axis_stator_base_link");

        // Move the line to the required position
        move_to_joint_position(target_pose_);
    }

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    
    // Declare variables
    std::vector<double> target_pose_;

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
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("move_robot_pose");

    double middle_line = 1.5; // meters
    double correct_rotation = 90.0; // degrees

    // Launch the ABB RWS Client
    std::string launch_command = "ros2 launch abb_bringup abb_rws_client.launch.py robot_ip:=192.168.125.1 &";
    std::cout << "Launching ABB RWS Client..." << std::endl;
    
    int result = std::system(launch_command.c_str());  // Launch process in background
    if (result != 0) {
        std::cerr << "Failed to launch ABB RWS Client!" << std::endl;
        return -1; // Exit if launch fails
    }

    // Wait a few seconds to allow the launch process to initialize
    std::this_thread::sleep_for(std::chrono::seconds(2));

    /*
    The first three Pick&Place operations are for the bigger boxes
    */

    // Move the line to the first position
    std::vector<double> target_pose1 = {middle_line - 0.251};
    auto move_line1 = std::make_shared<MoveLine>(node, target_pose1);

    // Wait a few seconds to allow the launch process to initialize
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // First Pick & Place set
    std::vector<double> home_pose1 = {(-90.31 + correct_rotation) * M_PI / 180.0, 11.78 * M_PI / 180.0, 0.21 * M_PI / 180.0, 0.28 * M_PI / 180.0, 76.70 * M_PI / 180.0, -20.20 * M_PI / 180.0};
    std::vector<double> pre_pick_pose1 = {(-15.50 + correct_rotation) * M_PI / 180.0, 53.11 * M_PI / 180.0, -16.97 * M_PI / 180.0, 0.00 * M_PI / 180.0, 53.85 * M_PI / 180.0, -195.50 * M_PI / 180.0};
    std::vector<double> pick_pose1 = {(-15.50 + correct_rotation) * M_PI / 180.0, 62.44 * M_PI / 180.0, -13.69 * M_PI / 180.0, 0.00 * M_PI / 180.0, 41.25 * M_PI / 180.0, -195.50 * M_PI / 180.0};
    std::vector<double> pre_place_pose1 = {(-143.41 + correct_rotation) * M_PI / 180.0, 29.99 * M_PI / 180.0, 34.40 * M_PI / 180.0, 0.00 * M_PI / 180.0, 25.60 * M_PI / 180.0, 36.59 * M_PI / 180.0};
    std::vector<double> place_pose1 = {(-143.41 + correct_rotation) * M_PI / 180.0, 45.69 * M_PI / 180.0, 36.96 * M_PI / 180.0, 0.00 * M_PI / 180.0, 7.36 * M_PI / 180.0, 36.59 * M_PI / 180.0};

    bool use_cartesian1 = false;
    auto pick_and_place1 = std::make_shared<PickAndPlace>(node, use_cartesian1, home_pose1, pre_pick_pose1, pick_pose1, pre_place_pose1, place_pose1);
    

//    rclcpp::shutdown();
//    return 0;

    // Wait a few seconds to allow the launch process to initialize
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Move the line to the second position
    std::vector<double> target_pose2 = {middle_line - 0.269};
    auto move_line2 = std::make_shared<MoveLine>(node, target_pose2);

    // Wait a few seconds to allow the launch process to initialize
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Second Pick&Place set
    std::vector<double> home_pose2 = {(-90.31 + correct_rotation) * M_PI / 180.0, 11.78 * M_PI / 180.0, 0.21 * M_PI / 180.0, 0.28 * M_PI / 180.0, 76.70 * M_PI / 180.0, -20.20 * M_PI / 180.0};
    std::vector<double> pre_pick_pose2 = {(-16.6 + correct_rotation) * M_PI / 180.0, 47.08 * M_PI / 180.0, -4.15 * M_PI / 180.0, 0.00 * M_PI / 180.0, 47.07 * M_PI / 180.0, -196.66 * M_PI / 180.0};
    std::vector<double> pick_pose2 = {(-16.66 + correct_rotation) * M_PI / 180.0, 57.38 * M_PI / 180.0, -1.14 * M_PI / 180.0, 0.00 * M_PI / 180.0, 33.75 * M_PI / 180.0, -196.66 * M_PI / 180.0};
    std::vector<double> pre_place_pose2 = {(-138.76 + correct_rotation) * M_PI / 180.0, 26.16 * M_PI / 180.0, 42.28 * M_PI / 180.0, 0.00 * M_PI / 180.0, 21.55 * M_PI / 180.0, 41.24 * M_PI / 180.0};
    std::vector<double> place_pose2 = {(-138.76 + correct_rotation) * M_PI / 180.0, 43.83 * M_PI / 180.0, 45.03 * M_PI / 180.0, 0.00 * M_PI / 180.0, 1.14 * M_PI / 180.0, 41.24 * M_PI / 180.0};

    bool use_cartesian2 = false;
    auto pick_and_place2 = std::make_shared<PickAndPlace>(node, use_cartesian2, home_pose2, pre_pick_pose2, pick_pose2, pre_place_pose2, place_pose2);

    // Wait a few seconds to allow the launch process to initialize
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Move the line to the third position
    std::vector<double> target_pose3 = {middle_line - 0.270};
    auto move_line3 = std::make_shared<MoveLine>(node, target_pose3);

    // Wait a few seconds to allow the launch process to initialize
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Third Pick&Place set
    std::vector<double> home_pose3 = {(-90.31 + correct_rotation) * M_PI / 180.0, 11.78 * M_PI / 180.0, 0.21 * M_PI / 180.0, 0.28 * M_PI / 180.0, 76.70 * M_PI / 180.0, -20.20 * M_PI / 180.0};
    std::vector<double> pre_pick_pose3 = {(-18.34 + correct_rotation) * M_PI / 180.0, 40.71 * M_PI / 180.0, 9.19 * M_PI / 180.0, 0.00 * M_PI / 180.0, 40.09 * M_PI / 180.0, -198.34 * M_PI / 180.0};
    std::vector<double> pick_pose3 = {(-18.34 + correct_rotation) * M_PI / 180.0, 52.30 * M_PI / 180.0, 12.11 * M_PI / 180.0, 0.00 * M_PI / 180.0, 25.59 * M_PI / 180.0, -198.34 * M_PI / 180.0};
    std::vector<double> pre_place_pose3 = {(-134.23 + correct_rotation) * M_PI / 180.0, 23.34 * M_PI / 180.0, 48.02 * M_PI / 180.0, 0.00 * M_PI / 180.0, 18.64 * M_PI / 180.0, 45.77 * M_PI / 180.0};
    std::vector<double> place_pose3 = {(-134.23 + correct_rotation) * M_PI / 180.0, 42.83 * M_PI / 180.0, 50.97 * M_PI / 180.0, 0.00 * M_PI / 180.0, -3.80 * M_PI / 180.0, 45.77 * M_PI / 180.0};

    bool use_cartesian3 = false;
    auto pick_and_place3 = std::make_shared<PickAndPlace>(node, use_cartesian3, home_pose3, pre_pick_pose3, pick_pose3, pre_place_pose3, place_pose3);

    
    //The second three Pick&Place operations are for the smaller boxes
    // Wait a few seconds to allow the launch process to initialize
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Move the line to the fourth position
    std::vector<double> target_pose4 = {middle_line + 0.150};
    auto move_line4 = std::make_shared<MoveLine>(node, target_pose4);

    // Wait a few seconds to allow the launch process to initialize
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Fourth Pick & Place set
    std::vector<double> home_pose4 = {(-90.31 + correct_rotation) * M_PI / 180.0, 11.78 * M_PI / 180.0, 0.21 * M_PI / 180.0, 0.28 * M_PI / 180.0, 76.70 * M_PI / 180.0, -20.20 * M_PI / 180.0};
    std::vector<double> pre_pick_pose4 = {(-37.43 + correct_rotation) * M_PI / 180.0, 37.23 * M_PI / 180.0, 16.39 * M_PI / 180.0, 0.00 * M_PI / 180.0, 36.38 * M_PI / 180.0, 142.57 * M_PI / 180.0};
    std::vector<double> pick_pose4 = {(-37.43 + correct_rotation) * M_PI / 180.0, 49.69 * M_PI / 180.0, 19.33 * M_PI / 180.0, 0.00 * M_PI / 180.0, 20.98 * M_PI / 180.0, 142.57 * M_PI / 180.0};
    std::vector<double> pre_place_pose4 = {(-135.29 + correct_rotation) * M_PI / 180.0, 26.85 * M_PI / 180.0, 40.87 * M_PI / 180.0, 0.00 * M_PI / 180.0, 22.28 * M_PI / 180.0, 44.71 * M_PI / 180.0};
    std::vector<double> place_pose4 = {(-135.29 + correct_rotation) * M_PI / 180.0, 44.12 * M_PI / 180.0, 43.57 * M_PI / 180.0, 0.00 * M_PI / 180.0, 2.30 * M_PI / 180.0, 44.71 * M_PI / 180.0};

    bool use_cartesian4 = false;
    auto pick_and_place4 = std::make_shared<PickAndPlace>(node, use_cartesian4, home_pose4, pre_pick_pose4, pick_pose4, pre_place_pose4, place_pose4);

    // Wait a few seconds to allow the launch process to initialize
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Move the line to the fifth position
    std::vector<double> target_pose5 = {middle_line + 0.262};
    auto move_line5 = std::make_shared<MoveLine>(node, target_pose5);

    // Wait a few seconds to allow the launch process to initialize
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Fifth Pick&Place set
    std::vector<double> home_pose5 = {(-90.31 + correct_rotation) * M_PI / 180.0, 11.78 * M_PI / 180.0, 0.21 * M_PI / 180.0, 0.28 * M_PI / 180.0, 76.70 * M_PI / 180.0, -20.20 * M_PI / 180.0};
    std::vector<double> pre_pick_pose5 = {(-48.68 + correct_rotation) * M_PI / 180.0, 27.28 * M_PI / 180.0, 36.45 * M_PI / 180.0, 0.00 * M_PI / 180.0, 26.27 * M_PI / 180.0, 131.32 * M_PI / 180.0};
    std::vector<double> pick_pose5 = {(-48.68 + correct_rotation) * M_PI / 180.0, 43.27 * M_PI / 180.0, 39.71 * M_PI / 180.0, 0.00 * M_PI / 180.0, 7.02 * M_PI / 180.0, 131.32 * M_PI / 180.0};
    std::vector<double> pre_place_pose5 = {(-135.97 + correct_rotation) * M_PI / 180.0, 27.30 * M_PI / 180.0, 39.95 * M_PI / 180.0, 0.00 * M_PI / 180.0, 22.74 * M_PI / 180.0, 44.03 * M_PI / 180.0};
    std::vector<double> place_pose5 = {(-135.97 + correct_rotation) * M_PI / 180.0, 44.32 * M_PI / 180.0, 42.64 * M_PI / 180.0, 0.00 * M_PI / 180.0, 3.04 * M_PI / 180.0, 44.03 * M_PI / 180.0};

    bool use_cartesian5 = false;
    auto pick_and_place5 = std::make_shared<PickAndPlace>(node, use_cartesian5, home_pose5, pre_pick_pose5, pick_pose5, pre_place_pose5, place_pose5);

    // Wait a few seconds to allow the launch process to initialize
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Move the line to the sixth position
    std::vector<double> target_pose6 = {middle_line + 0.290};
    auto move_line6 = std::make_shared<MoveLine>(node, target_pose6);

    // Wait a few seconds to allow the launch process to initialize
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Sixth Pick&Place set
    std::vector<double> home_pose6 = {(-90.31 + correct_rotation) * M_PI / 180.0, 11.78 * M_PI / 180.0, 0.21 * M_PI / 180.0, 0.28 * M_PI / 180.0, 76.70 * M_PI / 180.0, -20.20 * M_PI / 180.0};
    std::vector<double> pre_pick_pose6 = {(-58.88 + correct_rotation) * M_PI / 180.0, 22.35 * M_PI / 180.0, 45.98 * M_PI / 180.0, 0.00 * M_PI / 180.0, 21.66 * M_PI / 180.0, 121.12 * M_PI / 180.0};
    std::vector<double> pick_pose6 = {(-58.88 + correct_rotation) * M_PI / 180.0, 41.02 * M_PI / 180.0, 49.60 * M_PI / 180.0, 0.00 * M_PI / 180.0, -0.62 * M_PI / 180.0, 121.12 * M_PI / 180.0};
    std::vector<double> pre_place_pose6 = {(-131.63 + correct_rotation) * M_PI / 180.0, 24.73 * M_PI / 180.0, 45.20 * M_PI / 180.0, 0.00 * M_PI / 180.0, 20.06 * M_PI / 180.0, 48.37 * M_PI / 180.0};
    std::vector<double> place_pose6 = {(-131.63 + correct_rotation) * M_PI / 180.0, 43.27 * M_PI / 180.0, 48.05 * M_PI / 180.0, 0.00 * M_PI / 180.0, -1.32 * M_PI / 180.0, 48.37 * M_PI / 180.0};

    bool use_cartesian6 = false;
    auto pick_and_place6 = std::make_shared<PickAndPlace>(node, use_cartesian6, home_pose6, pre_pick_pose6, pick_pose6, pre_place_pose6, place_pose6);

    rclcpp::shutdown();
    return 0;
}
