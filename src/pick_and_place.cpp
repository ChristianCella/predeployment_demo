#include <memory>
#include <vector>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <thread>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <abb_robot_msgs/srv/set_io_signal.hpp>

class PickAndPlace {
public:
    PickAndPlace(const rclcpp::Node::SharedPtr &node, bool use_cartesian,
                 const std::vector<double> &home_pose, const std::vector<double> &pre_pick_pose,
                 const std::vector<double> &pick_pose, const std::vector<double> &pre_place_pose,
                 const std::vector<double> &place_pose)
        : node_(node), move_group_(node, "manipulator"), cartesian_poses_(use_cartesian),
          home_pose_(home_pose), pre_pick_pose_(pre_pick_pose), pick_pose_(pick_pose),
          pre_place_pose_(pre_place_pose), place_pose_(place_pose), z_offset_(0.2)
    {
        move_group_.setPlannerId("RRTConnectkConfigDefault");
        move_group_.setMaxVelocityScalingFactor(0.1);
        move_group_.setMaxAccelerationScalingFactor(0.1);
        move_group_.setPoseReferenceFrame("base_link");

        cartesian_max_step_ = 0.0025;
        cartesian_fraction_threshold_ = 0.95;
        cartesian_jump_threshold_ = 0.00;
        cartesian_avoid_collisions_ = false;

        io_signal_client_ = node_->create_client<abb_robot_msgs::srv::SetIOSignal>("/rws_client/set_io_signal");

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
    rclcpp::Client<abb_robot_msgs::srv::SetIOSignal>::SharedPtr io_signal_client_;
    double cartesian_max_step_, cartesian_fraction_threshold_, cartesian_jump_threshold_;
    bool cartesian_avoid_collisions_;

    void set_io_signal(const std::string &signal_name, const std::string &value)
    {
        if (!io_signal_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(node_->get_logger(), "IO signal service is not available!");
            return;
        }
        auto request = std::make_shared<abb_robot_msgs::srv::SetIOSignal::Request>();
        request->signal = signal_name;
        request->value = value;
        io_signal_client_->async_send_request(request);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    void move_to_joint_position(const std::vector<double> &joint_positions)
    {
        move_group_.setJointValueTarget(joint_positions);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
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
        set_io_signal("ABB_Scalable_IO_0_DO1", "1");
        move_to_joint_position(pick_pose_);
        move_to_joint_position(pre_pick_pose_);
        move_to_joint_position(pre_place_pose_);
        move_to_joint_position(place_pose_);
        set_io_signal("ABB_Scalable_IO_0_DO1", "0");
        move_to_joint_position(pre_place_pose_);
        move_to_joint_position(home_pose_);
    }

    void execute_cartesian_pick_and_place() {}
};

class MoveLine {
public:
    MoveLine(const rclcpp::Node::SharedPtr &node, const std::vector<double> &target_pose)
        : node_(node), move_group_(node, "linear_guide"), target_pose_(target_pose)
    {
        move_group_.setPlannerId("RRTConnectkConfigDefault");
        move_group_.setMaxVelocityScalingFactor(0.5);
        move_group_.setMaxAccelerationScalingFactor(0.5);
        move_group_.setPoseReferenceFrame("linear_axis_stator_base_link");
        move_to_joint_position(target_pose_);
    }

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    std::vector<double> target_pose_;

    void move_to_joint_position(const std::vector<double> &joint_positions)
    {
        move_group_.setJointValueTarget(joint_positions);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
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

    // Declare and get correct_rotation and line_targets
    node->declare_parameter<double>("positions.correct_rotation_deg", 90.0);
    node->declare_parameter<std::vector<double>>("positions.line_targets", {});
    double correct_rotation = node->get_parameter("positions.correct_rotation_deg").as_double();
    std::vector<double> line_targets = node->get_parameter("positions.line_targets").as_double_array();

    auto apply_rotation = [&](std::vector<double> vec) {
        vec[0] += correct_rotation;
        for (auto &v : vec) v *= M_PI / 180.0;
        return vec;
    };

    for (size_t i = 0; i < line_targets.size(); ++i)
    {
        std::string base = "positions.poses." + std::to_string(i);
        std::vector<double> home, pre_pick, pick, pre_place, place;

        node->declare_parameter<std::vector<double>>(base + ".home", {});
        node->declare_parameter<std::vector<double>>(base + ".pre_pick", {});
        node->declare_parameter<std::vector<double>>(base + ".pick", {});
        node->declare_parameter<std::vector<double>>(base + ".pre_place", {});
        node->declare_parameter<std::vector<double>>(base + ".place", {});

        node->get_parameter(base + ".home", home);
        node->get_parameter(base + ".pre_pick", pre_pick);
        node->get_parameter(base + ".pick", pick);
        node->get_parameter(base + ".pre_place", pre_place);
        node->get_parameter(base + ".place", place);

        home = apply_rotation(home);
        pre_pick = apply_rotation(pre_pick);
        pick = apply_rotation(pick);
        pre_place = apply_rotation(pre_place);
        place = apply_rotation(place);

        // Move the linear axis to the target position
        auto move_line = std::make_shared<MoveLine>(node, std::vector<double>{line_targets[i]});
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Pick and place the object
        auto pick_and_place = std::make_shared<PickAndPlace>(node, false, home, pre_pick, pick, pre_place, place);
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    rclcpp::shutdown();
    return 0;
}


