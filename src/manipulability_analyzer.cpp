#include <memory>
#include <vector>
#include <map>
#include <chrono>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

using namespace std::chrono_literals;

/*
This class computes the manipulability measure of the robot manipulator.
For the moment this is a stand-alone code. WIP!!
*/

class ManipulabilityAnalyzer
{
public:
    explicit ManipulabilityAnalyzer(const rclcpp::Node::SharedPtr &node)
        : node_(node),
          move_group_(node, "manipulator")
    {
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description");
        robot_model_ = robot_model_loader_->getModel();
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);

        // Timer to recompute manipulability every second
        timer_ = node_->create_wall_timer(1s, std::bind(&ManipulabilityAnalyzer::compute_manipulability, this));
    }

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface move_group_;
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelConstPtr robot_model_;
    std::shared_ptr<moveit::core::RobotState> robot_state_;
    rclcpp::TimerBase::SharedPtr timer_;

    void compute_manipulability()
    {
        // Get joint values from MoveIt interface
        std::vector<double> raw_joint_values = move_group_.getCurrentJointValues();
        if (raw_joint_values.empty()) {
            RCLCPP_WARN(node_->get_logger(), "Joint values are empty. Skipping computation.");
            return;
        }

        const moveit::core::JointModelGroup *joint_model_group = robot_state_->getJointModelGroup(move_group_.getName());
        if (!joint_model_group) {
            RCLCPP_ERROR(node_->get_logger(), "Joint model group not found for group '%s'.", move_group_.getName().c_str());
            return;
        }

        const auto &expected_joint_names = joint_model_group->getVariableNames();
        const auto &actual_joint_names = move_group_.getJointNames();

        if (raw_joint_values.size() != actual_joint_names.size()) {
            RCLCPP_WARN(node_->get_logger(), "Mismatch in joint value size. Expected %zu, got %zu.",
                        actual_joint_names.size(), raw_joint_values.size());
            return;
        }

        // Map joint name → value
        std::map<std::string, double> joint_map;
        for (size_t i = 0; i < actual_joint_names.size(); ++i) {
            joint_map[actual_joint_names[i]] = raw_joint_values[i];
        }

        // Reorder joint values to match expected order
        std::vector<double> ordered_joint_values;
        for (const auto &name : expected_joint_names) {
            if (joint_map.find(name) == joint_map.end()) {
                RCLCPP_WARN(node_->get_logger(), "Missing joint value for: %s", name.c_str());
                return;
            }
            ordered_joint_values.push_back(joint_map[name]);
        }

        // Update robot state
        robot_state_->setJointGroupPositions(joint_model_group, ordered_joint_values);

        // Compute Jacobian
        Eigen::MatrixXd jacobian = robot_state_->getJacobian(joint_model_group);

        // Compute manipulability
        double manipulability = std::abs(jacobian.determinant());

        // Output
        RCLCPP_INFO(node_->get_logger(), "Manipulability Measure (|det(J)|): %f", manipulability);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Sim time enabled
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides({{"use_sim_time", true}});

    auto node = std::make_shared<rclcpp::Node>(
        "manipulability_analyzer",
        node_options.automatically_declare_parameters_from_overrides(true));

    auto analyzer = std::make_shared<ManipulabilityAnalyzer>(node);

    // Keep node alive
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
