#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
import numpy as np

def main():
    rclpy.init()
    node = Node("move_robot")

    # Manually specify robot parameters
    joint_positions = [0.35, 0.0, 0.0, 0.0, 0.0, 0.0]  # Adjust as needed
    group_name = "manipulator"  # Change to match your MoveIt group
    base_link = "base_link"  # Update based on your robot
    end_effector = "tool0"  # Update based on your robot
    planner_id = "RRTConnectkConfigDefault"

    # Initialize MoveIt 2 (Ensure joint_names is set manually!)
    moveit2 = MoveIt2(
        node=node,
        joint_names=[
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
        ],  # Update with actual joint names
        base_link_name=base_link,
        end_effector_name=end_effector,
        group_name=group_name,
    )
    moveit2.planner_id = planner_id

    # Set velocity/acceleration scaling
    moveit2.max_velocity = 0.5
    moveit2.max_acceleration = 0.5

    # Move to joint positions
    node.get_logger().info(f"Moving to: {joint_positions}")
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()  # Blocking execution

    # Shutdown and exit
    node.get_logger().info("Motion execution completed.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
