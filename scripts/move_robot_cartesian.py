#!/usr/bin/env python3

''' 
This is not the best way to move the REAL robot. You should create a plan.
'''

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
import numpy as np

def main():
    rclpy.init()
    node = Node("move_robot_pose")

    # Manually specify robot parameters
    group_name = "manipulator"  # Change to match your MoveIt group
    base_link = "base_link"  # Update based on your robot
    end_effector = "tool0"  # Update based on your robot
    #planner_id = "ompl"
    planner_id = "RRTConnectkConfigDefault"

    # Define target pose (Modify these values as needed)
    target_position = [0.6764, 0.715961, 0.54281]  # X, Y, Z in meters
    target_orientation = [0.0, 1.0, 0.0, 0.0]  # Quaternion (x, y, z, w)

    # Enable Cartesian Planning
    cartesian = False
    cartesian_max_step = 0.0025
    cartesian_fraction_threshold = 0.95
    cartesian_jump_threshold = 0.05
    cartesian_avoid_collisions = False

    # Initialize MoveIt 2
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
    moveit2.max_velocity = 0.25
    moveit2.max_acceleration = 0.25

    # Set Cartesian planning parameters
    moveit2.cartesian_avoid_collisions = cartesian_avoid_collisions
    moveit2.cartesian_jump_threshold = cartesian_jump_threshold

    # Move to Cartesian pose
    node.get_logger().info(
        f"Moving to Cartesian Pose: Position={target_position}, Orientation={target_orientation}"
    )
    moveit2.move_to_pose(
        position=target_position,
        quat_xyzw=target_orientation,
        cartesian=cartesian,
        cartesian_max_step=cartesian_max_step,
        cartesian_fraction_threshold=cartesian_fraction_threshold,
        tolerance_position = 0.001,
        tolerance_orientation = 0.001,
    )

    # Wait for execution to complete
    moveit2.wait_until_executed()

    # Shutdown and exit
    node.get_logger().info("Motion execution completed.")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
