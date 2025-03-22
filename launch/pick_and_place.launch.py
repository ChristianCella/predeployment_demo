from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Locate the YAML file inside the config folder of your package
    config_file = PathJoinSubstitution([
        FindPackageShare('predeployment_demo'),  # Replace if your package is named differently
        'config',
        'joint_positions.yaml'
    ])

    return LaunchDescription([
        Node(
            package='predeployment_demo',
            executable='pick_and_place',
            name='move_robot_pose_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                config_file
            ],
        )
    ])
