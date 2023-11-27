import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    package_description = "get_cube_pose"

    basic_grasp = Node(
        package='simple_grasping',
        executable='basic_grasping_perception_node',
        name='basic_grasping_perception_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'debug_topics': True}],
        )

    #moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit2_config").to_moveit_configs()
    moveit_config = MoveItConfigsBuilder("my").to_moveit_configs()

    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        package="moveit2_scripts",
        executable="pick_and_place_perception",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )
    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'perception.rviz')


    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    return launch.LaunchDescription(
    [
        basic_grasp,
        moveit_cpp_node,
        rviz_node
    ]    
    )