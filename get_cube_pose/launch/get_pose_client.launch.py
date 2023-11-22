import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    package_description = "get_cube_pose"

    basic_grasp = Node(
        package='simple_grasping',
        executable='basic_grasping_perception_node',
        name='basic_grasping_perception_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'debug_topics': True}]
        )

    get_pose = Node(
    package='get_cube_pose',
    executable='get_pose_client',
    name='get_pose_client',
    output='screen',
    emulate_tty=True
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
        get_pose,
        rviz_node
    ]    
    )