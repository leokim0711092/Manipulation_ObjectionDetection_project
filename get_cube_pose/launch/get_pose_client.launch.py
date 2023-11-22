import launch
from launch_ros.actions import Node


def generate_launch_description():

    basic_grasp = Node(
        package='simple_grasping',
        executable='basic_grasping_perception_node',
        name='basic_grasping_perception_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'debug_topics': True}]
        )

    return launch.LaunchDescription(
    [
        basic_grasp
    ]    
    )