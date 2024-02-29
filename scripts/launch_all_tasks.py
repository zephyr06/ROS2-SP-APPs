from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tsp_solver',
            namespace='',
            executable='tsp_solver_listener',
            name='tsp_listener'
        ),
        Node(
            package='cpp_pubsub',
            namespace='',
            executable='talker',
            name='talker_tsp',
            arguments=['tsp', '1000']
        ),
    ])