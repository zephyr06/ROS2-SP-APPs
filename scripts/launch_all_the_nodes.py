from launch import LaunchDescription
from launch_ros.actions import Node


mpc_period=500
rrt_period=1000
slam_period=2000
tsp_period=1000

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_pubsub',
            namespace='',
            executable='talker',
            name='cpp_pubsub',
            arguments=["mpc", mpc_period]
        ),
        Node(
            package='cpp_pubsub',
            namespace='',
            executable='talker',
            name='cpp_pubsub',
            arguments=["rrt", rrt_period]
        ),
        Node(
            package='cpp_pubsub',
            namespace='',
            executable='talker',
            name='cpp_pubsub',
            arguments=["slam", slam_period]
        ),
        Node(
            package='cpp_pubsub',
            namespace='',
            executable='talker',
            name='cpp_pubsub',
            arguments=["tsp", tsp_period]
        ),
        #  *************** Listeners ********************
        Node(
            package='mpc',
            name='mpc',
            executable='listener_mpc'
        ),
        Node(
            package='rrt_solver',
            name='rrt_solver',
            executable='rrt_listener'
        ),
        Node(
            package='dynaslam',
            name='dynaslam',
            executable='listener_slam'
        ),
        Node(
            package='tsp_solver',
            name='tsp_solver',
            executable='tsp_solver_listener'
        ),
    ])