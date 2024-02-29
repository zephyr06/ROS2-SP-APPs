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
            name='talker_tsp',
            arguments=["mpc", str(mpc_period)]
        ),
        Node(
            package='cpp_pubsub',
            namespace='',
            executable='talker',
            name='talker_rrt',
            arguments=["rrt", str(rrt_period)]
        ),
        Node(
            package='cpp_pubsub',
            namespace='',
            executable='talker',
            name='talker_slam',
            arguments=["slam", str(slam_period)]
        ),
        Node(
            package='cpp_pubsub',
            namespace='',
            executable='talker',
            name='talker_tsp',
            arguments=["tsp", str(tsp_period)]
        ),
        #  *************** Listeners ********************
        Node(
            package='mpc',
            name='mpc',
            namespace='',
            executable='listener_mpc'
        ),
        Node(
            package='rrt_solver',
            name='rrt_solver',
            namespace='',
            executable='rrt_listener'
        ),
        Node(
            package='dynaslam',
            name='dynaslam',
            namespace='',
            executable='listener_slam'
        ),
        Node(
            package='tsp_solver',
            name='tsp_solver',
            namespace='',
            executable='tsp_solver_listener'
        ),
    ])