from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
import os
import argparse

node_scheduler = Node(
    package='real_time_manager',
    name='listener_scheduler',
    executable='listener_scheduler',
    output="screen",
    arguments=[LaunchConfiguration('scheduler')] # Options: CFS, RM, optimizerBF, optimizerIncremental
)

current_file_path = os.path.abspath(__file__)
parent_directory = os.path.dirname(os.path.dirname(current_file_path))
task_characteristics_yaml = parent_directory + \
    '/all_time_records/task_characteristics.yaml'

print(task_characteristics_yaml)

# Read YAML file
with open(task_characteristics_yaml, 'r') as file:
    tasks_data = yaml.safe_load(file)

mpc_period = 20
rrt_period = 2000
slam_period = 2000
tsp_period = 10000
scheduler_period = 60000

# Update periods from the task_characteristics.yaml
for task in tasks_data['tasks']:
    period = task['period']
    if task['name'] == 'MPC':
        mpc_period = period
    elif task['name'] == 'RRT':
        rrt_period = period
    elif task['name'] == 'SLAM':
        slam_period = period
    elif task['name'] == 'TSP':
        tsp_period = period

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_pubsub',
            executable='talker',
            name='talker_mpc',
            arguments=["mpc", str(mpc_period)]
        ),
        Node(
            package='cpp_pubsub',
            executable='talker',
            name='talker_rrt',
            arguments=["rrt", str(rrt_period)]
        ),
        Node(
            package='cpp_pubsub',
            executable='talker',
            name='talker_slam',
            arguments=["slam", str(slam_period)]
        ),
        Node(
            package='cpp_pubsub',
            executable='talker',
            name='talker_tsp',
            arguments=["tsp", str(tsp_period)]
        ),
        Node(
            package='cpp_pubsub',
            executable='talker',
            name='talker_scheduler',
            arguments=["scheduler", str(scheduler_period)]
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
        node_scheduler
    ])
