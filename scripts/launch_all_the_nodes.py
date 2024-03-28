from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
import os

current_file_path = os.path.abspath(__file__)
parent_directory = os.path.dirname(os.path.dirname(current_file_path))
task_characteristics_yaml = parent_directory + '/all_time_records/task_characteristics.yaml'

print(task_characteristics_yaml)

# Read YAML file
with open(task_characteristics_yaml, 'r') as file:
    tasks_data = yaml.safe_load(file)

mpc_period=50
rrt_period=1000
slam_period=1000
tsp_period=10000
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
    elif task['name'] == 'TSP:':
        tsp_period = period

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_pubsub',
            namespace='',
            executable='talker',
            name='talker_mpc',
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
        Node(
            package='cpp_pubsub',
            namespace='',
            executable='talker',
            name='talker_scheduler',
            arguments=["scheduler", str(scheduler_period)]
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
        # Node(
        #     package='real_time_manager',
        #     name='listener_scheduler_fixed_priority',
        #     namespace='',
        #     executable='listener_scheduler_fixed_priority'
        # ),
         Node(
            package='real_time_manager',
            name='listener_scheduler',
            namespace='',
            executable='listener_scheduler'
        ),
    ])
