import os
import argparse

parser = argparse.ArgumentParser(description='Set the maximum time for the TSP algorithm')
parser.add_argument('--time_limit', default = 0.2)
args = parser.parse_args()
time_limit_input = args.time_limit

tsp_config_path = os.path.join(os.path.dirname(__file__), '../SP_Metric_Opt/applications/tsp_solver_osm/config/algorithm_config.yaml')
with open(tsp_config_path, 'r') as f:
    lines = f.readlines()
    line_max_time = lines[6]
    if 'max_time' in line_max_time:
        lines[6] = '  max_time: ' + str(time_limit_input) + '\n'
    else:
        print('Error: max_time not found in algorithm_config.yaml')
with open(tsp_config_path, 'w') as f:
    f.writelines(lines)
