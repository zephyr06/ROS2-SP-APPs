import yaml

for i in range(100):
	filename = f'taskset_characteristics_{i}.yaml'
	# Load the YAML file
	with open(filename, 'r') as file:
		data = yaml.safe_load(file)

	# Modify the sp_weight for each task
	sp_weight_total = 0
	n_tasks = 0
	for task in data['tasks']:
		if 'performance_records_perf' in task:
			#print(task)
			#quit()
			sp_weight_total += 1
		else:
			sp_weight_total += 10
	
	for task in data['tasks']:
		if 'performance_records_perf' in task:
			task['sp_weight'] = 5/sp_weight_total
			task['sp_threshold'] = 0.6013947903486734
		else:
			task['sp_weight'] = 10*5/sp_weight_total

	# Save the modified data back to the YAML file
	with open(filename, 'w') as file:
		yaml.dump(data, file, default_flow_style=False)

	# Print out the modified YAML data
	print(yaml.dump(data, default_flow_style=False))