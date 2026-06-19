import os
import yaml

# Folder containing the files
folder_path = '../taskset_cfg_multip_10_4'  # replace with your folder path

pos = {}

for n in range(100):
    new_name = f"taskset_characteristics_i{n}_p0.yaml"
    old_path = os.path.join(folder_path, new_name)
    
    
    # Load the YAML file
    with open(old_path, 'r') as file:
        data = yaml.safe_load(file)
    
    # Step 2: Sort tasks
    data['tasks'] = sorted(
        data['tasks'],
        key=lambda task: (task['period'], -task['sp_weight'])
    )
	
    n_tasks = len(data['tasks'])
    for i in range(n_tasks):
        old_id = data['tasks'][i]['gid']
        data['tasks'][i]['id'] = i
        data['tasks'][i]['gid'] = i
        data['tasks'][i]['name'] = f'task_{i}'
        if n==0:
            pos[old_id] = i
	
    # Step 3: Save back to YAML
    with open(new_name, 'w') as f:
        yaml.dump(data, f, sort_keys=False)

print('pos',pos)

import shutil

# path_Et_task_9_0_7
for t in range(n_tasks):
    newt = pos[t]
    for i in range(8):
        oldpath = os.path.join(folder_path,f'path_Et_task_{t}_0_{i}.txt')
        newpath = f'path_Et_task_{newt}_0_{i}.txt'
        shutil.copyfile(oldpath, newpath)
		