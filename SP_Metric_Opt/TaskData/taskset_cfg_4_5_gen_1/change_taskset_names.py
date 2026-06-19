import os

# Folder containing the files
folder_path = './'  # replace with your folder path

for n in range(100):
    old_name = f"taskset_characteristics_{n}.yaml"
    new_name = f"taskset_characteristics_i{n}_p0.yaml"
    
    old_path = os.path.join(folder_path, old_name)
    new_path = os.path.join(folder_path, new_name)
    
    if os.path.exists(old_path):
        os.rename(old_path, new_path)
        print(f"Renamed: {old_name} → {new_name}")
    else:
        print(f"Skipped (not found): {old_name}")