import os

folder_path = "all_time_records"

# Get the list of files in the folder
file_list = os.listdir(folder_path)

# Iterate over each file and clear its contents
for file_name in file_list:
    file_path = os.path.join(folder_path, file_name)
    with open(file_path, "w") as file:
        file.truncate(0)
