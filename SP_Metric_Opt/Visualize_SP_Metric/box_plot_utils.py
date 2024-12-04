import os


def get_subfolder_path(folder_path):
    # Check if the given path is a directory
    if not os.path.isdir(folder_path):
        raise ValueError(f"The path {folder_path} is not a directory.")

    # List all items in the directory
    items = os.listdir(folder_path)

    # Filter for subfolders
    subfolders = [os.path.join(folder_path, item) for item in items if os.path.isdir(os.path.join(folder_path, item))]

    # Check if there is exactly one subfolder
    if len(subfolders) == 1:
        return subfolders[0]
    elif len(subfolders) > 1:
        raise ValueError("The folder contains more than one subfolder.")
    else:
        raise ValueError("The folder does not contain any subfolders.")


def clear_folder(folder_path):
    try:
        # Check if folder exists
        if not os.path.exists(folder_path):
            print(f"The folder {folder_path} does not exist.")
            return

        # Loop through all files in the folder and delete them
        for filename in os.listdir(folder_path):
            file_path = os.path.join(folder_path, filename)

            # Check if it's a file (not a subdirectory)
            if os.path.isfile(file_path):
                os.remove(file_path)
                print(f"Deleted: {file_path}")
            else:
                print(f"Skipping: {file_path} (not a file)")

        print(f"All files have been deleted from {folder_path}")

    except Exception as e:
        print(f"An error occurred: {e}")
