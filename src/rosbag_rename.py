import os
import shutil
import datetime

def rename_rosbags_in_folder(folder_path):
    # Check if the folder exists
    if not os.path.exists(folder_path):
        print(f"The folder path '{folder_path}' does not exist.")
        return
    
    # Iterate through all files in the folder
    for filename in os.listdir(folder_path):
        if filename.endswith(".bag"):
            # Get the full file path
            file_path = os.path.join(folder_path, filename)

            # Get the modification time of the file
            mtime = os.path.getmtime(file_path)

            # Convert the timestamp to a formatted date-time string
            formatted_datetime = datetime.datetime.fromtimestamp(mtime).strftime("%Y_%m_%d_%H_%M_%S")

            # Construct the new filename with the format "hound_{date-time}.bag"
            new_filename = f"hound_{formatted_datetime}.bag"

            # Get the full new file path
            new_file_path = os.path.join(folder_path, new_filename)

            # Rename the file
            os.rename(file_path, new_file_path)
            print(f"Renamed: {filename} -> {new_filename}")

if __name__ == "__main__":
    folder_path = "/root/catkin_ws/src/new_bags"  # Replace this with your actual folder path
    rename_rosbags_in_folder(folder_path)