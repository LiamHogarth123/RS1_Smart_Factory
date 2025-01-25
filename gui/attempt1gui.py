import tkinter as tk
import subprocess
import os

# Function to run the ROS2 launch file
def run_ros2_launch():
    # Specify your ROS2 launch file command
    launch_command = "ros2 launch global_controller_single_robot global_controller_single_launch.py"
    
    # Run the launch file using subprocess
    # Ensure the ROS2 environment is sourced by adding `source` command
    subprocess.run(f"source /opt/ros/humble/setup.bash && {launch_command}", shell=True, executable='/bin/bash')

# Initialize the Tkinter window
window = tk.Tk()
window.title("ROS2 Launch GUI")
window.geometry("300x200")

# Create a button widget
launch_button = tk.Button(window, text="Run Launch File", command=run_ros2_launch)
launch_button.pack(pady=20)

# Run the Tkinter event loop
window.mainloop()
