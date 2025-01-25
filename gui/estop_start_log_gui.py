import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import threading

# ROS2 Node Class
class GuiNode(Node):
    def __init__(self, log_callback):
        super().__init__("gui_node")
        self.e_stop_publisher = self.create_publisher(Bool, "e_stop", 10)
        self.start_publisher = self.create_publisher(Bool, "start", 10)
        self.log_callback = log_callback

    def publish_e_stop(self):
        msg = Bool()
        msg.data = True
        self.e_stop_publisher.publish(msg)
        self.log_callback("Published: E-Stop Activated!")

    def publish_start(self):
        msg = Bool()
        msg.data = True
        self.start_publisher.publish(msg)
        self.log_callback("Published: Start Operation!")

# Function to run ROS2 in a separate thread
def ros2_thread():
    rclpy.spin(node)

# Function for the E-Stop button
def emergency_stop():
    node.publish_e_stop()

# Function for the Start button
def start_operation():
    node.publish_start()

# Function to add log messages to the text box
def log_message(message):
    log_box.insert(tk.END, message + "\n")  # Append the message to the log box
    log_box.see(tk.END)  # Scroll to the end of the text box

# Initialize ROS2
rclpy.init()
node = GuiNode(log_message)
thread = threading.Thread(target=ros2_thread, daemon=True)
thread.start()

# Create the main GUI window
window = tk.Tk()
window.title("Warehouse Automation GUI")
window.geometry("400x300")

# E-Stop Button
e_stop_button = tk.Button(window, text="E-Stop", fg="white", bg="red", font=("Arial", 12, "bold"), command=emergency_stop)
e_stop_button.pack(pady=10)

# Start Button
start_button = tk.Button(window, text="Start", fg="white", bg="green", font=("Arial", 12, "bold"), command=start_operation)
start_button.pack(pady=10)

# Log Box
log_box = tk.Text(window, height=10, width=50, state=tk.NORMAL)
log_box.pack(pady=10)
log_box.insert(tk.END, "Log started...\n")  # Initial log message

# Run the GUI loop
window.mainloop()

# Shutdown ROS2 after GUI closes
rclpy.shutdown()
