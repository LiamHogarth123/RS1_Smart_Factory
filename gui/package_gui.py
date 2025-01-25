import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import threading

# Mapping status codes to human-readable status
STATUS_MAP = {
    1: "Stored",
    2: "Robot En Route to Pick",
    3: "Package Pickup En Route to Delivery",
    4: "Delivered",
    5: "Package Confirmed",
    6: "Package Delivered",
    8: "Package Not Found",
    9: "Package Incorrect"
}

# ROS2 Node Class
class GuiNode(Node):
    def __init__(self, update_status_callback):
        super().__init__("gui_node")
        self.update_status_callback = update_status_callback

        # Subscriber for package status updates
        self.package_status_subscriber = self.create_subscription(
            Int32, "package_status", self.package_status_callback, 10
        )

    # Callback for package status updates
    def package_status_callback(self, msg):
        # Extract package ID and status from the message
        package_id = msg.data // 10  # Package ID is the tens digit
        status_code = msg.data % 10  # Status code is the units digit

        # Update the GUI using the callback
        if status_code in STATUS_MAP:
            self.update_status_callback(package_id, STATUS_MAP[status_code])

# Function to run ROS2 in a separate thread
def ros2_thread():
    rclpy.spin(node)

# Function to update the status in the table
def update_status(package_id, status):
    if 1 <= package_id <= 8:  # Ensure package ID is valid
        status_labels[package_id - 1].config(text=status)

# Initialize ROS2
rclpy.init()
node = GuiNode(update_status)
thread = threading.Thread(target=ros2_thread, daemon=True)
thread.start()

# Create the main GUI window
window = tk.Tk()
window.title("Warehouse Automation GUI")
window.geometry("500x400")

# Create table headers
tk.Label(window, text="Package", font=("Arial", 12, "bold")).grid(row=0, column=0, padx=10, pady=5)
tk.Label(window, text="Status", font=("Arial", 12, "bold")).grid(row=0, column=1, padx=10, pady=5)

# Create the table with 8 rows
status_labels = []
for i in range(8):
    # Left column: Package name
    tk.Label(window, text=f"Package {i + 1}", font=("Arial", 10)).grid(row=i + 1, column=0, padx=10, pady=5)

    # Right column: Status
    status_label = tk.Label(window, text="Unknown", font=("Arial", 10))
    status_label.grid(row=i + 1, column=1, padx=10, pady=5)
    status_labels.append(status_label)

# Run the GUI loop
window.mainloop()

# Shutdown ROS2 after GUI closes
rclpy.shutdown()
