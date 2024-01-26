import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import tkinter as tk

class ClickLockPublisher(Node):
    def __init__(self):
        super().__init__('click_loc')
        self.publisher_ = self.create_publisher(Bool, '/click_loc', 10)
        self.timer_ = self.create_timer(1.0, self.publish_message)

        # Create GUI window
        self.root = tk.Tk()
        self.root.title("click loc")
        self.root.geometry("300x200")  # Set the window size to 300x200

        # Create button
        self.publish_button = tk.Button(self.root, text="localize", command=self.publish_message)
        self.publish_button.pack()

        # Expand the size of the button
        self.publish_button.config(width=20, height=10)

    def publish_message(self):
        msg = Bool()
        msg.data = True  # Change this value based on your requirements
        self.publisher_.publish(msg)

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)

    publisher_node = ClickLockPublisher()
    publisher_node.run()

    rclpy.shutdown()

if __name__ == '__main__':
    main()