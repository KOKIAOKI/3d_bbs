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
        self.root.title("ROS2 Publisher GUI")

        # Create button
        self.publish_button = tk.Button(self.root, text="localize", command=self.publish_message)
        self.publish_button.pack()

    def publish_message(self):
        msg = Bool()
        msg.data = True  # Change this value based on your requirements
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing /click_loc: {}'.format(msg.data))

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)

    publisher_node = ClickLockPublisher()
    publisher_node.run()

    rclpy.shutdown()

if __name__ == '__main__':
    main()