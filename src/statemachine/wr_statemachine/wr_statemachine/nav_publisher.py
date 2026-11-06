from rclpy.node import Node
from std_msgs.msg import Int32


class NavPublisher(Node):
    def __init__(self):
        super().__init__("nav_publisher")
        self.publisher = self.create_publisher(Int32, "/nav_mode", 1)
