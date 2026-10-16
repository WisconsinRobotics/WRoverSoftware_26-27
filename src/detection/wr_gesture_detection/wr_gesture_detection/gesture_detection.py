import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import Empty

class GestureDetection(Node):
    def __init__(self):
        super().__init__('gesture_detection_node')

        # self.turtle_sub = self.create_subscription(
        #     Point, 
        #     '/subscription', 
        #     self.gesture_callback,
        #     10
        #     )

        self.gps_pub = self.create_publisher(
            Empty,
            '/person',
            10
        )

        self.get_logger().info('Running GestureDetection node')

    def gesture_callback(self, msg: Pose):
        pass


def main(args=None):

    rclpy.init(args=args)
    node = GestureDetection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()