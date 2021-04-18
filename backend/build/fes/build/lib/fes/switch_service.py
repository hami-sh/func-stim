import rclpy
from std_msgs.msg import String


class SwitchService:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('switch_service')

    def mainloop(self):
        subscription = self.node.create_subscription(
            String, 'topic', lambda msg: self.node.get_logger().info('I heard: "%s"' % msg.data), 10)

        rclpy.spin(self.node)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    switch = SwitchService()
    switch.mainloop()
