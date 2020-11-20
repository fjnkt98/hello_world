import rclpy
import rclpy.node
import rclpy.qos
import std_msgs.msg


class Listener(rclpy.node.Node):

    def __init__(self):
        super().__init__('listener')

        # Create subscription with topic name 'chatter'
        qos = rclpy.qos.QoSProfile(depth=10)
        self._subscription = self.create_subscription(
                std_msgs.msg.String,
                'chatter',
                self.listener_callback,
                qos)
        # self._subscription

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    # Node initialization
    rclpy.init(args=args)

    # Create node object and spin
    listener = Listener()
    rclpy.spin(listener)

    # Shutting down the node
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
