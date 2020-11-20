import rclpy
import rclpy.node
import rclpy.qos
import std_msgs.msg


class Talker(rclpy.node.Node):

    def __init__(self):
        super().__init__('talker')

        # Create publisher with topic name 'chatter'
        # QoS profile: depth = 10
        qos = rclpy.qos.QoSProfile(depth=10)
        self._publisher = self.create_publisher(
                std_msgs.msg.String,
                'chatter',
                qos)

        # Create timer with period 0.5 [seconds]
        self._timer = self.create_timer(0.5, self.timer_callback)

        # Increment variable
        self._i = 0

    def timer_callback(self):
        msg = std_msgs.msg.String()
        msg.data = 'Hello World: %d' % self._i
        self._publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self._i += 1


def main(args=None):
    # Node initialization
    rclpy.init(args=args)

    # Create node object and spin
    talker = Talker()
    rclpy.spin(talker)

    # Shutting down the node
    talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
