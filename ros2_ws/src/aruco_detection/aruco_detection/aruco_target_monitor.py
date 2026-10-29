
import rclpy

from rclpy.node import Node
from std_msgs.msg import Int32, String


class ArucoTargetMonitor(Node):

    def __init__(self):
        super().__init__('aruco_target_monitor')

        self.target_id = None
        self.detected_id = None
        self.detected_position = None
        self.target_reported = False

        self.target_subscription = self.create_subscription(
            Int32,
            '/target_aruco_id',
            self.target_callback,
            10
        )

        self.id_subscription = self.create_subscription(
            Int32,
            '/aruco/detected_id',
            self.id_callback,
            10
        )

        self.position_subscription = self.create_subscription(
            String,
            '/aruco/detected_position',
            self.position_callback,
            10
        )

        self.event_publisher = self.create_publisher(
            String,
            '/target_aruco_event',
            10
        )

        self.get_logger().info('ArUco target monitor started.')

    def target_callback(self, msg):
        if msg.data < 0:
            self.get_logger().warning('Target ID must be non-negative.')
            return

        if msg.data != self.target_id:
            self.target_id = msg.data
            self.target_reported = False

            self.get_logger().info(
                f'Target ID set to: {self.target_id}'
            )

            self.check_target()

    def id_callback(self, msg):
        self.detected_id = msg.data
        self.check_target()

    def position_callback(self, msg):
        self.detected_position = msg.data
        self.check_target()

    def check_target(self):
        if self.target_id is None or self.target_reported:
            return

        if self.detected_id != self.target_id:
            return

        if self.detected_position not in ('LEFT', 'RIGHT'):
            return

        event = (
            f'TARGET_REACHED:{self.target_id}:'
            f'{self.detected_position}'
        )

        message = String()
        message.data = event
        self.event_publisher.publish(message)

        self.get_logger().info(f'Published event: {event}')
        self.target_reported = True


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTargetMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
