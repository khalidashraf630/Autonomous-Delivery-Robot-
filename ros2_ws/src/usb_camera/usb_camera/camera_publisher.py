
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class USBCamera(Node):

    def __init__(self):
        super().__init__('usb_camera')

        self.bridge = CvBridge()

        # Keep only the latest frame
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            qos
        )

        self.cap = cv2.VideoCapture('/dev/video2', cv2.CAP_V4L2)

        if not self.cap.isOpened():
            raise RuntimeError('Could not open USB camera')

        self.cap.set(cv2.CAP_PROP_FOURCC,
                     cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)

        # Disable unnecessary buffering if supported
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.timer = self.create_timer(
            1.0 / 120.0,
            self.publish_frame
        )

        self.get_logger().info('USB camera publisher started')

    
    def publish_frame(self):

        import time

        t0 = time.perf_counter()

        ret, frame = self.cap.read()
        t1 = time.perf_counter()

        if not ret:
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        t2 = time.perf_counter()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'usb_camera_frame'

        self.publisher.publish(msg)
        t3 = time.perf_counter()

        self.get_logger().info(
            f'Capture: {(t1-t0)*1000:.2f} ms | '
            f'Conversion: {(t2-t1)*1000:.2f} ms | '
            f'Publish: {(t3-t2)*1000:.2f} ms'
        )



def main(args=None):

    rclpy.init(args=args)
    node = USBCamera()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
