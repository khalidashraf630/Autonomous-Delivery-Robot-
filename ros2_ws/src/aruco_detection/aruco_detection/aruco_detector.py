
import cv2
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        self.parameters = cv2.aruco.DetectorParameters_create()

        self.id_publisher = self.create_publisher(
            Int32,
            '/aruco/detected_id',
            10
        )

        self.position_publisher = self.create_publisher(
            String,
            '/aruco/detected_position',
            10
        )

        self.last_detected_id = None
        self.last_detected_position = None

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info('ArUco detector started.')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as error:
            self.get_logger().error(
                f'Image conversion failed: {error}'
            )
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters
        )

        if ids is None:
            self.last_detected_id = None
            self.last_detected_position = None
            return

        frame_width = frame.shape[1]

        for marker_corners, marker_id in zip(
            corners, ids.flatten()
        ):
            marker_id = int(marker_id)

            marker_points = marker_corners[0]
            center_x = float(marker_points[:, 0].mean())

            if center_x < frame_width / 2:
                position = 'LEFT'
            else:
                position = 'RIGHT'

            id_message = Int32()
            id_message.data = marker_id
            self.id_publisher.publish(id_message)

            position_message = String()
            position_message.data = position
            self.position_publisher.publish(position_message)

            if (
                marker_id != self.last_detected_id
                or position != self.last_detected_position
            ):
                self.get_logger().info(
                    f'ArUco ID: {marker_id}, Position: {position}'
                )

            self.last_detected_id = marker_id
            self.last_detected_position = position


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
