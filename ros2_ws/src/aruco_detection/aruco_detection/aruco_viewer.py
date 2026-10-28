
import cv2
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ArucoViewer(Node):

    def __init__(self):
        super().__init__('aruco_viewer')

        self.bridge = CvBridge()

        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info('ArUco viewer started.')

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

        height, width = frame.shape[:2]
        middle_x = width // 2

        cv2.line(
            frame,
            (middle_x, 0),
            (middle_x, height),
            (0, 255, 255),
            2
        )

        cv2.putText(
            frame, 'LEFT', (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8,
            (0, 255, 0), 2
        )
        cv2.putText(
            frame, 'RIGHT', (middle_x + 20, 35),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8,
            (0, 255, 0), 2
        )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters
        )

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for marker_corners, marker_id in zip(
                corners, ids.flatten()
            ):
                points = marker_corners[0]
                center_x = int(points[:, 0].mean())
                center_y = int(points[:, 1].mean())

                position = (
                    'LEFT' if center_x < middle_x else 'RIGHT'
                )

                label = f'ID: {int(marker_id)} | {position}'

                cv2.putText(
                    frame,
                    label,
                    (max(5, center_x - 80), max(60, center_y - 15)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 0, 255),
                    2
                )

        cv2.imshow('ArUco Camera Viewer', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
