
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info('Line detector started.')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8'
        )

        height, width = frame.shape[:2]

        # Only examine the lower part of the image.
        roi_y = int(height * 0.50)
        roi = frame[roi_y:, :]

        # Convert to grayscale and isolate dark pixels.
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(
            gray, 70, 255, cv2.THRESH_BINARY_INV
        )

        # Find connected dark regions.
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # Ignore tiny dark regions.
        contours = [
            c for c in contours
            if cv2.contourArea(c) > 20
        ]

        if contours:
            # Use the largest candidate as the line.
            contour = max(contours, key=cv2.contourArea)
            moments = cv2.moments(contour)

            if moments['m00'] > 0:
                line_x = int(moments['m10'] / moments['m00'])
                line_y = int(moments['m01'] / moments['m00'])

                image_center = width // 2
                error = line_x - image_center

                # Draw the detected center and image center.
                cv2.circle(
                    frame, (line_x, roi_y + line_y),
                    8, (0, 0, 255), -1
                )
                cv2.line(
                    frame, (image_center, roi_y),
                    (image_center, height), (255, 0, 0), 2
                )

                cv2.putText(
                    frame, f'Error: {error}px',
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 0), 2
                )

        # Show both the camera image and threshold mask.
        cv2.imshow('Line Detection', frame)
        cv2.imshow('Threshold Mask', mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
