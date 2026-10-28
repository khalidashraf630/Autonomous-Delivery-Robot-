import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from std_msgs.msg import Int32


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

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.detected_aruco_id = None

        self.aruco_subscription = self.create_subscription(
            Int32,
            '/aruco/detected_id',
            self.aruco_callback,
            10
        )

        # PID settings.
        self.kp = 0.8
        self.ki = 0.02
        self.kd = 0.04

        self.linear_speed = 0.3
        self.corner_speed = 0.025
        self.max_angular_speed = 0.6
        self.corner_angular_speed = 0.5

        # PID state
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

        # Prevent integral windup
        self.max_integral = 2.0

        # Smooth the derivative to reduce camera noise (chat-GPT suggestion)
        self.filtered_derivative = 0.0
        self.derivative_alpha = 0.3

        # ---------------- CORNER SETTINGS ----------------
        self.state = 'FOLLOW_LINE'
        self.turn_direction = 0.0

        # Require several centered frames before resuming PID
        self.reacquired_frames = 0
        self.required_reacquired_frames = 3
        self.reacquire_tolerance = 0.18
        # (chat-GPT suggestion) stops here 

        # Image threshold for a black line
        self.threshold_value = 70
        self.min_contour_area = 10

        self.get_logger().info(
            'PID line follower with 90-degree corner handling started.'
        )

    def aruco_callback(self, msg):
        marker_id = msg.data

        if marker_id != self.detected_aruco_id:
            self.detected_aruco_id = marker_id

            self.get_logger().info(
                f'Received ArUco ID: {marker_id}'
            )

    def find_line_center(self, mask, y_start, y_end):
        """
        Find the horizontal center of the largest black-line
        candidate inside a horizontal image band.

        Returns:
            (center_x, contour_area), or (None, 0)
        """
        band = mask[y_start:y_end, :]

        contours, _ = cv2.findContours(
            band,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        contours = [
            contour for contour in contours
            if cv2.contourArea(contour) >= self.min_contour_area
        ]

        if not contours:
            return None, 0

        contour = max(contours, key=cv2.contourArea)
        moments = cv2.moments(contour)

        if moments['m00'] <= 0:
            return None, 0

        center_x = int(moments['m10'] / moments['m00'])
        area = cv2.contourArea(contour)

        return center_x, area

    #(chat-GPT suggestion)
    def reset_pid(self):
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None
        self.filtered_derivative = 0.0
    #(chat-GPT suggestion) stops here

    def publish_velocity(self, linear, angular):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )
        except Exception as error:
            self.get_logger().error(
                f'Could not convert camera image: {error}'
            )
            self.stop_robot()
            return

        height, width = frame.shape[:2]
        image_center = width / 2.0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        _, mask = cv2.threshold(
            gray,
            self.threshold_value,
            255,
            cv2.THRESH_BINARY_INV
        )

        # Three horizontal bands, from bottom to top.
        bottom_y1 = int(height * 0.70)
        middle_y1 = int(height * 0.40)
        middle_y2 = int(height * 0.70)
        top_y2 = int(height * 0.40)

        bottom_x, bottom_area = self.find_line_center(
            mask, bottom_y1, height
        )

        middle_x, middle_area = self.find_line_center(
            mask, middle_y1, middle_y2
        )

        top_x, top_area = self.find_line_center(
            mask, 0, top_y2
        )

        # Draw the image bands for debugging.
        cv2.line(
            frame, (0, bottom_y1), (width, bottom_y1),
            (255, 255, 0), 1
        )
        cv2.line(
            frame, (0, middle_y1), (width, middle_y1),
            (255, 255, 0), 1
        )
        cv2.line(
            frame,
            (int(image_center), 0),
            (int(image_center), height),
            (255, 0, 0),
            1
        )

        # Draw detected centers.
        for x, y in [
            (bottom_x, int(height * 0.85)),
            (middle_x, int(height * 0.55)),
            (top_x, int(height * 0.20))
        ]:
            if x is not None:
                cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

        # Normalized horizontal error
        bottom_error = None
        if bottom_x is not None:
            bottom_error = (
                bottom_x - image_center
            ) / image_center

        middle_error = None
        if middle_x is not None:
            middle_error = (
                middle_x - image_center
            ) / image_center



        if middle_error is not None:
            self.pid_follow(middle_error)
            self.prev_middle_error = middle_error
        else:
            if bottom_error is not None:
                self.pid_follow(bottom_error)
            else:
                self.pid_follow(self.prev_middle_error)

        #(chat-GPT suggestion)
        # ------------------------------------------------
        # STATE 1: FOLLOW THE LINE WITH PID
        # ------------------------------------------------
        # if self.state == 'FOLLOW_LINE':

        #     if bottom_x is not None:
        #         self.pid_follow(bottom_error)

        #     else:
        #         # A possible corner: line absent near the
        #         # robot but still visible in the middle.
        #         if middle_x is not None:
        #             if middle_error < -0.08:
        #                 # Line bends toward the left.
        #                 self.turn_direction = 1.0
        #             elif middle_error > 0.08:
        #                 # Line bends toward the right.
        #                 self.turn_direction = -1.0
        #             elif top_x is not None:
        #                 top_error = (
        #                     top_x - image_center
        #                 ) / image_center

        #                 if top_error < -0.08:
        #                     self.turn_direction = 1.0
        #                 elif top_error > 0.08:
        #                     self.turn_direction = -1.0
        #                 else:
        #                     self.turn_direction = 0.0
        #             else:
        #                 self.turn_direction = 0.0

        #             if self.turn_direction != 0.0:
        #                 self.state = 'TURN_CORNER'
        #                 self.reacquired_frames = 0
        #                 self.reset_pid()

        #                 self.get_logger().info(
        #                     'Possible 90-degree corner detected.'
        #                 )
        #             else:
        #                 self.stop_robot()

        #         else:
        #             # Line is not visible in either region.
        #             # Stop rather than drive blindly.
        #             self.stop_robot()

        # # ------------------------------------------------
        # # STATE 2: TURN UNTIL THE LINE IS REACQUIRED
        # # ------------------------------------------------
        # elif self.state == 'TURN_CORNER':

        #     if bottom_error is not None:
        #         # Only resume when the line is reasonably
        #         # close to the bottom-center of the image.
        #         if abs(bottom_error) < self.reacquire_tolerance:
        #             self.reacquired_frames += 1
        #         else:
        #             self.reacquired_frames = 0

        #         if (
        #             self.reacquired_frames
        #             >= self.required_reacquired_frames
        #             ):
        #             self.state = 'FOLLOW_LINE'
        #             self.reset_pid()

        #             self.get_logger().info(
        #                 'Line reacquired. Resuming PID.'
        #             )

        #             self.pid_follow(bottom_error)

        #         else:
        #             self.publish_velocity(
        #                 self.corner_speed,
        #                 self.turn_direction
        #                 * self.corner_angular_speed
        #             )

        #     else:
        #         self.reacquired_frames = 0

        #         # Rotate slowly in the estimated direction.
        #         self.publish_velocity(
        #             0.0,
        #             self.turn_direction
        #             * self.corner_angular_speed
        #         )
        #(chat-GPT suggestion) stops here
        
        cv2.putText(
            frame,
            f'State: {self.state}',
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2
        )

        if bottom_error is not None:
            cv2.putText(
                frame,
                f'Error: {bottom_error:.2f}',
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2
            )

        cv2.imshow('Line Following', frame)
        cv2.imshow('Threshold Mask', mask)
        cv2.waitKey(1)

    def pid_follow(self, error):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.previous_time is None:
            dt = 0.0
        else:
            dt = now - self.previous_time

        # Avoid unstable calculations if simulation time
        # pauses or a frame arrives after a long delay.
        if dt <= 0.0 or dt > 0.5:
            self.previous_error = error
            self.previous_time = now
            self.filtered_derivative = 0.0

            self.publish_velocity(self.linear_speed, 0.0)
            return

        # Integral term
        self.integral += error * dt
        self.integral = max(
            -self.max_integral,
            min(self.max_integral, self.integral)
        )

        # Derivative term
        if self.previous_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.previous_error) / dt

        # Low-pass filter to reduce noisy derivative response.
        self.filtered_derivative = (
            self.derivative_alpha * derivative
            + (1.0 - self.derivative_alpha)
            * self.filtered_derivative
        )

        # PID control
        control = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * self.filtered_derivative
        )

        # If the line is left of center, turn left.
        angular = -control
        angular = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, angular)
        )

        self.publish_velocity(self.linear_speed, angular)

        self.previous_error = error
        self.previous_time = now

    def destroy_node(self):
        self.stop_robot()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
