import rclpy
import serial

from rclpy.node import Node
from std_msgs.msg import Float32, String


class ESP32Bridge(Node):

    def __init__(self):
        super().__init__('esp32_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('port').value
        baud_rate = self.get_parameter('baud_rate').value

        self.serial_connection = serial.Serial(
            port=port,
            baudrate=baud_rate,
            timeout=1
        )

        self.get_logger().info(
            f'Serial connected on {port}'
        )

        self.error_sub = self.create_subscription(
            Float32,
            '/line_error',
            self.error_callback,
            10
        )

        self.target_sub = self.create_subscription(
            String,
            '/target_aruco_event',
            self.target_callback,
            10
        )

        # TEST ONLY: Read responses from ESP32
        self.read_timer = self.create_timer(
            0.05,
            self.read_from_esp32
        )

    def error_callback(self, msg):
        error = msg.data
        serial_message = f'E:{error:.4f}\n'

        self.serial_connection.write(
            serial_message.encode('utf-8')
        )

        self.get_logger().info(
            f'Sent message: {serial_message.strip()}'
        )


    def target_callback(self, msg):
        parts = msg.data.split(':')

        if len(parts) != 3 or parts[0] != 'TARGET_REACHED':
            return

        target_id = parts[1]
        position = parts[2]

        if not target_id.isdigit():
            return

        if position == 'LEFT':
            position_code = 'L'
        elif position == 'RIGHT':
            position_code = 'R'
        else:
            return

        serial_message = f'A:{target_id}:{position_code}\n'

        self.serial_connection.write(
            serial_message.encode('utf-8')
        )

        self.get_logger().info(
            f'Sent arrival message: {serial_message.strip()}'
        )

    # TEST ONLY: Read confirmation from ESP32
    def read_from_esp32(self):
        if self.serial_connection.in_waiting == 0:
            return

        response = self.serial_connection.readline().decode(
            'utf-8', errors='replace'
        ).strip()

        if response.startswith('RECEIVED:E:'):
            self.get_logger().info(
                f'ESP32 {response}'
            )

    def destroy_node(self):
        if self.serial_connection.is_open:
            self.serial_connection.close()

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    node = ESP32Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()