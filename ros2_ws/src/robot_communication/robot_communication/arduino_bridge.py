import rclpy
import serial
import time

from rclpy.node import Node
from std_msgs.msg import Int32

class ArduinoBridge(Node):

    def __init__(self):
        super().__init__('arduino_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('port').value
        baud_rate = self.get_parameter('baud_rate').value

        self.serial_connection = serial.Serial(
            port=port,
            baudrate=baud_rate,
            timeout=0.1
        )

        time.sleep(2)
        self.serial_connection.reset_input_buffer()
        self.serial_connection.write(b'READY\n')
        

        self.target_pub = self.create_publisher(
            Int32,
            '/target_aruco_id',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.read_serial
        )

    def read_serial(self):
        if self.serial_connection.in_waiting == 0:
            return
        
        message = self.serial_connection.readline().decode(
            'utf-8', errors='replace'
        ).strip()

        if message.startswith('T:'):
            try:
                target_id = int(message.split(':', 1)[1])
                msg = Int32()
                msg.data = target_id

                self.target_pub.publish(msg)

                self.get_logger().info(
                    f'Target received: {target_id}'
                )

            except ValueError:
                self.get_logger().warning(
                    f'Invalid target message: {message}'
                )

    def destroy_node(self):
        if self.serial_connection.is_open:
            self.serial_connection.close()

        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()