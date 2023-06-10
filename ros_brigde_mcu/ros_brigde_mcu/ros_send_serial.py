import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import struct

serialPort = serial.Serial(port = "/dev/ttyUSB0", baudrate=9600,
                           bytesize=8, timeout=0)


class ros_send_serial(Node):

    def __init__(self):
        super().__init__('ros_send_serial')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.ros_send_serial_callback,
            10)
        self.subscription  # prevent unused variable warning

    def ros_send_serial_callback(self, msg):
        linear_x=msg.linear.x
        linear_y=msg.linear.y
        angular_z=msg.angular.z
        serialPort.write(b"$")
        serialPort.write(b"%f",linear_x)
        serialPort.write(b"%f",linear_y)
        serialPort.write(b"%f",angular_z)

        ser=serialPort.readline()
        self.get_logger().info('I heard: "%s"' % ser)

        time.sleep(1)
        serialPort.flush()

def main(args=None):
    rclpy.init(args=args)

    serial_send = ros_send_serial()

    rclpy.spin(serial_send)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ros_send_serial.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()