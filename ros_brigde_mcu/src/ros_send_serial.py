import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

serialPort = serial.Serial(port = "/dev/ttyUSB0", baudrate=115200,
                           bytesize=8, timeout=0)


class ros_send_serial(Node):

    def __init__(self):
        super().__init__('ros_send_serial')
        self.subscription = self.create_subscription(
            Twist,
            'topic',
            self.ros_send_serial_callback,
            10)
        self.subscription  # prevent unused variable warning

    def ros_send_serial_callback(self, msg):
        linear_x=msg.linear.x
        linear_y=msg.linear.y
        angular_z=msg.angular.z
        serialPort.write('$')
        serialPort.write(linear_x)
        serialPort.write(linear_y)
        serialPort.write(angular_z)
        time.sleep(1)
        serialPort.flush()

def main(args=None):
    rclpy.init(args=args)

    ros_send_serial = ros_send_serial()

    rclpy.spin(ros_send_serial)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ros_send_serial.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()