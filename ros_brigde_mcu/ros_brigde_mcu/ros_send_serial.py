import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import struct

serialPort = serial.Serial(port = "/dev/ttyUSB0", baudrate=115200,
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
        linear_x = msg.linear.x #convert to bytes
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        data= struct.pack('cfff', '$'.encode(), linear_x, linear_y, angular_z)

        #self.get_logger().info('send: "%s"' % data) #for debug

        serialPort.write(data)

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