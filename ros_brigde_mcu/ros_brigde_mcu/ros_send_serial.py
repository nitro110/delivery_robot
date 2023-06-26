import rclpy
import serial

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import (Quaternion, Vector3)

import time
import struct
import math
import threading
import struct

class ros_serial_com(Node):

    def __init__(self):
        super().__init__('ros_serial_com')
        self.get_logger().info("ros_serial_com node started")

        self.imu_frame = self.declare_parameter("~imu_frame", 'imu_link').value
        self.pub_topic = self.declare_parameter("~imu_pub_topic", '/imu_msg/raw').value
        self.publish_rate = self.declare_parameter("~publish_rate", 100).value
        self.get_logger().info(f"imu_frame:: {self.imu_frame} pub_topic:: {self.pub_topic} publish_rate::{self.publish_rate}")

        self.declare_parameter('linear_acceleration_stddev', 0.0)
        self.declare_parameter('angular_velocity_stddev', 0.0)
        self.declare_parameter('orientation_stddev', 0.0)

        linear_acceleration_stddev = self.get_parameter('linear_acceleration_stddev').value
        angular_velocity_stddev = self.get_parameter('angular_velocity_stddev').value
        orientation_stddev = self.get_parameter('orientation_stddev').value

        #IMU data
        self.imu = Imu()
        self.data_buffer = []

        self.imu.linear_acceleration_covariance[0] = linear_acceleration_stddev
        self.imu.linear_acceleration_covariance[4] = linear_acceleration_stddev
        self.imu.linear_acceleration_covariance[8] = linear_acceleration_stddev

        self.imu.angular_velocity_covariance[0] = angular_velocity_stddev
        self.imu.angular_velocity_covariance[4] = angular_velocity_stddev
        self.imu.angular_velocity_covariance[8] = angular_velocity_stddev

        self.imu.orientation_covariance[0] = orientation_stddev
        self.imu.orientation_covariance[4] = orientation_stddev
        self.imu.orientation_covariance[8] = orientation_stddev

        
        try:
            self.serialPort = serial.Serial(port = "/dev/ttyUSB1", baudrate=9600, bytesize=8, timeout=0)
            self.get_logger().info('Serial port initialized and opened.')
        except serial.SerialException:
            self.get_logger().error('Unable to open serial port.')
            return
            
        
        #subcribe to cmd_vel topic and send to serial
        self.cmd_send_ = self.create_subscription(
            Twist,
            'cmd_vel',
            self.ros_send_cmdvel_callback,
            10)
        self.cmd_send_  # prevent unused variable warning


        #read serial for imu data and publish /imu topic
        self.sensor_pub_cb_grp = ReentrantCallbackGroup()
        self.sensor_publisher_ = self.create_publisher(
            Imu,
            'imu',
            10,
            callback_group=self.sensor_pub_cb_grp)

        self.stop_queue = threading.Event()
        self.imu_thread = threading.Thread(target=self.publish_imu_data)
        
        self.get_logger().info(f"IMU node successfully created. publishing on topic:: {self.pub_topic}")

    def start_imu_publisher(self):
        self.imu_thread.start()
        #self.odometry_thread.start()

    def publish_imu_data(self):   
        self.get_logger().info(f"Publishing messages at {self.publish_rate} Hz.")
        self.rate = self.create_rate(self.publish_rate)
        while not self.stop_queue.is_set() and rclpy.ok():
            try:
                self.read_Sensor_data()
                self.rate.sleep()
            except Exception as ex:
                self.get_logger().error(f"Failed to create IMU message: {ex}")      
        
    def read_Sensor_data(self):  
            try:
                while self.serialPort.is_open and self.serialPort.in_waiting > 0:
                    line = self.serialPort.readline().decode('utf-8').strip()
                
                    #self.data_buffer.append(line)
                    #data = self.data_buffer.pop(0)
                    if line.startswith("#"):
                        imu_data = line.split(",")[1:]
                        self.get_logger().info(f"i read: {imu_data}")

                        self.imu.orientation.w = int(imu_data[1]) / 16384.0
                        self.imu.orientation.x = int(imu_data[2]) / 16384.0
                        self.imu.orientation.y = int(imu_data[3]) / 16384.0
                        self.imu.orientation.z = int(imu_data[4]) / 16384.0
                        self.imu.angular_velocity.x = int(imu_data[5]) * (4000.0 / 65536.0) * (math.pi / 180.0) * 25.0
                        self.imu.angular_velocity.y = int(imu_data[6]) * (4000.0 / 65536.0) * (math.pi / 180.0) * 25.0
                        self.imu.angular_velocity.z = int(imu_data[7]) * (4000.0 / 65536.0) * (math.pi / 180.0) * 25.0
                        self.imu.linear_acceleration.x = int(imu_data[8]) * (8.0 / 65536.0) * 9.81
                        self.imu.linear_acceleration.y = int(imu_data[9]) * (8.0 / 65536.0) * 9.81
                        self.imu.linear_acceleration.z = int(imu_data[10]) * (8.0 / 65536.0) * 9.81
        
                        self.get_logger().info(f"Orientation = {self.imu.orientation.w},{self.imu.orientation.x}, {self.imu.orientation.y}, {self.imu.orientation.z} ")
            
            except serial.SerialException as e:
                self.get_logger().error(f"Error reading from serial port: {e}")


    def ros_send_cmdvel_callback(self, msg):
        linear_x = msg.linear.x #convert to bytes
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        data= struct.pack('cfff', '$'.encode(), linear_x, linear_y, angular_z)

        #self.get_logger().info('send: "%s"' % data) #for debug

        self.serialPort.write(data)

        time.sleep(1)
        self.serialPort.flush()


    def shutdown(self):
        self.stop_queue.set()  # Signal the thread to stop
        self.serialPort.close()


def main(args=None):
    rclpy.init(args=args)

    serial_com = ros_serial_com()

    executor = MultiThreadedExecutor()
    executor.add_node(serial_com)

    serial_com.start_imu_publisher()

    try:
        executor.spin()
    
    finally:
        executor.shutdown()
        serial_com.shutdown()
        serial_com.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()