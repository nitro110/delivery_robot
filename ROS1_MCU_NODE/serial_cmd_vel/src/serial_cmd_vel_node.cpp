#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "serial/serial.h"
#include <sstream>

serial::Serial ser;

typedef union _data {
  float f;
  char  s[4];
} DataToByte;

uint8_t cmdPacket[18] = {'$', 0x00, 0x00, 0x00, 0,0,0,0, 0,0,0,0, 0,0,0,0, '\r','\n' };

void send_cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{   
    DataToByte x_linear_vel;
    DataToByte y_linear_vel;
    DataToByte w_angular_vel;
    x_linear_vel.f = msg->linear.x;
    y_linear_vel.f = msg->linear.y;
    w_angular_vel.f = msg->angular.z;

    if (msg->linear.z < 0){
        cmdPacket[18] = {'$', 0x01, 0x00, 0x00, 0,0,0,0, 0,0,0,0, 0,0,0,0, '\r','\n' };//reset mcu
        ser.write(cmdPacket,sizeof(cmdPacket));
    }
    else{    

        //linear_x_bytes
        cmdPacket[4] = x_linear_vel.s[0];
        cmdPacket[5] = x_linear_vel.s[1];
        cmdPacket[6] = x_linear_vel.s[2];
        cmdPacket[7] = x_linear_vel.s[3];

        //linear_y_bytes
        cmdPacket[8] = y_linear_vel.s[0];
        cmdPacket[9] = y_linear_vel.s[1];
        cmdPacket[10] = y_linear_vel.s[2];
        cmdPacket[11] = y_linear_vel.s[3];

        //angular_w_bytes
        cmdPacket[12] = w_angular_vel.s[0];
        cmdPacket[13] = w_angular_vel.s[1];
        cmdPacket[14] = w_angular_vel.s[2];
        cmdPacket[15] = w_angular_vel.s[3];

        ser.write(cmdPacket,sizeof(cmdPacket));
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_cmd_vel_node");
    ros:: NodeHandle nh;

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Failed to open the serial port!");
        return -1;
    }
      // Check if the serial port is open
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial port initialized!");

        // Create a subscriber for cmd_vel messages
        ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, send_cmd_vel_callback);

        // Spin the ROS node
        ros::spin();
        return 0;
    }
    else
    {
        ROS_ERROR_STREAM("Serial port is not open!");
        return -1;
    }
}