#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>

#define BaudRate  9600
#define TimeOut   1000
//You are going to write a subscriber to get keyborad(publisher)'s command
unsigned char s_buffer[1];
serial::Serial ser;
std_msgs::UInt8 command;

void bluecar_keyctl_callback(const std_msgs::UInt8& command) {
    ROS_INFO_STREAM("I Recived " << command.data);
    s_buffer[0] = command.data;
    ser.write(s_buffer, 1);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    ros::Subscriber command_sub = nh.subscribe("/bluecar/key_vel", 1, bluecar_keyctl_callback);

    try {
        ser.setPort("/dev/ttyTHS1");
        ser.setBaudrate(BaudRate);
        serial::Timeout timeOut = serial::Timeout::simpleTimeout(TimeOut);
        ser.setTimeout(timeOut);
        ser.open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open port! please check again.");
        return -1;
    }
     
    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port Initialized!");
    } else {
        ROS_ERROR_STREAM("Someting went wrong!");
        return -1;
    }

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        if (ser.available()) {

            std_msgs::String r_data;
            r_data.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read " << r_data.data);
        }
        loop_rate.sleep();
    }
}