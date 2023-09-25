#include <ros/ros.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>


#define BaudRate  9600
#define TimeOut   1000
#define SerialPath "/dev/ttyTHS1"

typedef union {
    float data32;
    uint8_t data8[4];
}data_u;

serial::Serial ser;

void bluecar_ctl_callback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
    uint8_t _cnt = 0;
    data_u _temp; // 声明一个联合体实例，使用它将待发送数据转换为字节数组
    uint8_t data_to_send[100]; // 待发送的字节数组		
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x55;
    uint8_t _start = _cnt;
    float datas[] = {(float)cmd_vel->linear.x,
                     (float)cmd_vel->linear.y,
                     (float)cmd_vel->angular.z};		
    for(int i = 0; i < sizeof(datas) / sizeof(float); i++){
        // 将要发送的数据赋值给联合体的float成员
        // 相应的就能更改字节数组成员的值
        _temp.data32 = datas[i];
        data_to_send[_cnt++]=_temp.data8[0];
        data_to_send[_cnt++]=_temp.data8[1];
        data_to_send[_cnt++]=_temp.data8[2];
        data_to_send[_cnt++]=_temp.data8[3]; // 最高位
    }
    // 添加校验码
    char checkout = 0;
    for(int i = _start; i < _cnt; i++)
        checkout += data_to_send[i];
    data_to_send[_cnt++] = checkout;
    // 串口发送
    ser.write(data_to_send, _cnt);
}

int main(int argc, char** argv) {
    try {
    ser.setPort(SerialPath);
    ser.setBaudrate(BaudRate);
    serial::Timeout timeOut = serial::Timeout::simpleTimeout(TimeOut);
    ser.setTimeout(timeOut);
    ser.open();
    } catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open port! please check again.");
    }   
    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port Initialized!");
    } else {
        ROS_ERROR_STREAM("Someting went wrong!");
    }


    ros::init(argc, argv, "contorl_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, bluecar_ctl_callback);
    while (nh.ok()) {
        ros::spinOnce();
    }
}
