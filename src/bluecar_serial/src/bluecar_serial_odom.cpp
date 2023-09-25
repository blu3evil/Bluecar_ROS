#include <ros/ros.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>


#define BaudRate  9600
#define TimeOut   1000
#define SerialPath "/dev/ttyTHS1"

typedef union {
    float data32;
    uint8_t data8[4];
}data_u;

class OdomPublisherNode {
    private:
        ros::Publisher _odompub;
        ros::Publisher _imupub;
        tf::TransformBroadcaster _cast;
        serial::Serial _ser;
        uint8_t _state = 0;
        uint8_t _data_receive[100] = {0};
    public:
        ros::NodeHandle _n;
        float vx, vy, vthz, th, x = 0.0f, y = 0.0f, accx, accy, accz, quatx, quaty, quatz, quatw, vthx, vthy;
        ros::Time current_time, last_time;
        OdomPublisherNode(std::string serpath) {
            try {
                this->_ser.setPort(serpath);
                this->_ser.setBaudrate(BaudRate);
                serial::Timeout timeOut = serial::Timeout::simpleTimeout(TimeOut);
                this->_ser.setTimeout(timeOut);
                this->_ser.open();
            } catch (serial::IOException &e) {
                ROS_ERROR_STREAM("Unable to open port! please check again.");
            }   
            if (this->_ser.isOpen()) {
                ROS_INFO_STREAM("Serial Port Initialized!");
            } else {
                ROS_ERROR_STREAM("Someting went wrong!");
            }
            
            _odompub = _n.advertise<nav_msgs::Odometry>("odom", 50);
            _imupub = _n.advertise<sensor_msgs::Imu>("imu", 50);

        }

        uint8_t read_odom();
        void cal_odom();
        void send_odom();
};

uint8_t OdomPublisherNode::read_odom() {
    uint8_t buffer = 0;
    data_u odom_struct;
    data_u imu_struct;
    uint8_t checksum = 0;
    //printf("%x    ", buffer);
    //printf("state = %d\n", this->_state);
    switch (this->_state)
    {
        case 0 ... 1:
            this->_ser.read(&buffer, 1); 
            if (buffer == 0xAA || buffer ==0x55)
                this->_state++;
            break;
        case 2 ... 53:
            this->_ser.read(&buffer, 1); 
            this->_data_receive[this->_state - 2] = buffer;
            this->_state++;
            break;
        case 54:
            this->_ser.read(&buffer, 1); 
            for (auto data : this->_data_receive) {
                checksum += data;
            }
            if (checksum != buffer) {
                ROS_ERROR_STREAM("checksum not fit!");
                this->_state = 0;                
            } else {
                this->_state++;
            }
            break;
        case 55:
            for (int i = 0; i < 4; i++) {
                float* datas_ptr[] = {&this->vx, &this->vy, &this->vthz, &this->th};
                odom_struct.data8[0] = this->_data_receive[4 * i + 0];
                odom_struct.data8[1] = this->_data_receive[4 * i + 1];
                odom_struct.data8[2] = this->_data_receive[4 * i + 2];
                odom_struct.data8[3] = this->_data_receive[4 * i + 3];
                *(datas_ptr[i]) = odom_struct.data32;
                //printf("%f", odom_struct.data32);
            }
            for (int i =4; i < 13; i++) {
                float* datas_ptr[] = {&this->accx, &this->accy, &this->accz, &this->vthx,
                &this->vthy, &this->quatx, &this->quaty, &this->quatz, &this->quatw};
                imu_struct.data8[0] = this->_data_receive[4 * i + 0];
                imu_struct.data8[1] = this->_data_receive[4 * i + 1];
                imu_struct.data8[2] = this->_data_receive[4 * i + 2];
                imu_struct.data8[3] = this->_data_receive[4 * i + 3];
                *(datas_ptr[i]) = imu_struct.data32;
            }
            memset(this->_data_receive, 0, sizeof(this->_data_receive));
            //ROS_ERROR_STREAM(this->th);
            ROS_INFO_STREAM("odom calculate successfully");
            this->_state = 0;
            return 1;
        default:
            ROS_ERROR_STREAM("Unrecognize status code!");
            this->_state = 0;
            break;
    }
    return 0;
}

void OdomPublisherNode::cal_odom() {
    this->current_time = ros::Time::now();            
    this->th *= M_PI / 180;
    double dt = (this->current_time - this->last_time).toSec();
    this->last_time = current_time;
    double delta_x = (this->vx * cos(this->th) - this->vy * sin(this->th)) * dt;
    double delta_y = (this->vx * sin(this->th) + this->vy * cos(this->th)) * dt;
    double delta_th = this->vthz * dt;
    this->x += delta_x;
    this->y += delta_y;
    this->th += delta_th;
}

void OdomPublisherNode::send_odom() {
    geometry_msgs::Quaternion odom_quat;
    odom_quat.w = this->quatw;
    odom_quat.x = this->quatx;
    odom_quat.y = this->quaty;
    odom_quat.z = this->quatz;
    nav_msgs::Odometry odom;
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::Imu imu;
    odom_trans.header.stamp = this->current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = this->x;
    odom_trans.transform.translation.y = this->y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    ROS_INFO_STREAM("yaw from quat: " + std::to_string(tf::getYaw(odom_quat)));
    ROS_INFO_STREAM("yaw from yaw: " + std::to_string(this->th));
    
    this->_cast.sendTransform(odom_trans);
    
    odom.header.stamp = this->current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = this->x;
    odom.pose.pose.position.y = this->y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = this->vx;
    odom.twist.twist.linear.y = this->vy;
    odom.twist.twist.linear.z = this->vthz;
    this->_odompub.publish(odom);

    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = "base_footprint";

    imu.angular_velocity.x = this->vthx;
    imu.angular_velocity.y = this->vthy;
    imu.angular_velocity.z = this->vthz;
    
    imu.linear_acceleration.x = this->accx;
    imu.linear_acceleration.y = this->accy;
    imu.linear_acceleration.z = this->accz;
    
    imu.orientation.x = this->quatx;
    imu.orientation.y = this->quaty;
    imu.orientation.z = this->quatz;
    imu.orientation.w = this->quatw;
    
    this->_imupub.publish(imu);

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "odometryNimu_publisher");
    OdomPublisherNode odomNode(SerialPath);


    odomNode.current_time  =  ros::Time::now();
    odomNode.last_time = ros::Time::now();

    while (odomNode._n.ok()) {
        if (odomNode.read_odom()) {
            odomNode.cal_odom();
            odomNode.send_odom();
        }
    }
}
