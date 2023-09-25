#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <termios.h>
#include <sys/poll.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_W           0x77
#define KEYCODE_A           0x61
#define KEYCODE_S           0x73
#define KEYCODE_D           0x64
#define KEYCODE_EQUAL       0x3D
#define KEYCODE_MINUS       0x2D
#define KEYCODE_SPACE       0x20
#define KEYCODE_UNDERSCORE  0x5F
#define KEYCODE_PLUS        0x2B

#define KEYCODE_A_CAP       0x41
#define KEYCODE_D_CAP       0x44
#define KEYCODE_S_CAP       0x53
#define KEYCODE_W_CAP       0x57


class BlueCarKeyboardTelepoNode {
    private:
        geometry_msgs::Twist cmdvel_;
        ros::NodeHandle n_;
        ros::Publisher pub_;

    public:
        BlueCarKeyboardTelepoNode() {
            pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
            
        }
        ~BlueCarKeyboardTelepoNode()  {}
        void keyboardLoop();

        void stopRobot() {
            cmdvel_.linear.x = 0;
            cmdvel_.linear.y = 0;
            cmdvel_.angular.z = 0;
            pub_.publish(cmdvel_);
        }
};

BlueCarKeyboardTelepoNode* tbk;

int kfd = 0;

struct termios cooked, raw;
bool done;

int main(int argc, char** argv) {
    ros::init(argc, argv, "tbk", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    BlueCarKeyboardTelepoNode tbk;

    boost::thread t = boost::thread(boost::bind(&BlueCarKeyboardTelepoNode::keyboardLoop, &tbk));
    ros::Rate r(25);
    ros::spin();
    
    t.interrupt();
    t.join();
    tbk.stopRobot();

    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}

void BlueCarKeyboardTelepoNode::keyboardLoop() {
    static float linear_speed = 0.5;
    static float angular_speed = 1;
    char c;
    bool dirty = false;
    
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));

    raw.c_lflag &=~ (ICANON | ECHO);

    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOL] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("Use WASD keys to control the robot");
    puts("Press Shift to move faster");
    
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    
    for (;;) {
        boost::this_thread::interruption_point();

        int num;
        if ((num = poll(&ufd, 1, 250)) < 0) {
            perror("poll():");
            return;
        }
        else if (num > 0) {
            if (read(kfd, &c, 1) < 0) {
                perror("read():");
                return;
            }
        } else {
            if (dirty == true) {
                stopRobot();
                dirty = false;
            }
            continue;
        }

        switch (c)
        {
        case KEYCODE_W:
            cmdvel_.linear.x = linear_speed;
            cmdvel_.linear.y = 0;
            dirty = true;
            break;
        case KEYCODE_S:
            cmdvel_.linear.x = -1 * linear_speed;
            cmdvel_.linear.y = 0;
            dirty = true;
            break;
        case KEYCODE_A:
            cmdvel_.linear.x = 0;
            cmdvel_.linear.y = linear_speed;
            dirty = true;
            break;
        case KEYCODE_D:
            cmdvel_.linear.x = 0;
            cmdvel_.linear.y = -1 * linear_speed;
            dirty = true;
            break;
        case KEYCODE_SPACE:
            cmdvel_.linear.x = 0;
            cmdvel_.linear.y = 0;
            cmdvel_.angular.z = 0;
            break;
        case KEYCODE_A_CAP:
            cmdvel_.angular.z = 3;
            dirty = true;
            break;
        case KEYCODE_D_CAP:
            cmdvel_.angular.z = -3;
            dirty = true;
            break;
        case KEYCODE_EQUAL:
            linear_speed += 0.1;
            ROS_INFO_STREAM("linear speed now: " + std::to_string(linear_speed));
            break;
        case KEYCODE_MINUS:
            linear_speed -= 0.1;
            ROS_INFO_STREAM("linear speed now: " + std::to_string(linear_speed));
            break;
        case KEYCODE_PLUS:
            angular_speed += 0.1;
            ROS_INFO_STREAM("angular speed now: " + std::to_string(angular_speed));
            break;
        case KEYCODE_UNDERSCORE:
            angular_speed -= 0.1;
            ROS_INFO_STREAM("angular speed now: " + std::to_string(angular_speed));
            break;
        }
        pub_.publish(cmdvel_);
    }
}