#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <std_msgs/UInt8.h>
#include <termios.h>
#include <sys/poll.h>


#define KEYCODE_W   0x77
#define KEYCODE_A   0x61
#define KEYCODE_S   0x73
#define KEYCODE_D   0x64

#define KEYCODE_A_CAP   0x41
#define KEYCODE_D_CAP   0x44
#define KEYCODE_S_CAP   0x53
#define KEYCODE_W_CAP   0x57

class BlueCarKeyboardTelepoNode {
    private:
        std_msgs::UInt8 keyvel_;
        ros::NodeHandle n_;
        ros::Publisher pub_;

    public:
        BlueCarKeyboardTelepoNode() {
            pub_ = n_.advertise<std_msgs::UInt8>("bluecar/key_vel", 1);
        }
        ~BlueCarKeyboardTelepoNode()  {}
        void keyboardLoop();

        void stopRobot() {
            keyvel_.data = 0x2e;
            pub_.publish(keyvel_);
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

    ros::spin();
    
    t.interrupt();
    t.join();
    tbk.stopRobot();

    tcsetattr(kfd, TCSANOW, &cooked);

    return(0);
}

void BlueCarKeyboardTelepoNode::keyboardLoop() {
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
    
    int dirty_dirty = 0; //I have to add this dumb flag because of some unexpected reason which cause this program not so perfect
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
                dirty_dirty = 0;
            }
            continue;
        }

        switch (c)
        {
        case KEYCODE_W:
            keyvel_.data = 0x2A;
            dirty = true;
            dirty_dirty++;
            break;
        case KEYCODE_S:
            keyvel_.data = 0x2B;
            dirty = true;
            dirty_dirty++;
            break;
        case KEYCODE_A:
            keyvel_.data = 0x2D;
            dirty = true;
            dirty_dirty++;
            break;
        case KEYCODE_D:
            keyvel_.data = 0x2C;
            dirty = true;
            dirty_dirty++;
            break;
        }
        if (dirty_dirty < 2) {
            continue;
        }
        pub_.publish(keyvel_);
    }
}
