#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class LaserController {
public:
    LaserController() : driving_state(0), counterL(0), counterR(0), isOkay(true) {
        pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        sub_scan = nh.subscribe("/hokuyo_scan", 1, &LaserController::speed_control, this);
        sub_scan2 = nh.subscribe("/hokuyo_scan2", 1, &LaserController::laser_callback2, this);
    }

    void speed_control(const sensor_msgs::LaserScan::ConstPtr& data) {
        geometry_msgs::Twist msg;

        if (driving_state == 0) {
            msg.linear.x = 0.7;
            msg.angular.z = 0;
            ROS_INFO("Driving");
            counterL = 0;
            counterR = 0;
        } else if (driving_state == 1) { // Turn right
            msg.linear.x = 0;
            msg.angular.z = -0.7;
            ROS_INFO("Turning Right");
        } else { // Turn left
            msg.linear.x = 0;
            msg.angular.z = 0.7;
            ROS_INFO("Turning Left");
        }

        pub.publish(msg);
    }

    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& data) {
        // Your laser callback logic here
    }

    void laser_callback2(const sensor_msgs::LaserScan::ConstPtr& data) {
        // Your second laser callback logic here
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub_scan;
    ros::Subscriber sub_scan2;

    int driving_state;
    int counterL;
    int counterR;
    bool isOkay;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_controller_node");
    LaserController controller;
    ros::spin();
    return 0;
}
