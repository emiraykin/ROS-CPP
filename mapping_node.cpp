#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace std;

const int pixel = 360;

class MappingNode {
public:
    MappingNode() : resim(pixel, pixel, CV_8UC1, cv::Scalar(0)) {}

    void get_rotation(const nav_msgs::Odometry::ConstPtr& msg) {
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        myPoseTheta = yaw;

        myPoseX = msg->pose.pose.position.x;
        myPoseY = msg->pose.pose.position.y;
    }

    void cb_scan(const sensor_msgs::LaserScan::ConstPtr& data) {
        double x, y;
        double normalizer = pixel / (2 * M_PI);

        cv::Mat infoMatrix(360, 2, CV_32FC1, cv::Scalar(0));

        int j = 0;
        for (int i = 0; i < 360; ++i) {
            double cor = data->ranges[i];
            if (!std::isinf(cor)) {
                double angle = (i / 180.0 * M_PI) + myPoseTheta;
                x = cor * std::cos(angle) + myPoseX;
                y = cor * std::sin(angle) + myPoseY;

                infoMatrix.at<float>(j, 0) = round(180 + y * normalizer) * 1;
                infoMatrix.at<float>(j, 1) = round(180 - x * normalizer) * 1;
                ++j;
            }
        }

        for (int k = 0; k < j; ++k) {
            int x_coord = infoMatrix.at<float>(k, 1);
            int y_coord = infoMatrix.at<float>(k, 0);
            resim.at<uchar>(y_coord, x_coord) = 255;
        }

        resim.at<uchar>(round(180 - myPoseY * normalizer), round(180 + myPoseX * normalizer)) = 122;
    }

    void run() {
        ros::NodeHandle nh;

        ros::Subscriber sub_scan = nh.subscribe("/scan", 1, &MappingNode::cb_scan, this);
        ros::Subscriber sub_odom = nh.subscribe("/odom", 1, &MappingNode::get_rotation, this);

        ros::Rate loop_rate(30); // Assuming a loop rate of 30 Hz

        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    double myPoseX = 0.0;
    double myPoseY = 0.0;
    double myPoseTheta = 0.0;
    cv::Mat resim;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapping_node");

    MappingNode mapping_node;
    mapping_node.run();

    return 0;
}
