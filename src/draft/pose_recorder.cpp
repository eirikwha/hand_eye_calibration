//
// Created by eirik on 08.06.19.
//
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <signal.h>

using namespace std;

int n_pose = 0;
int n_it = 1;
string posePath;

void poseCallback(const geometry_msgs::PoseStamped &robotPose){
    vector<double> pose;
    pose.push_back(robotPose.pose.position.x);
    pose.push_back(robotPose.pose.position.y);
    pose.push_back(robotPose.pose.position.z);
    pose.push_back(robotPose.pose.orientation.x);
    pose.push_back(robotPose.pose.orientation.y);
    pose.push_back(robotPose.pose.orientation.z);
    pose.push_back(robotPose.pose.orientation.w);

    stringstream sstream;
    sstream << setw(2) << std::setfill('0') << n_pose;
    std::string filename = posePath + string("pose") + sstream.str() + string(".yml");

    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs << "pose" << pose;
    fs.release();
    n_pose += 1;

    ROS_INFO_STREAM("Saved: " << filename);
}

void wait()
{
    cout << "-------------------------------------------------" << endl;
    cout << "Press ENTER store a pose" << endl;
    cout << "-------------------------------------------------" << endl;
    cin.ignore( numeric_limits <streamsize> ::max(), '\n' );
    n_it += 1;
}

int main(int argc, char** argv){

    const char *poseTopic = argv[1];
    posePath = argv[2];

    ROS_INFO_STREAM("Listening for robot poses at: " << poseTopic);

    ros::init(argc, argv, "PoseRecorder");
    ros::NodeHandle nh;
    ros::Subscriber subPose = nh.subscribe(poseTopic, 1, poseCallback);

    ros::spin();
}