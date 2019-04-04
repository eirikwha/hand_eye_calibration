//
// Created by eirik on 30.03.19.
//

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>

#include <pcl/console/print.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/console/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>


using namespace pcl::console;
using namespace pcl;
using namespace pcl::io;

using namespace std;
using namespace cv;

vector<geometry_msgs::PoseStamped> poseList;
string pointCloudPath;
string posePath;
int height = 1200;
int width = 1920;
int n_cloud = 0, n_pose = 0;

//TODO: MAKE IT WORK FOR OTHER POINTCLOUDS AS WELL

void pointCloudCallback(const sensor_msgs::PointCloud2 &cloud) {

    cout << "cloud.fields[4].name: " << cloud.fields[4].name << endl <<
         "cloud.fields[4].offset: " << cloud.fields[4].offset << endl <<
         "cloud.fields[4].count: " << cloud.fields[4].count << endl <<
         "cloud.fields[4].datatype: " << cloud.data.at(1) << endl << endl;

    std::vector<float> p;
    int rows = height;
    int cols = width;

    for (sensor_msgs::PointCloud2ConstIterator<float> it(cloud, "rgba"); it != it.end(); ++it) {
        float i = it[0];
        p.emplace_back(it[0]);
    }

    std::vector<uint8_t> r;
    std::vector<uint8_t> g;
    std::vector<uint8_t> b;

    for (int i = 0; i < rows*cols; i++){
        uint32_t rgb = *reinterpret_cast<uint32_t *>(&p[i]);
        r.emplace_back((rgb >> 16) & 0x0000ff);
        g.emplace_back((rgb >> 8) & 0x0000ff);
        b.emplace_back((rgb) & 0x0000ff);
    }

    cv::Mat color;
    if(p.size() == rows*cols) // check that the rows and cols match the size of your vector
    {
        Mat mr = Mat(rows, cols, CV_8UC1); // initialize matrix of uchar of 1-channel where you will store vec data
        Mat mg = Mat(rows, cols, CV_8UC1);
        Mat mb = Mat(rows, cols, CV_8UC1);
        //copy vector to mat
        memcpy(mr.data, r.data(), r.size()*sizeof(uint8_t)); // change uchar to any type of data values that you want to use instead
        memcpy(mg.data, g.data(), g.size()*sizeof(uint8_t));
        memcpy(mb.data, b.data(), b.size()*sizeof(uint8_t));

        std::vector<cv::Mat> array_to_merge;
        array_to_merge.emplace_back(mb);
        array_to_merge.emplace_back(mg);
        array_to_merge.emplace_back(mr);
        cv::merge(array_to_merge, color);

    }
    stringstream sstream;
    sstream << pointCloudPath << string("/") << string("img") << setw(2) << setfill('0') << n_cloud << ".png"; // possible error with "/
    cv::imwrite(sstream.str(),color);
    cout << sstream.str() << endl;
    ++n_cloud;
}

void poseCallback(const geometry_msgs::PoseStamped &robotPose){

    vector<float> pose;
    pose.push_back(robotPose.pose.position.x);
    pose.push_back(robotPose.pose.position.y);
    pose.push_back(robotPose.pose.position.z);
    pose.push_back(robotPose.pose.orientation.x);
    pose.push_back(robotPose.pose.orientation.y);
    pose.push_back(robotPose.pose.orientation.z);
    pose.push_back(robotPose.pose.orientation.w);

    ostringstream oss;
    oss << setw(2) << std::setfill('0') << n_pose;
    std::string filename = posePath + string("/") + string("pose") + oss.str() + string(".yml");

    FileStorage fs(filename, FileStorage::WRITE);
    fs << "pose" << pose;
    fs.release();
    ++n_pose;
}

void printHelp(int, char **argv) {
    cout << "Syntax is: "<< argv[0] <<  "/listenTopic /pointCloudTopic /poseTopic /image/path/ /pose/path/ \n" << endl;
}

/* ---[ */
int
main (int argc, char** argv)
{
    cout << "Record a set of calib1_pose and pointclouds." << endl;

    if (argc < 4)
    {
        printHelp (argc, argv);
        return (-1);
    }

    const char* pointCloudTopic = argv[1];
    const char* poseTopic = argv[2];
    pointCloudPath = argv[3];
    posePath = argv[4];

    ROS_INFO_STREAM("Listening for pointclouds at: " << pointCloudTopic << " and calib1_pose at: " << poseTopic);
    cout << endl;

    // Initialize ROS
    ros::init(argc, argv, "PoseRecorder");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe(pointCloudTopic, 1, pointCloudCallback);
    //ros::topic::waitForMessage<geometry_msgs::PoseStamped>(poseTopic);

    ros::Subscriber sub2 = nh.subscribe(poseTopic,1,poseCallback);
    ros::spin();
}
