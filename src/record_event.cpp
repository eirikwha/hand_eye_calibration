//
// Created by eirik on 30.03.19.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

using namespace pcl;
using namespace pcl::io;

using namespace std;
using namespace cv;

vector<geometry_msgs::PoseStamped> poseList;
string calibrationPath;
int height = 1200;
int width = 1920;
int n_cloud = 0, n_pose = 0;

// TODO: MAKE IT WORK FOR OTHER POINTCLOUDS AS WELL
// TODO: Add a function that runs matching (findcorners or some 6d pose est, and only saves if matches are found)
// TODO: Maybe run in livemode?
// TODO: Publish image to view in rviz

void pointCloudCallback(const sensor_msgs::PointCloud2 &cloud) { // TODO: check data types - double more correct??

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
    sstream << calibrationPath << string("img/") << string("img") << setw(2) << setfill('0') << n_cloud << ".png"; // possible error with "/
    cv::imwrite(sstream.str(),color);
    cout << sstream.str() << endl;

    // TODO: Make it possible to save pointclouds as well, as pointcloud2 or ply. For matching with PPF

    /// Show your results
    namedWindow( "Image", CV_WINDOW_AUTOSIZE );
    imshow( "Image", color);
    waitKey(0);

    ++n_cloud;
}

void poseCallback(const geometry_msgs::PoseStamped &robotPose){

    // For every recorded point cloud, a pose is recorded. n_pose and n_cloud manages the sequencing of the recording.
    if (n_pose < n_cloud){
        vector<double> pose;
        pose.push_back(robotPose.pose.position.x);
        pose.push_back(robotPose.pose.position.y);
        pose.push_back(robotPose.pose.position.z);
        pose.push_back(robotPose.pose.orientation.x);
        pose.push_back(robotPose.pose.orientation.y);
        pose.push_back(robotPose.pose.orientation.z);
        pose.push_back(robotPose.pose.orientation.w);

        ostringstream oss;
        oss << setw(2) << std::setfill('0') << n_pose;
        std::string filename = calibrationPath + string("pose/") + string("pose") + oss.str() + string(".yml");

        FileStorage fs(filename, FileStorage::WRITE);
        fs << "pose" << pose;
        fs.release();
        ++n_pose;
    }
}

void printHelp(int, char **argv) {
    cout << "Syntax is: "<< argv[0] <<  " /pointCloudTopic /poseTopic /calibration/path/ \n" << endl;
}

/* ---[ */
int
main (int argc, char** argv)
{
    ROS_INFO_STREAM("Record a set of calib1_pose and pointclouds.");

    if (argc < 4)
    {
        printHelp (argc, argv);
        return (-1);
    }

    const char* pointCloudTopic = argv[1];
    const char* poseTopic = argv[2];
    calibrationPath = argv[3];

    ROS_INFO_STREAM("Listening for pointclouds at: " << pointCloudTopic << " and calib1_pose at: " << poseTopic << endl);
    ROS_INFO_STREAM("Wait 20 seconds for the camera to boot up..." << endl);

    /// Initialize ROS
    ros::init(argc, argv, "PoseRecorder");
    ros::NodeHandle nh;

    /// Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe(pointCloudTopic, 1, pointCloudCallback);

    /// Create a ROS subscriber for the input pose
    ros::Subscriber sub2 = nh.subscribe(poseTopic,1,poseCallback);

    ros::spin();
}
