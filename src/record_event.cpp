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
#include <pcl/filters/filter.h>
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
int n_img = 0, n_pc = 0, n_pose = 0;

// TODO: MAKE IT WORK FOR OTHER POINTCLOUDS AS WELL
// TODO: Publish image to view in rviz

void imageCallback(const sensor_msgs::Image &img) { /// For regular cameras

    cv::Mat color;
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        color =  cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    stringstream sstream;
    sstream << calibrationPath << string("img/") << string("img") << setw(2) << setfill('0') << n_img << ".png"; // possible error with "/
    cv::imwrite(sstream.str(),color);
    cout << sstream.str() << endl;

    /// Show your results
    namedWindow( "Image", CV_WINDOW_AUTOSIZE );
    imshow( "Image", color);
    waitKey(0);

    ++n_img;
}

void pointCloudToImgCallback(const sensor_msgs::PointCloud2 &cloud) { // TODO: check data types - double more correct??

    ROS_INFO_STREAM("In pointcloud2img callback...");

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

        for (int i = 0; i < rows * cols; i++) {
            uint32_t rgb = *reinterpret_cast<uint32_t *>(&p[i]);
            r.emplace_back((rgb >> 16) & 0x0000ff);
            g.emplace_back((rgb >> 8) & 0x0000ff);
            b.emplace_back((rgb) & 0x0000ff);
        }

        cv::Mat color;
        if (p.size() == rows * cols) // check that the rows and cols match the size of your vector
        {
            Mat mr = Mat(rows, cols, CV_8UC1); // initialize matrix of uchar of 1-channel where you will store vec data
            Mat mg = Mat(rows, cols, CV_8UC1);
            Mat mb = Mat(rows, cols, CV_8UC1);
            //copy vector to mat
            memcpy(mr.data, r.data(),
                   r.size() * sizeof(uint8_t)); // change uchar to any type of data values that you want to use instead
            memcpy(mg.data, g.data(), g.size() * sizeof(uint8_t));
            memcpy(mb.data, b.data(), b.size() * sizeof(uint8_t));

            std::vector<cv::Mat> array_to_merge;
            array_to_merge.emplace_back(mb);
            array_to_merge.emplace_back(mg);
            array_to_merge.emplace_back(mr);
            cv::merge(array_to_merge, color);

        }

        stringstream sstream;
        sstream << calibrationPath << string("img/") << string("img") << setw(2) << setfill('0') << n_img
                << ".png"; // possible error with "/
        cv::imwrite(sstream.str(), color);
        ROS_INFO_STREAM("Saved: " << sstream.str());

        /// Show your results
        namedWindow("Image", CV_WINDOW_AUTOSIZE);
        imshow("Image", color);
        waitKey();

    ++n_img;
}

void pointCloudCallback(const sensor_msgs::PointCloud2 &cloud) {

    pcl::PointCloud<pcl::PointXYZ> tempCloud;
    pcl::PointCloud<pcl::PointXYZ> cloudFiltered;
    pcl::fromROSMsg(cloud, tempCloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(tempCloud,cloudFiltered,indices);

    stringstream sstream;
    sstream << calibrationPath << string("pointcloud/") << string("pointcloud") << setw(2)
            << setfill('0') << n_pc << ".pcd";

    //pcl::io::savePLYFileASCII(sstream.str(), cloudFiltered);
    pcl::io::savePCDFile(sstream.str(), cloudFiltered, false);
    ROS_INFO_STREAM("Saved: " << sstream.str());

    ++n_pc;
}

void poseCallback(const geometry_msgs::PoseStamped &robotPose){

    if (n_pose < n_img || n_pose < n_pc){
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
        std::string filename = calibrationPath + string("pose/") + string("pose") + sstream.str() + string(".yml");

        FileStorage fs(filename, FileStorage::WRITE);
        fs << "pose" << pose;
        fs.release();

        ROS_INFO_STREAM("Saved: " << filename);

        ++n_pose;
    }
}

void printHelp(int, char **argv) {
    cout << "Syntax is: "<< argv[0] <<  " /visualDataTopic /poseTopic /calibration/path/ {img | pointcloud_to_img | pointcloud} \n" << endl;
}

int main (int argc, char** argv) {

    ROS_INFO_STREAM("Record a set of robot poses and corresponding images/pointclouds.");
    if (argc < 5) {
        printHelp (argc, argv);
        return (-1);
    }

    const char *imageTopic;
    const char *pointCloudTopic;
    const char *poseTopic = argv[2];
    calibrationPath = argv[3];

    if (argv[4] == string("img")) {
        imageTopic = argv[1];
    } else if (argv[4] == string("pointcloud_to_img") || argv[4] == string("pointcloud")) {
        pointCloudTopic = argv[1];
    } else {
        printHelp(argc, argv);
        return (-1);
    }

    ROS_INFO_STREAM("Listening for robot poses at: " << poseTopic);

    /// Initialize ROS
    ros::init(argc, argv, "PoseRecorder");
    ros::NodeHandle nh;

    /// Create a ROS subscriber for the input image or pointcloud
    ros::Subscriber subImg;
    ros::Subscriber subCloud;

    if (argv[4] == string("img")) {
        subImg = nh.subscribe(imageTopic, 1, imageCallback);
        ROS_INFO_STREAM("Listening for images at: " << imageTopic);
    }
    else if (argv[4] == string("pointcloud_to_img")) {
        subImg = nh.subscribe(pointCloudTopic, 1, pointCloudToImgCallback);
        ROS_INFO_STREAM("Listening for pointclouds at: " << pointCloudTopic);
    }
    else {
        subImg = nh.subscribe(pointCloudTopic, 1, pointCloudToImgCallback);
        subCloud = nh.subscribe(pointCloudTopic, 1, pointCloudCallback);
        ROS_INFO_STREAM("Listening for pointclouds at: " << pointCloudTopic);
    }

    /// Create a ROS subscriber for the input pose
    ros::Subscriber subPose = nh.subscribe(poseTopic, 1, poseCallback);

    ros::spin();
}
