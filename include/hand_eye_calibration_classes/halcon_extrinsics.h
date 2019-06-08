//
// Created by eirik on 07.06.19.
//

#pragma once
#ifndef HAND_EYE_CALIBRATION_HALCON_EXTRINSICS_H
#define HAND_EYE_CALIBRATION_HALCON_EXTRINSICS_H

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <pose_estimator_msgs/GetPoseStamped.h>
#include <hand_eye_calibration/pose_io.h>


class HalconExtrinsics {
public:

    HalconExtrinsics(std::vector<std::string> &pointCloudList, bool edges, ros::NodeHandle* nodehandle);

    std::vector <Eigen::Matrix4d> getPartPosesAsEigenMat();
    std::vector<int> getInvalids();

private:

    ros::NodeHandle nh;
    ros::Publisher pubPointCloud2;
    std::string pointCloudTopic;
    pcl::PointCloud<pcl::PointXYZRGBA> tmpCloud;
    sensor_msgs::PointCloud2 tmpMsg;

    std::vector<std::string> pointCloudList;
    bool edges;

    std::vector<int> invalids;
    geometry_msgs::PoseStamped pose;
    Eigen::Matrix4d tmpMat;
    std::vector<Eigen::Matrix4d> TVec;

    void initializePublishers();
    void readPointCloud(std::string fileName);
    void estimatePoses();
    void poseToEigenMatrix(geometry_msgs::PoseStamped pose);

};

#endif //HAND_EYE_CALIBRATION_HALCON_EXTRINSICS_H
