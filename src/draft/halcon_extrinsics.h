//
// Created by eirik on 07.06.19.
//

#pragma once
#ifndef HAND_EYE_CALIBRATION_HALCON_EXTRINSICS_H
#define HAND_EYE_CALIBRATION_HALCON_EXTRINSICS_H

#include "../../../../../../../opt/ros/kinetic/include/ros/ros.h"
#include "../../../../../../../opt/ros/kinetic/include/ros/package.h"
#include "../../../../../../../usr/include/pcl-1.7/pcl/io/pcd_io.h"
#include "../../../../../../../opt/ros/kinetic/include/pcl_conversions/pcl_conversions.h"
#include "../../../../../../../usr/include/pcl-1.7/pcl/point_types.h"
#include "../../../../../../../usr/include/pcl-1.7/pcl/point_cloud.h"
#include "../../../../../../../usr/include/c++/5/iostream"
#include "../../../../../../../usr/include/c++/5/vector"
#include "../../../../../../../usr/include/eigen3/Eigen/Geometry"
#include "../../../../../../../opt/ros/kinetic/include/opencv-3.3.1-dev/opencv2/core/eigen.hpp"
#include "../../../../../../../opt/ros/kinetic/include/opencv-3.3.1-dev/opencv2/highgui.hpp"
#include "../../../../../../../opt/ros/kinetic/include/geometry_msgs/PoseStamped.h"
#include "../../../../../../../opt/ros/kinetic/include/sensor_msgs/PointCloud2.h"
#include "../../../../../../../opt/ros/kinetic/include/cv_bridge/cv_bridge.h"
#include "../../../../../../../opt/ros/kinetic/include/opencv-3.3.1-dev/opencv2/opencv.hpp"
#include "../../../../../../../opt/ros/kinetic/include/opencv-3.3.1-dev/opencv2/highgui.hpp"
#include "../../../../devel/include/pose_estimator_msgs/GetPoseStamped.h"
#include "../../include/hand_eye_calibration/pose_io.h"


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
