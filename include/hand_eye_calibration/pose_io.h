//
// Created by eirik on 29.03.19.
//
#pragma once
#ifndef PROJECT_POSE_IO_H
#define PROJECT_POSE_IO_H

#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>

namespace RobotPoseIO {

    void writePose(geometry_msgs::PoseStamped &robotPose, std::string folderPath, int i, bool isEigen4x4);

    std::vector<double> readPose(std::string fileName);

    int listPoses(const char *directoryPath, const char *fileType, std::vector<std::string> &list);

    void writePoseList(const char *filePath, const char *outputName, const char *fileType);

    void readPoseList(const char *filePath, std::vector<std::string> &poses);

    geometry_msgs::PoseStamped vectorToPose(std::vector<double> &poseVec);

    void convertTo4x4(std::vector<double> &poseVec, Eigen::Matrix4d &t);

    void writeTransformation(Eigen::Matrix4d T, const char *filePath);
}

#endif //PROJECT_POSE_IO_H
