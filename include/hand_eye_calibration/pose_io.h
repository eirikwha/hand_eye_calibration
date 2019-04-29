//
// Created by eirik on 29.03.19.
//
#pragma once
#ifndef PROJECT_POSE_IO_H
#define PROJECT_POSE_IO_H

#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>

void writePose(geometry_msgs::PoseStamped &robotPose, std::string folderPath, int i, bool isEigen4x4);

std::vector<float> readSinglePose(std::string fileName);

int listPoses(const char* directoryPath, const char* fileType, std::vector<std::string> &list);

void writePoseList(const char* filePath, const char* outputName, const char *fileType);

void readPoseList(const char* filePath,  std::vector <std::string> &poses);

geometry_msgs::PoseStamped vectorToPose(std::vector<float> &poseVec);

void convertTo4x4(geometry_msgs::PoseStamped &pose, Eigen::Matrix4f &t);

void readRobotTransformation(std::vector<Eigen::Matrix3f> &rotationRB_vec, std::vector<Eigen::Vector3f> &translationRB_vec);

#endif //PROJECT_POSE_IO_H
