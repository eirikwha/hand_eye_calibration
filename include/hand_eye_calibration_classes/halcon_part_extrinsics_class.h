//
// Created by eirik on 01.06.19.
//
#pragma once
#ifndef PROJECT_HALCON_PART_EXTRINSICS_CLASS_H
#define PROJECT_HALCON_PART_EXTRINSICS_CLASS_H

#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <halcon_pose_estimation/halcon_surface_matching.h>
#include <halcon_pose_estimation/halcon_io.h>
#include <halcon_pose_estimation/halcon_pose_conversion.h>
#include <halcon_pose_estimation/pcl_file_handler.h>
#include <halcon_pose_estimation/pcl_viz.h>
#include <halcon_pose_estimation/halcon_object_model.h>


class HalconPartExtrinsics {
public:
    HalconPartExtrinsics(std::vector <std::string> &pointCloudList, bool edges);

    ~HalconPartExtrinsics() = default;

    std::vector <Eigen::Matrix4d> getPartPosesAsEigenMat();
    std::vector<int> getInvalids();

private:
    std::vector<std::string> pointCloudList;
    bool edges;
    float unit;

    /// BAD QUICKFIX FOR VISUALIZATION. IS ROS BETTER?
    HTuple transformedModel;
    const char* surfModelPath = "/home/eirik/catkin_ws/src/halcon_pose_estimation/data/output/MG1_SURFMODEL.sfm";
    const char* plyModelPath = "/home/eirik/catkin_ws/src/halcon_pose_estimation/data/input/MG1_green.ply";
    const char* transformedPlyPath = "/home/eirik/catkin_ws/src/halcon_pose_estimation/data/output/MG1_transformed.ply";
    /// BAD QUICKFIX FOR VISUALIZATION. IS ROS BETTER?

    HTuple model, scene, bestPose, poses, matchingResultID;
    HTuple genParamName, genParamValue;
    HTuple sfmGenParamName, sfmGenParamValue, camparam;

    std::vector<int> invalids;
    Eigen::Matrix4f tmp;
    std::vector<Eigen::Matrix4d> TVec;

    void initializeMatchingParams();
    void initializeEdgeMatchingParams();

    void readPointCloud(const char* fileName);
    bool estimatePose(int i);
    void visualizePose(int i);
    void verifyAndStorePoses();
    void poseToEigenMatrix(HTuple pose);
};


#endif //PROJECT_HALCON_PART_EXTRINSICS_CLASS_H
