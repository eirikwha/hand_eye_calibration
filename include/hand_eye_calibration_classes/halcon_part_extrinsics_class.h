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


class HalconPartExtrinsics {
public:
    HalconPartExtrinsics(std::vector <std::string> &pointCloudList, bool edges);

    ~HalconPartExtrinsics() = default;

    std::vector <Eigen::Matrix4d> getPartPosesAsEigenMat();
    std::vector<int> getInvalids();

private:
    const char* surfModelPath;
    std::vector<std::string> pointCloudList;
    bool edges;
    float unit;

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
    void verifyAndStorePoses();
    void poseToEigenMatrix(HTuple pose);
};


#endif //PROJECT_HALCON_PART_EXTRINSICS_CLASS_H
