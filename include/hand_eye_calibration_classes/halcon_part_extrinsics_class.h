//
// Created by eirik on 01.06.19.
//
#pragma once
#ifndef PROJECT_HALCON_PART_EXTRINSICS_CLASS_H
#define PROJECT_HALCON_PART_EXTRINSICS_CLASS_H

#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>


class HalconPartExtrinsics {
public:
    HalconPartExtrinsics(std::vector <std::string> &pointCloudList);

    ~HalconPartExtrinsics() = default;

    bool edges;

    std::vector <Eigen::Matrix4d> getPartPosesAsEigenMat();
    std::vector<int> getInvalids();

private:
    std::vector<std::string> pointCloudList;

    std::vector<int> invalids;

    //std::vector<Eigen::Vector3d> tvecs;
    //std::vector<Eigen::Matrix3d> rvecs;
    std::vector<Eigen::Matrix4d> TVec;

    void readPointCloud(std::string &fileName);
    void estimatePose(int i);
    void verifyAndStorePoses();
    void posesToEigenMatrix();
};


#endif //PROJECT_HALCON_PART_EXTRINSICS_CLASS_H
