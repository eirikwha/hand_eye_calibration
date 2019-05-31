//
// Created by eirik on 30.03.19.
//

#pragma once

#ifndef PROJECT_PARKMARTIN_H
#define PROJECT_PARKMARTIN_H

#include <Eigen/Eigen>
#include <iostream>

struct PosePair{
    std::vector<Eigen::Matrix4d> A;
    std::vector<Eigen::Matrix4d> B;
};

namespace HandEye {

    Eigen::Vector3d logTheta(Eigen::Matrix3d R);

    Eigen::Matrix3d invsqrt(Eigen::Matrix3d M);

    Eigen::Matrix3d getR(Eigen::Matrix3d M);

    PosePair createPosePairs(std::vector<Eigen::Matrix4d> tRB, std::vector<Eigen::Matrix4d> tCB);

    Eigen::Matrix4d performEstimation(PosePair AB);
}

#endif //PROJECT_PARKMARTIN_H
