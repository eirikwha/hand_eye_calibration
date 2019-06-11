//
// Created by eirik on 28.05.19.
//
#pragma once
#ifndef PROJECT_HAND_EYE_CALIB_CLASS_H
#define PROJECT_HAND_EYE_CALIB_CLASS_H

#include <Eigen/Eigen>
#include <iostream>
#include <vector>

struct PosePair{
    std::vector<Eigen::Matrix4d> A;
    std::vector<Eigen::Matrix4d> B;
};

class HandEyeCalib {
public:
    HandEyeCalib(PosePair AB);
    HandEyeCalib(std::vector<Eigen::Matrix4d> tRB_vec, std::vector<Eigen::Matrix4d> tCB_vec);
    ~HandEyeCalib();

    Eigen::Matrix4d getX();
    PosePair getPosePairs();


private:

    PosePair AB;
    Eigen::Matrix4d X;
    std::vector<Eigen::Matrix4d> tRB_vec;
    std::vector<Eigen::Matrix4d> tCB_vec;

    void createPosePairs(std::vector<Eigen::Matrix4d> tRB, std::vector<Eigen::Matrix4d> tCB);

    Eigen::Vector3d logTheta(Eigen::Matrix3d R);

    Eigen::Matrix3d invsqrt(Eigen::Matrix3d M);

    Eigen::Matrix4d performEstimation();

};


#endif //PROJECT_HAND_EYE_CALIB_CLASS_H
