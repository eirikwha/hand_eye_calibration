//
// Created by eirik on 28.05.19.
//
#pragma once
#ifndef PROJECT_PARK_MARTIN_CLASS_H
#define PROJECT_PARK_MARTIN_CLASS_H

#include <Eigen/Eigen>
#include <iostream>
#include <vector>

struct PosePair{
    std::vector<Eigen::Matrix4d> A;
    std::vector<Eigen::Matrix4d> B;
};

class ParkMartin {
public:
    ParkMartin(std::vector<Eigen::Matrix4d> tRB_vec, std::vector<Eigen::Matrix4d> tCB_vec);
    ~ParkMartin();

    Eigen::Matrix4d getX();
    PosePair getPosePairs();

private:

    PosePair AB;
    Eigen::Matrix4d X;

    void createPosePairs(std::vector<Eigen::Matrix4d> tRB, std::vector<Eigen::Matrix4d> tCB);

    Eigen::Vector3d logTheta(Eigen::Matrix3d R);

    Eigen::Matrix3d invsqrt(Eigen::Matrix3d M);

    Eigen::Matrix4d performEstimation();

};


#endif //PROJECT_PARK_MARTIN_CLASS_H
