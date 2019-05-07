//
// Created by eirik on 30.03.19.
//

#pragma once

#ifndef PROJECT_PARKMARTIN_H
#define PROJECT_PARKMARTIN_H

#include <Eigen/Eigen>
#include <iostream>

    Eigen::Vector3d logTheta(Eigen::Matrix3d R);

    Eigen::Matrix3d invsqrt(Eigen::Matrix3d M);

    Eigen::Matrix3d getR(Eigen::Matrix3d M);

    Eigen::Matrix4d performEstimation(std::vector<Eigen::Matrix4d> tRB_vec, std::vector<Eigen::Matrix4d> tCB_vec);

#endif //PROJECT_PARKMARTIN_H
