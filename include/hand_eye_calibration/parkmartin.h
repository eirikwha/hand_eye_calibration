//
// Created by eirik on 30.03.19.
//

#pragma once

#ifndef PROJECT_PARKMARTIN_H
#define PROJECT_PARKMARTIN_H

#include <Eigen/Eigen>
#include <iostream>

    Eigen::Vector3f getLogTheta(Eigen::Matrix3f R);

    Eigen::Matrix3f invsqrt(Eigen::Matrix3f M);

    void performEstimation(std::vector<Eigen::Matrix4f> tRB_vec, std::vector<Eigen::Matrix4f> tCB_vec);

#endif //PROJECT_PARKMARTIN_H
