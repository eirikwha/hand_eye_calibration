//
// Created by eirik on 30.03.19.
//

#pragma once

#ifndef PROJECT_PARKMARTIN_H
#define PROJECT_PARKMARTIN_H

#include <Eigen/Eigen>
#include <iostream>


Eigen::Vector3f getLogTheta(Eigen::Matrix3f R);

void performEstimation(std::vector<Eigen::Matrix3f> rotationRB_vec, std::vector<Eigen::Vector3f> translationRB_vec,
                       std::vector<Eigen::Matrix3f> rotationCB_vec,std::vector<Eigen::Vector3f> translationCB_vec);



#endif //PROJECT_PARKMARTIN_H
