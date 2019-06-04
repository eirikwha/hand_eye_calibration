//
// Created by eirik on 01.06.19.
//

#include "hand_eye_calibration_classes/halcon_part_extrinsics_class.h"
// List of files
// Input is .pcd in pointxyz format, same as pose estimator pkg

// Use pose estimator pkg library to estimate and visualize pose

// Convert pose stamped OR pose to 4x4 eigen

// Save in the same way as for chessboard

HalconPartExtrinsics::HalconPartExtrinsics(std::vector<std::string> &pointCloudList) {

}

std::vector <Eigen::Matrix4d> HalconPartExtrinsics::getPartPosesAsEigenMat(){

}

std::vector<int> HalconPartExtrinsics::getInvalids() {

}

void HalconPartExtrinsics::readPointCloud(std::string &fileName) {

}


void HalconPartExtrinsics::estimatePose(int i) {
    //HalconSurfaceMatching::findSurfaceModel3D()
}

void HalconPartExtrinsics::verifyAndStorePoses(){

}

void HalconPartExtrinsics::posesToEigenMatrix(){

}