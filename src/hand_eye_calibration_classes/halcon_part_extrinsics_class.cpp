//
// Created by eirik on 01.06.19.
//

#include "hand_eye_calibration_classes/halcon_part_extrinsics_class.h"
// List of files
// Input is .pcd in pointxyz format, same as pose estimator pkg

// Use pose estimator pkg library to estimate and visualize pose

// Convert pose stamped OR pose to 4x4 eigen

// Save in the same way as for chessboard

HalconPartExtrinsics::HalconPartExtrinsics(std::vector<std::string> &pointCloudList): pointCloudList(pointCloudList) {

    HalconIO::readSurfaceModel(surfModelPath, model);
    initializeMatchingParams();
    if (edges){
        initializeEdgeMatchingParams();
    }

}

void HalconPartExtrinsics::initializeMatchingParams(){

    genParamName.Append("num_matches");
    genParamName.Append("max_overlap_dist_rel");
    genParamName.Append("scene_normal_computation");
    genParamName.Append("sparse_pose_refinement");
    genParamName.Append("score_type");
    genParamName.Append("pose_ref_use_scene_normals");
    genParamName.Append("dense_pose_refinement");
    genParamName.Append("pose_ref_num_steps");
    genParamName.Append("pose_ref_sub_sampling");
    genParamName.Append("pose_ref_dist_threshold_rel");
    genParamName.Append("pose_ref_scoring_dist_rel");

    genParamValue.Append(5);
    genParamValue.Append(0.5);
    genParamValue.Append("mls");
    genParamValue.Append("true");
    genParamValue.Append("model_point_fraction");
    genParamValue.Append("true");
    genParamValue.Append("true");
    genParamValue.Append(10);
    genParamValue.Append(2);
    genParamValue.Append(0.1);
    genParamValue.Append(0.005);

}

void HalconPartExtrinsics::initializeEdgeMatchingParams(){

    sfmGenParamName.Append("camera_parameter 0");
    // TODO: FOR LOOP, GET VALUES DIRECTLY FROM YAML FILE
    camparam.Append("area_scan_polynomial");
    camparam.Append(1.61928e-02);
    camparam.Append(9.52772e+02);
    camparam.Append(2.61989e-02);
    camparam.Append(7.68154e-07);
    camparam.Append(-1.21351e-02);
    camparam.Append(2.72969e-02);
    camparam.Append(5.86000e-06);
    camparam.Append(5.86000e-06);
    camparam.Append(9.56770e+02);
    camparam.Append(6.19175e+02);
    camparam.Append(1920);
    camparam.Append(1200);
    sfmGenParamValue.Append(camparam);

    try {
        SetSurfaceModelParam(model, sfmGenParamName, sfmGenParamValue);
    }
    catch(HException &exc) {
        cout << exc.ErrorMessage() << endl;
    }
}

std::vector <Eigen::Matrix4d> HalconPartExtrinsics::getPartPosesAsEigenMat(){
    return TVec;
}

std::vector<int> HalconPartExtrinsics::getInvalids(){
    return invalids;
}

void HalconPartExtrinsics::readPointCloud(const char* fileName) {
    HalconIO::readObjectModel3D(fileName, 1, scene);
}

bool HalconPartExtrinsics::estimatePose(int i) {
    readPointCloud(pointCloudList[i].c_str());

    HalconSurfaceMatching::findSurfaceModel3D(model, scene, poses, matchingResultID,
                                              genParamName, genParamValue);

    return poses.Length() / 7 >= 1;
    // store as something that can be visualized
}

void HalconPartExtrinsics::verifyAndStorePoses(){
    for (int i = 0; i < pointCloudList.size(); i++){
        estimatePose(i);

        if (estimatePose(i)){
            // Some visualization step and waitkey here
            //if(key == 0) {
            poseToEigenMatrix(poses[0]);
            TVec.emplace_back(tmp);
            //}
            //else {
            // add to invalids list?
            // }
        }
        else{
            invalids.emplace_back(i);
        }

    }
}

void HalconPartExtrinsics::poseToEigenMatrix(HTuple pose) {
    HalconPoseConversion::poseToEigen4fPose(pose, tmp);
}