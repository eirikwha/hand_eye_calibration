//
// Created by eirik on 01.06.19.
//

#include "halcon_part_extrinsics_class.h"
// List of files
// Input is .pcd in pointxyz format, same as pose estimator pkg

// Use pose estimator pkg library to estimate and visualize pose

// Convert pose stamped OR pose to 4x4 eigen

// Save in the same way as for chessboard

HalconPartExtrinsics::HalconPartExtrinsics(std::vector<std::string> &pointCloudList, bool edges):
                                                pointCloudList(pointCloudList), edges(edges) {

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

    genParamValue.Append(1);
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
    // TODO: MAKE PARAM HPP FILE

    sfmGenParamName.Append("camera_parameter 0");
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

std::vector <Eigen::Matrix4f> HalconPartExtrinsics::getPartPosesAsEigenMat(){ // TODO: Should be double?
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

void HalconPartExtrinsics::visualizePose(int i){

    HTuple modelVis;
    HalconIO::readObjectModel3D(plyModelPath, 1, modelVis);

    cout << "Trying to transform model into best pose for visualization" << endl;
    HalconObjectModel::rigidTrans(bestPose, modelVis, transformedModel);

    cout << "Trying to save transformed model." << endl;
    HalconIO::writeObjectModel3DPly(transformedPlyPath, transformedModel);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model1 (new pcl::PointCloud<pcl::PointXYZRGB>());
    model1 = PCLFileHandler::loadPlyToPointXYZRGB(transformedPlyPath);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene1 (new pcl::PointCloud<pcl::PointXYZRGB>());
    scene1 = PCLFileHandler::loadPlyToPointXYZRGB(pointCloudList[i].c_str());
    PCLViz::twoInOneVis(model1,scene1);

}

void HalconPartExtrinsics::verifyAndStorePoses(){ // TODO: ROS visualization rviz
    for (int i = 0; i < pointCloudList.size(); i++){
        estimatePose(i);

        if (estimatePose(i)){
            // TODO: Read green ply and visualize with pcl (Not good, but ok for now)
            visualizePose(i);
            int key = cv::waitKey(0);
            switch (key) {
                case ((int) ('d')):
                    invalids.emplace_back(i);
                    cout << "Marked pose in: " << pointCloudList[i]
                    << " as invalid by keypress d. " << endl;
                    break;

                default:
                    cout << "Stored pose in: "
                         << pointCloudList[i] << endl;
                    poseToEigenMatrix(poses[0]);
                    TVec.emplace_back(tmp);
                    break;
            }
        }
        else{
            invalids.emplace_back(i);
        }
    }
}

void HalconPartExtrinsics::poseToEigenMatrix(HTuple pose) {
    HalconPoseConversion::poseToEigen4fPose(pose, tmp);
}