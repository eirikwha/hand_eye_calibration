//
// Created by eirik on 31.05.19.
//

//
// Created by eirik on 26.03.19.
//

#include "hand_eye_calibration_classes/chessboard_calib_class.h"
#include "hand_eye_calibration_classes/chessboard_extrinsics_class.h"
#include "hand_eye_calibration_classes/hand_eye_calib_class.h"
#include "hand_eye_calibration/pose_io.h"
#include "hand_eye_calibration/camparam_io.h"

#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <vector>

#define CALIBRATION_DEBUG 0
#define DEBUG_IMAGES 0

// TODO: Args with filepath of poses and images
using namespace std;

int main(){

    const char* intrinsicPath = "/home/eirik/catkin_ws/src/"
                                "hand_eye_calibration/data/calib080419/intrinsics.yml";

    const char* imgPath = "/home/eirik/catkin_ws/src/"
                          "hand_eye_calibration/data/calib080419/img/";

    const char* robotPosePath = "/home/eirik/catkin_ws/src/"
                                "hand_eye_calibration/data/calib080419/pose/";

    const char* extCalibPath = "/home/eirik/catkin_ws/src/"
                               "hand_eye_calibration/data/calib080419/extrinsics.yml";

    vector<string> imagelist;
    CamParamIO::listFiles(imgPath ,"png", imagelist);

    cout << "Size of imagelist: " << imagelist.size() << endl << endl;

    /// CALIBRATION OF LENS
    ChessBoard CB;
    ChessBoardCalib C (imagelist, intrinsicPath, CB);

    ChessBoardCalibResult calResult = C.getCalibrationResult();

    cout << "Number of images with detected corners: "
            << C.getValidPointsImage().size() << endl << endl;

    /// VERIFY CALIBRATON BY UNDISTORTING IMAGE
    cv::Mat img = C.getColorImage(imagelist[1]);

    cv::namedWindow("Before undistortion",CV_WINDOW_NORMAL);

    cv::resizeWindow("Before undistortion", 1080,720);
    cv::imshow("Before undistortion", img);
    cv::waitKey(0);

    cv::namedWindow("After undistortion", CV_WINDOW_NORMAL);
    cv::resizeWindow("After undistortion", 1080,720);
    cv::imshow("After undistortion", C.undistortImage(img));
    cv::waitKey(0);

    cv::destroyAllWindows();

    vector<int> invalidImgs = C.getInvalids();
    vector<vector<cv::Point2f>> pointsImage = C.getValidPointsImage();
    vector<vector<cv::Point3f>> points3d = C.getValidPoints3d();

    /// ERASE INVALID IMAGES
    for(int i =0; i<invalidImgs.size(); i++){
        imagelist.erase(imagelist.begin() + invalidImgs[i]);
    }

    /// POSE ESTIMATION OF CHECKERBOARD
    ChessBoardExtrinsics CBE(imagelist,intrinsicPath,
            CB, pointsImage, points3d);

    vector<Eigen::Matrix4d> tCB_vec = CBE.getChessboardPosesAsEigenMat();

    vector<int> invalidPoses = CBE.getInvalids();

    for(int i =0; i<invalidPoses.size(); i++){
        imagelist.erase(imagelist.begin()+ invalidPoses[i]);
        tCB_vec.erase(tCB_vec.begin() + invalidPoses[i]);
        pointsImage.erase(pointsImage.begin() + invalidPoses[i]);
        points3d.erase(points3d.begin() + invalidPoses[i]);
    }

    /// READ ROBOT END EFFECTOR POSE LIST AND REMOVE POSES WITHOUT MATCHES IN IMAGES
    vector<string> poselist;
    RobotPoseIO::listPoses(robotPosePath ,"yml",poselist);

    for(int i =0; i<invalidImgs.size(); i++){
        poselist.erase(poselist.begin()+ invalidImgs[i]);
    }

    for(int i =0; i<invalidPoses.size(); i++){
        poselist.erase(poselist.begin()+ invalidPoses[i]);
    }
    cout << "Size of poselist: " << poselist.size() << endl;
    vector<Eigen::Matrix4d> tRB_vec;

    // TODO: This into a function!!
    for (int i = 0; i<poselist.size();i++) {
        vector<double> poseVec = RobotPoseIO::readPose(poselist[i]);
        Eigen::Matrix4d t1;
        RobotPoseIO::convertTo4x4(poseVec, t1);
        t1.block(0, 3, 3, 1) = t1.block(0, 3, 3, 1);
        tRB_vec.push_back(t1);
    }

#if CALIBRATION_DEBUG
        cout << "tRB: " << tRB_vec[i] << endl << endl;
        cout << "tCB: " << tCB_vec[i] << endl << endl;
#endif

    cout << "Size of tRB_vec: " << tRB_vec.size() << endl;

    /// HAND EYE CALIBRATION
    Eigen::Matrix4d X;
    vector<Eigen::Matrix4d> T;

    if (tRB_vec.size() < 3){
        cout << "Insufficient data" << endl;
    }
    else {
        if (tRB_vec.size() > 3 && tRB_vec.size() < 10) {
            cout << "At least 10 pose pairs are recommended. "
                    "See the original paper for further explaination" << endl;
        }

        HandEyeCalib PM(tRB_vec, tCB_vec);
        X = PM.getX();
        cout << "X:\n" << X << endl << endl;
    }

    RobotPoseIO::writeTransformation(X, extCalibPath);

#if CALIBRATION_DEBUG
    cout << "Trying to apply all transformations from the robot base to the camera" << endl
    << "All should be quite similar" << endl << endl;
#endif

    /// COMPUTE ACTUAL TRANSFORMATION

    for (int i = 0; i < pointsImage.size(); i++){

        T.emplace_back(tRB_vec[i] * (X * tCB_vec[i].inverse()));

#if CALIBRATION_DEBUG
        cout << T[i] << endl << endl;
#endif
    }

    cout << "For now, we average the transformation elements." << endl
         << "For the future, a median computation of each element should "
            "be considered for outlier robustness." << endl << endl;

    Eigen::Matrix4d T_mat;
    T_mat.setZero();

    for (int i = 0; i < pointsImage.size(); i++){
        T_mat += T[i];
    }

    T_mat = T_mat / pointsImage.size();
    cout << "Final transformation: \n" << T_mat << endl;
}