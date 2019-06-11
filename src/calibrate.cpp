//
// Created by eirik on 26.03.19.
//

#include "hand_eye_calibration/chessboard.h"
#include "hand_eye_calibration/camparam_io.h"
#include "hand_eye_calibration/pose_io.h"
#include "hand_eye_calibration/parkmartin.h"

#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <vector>

#define CALIBRATION_DEBUG 0
#define DEBUG_IMAGES 0

// TODO: Agrs with filepath of poses and images
int main(){

    const char* intrinsicPath = "/home/eirik/catkin_ws/src/"
                                "hand_eye_calibration/data/calib080419/intrinsics.yml";

    const char* imgPath = "/home/eirik/catkin_ws/src/"
                          "hand_eye_calibration/data/calib080419/img/";

    const char* robotPosePath = "/home/eirik/catkin_ws/src/"
                                "hand_eye_calibration/data/calib080419/pose/";

    const char* extCalibPath = "/home/eirik/catkin_ws/src/"
                               "hand_eye_calibration/data/calib080419/extrinsics.yml";

    vector<vector<cv::Point2f>> pointsImage;
    vector<vector<cv::Point3f>> points3d;
    vector<cv::Point3f> obj = genPatternPoints();
    vector<int> invalidImgs, invalidPoses;

    /// MAKE LIST OF CALIBRATION IMAGES
    vector<string> imagelist;
    CamParamIO::listFiles(imgPath,"png", imagelist);

    /// DETECT CORNERS AND ERASE INVALID IMAGES
    cout << "Size of imagelist: " << imagelist.size() << endl << endl;

    for (int i = 0; i<imagelist.size();i++){
        cv::Mat image = readColorImage(imagelist[i]);

#if DEBUG_IMAGES
        cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);
        cv::imshow("Display window", image);
        cv::waitKey(0);
#endif
        auto getCorner = findCorners(image); // what if no corners are found???
        auto found = get<1>(getCorner);

#if CALIBRATION_DEBUG
        cout << imagelist[i] << ": ";
#endif

        if (found){
            auto corner = get<0>(getCorner);
#if DEBUG_IMAGES
            drawCorners(image,corner);
            cv::imshow("Display window", image);
            cv::waitKey(0);
#endif
            saveCorners(pointsImage, points3d, obj, corner);

#if CALIBRATION_DEBUG
            cout << "Corners stored." << endl;
#endif
        }
        else {

#if CALIBRATION_DEBUG
            cout << "No corners found in image." << endl;
#endif
            invalidImgs.emplace_back(i);
        }
    }
    cout << "Number of images with detected corners: " << pointsImage.size() << endl << endl;

    /// CALIBRATION OF LENS
    cv::Mat image = readColorImage(imagelist[1]); // image only to get correct image size
    tuple<cv::Mat, cv::Mat, vector<cv::Mat>, vector<cv::Mat>> calib;
    calib = calibrateLens(pointsImage,points3d,image);

    CamParamIO::writeCamParams(get<0>(calib),get<1>(calib),intrinsicPath);

    cv::Mat cameraMatrix, distCoeffs;
    CamParamIO::readCamParams(intrinsicPath,cameraMatrix,distCoeffs);

    /// VERIFY CALIBRATON BY UNDISTORTING IMAGE
    cv::namedWindow("Before undistortion",CV_WINDOW_NORMAL);
    cv::resizeWindow("Before undistortion", 1080,720);
    cv::imshow("Before undistortion", readColorImage(imagelist[1]));
    cv::waitKey(0);

    cv::namedWindow("After undistortion", CV_WINDOW_NORMAL);
    cv::resizeWindow("After undistortion", 1080,720);
    cv::imshow("After undistortion", undistortImage(readColorImage(imagelist[1]),
            cameraMatrix,distCoeffs));
    cv::waitKey(0);

    cv::destroyAllWindows();

    /// POSE ESTIMATION OF CHECKERBOARD
    vector<Eigen::Vector3d> tvecs;
    vector<Eigen::Matrix3d> rvecs;

    for(int i =0; i<invalidImgs.size(); i++){
        imagelist.erase(imagelist.begin()+ invalidImgs[i]);
    }

    cout << "Delete image poses by pressing d, accept by pressing ENTER button." << endl;

    for (int i = 0; i<pointsImage.size(); i++) {
        tuple<cv::Mat, cv::Mat> pnp = getObjectPosePnP(pointsImage[i], points3d[i],
                cameraMatrix, distCoeffs, true);

        Eigen::Vector3d tvec;
        cv::cv2eigen(get<1>(pnp), tvec);
        tvecs.emplace_back(tvec*0.001);
        cv::Mat r;
        cv::Rodrigues(get<0>(pnp), r);
        Eigen::Matrix3d rMat;
        cv::cv2eigen(r, rMat); // TODO: CHECK!!!
        rvecs.push_back(rMat);

        cv::Mat img = readColorImage(imagelist[i]);
        drawAxis(get<0>(pnp),get<1>(pnp),cameraMatrix,distCoeffs,img);

        cv::namedWindow("Poses",CV_WINDOW_NORMAL);
        cv::imshow("Poses", img);
        cv::resizeWindow("Poses", 1080,720);
        //cv::waitKey(0);

        int key = cv::waitKey(0);

        switch(key)
        {
            case ((int)('d')):

                invalidPoses.emplace_back(i);
                cout << "Marked pose in: "
                        << imagelist[i] << " as invalid by keypress d. " << endl;
                break;
        }


#if CALIBRATION_DEBUG
        cout << "Checking the validity of the rotation matrices" << endl << endl;
        cout << "Should be I: " << endl << rMat*rMat.transpose() << endl << endl << "detR: " << rMat.determinant() << endl<< endl;
#endif
    }

    for(int i =0; i<invalidPoses.size(); i++){
        imagelist.erase(imagelist.begin()+ invalidPoses[i]);
        pointsImage.erase(pointsImage.begin() + i);
        points3d.erase(points3d.begin() + i);
    }

    /// READ ROBOT END EFFECTOR POSE LIST AND REMOVE POSES WITHOUT MATCHES IN IMAGES
    vector<string> poselist;
    RobotPoseIO::listPoses(robotPosePath,"yml",poselist);

    for(int i =0; i<invalidImgs.size(); i++){
        poselist.erase(poselist.begin()+ invalidImgs[i]);
    }

    for(int i =0; i<invalidPoses.size(); i++){
        poselist.erase(poselist.begin()+ invalidPoses[i]);
    }

    cout << "Size of poselist: " << poselist.size() << endl;

    /// IMPORT AND CONVERT TO EIGEN 4f TRANSFORMATION MATRIX
    vector<Eigen::Matrix4d> tRB_vec;
    vector<Eigen::Matrix4d> tCB_vec;

    for (int i = 0; i<poselist.size();i++) {
        vector<double> poseVec = RobotPoseIO::readPose(poselist[i]);
        Eigen::Matrix4d t1;
        RobotPoseIO::convertTo4x4(poseVec,t1);
        t1.block(0,3,3,1) = t1.block(0,3,3,1);
        tRB_vec.push_back(t1);

#if CALIBRATION_DEBUG
        cout << "Checking the validity of the rotation matrices" << endl << endl;
        cout << "Should be I: " << endl << t1.block(0,0,3,3)*t1.block(0,0,3,3).transpose() << endl << endl << "detR: " << t1.block(0,0,3,3).determinant() << endl<< endl;
#endif

        Eigen::Matrix4d t2;

        t2.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
        t2.block(0,0,3,3) = rvecs[i];
        t2.block(0,3,3,1) = tvecs[i];
        tCB_vec.push_back(t2);

#if CALIBRATION_DEBUG
        cout << "tRB: " << tRB_vec[i] << endl << endl;
        cout << "tCB: " << tCB_vec[i] << endl << endl;
#endif
    }

    cout << "Size of tRB_vec: " << tRB_vec.size() << endl;

    // HAND EYE CALIBRATION
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

        PosePair AB = HandEye::createPosePairs(tRB_vec,tCB_vec);
        X = HandEye::performEstimation(AB);
        cout << "X:\n" << X << endl << endl;
    }

#if CALIBRATION_DEBUG
    cout << "Trying to apply all transformations from the robot base to the camera" << endl
    << "All should be quite similar" << endl << endl;
#endif

    /// COMPUTE ACTUAL TRANSFORMATION
    for (int i = 0; i < pointsImage.size(); i++){

        T.emplace_back(tRB_vec[i] * (X * tCB_vec[i].inverse()));

        // TODO: Visualize pose of gripper
#if CALIBRATION_DEBUG
        cout << T[i] << endl << endl;
#endif
    }

    cout << "For now, we average the transformation elements." << endl
         << "For the future, a median computation of each element "
            "should be considered for outlier robustness." << endl << endl;

    Eigen::Matrix4d T_mat;
    T_mat.setZero();

    for (int i = 0; i < pointsImage.size(); i++){
        T_mat += T[i];
    }

    T_mat = T_mat / pointsImage.size();
    cout << "Final transformation: \n" << T_mat << endl;

    RobotPoseIO::writeTransformation(X, extCalibPath);

}