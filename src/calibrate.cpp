//
// Created by eirik on 26.03.19.
//

#include "hand_eye_calibration/chessboard.h"
#include "hand_eye_calibration/camparam_io.h"
#include "hand_eye_calibration/pose_io.h"
#include "hand_eye_calibration/parkmartin.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <Eigen/Core>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <vector>


int main(){

    /////////////////////////////////////////
    // MAKE LIST OF CALIBRATION IMAGES
    /////////////////////////////////////////

    vector<vector<cv::Point2f>> pointsImage;
    vector<vector<cv::Point3f>> points3d;
    vector<cv::Point3f> obj;
    vector<int> invalids;

    vector<string> imagelist;
    listImages("/home/eirik/catkin_ws/src/hand_eye_calibration/data/calib080419/img/","png", imagelist);

    /////////////////////////////////////////
    // DETECT CORNERS AND ERASE INVALID IMAGES
    /////////////////////////////////////////

    cout << "Size of imagelist: " << imagelist.size() << endl << endl;

    for (int i = 0; i<imagelist.size();i++){ // bug???
        cv::Mat image = readColorImage(imagelist[i]); // TODO argv

        cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);
        cv::imshow("Display window", image);
        cv::waitKey(0);

        auto getCorner = findCorners(image); // what if no corners are found???
        auto found = get<1>(getCorner);

        cout << imagelist[i] << ": ";

        if (found){
            auto corner = get<0>(getCorner);
            drawCorners(image,corner);
            cv::imshow("Display window", image);
            cv::waitKey(0);
            saveCorners(pointsImage,points3d,obj,corner);
        }
        else {
            cout << "No corners found in image" << endl;
            invalids.push_back(i);
        }
    }

    cout << "\nNumber of images with detected corners: " << pointsImage.size() << endl << endl;

    /////////////////////////////////////////
    // CALIBRATION OF LENS
    /////////////////////////////////////////

    cv::Mat image = readColorImage(imagelist[1]); // image only to get correct image size
    tuple<cv::Mat, cv::Mat, vector<cv::Mat>, vector<cv::Mat>> calib = calibrateLens(pointsImage,points3d,image);

    const char* filepath1 = "/home/eirik/catkin_ws/src/hand_eye_calibration/data/calib080419/calib.yml";
    writeCamParams(get<0>(calib),get<1>(calib),filepath1,5);

    cv::Mat camMat1, distCoeffs1;
    readCamParams(filepath1,camMat1,distCoeffs1);

    /////////////////////////////////////////
    // POSE ESTIMATION OF CHECKERBOARD
    /////////////////////////////////////////

    vector<Eigen::Vector3f> tvecs;
    vector<Eigen::Matrix3f> rvecs;

    for (int i = 0; i<pointsImage.size();i++) {
        tuple<cv::Mat, cv::Mat> pnp = getObjectPosePnP(pointsImage[i],points3d[i],camMat1,distCoeffs1);

        Eigen::Vector3f tvec;
        cv::cv2eigen(get<1>(pnp),tvec);
        tvecs.push_back(tvec);

        cv::Mat r;
        cv::Rodrigues(get<0>(pnp),r);
        Eigen::Matrix3f rMat;
        cv::cv2eigen(r,rMat);

        rvecs.push_back(rMat);
    }

    /////////////////////////////////////////
    // READ ROBOT END EFFECTOR POSE LIST AND REMOVE POSES WITHOUT MATCHES IN IMAGES
    /////////////////////////////////////////

    vector<string> poselist;
    listPoses("/home/eirik/catkin_ws/src/hand_eye_calibration/data/calib080419/pose/","yml",poselist);

    for(int i =0; i<invalids.size(); i++){
        poselist.erase(poselist.begin()+ invalids[i]);
    }

    cout << "Size of poselist: " << poselist.size() << endl;

    /////////////////////////////////////////
    // IMPORT AND CONVERT TO EIGEN 4f TRANSFORMATION MATRIX
    /////////////////////////////////////////

    vector<Eigen::Matrix4f> tRB_vec;
    vector<Eigen::Matrix4f> tCB_vec;

    for (int i = 0; i<poselist.size();i++) {
        vector<float> posevec = readSinglePose(poselist[i]);
        geometry_msgs::PoseStamped pose = vectorToPose(posevec);
        Eigen::Matrix4f t1;
        convertTo4x4(pose,t1);
        tRB_vec.push_back(t1);

        Eigen::Matrix4f t2;
        t2.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
        t2.block<3,3>(0,0) = rvecs[i];
        t2.block<3,1>(0,3) =tvecs[i];

        tCB_vec.push_back(t2);
    }

    cout << "Size of tRB_vec: " << tRB_vec.size() << endl;

    // TODO: Parkmartin return X
    /////////////////////////////////////////
    // HAND EYE CALIBRATION
    /////////////////////////////////////////

    performEstimation(tRB_vec,tCB_vec);
}