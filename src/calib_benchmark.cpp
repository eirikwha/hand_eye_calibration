//
// Created by eirik on 10.06.19.
//

// Ros node
// 12 publishers, 12 topics
// loop som genererer ett og ett pose par
// beregner x
// publiserer alle 12 verdier


#include "hand_eye_calibration/chessboard.h"
#include "hand_eye_calibration/camparam_io.h"
#include "hand_eye_calibration/pose_io.h"
#include "hand_eye_calibration/parkmartin.h"

#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv){

    const char* intrinsicPath = "/home/eirik/catkin_ws/src/"
                                "hand_eye_calibration/data/calib080419/intrinsics.yml";

    const char* imgPath = "/home/eirik/catkin_ws/src/"
                          "hand_eye_calibration/data/calib080419/img/";

    const char* robotPosePath = "/home/eirik/catkin_ws/src/"
                                "hand_eye_calibration/data/calib080419/pose/";

    const char* extCalibPath = "/home/eirik/catkin_ws/src/"
                               "hand_eye_calibration/data/calib080419/extrinsics.yml";

    cv::Mat cameraMatrix, distCoeffs;
    CamParamIO::readCamParams(intrinsicPath,cameraMatrix,distCoeffs);

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

        auto getCorner = findCorners(image); // what if no corners are found???
        auto found = get<1>(getCorner);


        if (found){
            auto corner = get<0>(getCorner);
            saveCorners(pointsImage, points3d, obj, corner);

        }
        else {
            invalidImgs.emplace_back(i);
        }
    }
    cout << "Number of images with detected corners: " << pointsImage.size() << endl << endl;

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
        tvecs.push_back(tvec*0.001);

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

        Eigen::Matrix4d t2;

        t2.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
        t2.block(0,0,3,3) = rvecs[i];
        t2.block(0,3,3,1) = tvecs[i];
        tCB_vec.push_back(t2);
    }

    cout << "Size of tRB_vec: " << tRB_vec.size() << endl;

    // HAND EYE CALIBRATION
    Eigen::Matrix4d X;

    if (tRB_vec.size() < 3){
        cout << "Insufficient data" << endl;
    }
    else {
        if (tRB_vec.size() > 3 && tRB_vec.size() < 10) {
            cout << "At least 10 pose pairs are recommended. "
                    "See the original paper for further explaination" << endl;
        }

        /// Create all pose pairs.
        PosePair AB = HandEye::createPosePairs(tRB_vec,tCB_vec);
        cout << "Number of pose pairs: " << AB.A.size() << endl;

        /// Criterion from reference paper, at least 3 pairs to start with
        PosePair AB_tmp;
        AB_tmp.A.insert(AB_tmp.A.end(),{AB.A[0], AB.A[1], AB.A[2]});
        AB_tmp.B.insert(AB_tmp.B.end(),{AB.B[0], AB.B[1], AB.B[2]});

        ros::init(argc, argv, "talker");
        ros::NodeHandle n;
        ros::Publisher r11, r12, r13, r21, r22, r23, r31, r32, r33, tx, ty, tz;

        std_msgs::Float64 val;
        r11 = n.advertise<std_msgs::Float64>("r11", 1000);
        r12 = n.advertise<std_msgs::Float64>("r12", 1000);
        r13 = n.advertise<std_msgs::Float64>("r13", 1000);
        r21 = n.advertise<std_msgs::Float64>("r21", 1000);
        r22 = n.advertise<std_msgs::Float64>("r22", 1000);
        r23 = n.advertise<std_msgs::Float64>("r23", 1000);
        r31 = n.advertise<std_msgs::Float64>("r31", 1000);
        r32 = n.advertise<std_msgs::Float64>("r32", 1000);
        r33 = n.advertise<std_msgs::Float64>("r33", 1000);

        tx = n.advertise<std_msgs::Float64>("tx", 1000);
        ty = n.advertise<std_msgs::Float64>("ty", 1000);
        tz = n.advertise<std_msgs::Float64>("tz", 1000);

        for (int i = 3; i < AB.A.size(); i++){
            // start with 3 pose pairs
            X = HandEye::performEstimation(AB_tmp);
            cout << X << endl;

            AB_tmp.A.emplace_back(AB.A[i]);
            AB_tmp.B.emplace_back(AB.B[i]);

            cout << "Number of pose pairs: " << AB_tmp.A.size() << endl;

            val.data = X(0);
            r11.publish(val);
            val.data = X(4);
            r12.publish(val);
            val.data = X(8);
            r13.publish(val);
            val.data = X(1);
            r21.publish(val);
            val.data = X(5);
            r22.publish(val);
            val.data = X(9);
            r23.publish(val);
            val.data = X(2);
            r31.publish(val);
            val.data = X(6);
            r32.publish(val);
            val.data = X(10);
            r33.publish(val);

            val.data = X(12);
            tx.publish(val);
            val.data = X(13);
            ty.publish(val);
            val.data = X(14);
            tz.publish(val);
            ros::Duration(0.1).sleep();
        }
    }
}


// launch rqt_plot, plot alle 9 r topics
// launch nytt rqt_plot, alle 3 t topics
