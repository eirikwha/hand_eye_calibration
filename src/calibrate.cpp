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
#include <iostream>
#include <vector>


int main(){

    vector<vector<cv::Point2f>> pointsImage;
    vector<vector<cv::Point3f>> points3d;
    vector<cv::Point3f> obj;

    writeImageList("/home/eirik/catkin_ws/src/hand_eye_calibration/data/calib1_img/","imglist.yml","png");

    vector<string> imagelist;
    readImageList("/home/eirik/catkin_ws/src/hand_eye_calibration/data/calib1_img/imglist.yml", imagelist);

    for (int i = 0; i<imagelist.size();i++){
        cv::Mat image = readColorImage(imagelist[i]); // TODO argv

        cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);
        cv::imshow("Display window", image);
        cv::waitKey(0);

        auto getCorner = findCorners(image);
        auto corner = get<0>(getCorner);
        auto found = get<1>(getCorner);

        if (found){
            drawCorners(image,corner);
            cv::imshow("Display window", image);
            cv::waitKey(0);
            saveCorners(pointsImage,points3d,obj,corner);
        }
        else {
            cout << "No corners found in image" << endl;
            return -1;
        }
    }

    cv::Mat image = readColorImage(imagelist[1]);
    tuple<cv::Mat, cv::Mat, vector<cv::Mat>, vector<cv::Mat>> calib = calibrateLens(pointsImage,points3d,image);

    //cout << get<0>(calib) << endl;

    const char* filepath1 = "/home/eirik/catkin_ws/src/hand_eye_calibration/data/calib1_result/calib1.yml";
    writeCamParams(get<0>(calib),get<1>(calib),filepath1,5);
    cv::Mat camMat1, distCoeffs1;
    readCamParams(filepath1,camMat1,distCoeffs1);

    vector<cv::Mat> rvecs;
    vector<cv::Mat> tvecs;
    for (int i = 0; i<pointsImage.size();i++) {
        tuple<cv::Mat, cv::Mat> pnp = getObjectPosePnP(pointsImage[i],points3d[i],camMat1,distCoeffs1);
        rvecs.push_back(get<0>(pnp));
        tvecs.push_back(get<1>(pnp));
    }

    writePoseList("/home/eirik/catkin_ws/src/hand_eye_calibration/data/calib1_pose/","poselist.yml","yml");

    vector<string> poselist;
    readPoseList("/home/eirik/catkin_ws/src/hand_eye_calibration/data/calib1_pose/poselist.yml",poselist);

    ////////////////////////////////////////////////

    vector<float> posevec = readSinglePose("/home/eirik/catkin_ws/src/hand_eye_calibration/data/calib1_pose/pose00.yml");
    geometry_msgs::PoseStamped pose = vectorToPose(posevec);

    writePose(pose,"/home/eirik/catkin_ws/src/hand_eye_calibration/data/",3,true);

    writePoseList("/home/eirik/catkin_ws/src/hand_eye_calibration/data/torstem_test/calib1_pose", "calib1_pose.yml","yml");

    Eigen::Matrix4d t;
    convertTo4x4(pose,t);

    cout << t <<endl;



    // TODO: PARK MARTIN


}