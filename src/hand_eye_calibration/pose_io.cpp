//
// Created by eirik on 29.03.19.
//

// Write single pose to txt

#include <opencv2/core/core.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include "dirent.h"
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "hand_eye_calibration/pose_io.h"

using namespace std;
using namespace cv;

namespace RobotPoseIO {

/// Read single pose from txt // Read all calib1_pose from yml
    vector<double> readPose(string fileName) {
        vector<double> poseRead;
        FileStorage fsRead(fileName, FileStorage::READ);
        fsRead["pose"] >> poseRead;
        fsRead.release();
        return poseRead;
    }

    int listPoses(const char *directoryPath, const char *fileType, vector<string> &list) {

        DIR *d;
        struct dirent *dir;
        d = opendir(directoryPath);
        int ret = 0;
        int n = 0;
        char *tok1, *tok2;

        if (d) {
            while ((dir = readdir(d)) != nullptr) {
                tok1 = strtok(dir->d_name, ".");
                tok2 = strtok(nullptr, ".");
                if (strcmp("yml", fileType) == 0) {
                    if (tok1 != nullptr) {
                        ret = strcmp(tok2, fileType);
                        if (ret == 0) {
                            string name = string(directoryPath) + dir->d_name + string(".") + fileType;
                            list.emplace_back(name);
                            ++n;
                        }
                    }
                } else {
                    cout << "Wrong filetype! Try txt." << endl;
                    break;
                }
            }
            closedir(d);
        }
        std::sort(list.begin(), list.end());
        return 0;
    }

    void writePoseList(const char *filePath, const char *outputName,
                       const char *fileType) { // TODO: DONT LIST THE FILENAME ITSELF
        vector<string> list;
        listPoses(filePath, fileType, list);
        string path = string(filePath) + string("/") + string(outputName);
        FileStorage fs(path, FileStorage::WRITE);
        fs << "calib1_pose" << "[";
        for (int i = 0; i < list.size(); i++) {
            fs << list[i];
        }
        fs << "]";
    }

    void readPoseList(const char *filePath, vector<string> &poses) {
        FileStorage fs(filePath, FileStorage::READ);
        fs["calib1_pose"] >> poses;
        cout << "Pose list:" << endl << endl;
        for (int i = 0; i < poses.size(); i++) {
            cout << poses[i] << endl;
        }

        fs.release();
    }

    geometry_msgs::PoseStamped vectorToPose(vector<double> &poseVec) {
        geometry_msgs::PoseStamped pose;

        pose.pose.position.x = poseVec[0];
        pose.pose.position.y = poseVec[1];
        pose.pose.position.z = poseVec[2];

        pose.pose.orientation.x = poseVec[3];
        pose.pose.orientation.y = poseVec[4];
        pose.pose.orientation.z = poseVec[5];
        pose.pose.orientation.w = poseVec[6];

        return pose;
    }

    void convertTo4x4(vector<double> &poseVec, Eigen::Matrix4d &t) {
        Eigen::Quaterniond q;
        q.x() = poseVec[3];
        q.y() = poseVec[4];
        q.z() = poseVec[5];
        q.w() = poseVec[6];
        Eigen::Matrix3d m = q.normalized().toRotationMatrix();

        t(0, 0) = (double) m(0, 0);
        t(0, 1) = (double) m(0, 1);
        t(0, 2) = (double) m(0, 2);
        t(0, 3) = poseVec[0];
        t(1, 0) = (double) m(1, 0);
        t(1, 1) = (double) m(1, 1);
        t(1, 2) = (double) m(1, 2);
        t(1, 3) = poseVec[1];
        t(2, 0) = (double) m(2, 0);
        t(2, 1) = (double) m(2, 1);
        t(2, 2) = (double) m(2, 2);
        t(2, 3) = poseVec[2];
        t(3, 0) = (double) 0;
        t(3, 1) = (double) 0;
        t(3, 2) = (double) 0;
        t(3, 3) = (double) 1;

    }

    void writeTransformation(Eigen::Matrix4d T, const char *filePath) {

        cv::Mat Tv;
        eigen2cv(T, Tv);
        FileStorage fs(filePath, FileStorage::WRITE);
        time_t rawtime;
        time(&rawtime);
        fs << "calibrationDate" << asctime(localtime(&rawtime));
        fs << "transformationMatrix" << Tv;
        fs.release();
    }

}

