//
// Created by eirik on 29.03.19.
//

// Write single pose to txt

#include <opencv2/core/core.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include "dirent.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>


#include "hand_eye_calibration/pose_io.h"

using namespace std;
using namespace cv;

void writePose(geometry_msgs::PoseStamped &robotPose, string folderPath, int n, bool isEigen4x4) // this is the pose callback!!!!
{
    vector<float> pose;
    if (isEigen4x4){
        Eigen::Matrix4d t;
        convertTo4x4(robotPose,t);

        for (int row =0; row < 4; row++){
            for (int col =0; col < 4; col++)
            pose.push_back(t(row,col));
        }
    }
    else {
        pose.push_back(robotPose.pose.position.x);
        pose.push_back(robotPose.pose.position.y);
        pose.push_back(robotPose.pose.position.z);
        pose.push_back(robotPose.pose.orientation.x);
        pose.push_back(robotPose.pose.orientation.y);
        pose.push_back(robotPose.pose.orientation.z);
        pose.push_back(robotPose.pose.orientation.w);
    }
    ostringstream oss;
    oss << setw(2) << std::setfill('0') << n;
    std::string filename = folderPath + string("/") + string("pose") + oss.str() + string(".yml");

    FileStorage fs(filename, FileStorage::WRITE);
    fs << "pose" << pose;
    fs.release();
}

// Read single pose from txt // Read all calib1_pose from yml
vector<float> readSinglePose(string fileName){
    vector<float> poseRead;
    FileStorage fsRead(fileName, FileStorage::READ);
    fsRead["pose"] >> poseRead;
    fsRead.release();
    return poseRead;
}

int listPoses(const char* directoryPath, const char* fileType, vector<string> &list) {

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
            if (strcmp("yml",fileType) == 0){
                if (tok1 != nullptr) {
                    ret = strcmp(tok2, fileType);
                    if (ret == 0) {
                        string name = string(directoryPath) + dir->d_name + string(".") + fileType;
                        list.emplace_back(name);
                        ++n;
                    }
                }
            }
            else {
                cout << "Wrong filetype! Try txt." << endl;
                break;
            }
        }
        closedir(d);
    }
    std::sort(list.begin(), list.end());
    return 0;
}

void writePoseList(const char* filePath, const char* outputName, const char *fileType){ // TODO: DONT LIST THE FILENAME ITSELF
    vector <string> list;
    listPoses(filePath,fileType, list);
    string path = string(filePath) + string("/")+ string(outputName);
    FileStorage fs(path, FileStorage::WRITE);
    fs << "calib1_pose" << "[";
    for(int i = 0; i < list.size(); i++){
        fs << list[i];
    }
    fs << "]";
}

void readPoseList(const char* filePath,  vector <string> &poses){
    FileStorage fs(filePath, FileStorage::READ);
    fs["calib1_pose"] >> poses;
    cout << "Pose list:" << endl << endl;
    for (int i = 0 ; i < poses.size(); i++) {
        cout << poses[i] << endl;
    }

    fs.release();
}

geometry_msgs::PoseStamped vectorToPose(vector<float> &poseVec){
    geometry_msgs::PoseStamped pose;
    pose.pose.orientation.x = poseVec[0];
    pose.pose.orientation.y = poseVec[1];
    pose.pose.orientation.z = poseVec[2];
    pose.pose.orientation.w = poseVec[3];

    pose.pose.position.x = poseVec[4];
    pose.pose.position.x = poseVec[5];
    pose.pose.position.x = poseVec[6];

    return pose;
}

void convertTo4x4(geometry_msgs::PoseStamped &pose, Eigen::Matrix4d &t)
{

    tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    t(0, 0) = (float) m[0][0]; t(0, 1) = (float) m[0][1]; t(0, 2) = (float) m[0][2]; t(0, 3) = (float) pose.pose.position.x;
    t(1, 0) = (float) m[1][0]; t(1, 1) = (float) m[1][1]; t(1, 2) = (float) m[1][2]; t(1, 3) = (float) pose.pose.position.y;
    t(2, 0) = (float) m[2][0]; t(2, 1) = (float) m[2][1]; t(2, 2) = (float) m[2][2]; t(2, 3) = (float) pose.pose.position.z;
    t(3, 0) = (float) 0; t(3, 1) = (float) 0; t(3, 2) = (float) 0; t(3, 3) = (float) 1;

}

void readRobotTransformation(vector<Eigen::Matrix3f> &rotationRB_vec, vector<Eigen::Vector3f> &translationRB_vec){ // TODO: NOT REALLY NEEDED, BACK/FOURTH

    vector<string> poselist;
    readPoseList("/home/eirik/catkin_ws/src/hand_eye_calibration/data/calib1_pose/poselist.yml",poselist);

    for (int i = 0; i < poselist.size(); i++){
        Eigen::Matrix3f rotationRB;
        Eigen::Vector3f translationRB;
        vector<float> pose = readSinglePose(poselist[i]);
        rotationRB(0) = pose[0]; rotationRB(1) = pose[1]; rotationRB(2) = pose[2];
        rotationRB(3) = pose[4]; rotationRB(4) = pose[5]; rotationRB(5) = pose[6];
        rotationRB(6) = pose[8]; rotationRB(7) = pose[9]; rotationRB(8) = pose[10];

        translationRB[0] = pose [3]; translationRB[0] = pose [7]; translationRB[0] = pose [11];

        rotationRB_vec.push_back(rotationRB);
        translationRB_vec.push_back(translationRB);

    }

    //return make_tuple(rotationRB_vec,translationRB_vec);
}

void convertChessboardTransformation(){

}
