//
// Created by eirik on 07.06.19.
//

#include "halcon_extrinsics.h"

using namespace std;

HalconExtrinsics::HalconExtrinsics(std::vector<std::string> &pointCloudList, bool edges, ros::NodeHandle* nodehandle):nh(*nodehandle),
        pointCloudList(pointCloudList), edges(edges) {

    initializePublishers();

    if (!nh.getParam("pointcloud_topic", pointCloudTopic))
    {
        pointCloudTopic = "hdr_pointcloud";
        ROS_ERROR("HalconExtrinsics Class: Could not parse pointcloud topic. Setting default.");
    }

}

void HalconExtrinsics::initializePublishers(){
    ROS_INFO("Initializing Publishers");
    /// Create a ROS publisher for the output PoseArray
    pubPointCloud2 = nh.advertise<sensor_msgs::PointCloud2>(pointCloudTopic, 1);
}

std::vector <Eigen::Matrix4d> HalconExtrinsics::getPartPosesAsEigenMat(){
    return TVec;
}

std::vector<int> HalconExtrinsics::getInvalids(){
    return invalids;
}

void HalconExtrinsics::estimatePoses(){

    for (int i = 0; i < pointCloudList.size(); i++){
        readPointCloud(pointCloudList[i]);
        pcl::toROSMsg(tmpCloud,tmpMsg);
        pubPointCloud2.publish(tmpMsg);

        ros::ServiceClient client = nh.serviceClient<pose_estimator_msgs::GetPoseStamped>("pose_estimator_server");
        pose_estimator_msgs::GetPoseStamped srv;
        srv.request.req = 1;
        client.call(srv);
        srv.response.pose = pose;

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
                poseToEigenMatrix(pose);
                TVec.emplace_back(tmpMat);
                break;
            }
        }
}


void HalconExtrinsics::readPointCloud(std::string fileName) {
    pcl::io::loadPCDFile(fileName, tmpCloud);
}

void HalconExtrinsics::poseToEigenMatrix(geometry_msgs::PoseStamped pose) {
    vector<double> p;
    p.emplace_back(pose.pose.position.x);
    p.emplace_back(pose.pose.position.y);
    p.emplace_back(pose.pose.position.z);

    p.emplace_back(pose.pose.orientation.x);
    p.emplace_back(pose.pose.orientation.y);
    p.emplace_back(pose.pose.orientation.z);
    p.emplace_back(pose.pose.orientation.w);

    RobotPoseIO::convertTo4x4(p,tmpMat);
}