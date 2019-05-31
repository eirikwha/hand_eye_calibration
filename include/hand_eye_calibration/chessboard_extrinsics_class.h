//
// Created by eirik on 30.05.19.
//

#ifndef PROJECT_CHESSBOARD_EXTRINSICS_CLASS_H
#define PROJECT_CHESSBOARD_EXTRINSICS_CLASS_H

#include "../../include/hand_eye_calibration/chessboard_class.h"
#include "../../include/hand_eye_calibration/chessboard_calib_class.h"
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

class ChessBoardExtrinsics{
public:
    ChessBoardExtrinsics(std::vector<std::string> &imgList, const char* intrinsicsFilePath, ChessBoard CB);

    ChessBoardExtrinsics(std::vector<std::string> &imgList, const char* intrinsicsFilePath, ChessBoard CB,
                     std::vector<std::vector<cv::Point2f>> &pointsImage, std::vector<std::vector<cv::Point3f>> &points3d);

    ~ChessBoardExtrinsics() = default;

    bool pnpRansac;
    std::vector<Eigen::Matrix4d> getChessboardPosesAsEigenMat();


private:
    std::vector<std::string> imgList;
    const char* intrinsicsPath;
    cv::Mat img;

    ChessBoard CB;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    cv::Mat tvec;
    cv::Mat rvec;

    Eigen::Vector3d tvecEigen;
    Eigen::Matrix3d rMat;

    std::vector<std::vector<cv::Point2f>> pointsImage;
    std::vector<std::vector<cv::Point3f>> points3d;
    std::vector<int> invalids;

    std::vector<Eigen::Vector3d> tvecs;
    std::vector<Eigen::Matrix3d> rvecs;
    std::vector<Eigen::Matrix4d> TVec;

    void readColorImage(std::string &fileName);
    void readCamParams(const char* &fileName);
    void computeObjectPosePnP(int i);
    void verifyAndStorePoses();
    void posesToEigenMatrix();

    void drawVectorProjectPointsMethod(float x, float y, float z, float r,
            float g, float b, cv::Mat &rvec, cv::Mat &tvec, cv::Mat &cameraMatrix,
            cv::Mat &distCoeffs, cv::Mat &dst);

    void drawAxis(cv::Mat &rvec, cv::Mat &tvec, cv::Mat &cameraMatrix,
                                       cv::Mat &distCoeffs, cv::Mat &dst);

};


#endif //PROJECT_CHESSBOARD_EXTRINSICS_CLASS_H
