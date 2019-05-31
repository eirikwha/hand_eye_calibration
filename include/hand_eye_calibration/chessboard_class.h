//
// Created by eirik on 28.05.19.
//
#pragma once
#ifndef PROJECT_CHESSBOARD_CLASS_H
#define PROJECT_CHESSBOARD_CLASS_H

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

class ChessBoard {
public:
    ChessBoard();
    ChessBoard(int &numCornersHor, int &numCornersVer, float &squareSz);

    ~ChessBoard() = default;

    cv::Mat img;
    cv::Mat grayImg;

    //std::vector<cv::Point2f> cornersImg;
    std::vector<cv::Point3f> corners3d;


    void setNumCornersHor(int num);
    void setNumCornersVer(int num);
    void setSquareSize(int num);

    cv::Size getBoardSize();
    float getSquareSize();

    //void setImgWidth(int num);
    //void setImgHeight(int num);
/*
    ChessBoard(std::vector<std::vector<cv::Point2f>> pointsImage, std::vector<std::vector<cv::Point3f>> points3d);

    std::tuple<cv::Mat, cv::Mat, std::vector<cv::Mat>, std::vector<cv::Mat>> getCalibrationResult();

    void drawCorners(cv::Mat image, std::vector<cv::Point2f> corners);

    void drawVector_withProjectPointsMethod(float x, float y, float z, float r, float g, float b, cv::Mat &rvec,
                                            cv::Mat &tvec, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &dst);

    void drawAxis(cv::Mat &rvec, cv::Mat &tvec, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &dst);

    cv::Mat undistortImage(cv::Mat image, cv::Mat intrinsic, cv::Mat distCoeffs);
*/

private:
/*
    int imgWidth;
    int imgHeight;
    cv::Size imgDim;
*/
    int numCornersHor;
    int numCornersVer;
    float squareSize;
    cv::Size boardSize;


    //cv::Mat readColorImage(std::string filename);
    void genCorners3d();
    //bool findCornersImg();

/*
    std::tuple<cv::Mat, cv::Mat> objectPose;
    std::tuple<cv::Mat, cv::Mat, std::vector<cv::Mat>, std::vector<cv::Mat>> calibrationResult;
    std::vector<std::vector<cv::Point2f>> pointsImage;
    std::vector<std::vector<cv::Point3f>> points3d;

    const std::vector<cv::Point3f> genPatternPoints();

    cv::Mat readColorImage(std::string filename);

    std::tuple<std::vector<cv::Point2f>,bool> findCorners(cv::Mat image);

    void saveCorners(std::vector<std::vector<cv::Point2f>> &pointsImage, std::vector<std::vector<cv::Point3f>> &points3d,
                                                        std::vector<cv::Point3f> obj, std::vector<cv::Point2f> corners);

    std::tuple<cv::Mat, cv::Mat, std::vector<cv::Mat>, std::vector<cv::Mat>> calibrateLens(
                                            std::vector<std::vector<cv::Point2f>> &pointsImage,
                                            std::vector<std::vector<cv::Point3f>> &points3d, cv::Size &img_dim);

    std::tuple<cv::Mat, cv::Mat> getObjectPosePnP(std::vector<cv::Point2f> &pointsImage,
                                            std::vector<cv::Point3f> &points3d,
                                            cv::Mat &cameraMatrix, cv::Mat &distCoeffs, bool ransac);
*/
};


#endif //PROJECT_CHESSBOARD_CLASS_H
