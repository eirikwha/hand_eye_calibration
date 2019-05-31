//
// Created by eirik on 30.05.19.
//

#ifndef PROJECT_CHESSBOARD_CALIB_CLASS_H
#define PROJECT_CHESSBOARD_CALIB_CLASS_H

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "../../include/hand_eye_calibration/chessboard_class.h"

struct ChessBoardCalibResult{
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
};

class ChessBoardCalib {
public:

    ChessBoardCalib(std::vector<std::string> &imageList, const char* calibResultFilePath, ChessBoard &CB);
    ~ChessBoardCalib() = default;

    std::vector<std::string> imgList;
    const char* filePath;

    void setImgWidth(int num);
    void setImgHeight(int num);

    std::vector<int> getInvalids();
    std::vector<std::vector<cv::Point2f>> getValidPointsImage();
    std::vector<std::vector<cv::Point3f>> getValidPoints3d();
    ChessBoardCalibResult getCalibrationResult();
    cv::Mat getColorImage(std::string &fileName);
    cv::Mat undistortImage(cv::Mat &img);

private:

    int imgWidth;
    int imgHeight;
    cv::Size imgDim;

    ChessBoard CB;
    cv::Mat img;
    cv::Mat grayImg;

    std::vector<cv::Point2f> cornersImg;
    std::vector<std::vector<cv::Point2f>> pointsImage;
    std::vector<std::vector<cv::Point3f>> points3d;
    std::vector<int> invalids;

    cv::Mat intrinsics = cv::Mat(3, 3, CV_32FC1);
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    ChessBoardCalibResult calibResult;

    void readColorImage(std::string &fileName);
    bool findCornersImg();
    void detectValidCorners();
    void calibrateLens();
    void writeCalibrationResult();

};


#endif //PROJECT_CHESSBOARD_CALIB_CLASS_H
