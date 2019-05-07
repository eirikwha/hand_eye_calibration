//
// Created by eirik on 26.03.19.
//

#pragma once

#ifndef PROJECT_CHESSBOARD_H
#define PROJECT_CHESSBOARD_H

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

using namespace std;

const vector<cv::Point3f> genPatternPoints();

cv::Mat readColorImage(string filename);

tuple<vector<cv::Point2f>,bool> findCorners(cv::Mat image);

void drawCorners(cv::Mat image, vector<cv::Point2f> corners);

void saveCorners(vector<vector<cv::Point2f>> &pointsImage, vector<vector<cv::Point3f>> &points3d,
                 vector<cv::Point3f> obj, vector<cv::Point2f> corners);

tuple<cv::Mat, cv::Mat, vector<cv::Mat>, vector<cv::Mat> > calibrateLens(vector<vector<cv::Point2f>> &pointsImage,
                                                                         vector<vector<cv::Point3f>> &points3d, cv::Mat &image);

tuple<cv::Mat, cv::Mat> getObjectPosePnP(vector<cv::Point2f> &pointsImage, vector<cv::Point3f> &points3d,
                                         cv::Mat &cameraMatrix, cv::Mat &distCoeffs, bool ransac);

cv::Mat undistortImage(cv::Mat image, cv::Mat intrinsic, cv::Mat distCoeffs);

void drawVector_withProjectPointsMethod(float x, float y, float z, float r, float g, float b, cv::Mat &rvec, cv::Mat &tvec, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &dst);

void drawAxis(cv::Mat &rvec, cv::Mat &tvec, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &dst);

#endif //PROJECT_CHESSBOARD_H
