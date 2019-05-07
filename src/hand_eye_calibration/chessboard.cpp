//
// Created by eirik on 26.03.19.
//

#include "hand_eye_calibration/chessboard.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

constexpr int numCornersHor = 9;
constexpr int numCornersVer = 7;
constexpr float square_size = 10; //mm

cv::Size board_sz = cv::Size(numCornersHor, numCornersVer);

const vector<cv::Point3f> genPatternPoints() {
    vector<cv::Point3f> obj;
    for (int i = 0; i < board_sz.height; i++) {
        for (int j = 0; j < board_sz.width; j++) {
            obj.push_back(cv::Point3f((float) j * square_size, (float) i * square_size,0.0f));
        }
    }

    return obj;
}

cv::Mat readColorImage(string filename){
    cv::Mat image;
    image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);   // Read the file

    if(! image.data ){                              // Check for invalid input
        cout <<  "Could not open or find the image" << std::endl ;
    }
    return image;
}

tuple<vector<cv::Point2f>,bool> findCorners(cv::Mat image) {

    cv::Mat gray_image;
    vector<cv::Point2f> corners;

    if (image.channels() == 3) {
        cv::cvtColor(image, gray_image, CV_BGR2GRAY);
    }

    bool found = cv::findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    if(found)
    {
        cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        return make_tuple(corners, found);
    }
    else{
        return make_tuple(corners,found);
    }
}

void drawCorners(cv::Mat image, vector<cv::Point2f> corners) {

    cv::Mat color_image;
    if (image.channels() != 3) {
        cout << "Converting to color" << endl;
        cv::cvtColor(image, image, CV_GRAY2BGR);
    }

    if (!image.data){ // Check for invalid image data
        cout << "Image has no data" << std::endl;
    }

    cv::drawChessboardCorners(image,board_sz, corners,true);
}

void saveCorners(vector<vector<cv::Point2f>> &pointsImage, vector<vector<cv::Point3f>> &points3d,
                    vector<cv::Point3f> obj, vector<cv::Point2f> corners) {

    pointsImage.push_back(corners);
    points3d.push_back(obj);

}


tuple<cv::Mat, cv::Mat, vector<cv::Mat>, vector<cv::Mat>> calibrateLens(vector<vector<cv::Point2f>> &pointsImage,
                                                                        vector<vector<cv::Point3f>> &points3d, cv::Mat &image){

    cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
    cv::Mat distCoeffs;
    vector<cv::Mat> rvecs;
    vector<cv::Mat> tvecs;

    calibrateCamera(points3d, pointsImage, image.size(), intrinsic, distCoeffs, rvecs, tvecs);

    return make_tuple(intrinsic, distCoeffs,rvecs,tvecs);
}

tuple<cv::Mat, cv::Mat> getObjectPosePnP(vector<cv::Point2f> &pointsImage, vector<cv::Point3f> &points3d,
                             cv::Mat &cameraMatrix, cv::Mat &distCoeffs, bool ransac){

    cv::Mat rvec;
    cv::Mat tvec;

    try {
        if (ransac) {
            cv::solvePnPRansac(points3d, pointsImage, cameraMatrix, distCoeffs, rvec, tvec, false, CV_ITERATIVE);
        }
        else{
            cv::solvePnP(points3d, pointsImage, cameraMatrix, distCoeffs, rvec, tvec, false, CV_ITERATIVE);
        }
        return make_tuple(rvec,tvec);
    }
    catch(cv::Exception& e) {
        const char *err_msg = e.what();
        cout << "exception caught: " << err_msg << endl;
    }
}

cv::Mat undistortImage(cv::Mat image, cv::Mat intrinsic, cv::Mat distCoeffs){
    cv::Mat imageUndistorted;
    undistort(image, imageUndistorted, intrinsic, distCoeffs);
    return imageUndistorted;
}

void drawVector_withProjectPointsMethod(float x, float y, float z, float r, float g, float b, cv::Mat &rvec, cv::Mat &tvec, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &dst) {
    std::vector<cv::Point3f> points;
    std::vector<cv::Point2f> projectedPoints;

    //fills input array with 2 points
    points.push_back(cv::Point3f(0, 0, 0));
    points.push_back(cv::Point3f(x, y, z));


    //projects points using cv::projectPoints method
    cv::projectPoints(points, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

    //draws corresponding line
    cv::line(dst, projectedPoints[0], projectedPoints[1],
             CV_RGB(255 * r, 255 * g, 255 * b),5);
}

void drawAxis(cv::Mat &rvec, cv::Mat &tvec, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &dst) {

    drawVector_withProjectPointsMethod(100, 0, 0, 1, 0, 0, rvec, tvec, cameraMatrix, distCoeffs , dst);
    drawVector_withProjectPointsMethod(0, 100, 0, 0, 1, 0, rvec, tvec, cameraMatrix, distCoeffs, dst);
    drawVector_withProjectPointsMethod(0, 0, 100, 0, 0, 1, rvec, tvec, cameraMatrix, distCoeffs, dst);
}

