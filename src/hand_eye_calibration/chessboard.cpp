//
// Created by eirik on 26.03.19.
//

#include "hand_eye_calibration/chessboard.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

int numImages = 20; // set this value for desired number of input images (better to parse or pass as argument in main)
constexpr int numCornersHor = 9;
constexpr int numCornersVer = 7;
constexpr int numSquares = numCornersHor * numCornersVer;
cv::Size board_sz = cv::Size(numCornersHor, numCornersVer);


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

    if (!image.data) // Check for invalid image data
    {
        cout << "Image has no data" << std::endl;
    }

    cv::drawChessboardCorners(image,board_sz, corners,true);
}

void saveCorners(vector<vector<cv::Point2f>> &pointsImage, vector<vector<cv::Point3f>> &points3d,
                    vector<cv::Point3f> obj, vector<cv::Point2f> corners) {

    for (int j = 0; j < numSquares; j++) {
        obj.push_back(cv::Point3f(j / numCornersHor, j % numCornersHor, 0.0f));
    }
/*
    cout << "Store result for calibration? (y/n): ";
    string n;
    cin >> n;
    if (n == "y" || n == "yes") {*/
    pointsImage.push_back(corners);
    points3d.push_back(obj);

    printf("Corners stored.\n");
    /*} else {
        printf("Corners rejected.\n");
        return;
    }*/
}

tuple<cv::Mat, cv::Mat> getObjectPosePnP(vector<cv::Point2f> &pointsImage, vector<cv::Point3f> &points3d,
                             cv::Mat &cameraMatrix, cv::Mat &distCoeffs){

    cv::Mat rvec;
    cv::Mat tvec;


    try {
        cv::solvePnP(points3d, pointsImage, cameraMatrix, distCoeffs, rvec, tvec, false, CV_EPNP);
        return make_tuple(rvec,tvec);
    }
    catch(cv::Exception& e) {
        const char *err_msg = e.what();
        cout << "exception caught: " << err_msg << endl;
    }
}


tuple<cv::Mat, cv::Mat, vector<cv::Mat>, vector<cv::Mat>> calibrateLens(vector<vector<cv::Point2f>> &pointsImage,
                                                            vector<vector<cv::Point3f>> &points3d, cv::Mat &image){

    cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
    cv::Mat distCoeffs;
    vector<cv::Mat> rvecs;
    vector<cv::Mat> tvecs;

    //intrinsic.ptr<float>(0)[0] = 1; // focal length x
    //intrinsic.ptr<float>(1)[1] = 1; // focal length y

    calibrateCamera(points3d, pointsImage, image.size(), intrinsic, distCoeffs, rvecs, tvecs);

    return make_tuple(intrinsic, distCoeffs,rvecs,tvecs);
}


cv::Mat undistortImage(cv::Mat image, cv::Mat intrinsic, cv::Mat distCoeffs){
    cv::Mat imageUndistorted;

    undistort(image, imageUndistorted, intrinsic, distCoeffs);

    //imshow("win1", image);
    //imshow("win2", imageUndistorted);
    //cv::waitKey(1);
    return imageUndistorted;
}