//
// Created by eirik on 26.03.19.
//
#include "hand_eye_calibration/chessboard.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>



using namespace std;

int main() {
    int numImages = 20; // set this value for desired number of input images (better to parse or pass as argument in main)
    int numCornersHor = 9;
    int numCornersVer = 6;

    /*
    printf("Enter number of corners along width: ");
    scanf("%d", &numCornersHor);

    printf("Enter number of corners along height: ");
    scanf("%d", &numCornersVer);

    printf("Enter number of boards: ");
    scanf("%d", &numBoards);
     */



    int numSquares = numCornersHor * numCornersVer;
    cv::Size board_sz = cv::Size(numCornersHor, numCornersVer);

    vector<vector<cv::Point3f>> points_3d;
    vector<vector<cv::Point2f>> points_image;

    vector<cv::Point2f> corners;
    int successes = 0;

    cv::Mat image;
    cv::Mat gray_image;


    image = cv::imread("/home/eirik/catkin_ws/src/hand_eye_calibration/data/calib-checkerboard-320px.png",
                       CV_LOAD_IMAGE_COLOR);   // Read the file

    if (!image.data)                              // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    cv::namedWindow("Display window", CV_WINDOW_AUTOSIZE);// Create a window for display.
    cv::imshow("Display window", image);                   // Show our image inside it.

    cv::waitKey(0);                                          // Wait for a keystroke in the window

    vector<cv::Point3f> obj;
    for (int j = 0; j < numSquares; j++) {
        obj.push_back(cv::Point3f(j / numCornersHor, j % numCornersHor, 0.0f));
    }

    cv::cvtColor(image, gray_image, CV_BGR2GRAY);

    bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    if(found)
    {
        cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        drawChessboardCorners(image, board_sz, corners, found);
    }
    else{
        printf("No corners found");
    }

    cv::imshow("Display window", image);
    cv::waitKey(0);

    if(found!=0)
    {
        cout << "Corners found. Press y to store the result, or any other key to skip: ";
        string n;
        cin >> n;
        if (n == "y" || n=="yes"){
            points_image.push_back(corners);
            points_3d.push_back(obj);
            printf("Snap stored!");
        }
        else {
            printf("Snap not stored!");
            return 0;
        }
    }
/*
    cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
    cv::Mat distCoeffs;
    vector<cv::Mat> rvecs;
    vector<cv::Mat> tvecs;

    calibrateCamera(points_3d, points_image, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
*/


    // }
    return 0;
}
