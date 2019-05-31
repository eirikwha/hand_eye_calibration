//
// Created by eirik on 30.05.19.
//

#include <hand_eye_calibration/chessboard_calib_class.h>
#include <hand_eye_calibration/chessboard_calib_class.h>

using namespace std;

ChessBoardCalib::ChessBoardCalib(std::vector<std::string> &imageList, const char* calibResultFilePath, ChessBoard &CB): CB(CB) {

    imgWidth = 1920;
    imgHeight = 1200;
    imgDim = cv::Size(imgWidth, imgHeight);
    imgList = imageList;
    filePath = calibResultFilePath;

    calibrateLens();
    writeCalibrationResult();
}

void ChessBoardCalib::setImgWidth(int num){
    imgWidth = num;
}

void ChessBoardCalib::setImgHeight(int num) {
    imgHeight = num;
}

std::vector<int> ChessBoardCalib::getInvalids(){
    return invalids;
}

std::vector<std::vector<cv::Point2f>> ChessBoardCalib::getValidPointsImage(){
    return pointsImage;
}
std::vector<std::vector<cv::Point3f>> ChessBoardCalib::getValidPoints3d(){
    return points3d;
}

ChessBoardCalibResult ChessBoardCalib::getCalibrationResult() {
    return calibResult;
}

cv::Mat ChessBoardCalib::getColorImage(std::string &fileName) {
    readColorImage(fileName);
    return img;
}

cv::Mat ChessBoardCalib::undistortImage(cv::Mat &img){
    cv::Mat imageUndistorted;
    cv::undistort(img, imageUndistorted, intrinsics, distCoeffs);
    return imageUndistorted;
}


void ChessBoardCalib::readColorImage(string &fileName){
    img = cv::imread(fileName, CV_LOAD_IMAGE_COLOR);

    if(! img.data ){
        cout <<  "Could not open or find the image" << endl;
    }
}

bool ChessBoardCalib::findCornersImg() {

    if (img.channels() == 3) {
        cv::cvtColor(img, grayImg, CV_BGR2GRAY);
    }

    bool found = cv::findChessboardCorners(img, CB.getBoardSize(),
                                           cornersImg, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    if(found) {
        cornerSubPix(grayImg, cornersImg, cv::Size(11, 11), cv::Size(-1, -1),
                     cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        return true;
    }
    else {
        return false;
    }
}

void ChessBoardCalib::detectValidCorners() { // TODO: Probably makes the code pretty slow
    for (int i = 0; i < imgList.size(); i++) {
        readColorImage(imgList[i]);

        if (findCornersImg()) {
            pointsImage.push_back(cornersImg);
            points3d.push_back(CB.corners3d);
        } else {
            invalids.push_back(i);
        }
    }
}

void ChessBoardCalib::calibrateLens(){

    detectValidCorners();
    cv::calibrateCamera(points3d, pointsImage, imgDim, intrinsics, distCoeffs, rvecs, tvecs);
    calibResult = {intrinsics, distCoeffs, rvecs, tvecs};
}

void ChessBoardCalib::writeCalibrationResult(){

    cv::FileStorage fs(filePath, cv::FileStorage::WRITE);
    time_t rawtime; time(&rawtime);
    fs << "calibrationDate" << asctime(localtime(&rawtime));
    fs << "cameraMatrix" << calibResult.cameraMatrix << "distCoeffs" << calibResult.distCoeffs;
    fs.release();
}