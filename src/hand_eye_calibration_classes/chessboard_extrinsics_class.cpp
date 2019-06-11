//
// Created by eirik on 30.05.19.
//

#include "hand_eye_calibration_classes/chessboard_extrinsics_class.h"

using namespace std;

ChessBoardExtrinsics::ChessBoardExtrinsics(std::vector<std::string> &imgList,
                    const char* intrinsicsFilePath, ChessBoard CB): imgList(imgList),
                    CB(CB),intrinsicsPath(intrinsicsFilePath){

    ChessBoardCalib CBC(imgList,intrinsicsFilePath,CB);
    readCamParams(intrinsicsFilePath);
    points3d = CBC.getValidPoints3d();
    pointsImage = CBC.getValidPointsImage();
    pnpRansac = true;

}

ChessBoardExtrinsics::ChessBoardExtrinsics(std::vector<std::string> &imgList,
        const char* intrinsicsFilePath, ChessBoard CB, std::vector<std::vector<cv::Point2f>> &pointsImage,
        std::vector<std::vector<cv::Point3f>> &points3d): imgList(imgList), CB(CB),
        intrinsicsPath(intrinsicsFilePath), pointsImage(pointsImage), points3d(points3d){

    readCamParams(intrinsicsFilePath);
    pnpRansac = true;
}

std::vector<Eigen::Matrix4d> ChessBoardExtrinsics::getChessboardPosesAsEigenMat(){
    verifyAndStorePoses();
    posesToEigenMatrix();

    return TVec;
}

std::vector<int> ChessBoardExtrinsics::getInvalids(){
    return invalids;
}

void ChessBoardExtrinsics::readColorImage(string &fileName){
    img = cv::imread(fileName, CV_LOAD_IMAGE_COLOR);   // Read the file

    if(! img.data ){                              // Check for invalid input
        cout <<  "Could not open or find the image" << endl;
    }
}

void ChessBoardExtrinsics::readCamParams(const char* &fileName){
    cv::FileStorage fs(fileName, cv::FileStorage::READ);

    cv::String date;
    fs["calibrationDate"] >> date;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();
}


void ChessBoardExtrinsics::computeObjectPosePnP(int i){
    try {
        if (pnpRansac) {
            cv::solvePnPRansac(points3d[i], pointsImage[i], cameraMatrix,
                    distCoeffs, rvec, tvec, false, CV_ITERATIVE);
        }
        else {
            cv::solvePnP(points3d[i], pointsImage[i], cameraMatrix,
                    distCoeffs, rvec, tvec, false, CV_ITERATIVE);
        }
    }
    catch(cv::Exception& e) {
        const char *err_msg = e.what();
        cout << "exception caught: " << err_msg << endl;
    }
}

void ChessBoardExtrinsics::drawVectorProjectPointsMethod(float x, float y, float z,
                            float r, float g, float b, cv::Mat &rvec, cv::Mat &tvec,
                            cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &dst) {

    std::vector<cv::Point3f> points;
    std::vector<cv::Point2f> projectedPoints;

    points.emplace_back(cv::Point3f(0, 0, 0));
    points.emplace_back(cv::Point3f(x, y, z));

    cv::projectPoints(points, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

    cv::line(dst, projectedPoints[0], projectedPoints[1],
             CV_RGB(255 * r, 255 * g, 255 * b),5);
}

void ChessBoardExtrinsics::drawAxis(cv::Mat &rvec, cv::Mat &tvec, cv::Mat &cameraMatrix,
                                    cv::Mat &distCoeffs, cv::Mat &dst) {

    drawVectorProjectPointsMethod(100, 0, 0, 1, 0, 0, rvec, tvec, cameraMatrix, distCoeffs , dst);
    drawVectorProjectPointsMethod(0, 100, 0, 0, 1, 0, rvec, tvec, cameraMatrix, distCoeffs, dst);
    drawVectorProjectPointsMethod(0, 0, 100, 0, 0, 1, rvec, tvec, cameraMatrix, distCoeffs, dst);
}

void ChessBoardExtrinsics::verifyAndStorePoses() {

    cout << "Delete image poses by pressing d, accept by pressing ENTER button." << endl;

    for (int i = 0; i < pointsImage.size(); i++) {
        computeObjectPosePnP(i); // TODO: how to iterate here?
        cv::cv2eigen(tvec, tvecEigen);
        tvecs.emplace_back(tvecEigen*0.001);
        cv::Mat r;
        cv::Rodrigues(rvec, r);
        cv::cv2eigen(r, rMat); // TODO: CHECK!!!

        rvecs.emplace_back(rMat);

        readColorImage(imgList[i]);
        drawAxis(rvec, tvec, cameraMatrix, distCoeffs, img);

        cv::namedWindow("Poses", CV_WINDOW_NORMAL);
        cv::imshow("Poses", img);
        cv::resizeWindow("Poses", 1080, 720);

        int key = cv::waitKey(0);
        switch (key) {
            case ((int) ('d')):
                invalids.emplace_back(i);
                cout << "Marked pose in: "
                     << imgList[i] << " as invalid by keypress d. " << endl;
                break;
        }
    }
}

void ChessBoardExtrinsics::posesToEigenMatrix(){
    for (int i = 0; i<rvecs.size();i++) {
        Eigen::Matrix4d tmp;
        tmp.setIdentity();
        tmp.block(0, 0, 3, 3) = rvecs[i];
        tmp.block(0, 3, 3, 1) = tvecs[i];
        TVec.push_back(tmp);
    }
}