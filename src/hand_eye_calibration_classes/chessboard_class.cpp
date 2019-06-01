//
// Created by eirik on 28.05.19.
//

#include <hand_eye_calibration_classes/chessboard_class.h>

using namespace std;

ChessBoard::ChessBoard(){

    //img = readColorImage(fileName);
    numCornersHor = 9;
    numCornersVer = 7;
    squareSize = 10; // TODO: Is this mm???
    boardSize = cv::Size(numCornersHor, numCornersVer);
/*
    imgWidth = 1920;
    imgHeight = 1200;
    imgDim = cv::Size(imgWidth, imgHeight); // TODO: Private /protected variable?
*/
    //std::vector<cv::Point2f> cornersImg;
    //std::vector<cv::Point3f> corners3d;
    genCorners3d();

}

ChessBoard::ChessBoard(int &numCornersHor, int &numCornersVer, float &squareSz){
    boardSize = cv::Size(numCornersHor, numCornersVer);
    squareSize = squareSz;

    //std::vector<cv::Point3f> corners3d;
    genCorners3d();
}
/*
ChessBoard::ChessBoard(string fileName) {
    img = readColorImage(fileName);
}*/
/*
cv::Mat ChessBoard::readColorImage(string filename){ // TODO: Maybe void here as well? Or bool?
    img = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
    if(! img.data ){
        cout <<  "Could not open or find the image" << std::endl;
    }
    return img;
}
*/
void ChessBoard::genCorners3d() {
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            corners3d.emplace_back(cv::Point3f((float) j * squareSize, (float) i * squareSize,0.0f));
        }
    } // TODO: Maybe not possible without return, might have to use void.
}

/*
bool ChessBoard::findCornersImg() { // TODO: Maybe const wont work

    if (img.channels() == 3) {
        cv::cvtColor(img, grayImg, CV_BGR2GRAY);
    }

    bool found = cv::findChessboardCorners(img, boardSize,
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
*/

void ChessBoard::setNumCornersHor(int num){
    numCornersHor = num;
}

void ChessBoard::setNumCornersVer(int num){
    numCornersVer = num;
}

void ChessBoard::setSquareSize(int num){
    squareSize = num;
}

cv::Size ChessBoard::getBoardSize(){
    return boardSize;
}

float ChessBoard::getSquareSize(){
    return squareSize;
}
/*
void ChessBoard::setImgWidth(int num){
    imgWidth = num;
}

void ChessBoard::setImgHeight(int num){
    imgHeight = num;
}
*/
/*
ChessBoard::ChessBoard(vector<vector<cv::Point2f>> pointsImage, vector<vector<cv::Point3f>> points3d){
    calibrationResult = calibrateLens(pointsImage, points3d, img_dim);
}

std::tuple<cv::Mat, cv::Mat, std::vector<cv::Mat>, std::vector<cv::Mat>> ChessBoard::getCalibrationResult(){
    return calibrationResult;
}

const vector<cv::Point3f> ChessBoard::genPatternPoints() {
    vector<cv::Point3f> obj;
    for (int i = 0; i < board_sz.height; i++) {
        for (int j = 0; j < board_sz.width; j++) {
            obj.emplace_back(cv::Point3f((float) j * square_size, (float) i * square_size,0.0f));
        }
    }

    return obj;
}

cv::Mat ChessBoard::readColorImage(string filename){
    cv::Mat image;
    image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);   // Read the file

    if(! image.data ){                              // Check for invalid input
        cout <<  "Could not open or find the image" << std::endl ;
    }
    return image;
}

tuple<vector<cv::Point2f>,bool> ChessBoard::findCorners(cv::Mat image) {

    cv::Mat gray_image;
    vector<cv::Point2f> corners;

    if (image.channels() == 3) {
        cv::cvtColor(image, gray_image, CV_BGR2GRAY);
    }

    bool found = cv::findChessboardCorners(image, board_sz,
            corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    if(found)
    {
        cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

        return make_tuple(corners, found);
    }
    else{
        return make_tuple(corners,found);
    }
}

void ChessBoard::drawCorners(cv::Mat image, vector<cv::Point2f> corners) {

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

void ChessBoard::saveCorners(vector<vector<cv::Point2f>> &pointsImage, vector<vector<cv::Point3f>> &points3d,
                 vector<cv::Point3f> obj, vector<cv::Point2f> corners) {

    pointsImage.push_back(corners);
    points3d.push_back(obj);
}


tuple<cv::Mat, cv::Mat, vector<cv::Mat>, vector<cv::Mat>> ChessBoard::calibrateLens(vector<vector<cv::Point2f>> &pointsImage,
                                                                        vector<vector<cv::Point3f>> &points3d, cv::Size &img_dim){

    cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
    cv::Mat distCoeffs;
    vector<cv::Mat> rvecs;
    vector<cv::Mat> tvecs;

    calibrateCamera(points3d, pointsImage, img_dim, intrinsic, distCoeffs, rvecs, tvecs);

    return make_tuple(intrinsic, distCoeffs,rvecs,tvecs);
}

tuple<cv::Mat, cv::Mat> ChessBoard::getObjectPosePnP(vector<cv::Point2f> &pointsImage, vector<cv::Point3f> &points3d,
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

cv::Mat ChessBoard::undistortImage(cv::Mat image, cv::Mat intrinsic, cv::Mat distCoeffs){
    cv::Mat imageUndistorted;
    undistort(image, imageUndistorted, intrinsic, distCoeffs);
    return imageUndistorted;
}

void ChessBoard::drawVector_withProjectPointsMethod(float x, float y, float z, float r, float g, float b, cv::Mat &rvec, cv::Mat &tvec, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &dst) {
    std::vector<cv::Point3f> points;
    std::vector<cv::Point2f> projectedPoints;

    //fills input array with 2 points
    points.emplace_back(cv::Point3f(0, 0, 0));
    points.emplace_back(cv::Point3f(x, y, z));


    //projects points using cv::projectPoints method
    cv::projectPoints(points, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

    //draws corresponding line
    cv::line(dst, projectedPoints[0], projectedPoints[1],
             CV_RGB(255 * r, 255 * g, 255 * b),5);
}

void ChessBoard::drawAxis(cv::Mat &rvec, cv::Mat &tvec, cv::Mat &cameraMatrix, cv::Mat &distCoeffs, cv::Mat &dst) {

    drawVector_withProjectPointsMethod(100, 0, 0, 1, 0, 0, rvec, tvec, cameraMatrix, distCoeffs , dst);
    drawVector_withProjectPointsMethod(0, 100, 0, 0, 1, 0, rvec, tvec, cameraMatrix, distCoeffs, dst);
    drawVector_withProjectPointsMethod(0, 0, 100, 0, 0, 1, rvec, tvec, cameraMatrix, distCoeffs, dst);
}
 */