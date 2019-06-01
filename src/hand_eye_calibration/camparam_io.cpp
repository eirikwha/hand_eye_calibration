//
// Created by eirik on 29.03.19.
//

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <time.h>
#include "hand_eye_calibration/camparam_io.h"
#include "dirent.h"

using namespace cv;
using namespace std;

namespace CamParamIO {
    void readCamParams(const char *filePath, cv::Mat &cameraMatrix, cv::Mat &distCoeffs) {//, cv::FileNode features) {

        FileStorage fs(filePath, FileStorage::READ);

        String date;

        fs["calibrationDate"] >> date;
        fs["cameraMatrix"] >> cameraMatrix;
        fs["distCoeffs"] >> distCoeffs;
        cout << "Loaded calibration data" << endl << "------------------------" << endl
             << "calibration date: " << date
             << "camera matrix: \n" << cameraMatrix << endl
             << "distortion coeffs: \n" << distCoeffs << endl
             << "------------------------" << endl;

        fs.release();
    }

    void writeCamParams(cv::Mat cameraMatrix, cv::Mat distCoeffs, const char *filePath) {
        FileStorage fs(filePath, FileStorage::WRITE);
        time_t rawtime;
        time(&rawtime);
        fs << "calibrationDate" << asctime(localtime(&rawtime));
        fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;

        fs.release();
    }

    int listFiles(const char *directoryPath, const char *fileType, vector<string> &list) {

        DIR *d;
        struct dirent *dir;
        d = opendir(directoryPath);
        int ret = 0;
        int n = 0;
        char *tok1, *tok2;

        if (d) {
            while ((dir = readdir(d)) != nullptr) {
                tok1 = strtok(dir->d_name, ".");
                tok2 = strtok(nullptr, ".");
                if (strcmp("jpg", fileType) == 0 || strcmp("jpeg", fileType) == 0 || strcmp("png", fileType) == 0) {
                    if (tok1 != nullptr) {
                        ret = strcmp(tok2, fileType);
                        if (ret == 0) {
                            string name = string(directoryPath) + dir->d_name + string(".") + fileType;
                            list.emplace_back(name);
                            ++n;
                        }
                    }
                } else {
                    cout << "Wrong filetype! Try jpg, jpeg or png." << endl;
                    break;
                }
            }
            closedir(d);
        }
        std::sort(list.begin(), list.end());
        return 0;
    }

    void writeImageList(const char *filePath, const char *outputName, const char *fileType) {
        vector<string> list;
        listFiles(filePath, fileType, list);
        string path = filePath + string("/") + outputName;
        FileStorage fs(path, FileStorage::WRITE);
        fs << "images" << "[";
        for (int i = 0; i < list.size(); i++) {
            fs << list[i];
        }
        fs << "]";
    }

    void readImageList(const char *filePath, vector<string> &images) {
        FileStorage fs2(filePath, FileStorage::READ);
        fs2["images"] >> images;
        cout << "Image list:" << endl << endl;
        for (int i = 0; i < images.size(); i++) {
            cout << images[i] << endl;
        }

        fs2.release();
    }
}