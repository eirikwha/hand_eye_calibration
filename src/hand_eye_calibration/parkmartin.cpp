//
// Created by eirik on 30.03.19.
//

#include "hand_eye_calibration/parkmartin.h"
#include "hand_eye_calibration/pose_io.h"
#include "hand_eye_calibration/chessboard.h"
#include <Eigen/Eigen>
#include <iostream>
#include <vector>

#define ESTIMATION_DEBUG 0

using namespace std;
using namespace Eigen;

    Vector3f getLogTheta(Matrix3f R) {

        //Assuming that R is never an Identity
        float theta = acos((R.trace() - 1) / 2);
        Matrix3f logTheta = 0.5 * theta / sin(theta) * (R - R.transpose());
        return Vector3f(logTheta(2, 1), logTheta(0, 2), logTheta(1, 0));

    }

    Matrix3f invsqrt(Matrix3f M){

        Eigen::JacobiSVD<Matrix3f> svd(M,ComputeFullU | ComputeFullV);
        Eigen::Matrix3f S_d = (1/(svd.singularValues().array().sqrt())).matrix().asDiagonal();

#if ESTIMATION_DEBUG
        cout << "S: " << svd.singularValues() << endl << endl;
        cout << "U: " << svd.matrixU() << endl << endl;
        cout << "V: " << svd.matrixV() << endl << endl;
        cout << "S_d: " << S_d << endl << endl;
#endif

        return (svd.matrixU() * S_d.transpose()) * svd.matrixV().transpose();
    }


    void performEstimation(vector<Eigen::Matrix4f> tRB_vec, vector<Eigen::Matrix4f> tCB_vec) {
        if (tRB_vec.size() < 3) {
            std::cout << "Insufficient data" << std::endl;
            return;
        }

        if (tRB_vec.size() > 3 && tRB_vec.size() < 10) {
            std::cout << "At least 10 pose pairs are recommended. See the original paper for further explaination"
                      << std::endl;
            return;
        }

        Matrix3f M;
        Matrix4f rbi, rbj, cbi, cbj; // transformation matrices, useless conversion ??
        Matrix4f A, B;
        MatrixXf C(0, 3), d(0, 1);
        Vector3f ai, bi; // for storing logtheta, the Rotation matrix logarithm

        M.setZero();

        for (int i = 0; i < (int) tRB_vec.size(); i++) {
            for (int j = 0; j < (int) tRB_vec.size(); j++) {
                if (i != j) { // create pairs??
                    rbi = tRB_vec[i];
                    rbj = tRB_vec[j];
                    A = rbj.inverse() * rbi;

                    cbi = tCB_vec[i];
                    cbj = tCB_vec[j];
                    B = cbj * cbi.inverse();

#if ESTIMATION_DEBUG
                    cout << "A: " << A << endl << endl;
                    cout << "B: " << B << endl << endl;
#endif

                }
            }
        }
        cout << "Size of A: " << A.size() << endl;

        for (int i = 0; i < A.size(); i++) {
            ai = getLogTheta(A.block(0, 0, 3, 3)); // block of size (p,q) starting at (i,j) = matrix.block(i,j,p,q)
            bi = getLogTheta(B.block(0, 0, 3, 3));

            M += bi * ai.transpose(); // multiplying the rotation of the pose pairs from camera and robot
            // THIS IS THE ROTATION PART OF THE CALIB (save the A and B here)

            //cout << "ai: " << ai << endl << endl;
            //cout << "bi: " << bi << endl << endl;

            //cout << M << endl << endl;
        }

#if ESTIMATION_DEBUG
        cout << "M: " << M << endl << endl;
#endif

        Matrix3f Rx = invsqrt(M.transpose() * M) * M.transpose();
        cout << "\nOrientation of Robot tool-tip frame with respect to end-effector frame." << endl;
        cout << "Rx: " << Rx << endl << endl;

        for (int i = 0; i < A.size(); i++) {

            MatrixXf C_tmp = C;
            C.resize(C.rows() + 3, NoChange);
            C << C_tmp, Matrix3f::Identity() - A.block(0, 0, 3, 3);

            VectorXf d_tmp = d;
            d.resize(d.rows() + 3, NoChange);
            d << d_tmp, A.block(0, 3, 3, 1) - (Rx * B.block(0, 3, 3, 1));
        }

#if ESTIMATION_DEBUG
        cout << "C: " << C << endl << endl;
        cout << "d: " << d << endl << endl;
#endif
        Vector3f tx = ((C.transpose() * C).inverse()) * (C.transpose() * d);

        cout << "\nTranslation of Robot tool-tip frame with respect to end-effector frame." << endl;
        cout << "tx: " << tx << endl << endl;
    }