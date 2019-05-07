//
// Created by eirik on 30.03.19.
//

#include "hand_eye_calibration/parkmartin.h"
#include <Eigen/Eigen>
#include <iostream>
#include <vector>

#define ESTIMATION_DEBUG 0

//TODO: Separate so that it is possible to take in vector<Matrix4f> for A and B into a function, this way its easier
// to benchmark

using namespace std;
using namespace Eigen;

// AX = XB

    Vector3d logTheta(Matrix3d R) {
        //Assuming that R is never an Identity
        double theta = acos((R.trace() - 1) / 2);
        double logT = theta / (2*sin(theta));
        return Vector3d((R(2, 1)- R(1,2))*logT, (R(0, 2) - R(2,0))*logT, (R(1, 0)-R(0,1))*logT);
    }

    Matrix3d invsqrt(Matrix3d M){

        Eigen::JacobiSVD<Matrix3d> svd(M,ComputeFullU | ComputeFullV);
        Eigen::Vector3d S_v;
        S_v << 1,1,(svd.matrixV() * svd.matrixU().transpose()).determinant();
        Eigen::Matrix3d S = S_v.matrix().asDiagonal();

#if ESTIMATION_DEBUG
        cout << "U: " << svd.matrixU() << endl << endl;
        cout << "V: " << svd.matrixV() << endl << endl;
        cout << "S: " << S << endl << endl;
#endif
        if (S(2,2)>= 0.99 && S(2,2) <= 1.01){
            return (svd.matrixV() * S * svd.matrixU().transpose());
        }

        else {
            cout << "S should have 1,1,1 on the diagonal. The computed S is: " << endl << S << endl << endl;
            cout << "The matrix is passed, but it is possible that R isnt a rotation matrix." << endl << endl;
            return (svd.matrixV() * S * svd.matrixU().transpose());
        }
    }

    Matrix3d getR(Matrix3d M) {

        EigenSolver<Matrix3d> es(M.transpose() * M);
        Matrix3cd D = es.eigenvalues().asDiagonal();
        Matrix3cd V = es.eigenvectors();

        Matrix3cd Lambda = D.inverse().array().sqrt();

#if ESTIMATION_DEBUG
        cout << "D: " << D << endl << endl;
        cout << "V: " << V << endl << endl;
        cout << "Lambda: " << Lambda << endl << endl;
#endif
        return (V * Lambda * V.inverse() * M.transpose()).real();
    }

    Matrix4d performEstimation(vector<Eigen::Matrix4d> tRB_vec, vector<Eigen::Matrix4d> tCB_vec) {

        Matrix3d M;
        Matrix4d rbi, rbj, cbi, cbj; // transformation matrices, useless conversion ??
        vector<Matrix4d> A, B;
        Matrix4d X;
        MatrixXd C(0, 3), d(0, 1);
        Vector3d ai, bi; // for storing logtheta, the Rotation matrix logarithm

        M.setZero();

        for (int i = 0; i < tRB_vec.size(); i++) {
            for (int j = 0; j < tRB_vec.size(); j++) {
                if (i != j) { // create pairs??
                    rbi = tRB_vec[i];
                    rbj = tRB_vec[j];
                    A.push_back(rbj.inverse() * rbi);

                    cbi = tCB_vec[i];
                    cbj = tCB_vec[j];
                    B.push_back(cbj * cbi.inverse());

                    if (A.size() == 10){
                        break;
                    }

#if ESTIMATION_DEBUG
                    cout << "A: " << A[i] << endl << endl;
                    cout << "B: " << B[i] << endl << endl;
#endif
                }
            }
        }

#if ESTIMATION_DEBUG
        cout << "Size of A: " << A.size() << endl;
#endif

        for (int i = 0; i < A.size(); i++) {
            ai = logTheta(A[i].block(0, 0, 3, 3)); // block of size (p,q) starting at (i,j) = matrix.block(i,j,p,q)
            bi = logTheta(B[i].block(0, 0, 3, 3));

            M += ai * bi.transpose(); // multiplying the rotation of the pose pairs from camera and robot
        }

#if ESTIMATION_DEBUG
        cout << "M: " << M << endl << endl;
#endif
        Matrix3d Rx = getR(M);//(invsqrt(M * M.transpose())) * M.transpose();
        cout << "\nOrientation of Robot tool-tip frame with respect to end-effector frame." << endl;
        cout << "Rx: " << Rx << endl << endl;

        for (int i = 0; i < A.size(); i++) {

            MatrixXd C_tmp = C;
            C.resize(C.rows() + 3, NoChange);
            C << C_tmp, A[i].block(0, 0, 3, 3) - Matrix3d::Identity();

            VectorXd d_tmp = d;
            d.resize(d.rows() + 3, NoChange);
            d << d_tmp, (Rx * B[i].block(0, 3, 3, 1) - A[i].block(0, 3, 3, 1));
        }

#if ESTIMATION_DEBUG
        cout << "C: " << C << endl << endl;
        cout << "d: " << d << endl << endl;
#endif
        Vector3d tx = ((C.transpose() * C).inverse()) * (C.transpose() * d);

        cout << "\nTranslation of Robot tool-tip frame with respect to end-effector frame." << endl;
        cout << "tx: " << tx << endl << endl;

        X << Rx, tx, 0,0,0,1;

        return X;
    }