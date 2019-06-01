//
// Created by eirik on 28.05.19.
//

#include <hand_eye_calibration_classes/hand_eye_calib_class.h>

#define ESTIMATION_DEBUG 0

using namespace std;
using namespace Eigen;

HandEyeCalib::HandEyeCalib(std::vector<Eigen::Matrix4d> tRB_vec, std::vector<Eigen::Matrix4d> tCB_vec) {
    createPosePairs(tRB_vec, tCB_vec);
    X = performEstimation();
}

HandEyeCalib::~HandEyeCalib() = default;

Matrix4d HandEyeCalib::getX(){
    return X;
}

PosePair HandEyeCalib::getPosePairs(){
    return AB;
}

Vector3d HandEyeCalib::logTheta(Matrix3d R) {

    //Assuming that R is never an Identity
    double theta = acos((R.trace() - 1) / 2);
    double logT = theta / (2*sin(theta));
    return {(R(2, 1)- R(1,2))*logT, (R(0, 2) - R(2,0))*logT, (R(1, 0)-R(0,1))*logT};
}

Matrix3d HandEyeCalib::invsqrt(Matrix3d M){

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
        // TODO: CHANGE THIS, THIS IS THE UAYAMA CORRECTION
        cout << "S should have 1,1,1 on the diagonal. The computed S is: " << endl << S << endl << endl;
        cout << "The matrix is passed, but it is possible that R isnt a rotation matrix." << endl << endl;
        return (svd.matrixV() * S * svd.matrixU().transpose());
    }
}

void HandEyeCalib::createPosePairs(vector<Eigen::Matrix4d> tRB_vec, vector<Eigen::Matrix4d> tCB_vec){

    for (int i = 0; i < tRB_vec.size(); i++) {
        for (int j = 0; j < tRB_vec.size(); j++) {
            if (i != j) { // create pairs??
                AB.A.emplace_back(tRB_vec[j].inverse() * tRB_vec[i]);
                AB.B.emplace_back(tCB_vec[j] * tCB_vec[i].inverse());
            }
        }
    }
}

Matrix4d HandEyeCalib::performEstimation() {

    Matrix3d M;
    Matrix4d X;
    MatrixXd C(0, 3), d(0, 1);
    Vector3d ai, bi; // for storing logtheta, the Rotation matrix logarithm
    M.setZero();

#if ESTIMATION_DEBUG
    cout << "Size of A: " << A.size() << endl;
#endif

    for (int i = 0; i < AB.A.size(); i++) {
        ai = logTheta(AB.A[i].block(0, 0, 3, 3)); // block of size (p,q) starting at (i,j) = matrix.block(i,j,p,q)
        bi = logTheta(AB.B[i].block(0, 0, 3, 3));
        M += ai * bi.transpose(); // multiplying the rotation of the pose pairs from camera and robot
    }

#if ESTIMATION_DEBUG
    cout << "M: " << M << endl << endl;
#endif

    Matrix3d Rx = invsqrt(M.transpose());
    cout << "\nOrientation of Robot tool-tip frame with respect to end-effector frame." << endl;
    cout << "Rx: " << Rx << endl << endl;

    for (int i = 0; i < AB.A.size(); i++) {

        MatrixXd C_tmp = C;
        C.resize(C.rows() + 3, NoChange);
        C << C_tmp, AB.A[i].block(0, 0, 3, 3) - Matrix3d::Identity();

        VectorXd d_tmp = d;
        d.resize(d.rows() + 3, NoChange);
        d << d_tmp, (Rx * AB.B[i].block(0, 3, 3, 1) - AB.A[i].block(0, 3, 3, 1));
    }

#if ESTIMATION_DEBUG
    cout << "C: " << C << endl << endl;
    cout << "d: " << d << endl << endl;
#endif

    Vector3d tx = ((C.transpose() * C).inverse()) * (C.transpose() * d);
    cout << "\nTranslation of Robot tool-tip frame with respect to end-effector frame." << endl; // TODO: WHAT IS THIS???
    cout << "tx: " << tx << endl << endl;

    X << Rx, tx, 0,0,0,1;
    return X;
}
