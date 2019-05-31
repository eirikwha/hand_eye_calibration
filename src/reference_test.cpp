//
// Created by eirik on 18.05.19.
//

#include <hand_eye_calibration/park_martin_class.h>
#include <Eigen/Eigen>
#include <iostream>
#include <vector>


// VALIDATION OF THE ALGORITHM IN parkmartin.cpp, by running the example from the reference paper
using namespace std;
using namespace Eigen;

#define ESTIMATION_DEBUG 1

int main() {

    Matrix3d M;
    vector <Matrix4d> A, B;
    Matrix4d A1, A2, B1, B2, X;
    MatrixXd C(0, 3), d(0, 1);
    Vector3d ai, bi; // for storing logtheta, the Rotation matrix logarithm

    A1 << -0.989992, -0.141120, 0.000000, 0,
            0.141120, -0.989992, 0.000000, 0,
            0.000000, 0.000000, 1.000000, 0,
            0, 0, 0, 1;

    B1 << -0.989992, -0.138307, 0.028036, -26.9559,
            0.138307, -0.911449, 0.387470, -96.1332,
            -0.028036, 0.387470, 0.921456, 19.4872,
            0, 0, 0, 1;

    A2 << 0.070737, 0.000000, 0.997495, -400.000,
            0.000000, 1.000000, 0.000000, 0.000000,
            -0.997495, 0.000000, 0.070737, 400.000,
            0, 0, 0, 1;

    B2 << 0.070737, 0.198172, 0.977612, -309.543,
            -0.198172, 0.963323, -0.180936, 59.0244,
            -0.977612, -0.180936, 0.107415, 291.177,
            0, 0, 0, 1;

#if ESTIMATION_DEBUG
    cout << "A1:\n" << A1 << endl << endl;
#endif

    A.push_back(A1);
    A.push_back(A2);

    B.push_back(B1);
    B.push_back(B2);

    PosePair AB;
    AB.A = A;
    AB.B = B;

    X = ParkMartin(AB).getX();

    cout << "X:\n" << X << endl;

    return 0;
}