#ifndef CONVERT_H_
#define CONVERT_H_

#include <iostream>
#include <vector>
#include <fstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "TypeDefs.h"
#include "tic_toc.h"

using namespace std;
namespace IMU
{
void Vect_to_SkewMat(Vector_3 Vector, Matrix_3 &Matrix);
void Angular_to_Mat(Vector_3 Vector, Eigen::Matrix<double, 4, 4> &Matrix);
//void Rotation_to_Qaut(Matrix_3 rotation, Eigen::Quaterniond &quat);
Vector_4 Quaternion_to_Vect(Eigen::Quaterniond q);
Eigen::Quaterniond QuatMult(Eigen::Quaterniond q1, Eigen::Quaterniond q2);
Matrix_3 Quat_to_Matrix(Eigen::Quaterniond q);

// Eigen IO
Eigen::MatrixXd readFromfile(const string file);
bool writeTofile(Eigen::MatrixXd matrix, const string file);

// Eluer(Rotate vector) to rotation matrix
Matrix_3 Euler_to_RoatMat(Vector_3 Euler);

// Eluer(Rotate vector) to rotation matrix
Eigen::Quaterniond Euler_to_Quaternion(Vector_3 Euler);

// Quaternion to eulers
Vector_3 Quaternion_to_Euler(Eigen::Quaterniond q);

// Rotation matrix to eulers, the rotation order is X-Y-Z(roll,pitch and yaw)
Vector_3 Rotation_to_Euler(Matrix_3 Rotation);
// Rotation matrix to quaternion http://www.cs.ucr.edu/~vbz/resources/quatut.pdf
Eigen::Quaterniond Rotation_to_Quater(Matrix_3 Rotation);

// Indirect Kalman Filter for 3D Attitude Estimation (Roumeliotis)
Eigen::Quaterniond BuildUpdateQuat(Eigen::Vector3d DeltaTheta);


} //namesapce IMU

#endif