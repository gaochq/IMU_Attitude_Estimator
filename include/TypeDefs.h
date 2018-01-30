#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_


#include <iostream>
#include <Eigen/Core>

//#pragma once

namespace IMU
{

typedef Eigen::Matrix<double, 2, 1> Vector_2;
typedef Eigen::Matrix<double, 3, 1> Vector_3;
typedef Eigen::Matrix<double, 4, 1> Vector_4;
typedef Eigen::Matrix<double, 6, 1> Vector_6;
typedef Eigen::Matrix<double, 7, 1> Vector_7;
typedef Eigen::Matrix<double, 9, 1> Vector_9;
typedef Eigen::Matrix<double, 12, 1> Vector_12;

typedef Eigen::Matrix3d Matrix_3;
const Matrix_3 Zeros_Matrix3 = Matrix_3::Zero(3, 3);
const Matrix_3 Ones_Matrix3 = Matrix_3::Ones(3, 3);
const Matrix_3 Identity_Matrix3 = Matrix_3::Identity(3, 3);

} // namespace IMU

#endif 