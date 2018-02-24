#include <iostream>
#include <vector>
#include "time.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Convert.h"
#include "EKF_Attitude.h"
#include "Mahony_Attitude.h"
#include "ESKF_Attitude.h"

#include "matplotlibcpp.h"


using namespace std;
using namespace IMU;

namespace plt = matplotlibcpp;

int main(int argc, char **argv)
{
	Eigen::MatrixXd measurements;
	measurements =IMU::readFromfile("measurement2.bin");

	IMU::EKF_Attitude EKF_AHRS(true, 0.02);
    IMU::Mahony_Attitude Mahony(Eigen::Vector2d(0.1, 0.2), 0.02);

    Eigen::Matrix<double, 12, 1> ESKF_InitVec;
    ESKF_InitVec << 1e-5*Eigen::Vector3d::Ones(), 1e-9*Eigen::Vector3d::Ones(),
                    1e-3*Eigen::Vector3d::Ones(), 1e-4*Eigen::Vector3d::Ones();
    IMU::ESKF_Attitude ESKF_AHRS(ESKF_InitVec, 0.02);

	unsigned int i = 0;
	Eigen::MatrixXd Euler(measurements.rows(), 3);

    std::vector<double> Index, Roll, Pitch, Yaw;
	TicToc tc;
	do
	{
		Eigen::MatrixXd measure;
		Eigen::Quaterniond quaternion;
		
		Vector_3 Euler_single;

		//quaternion = EKF_AHRS.Run(measurements.row(i).transpose());
        //quaternion = Mahony.Run(measurements.row(i).transpose());
        quaternion = ESKF_AHRS.Run(measurements.row(i).transpose());

		Euler.row(i) = Quaternion_to_Euler(quaternion).transpose();
        std::cout << Euler.row(i)<< std::endl;

        Index.push_back(i*1.0);
        Roll.push_back(Euler.row(i)[0]);
        Pitch.push_back(Euler.row(i)[1]);
        Yaw.push_back(Euler.row(i)[2]);

		i++;
	} while (i<measurements.rows());

	cout << tc.toc() << "ms" << endl;
	writeTofile(Euler, "Euler.bin");

    plt::named_plot("cc", Index, Roll, "r");
    plt::named_plot("dd", Index, Pitch, "g");
    plt::named_plot("aa", Index, Yaw, "b");

	plt::xlim(0, 1000*20);

    plt::title("Sample figure");
    plt::legend();
    plt::show();

	return 0;
}

