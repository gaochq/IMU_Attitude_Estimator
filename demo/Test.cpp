#include <iostream>
#include <vector>
#include "time.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "EKF_Attitude.h"
#include "Convert.h"
#include "matplotlibcpp.h"


using namespace std;
using namespace IMU;

namespace plt = matplotlibcpp;

int main(int argc, char **argv)
{
	Eigen::MatrixXd measurements;
	measurements =IMU::readFromfile("measurement2.bin");

	IMU::EKF_Attitude EKF_AHRS(true, 0.02);
	unsigned int i = 0;
	Eigen::MatrixXd Euler(measurements.rows(), 3);

    std::vector<double> Index, Roll, Pitch, Yaw;
	TicToc tc;
	do
	{

		Eigen::MatrixXd measure;
		Eigen::Quaterniond quaternion;
		
		Vector_3 Euler_single;

		quaternion = EKF_AHRS.Run(measurements.row(i).transpose());

		Euler.row(i) = Quaternion_to_Euler(quaternion).transpose();

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

