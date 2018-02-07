#ifndef MAHONY_AHRS_H_
#define MAHONY_AHRS_H_

#include <iostream>
#include <vector>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "TypeDefs.h"

#ifdef _WIN32
#include "windows.h"
#else 
#include <unistd.h>  
#endif

using namespace std;
namespace IMU
{

class Mahony_Attitude
{
public:
	Mahony_Attitude(Vector_2 PI, double dt);
	void Params_Change(Vector_2 PI, double dt);
	void Mahony_Estimate();
	void Read_SensorData(Vector_9 measurement);

	Eigen::Quaterniond Run(Vector_9 measurement);
	void Release();
	void RequestStop();
	void RequestStart();
	bool Stop();
	bool isStopped();

private:
	Vector_9 cur_measurement;

	double Kp, Ki;
	double deltaT;
	Vector_3 Integ_angular;

	vector< Eigen::Quaterniond > quaternion;

	bool mbStopped;
	bool mbStopRequested;
	std::mutex mMutexStop;
};

} //namesapce IMU


#endif