#include <iostream>
#include "Mahony_Attitude.h"
#include "Convert.h"

using namespace std;
namespace IMU
{
	Mahony_Attitude::Mahony_Attitude(Vector_2 PI, double dt):
			mbStopped(false), mbStopRequested(false)
	{
		Kp = PI(0);
		Ki = PI(1);
		deltaT = dt;
		Integ_angular = Vector_3::Zero();
	}

	void Mahony_Attitude::Params_Change(Vector_2 PI, double dt)
	{
		if (isStopped())
		{
			Kp = PI(0);
			Ki = PI(1);
			deltaT = dt;
		}
	}

	void Mahony_Attitude::Mahony_Estimate()
	{
		Vector_3 angular_vel, acc, mag;
		angular_vel = cur_measurement.block<3, 1>(0, 0);
		acc = cur_measurement.block<3, 1>(3, 0);
		mag = cur_measurement.block<3, 1>(6, 0);

		Eigen::Quaterniond quat_temp;
		Vector_3 x_state, y_state, z_state;
		Matrix_3 Rotation_matrix;
		
		// if the quaternion is empty, the quaternion will be initialized by tha accelerometer and magnetometer
		if (quaternion.empty())
		{
			z_state = acc;
			y_state = z_state.cross(mag);
			x_state = y_state.cross(z_state);
			
			Rotation_matrix << x_state, y_state, z_state;
			quat_temp = Rotation_matrix;
			quaternion.push_back(quat_temp);
		}
		else
		{
			// get the error from the measurement from accelerometer and magnetometer
			quat_temp = quaternion.back();
			quat_temp.normalized();
			Rotation_matrix = quat_temp.toRotationMatrix();

			Vector_3 error_acc, error_mag, mag_ned, error_sum;
			error_acc = Rotation_matrix.block<3, 1>(0, 2);
			error_acc = acc.cross(error_acc);

			mag_ned = Rotation_matrix.transpose()*mag;
			error_mag << sqrt(mag_ned(0)*mag_ned(0) + mag_ned(1)*mag_ned(1)), 0, mag_ned(2);
			error_mag = Rotation_matrix*error_mag;
			error_mag = mag.cross(error_mag);
			error_sum = error_acc + error_mag;

			// get the modify value from the error
			Vector_3 Integ_angular, Correct_angular;
			Integ_angular = Integ_angular + Ki*deltaT*error_sum;
			Correct_angular = angular_vel + Kp*error_sum + Integ_angular;

			Eigen::Quaterniond Angular_quat;
			Angular_quat.w() = 0.0;
			Angular_quat.vec() = Correct_angular;

			// update the quaternion refer to the correct angular
			Eigen::Quaterniond quat_temp_new;
			quat_temp_new = QuatMult(quat_temp, Angular_quat);
			quat_temp_new.w() = quat_temp_new.w()*0.5*deltaT + quat_temp.w();
			quat_temp_new.vec() = quat_temp_new.vec()*0.5*deltaT + quat_temp.vec();
			quat_temp_new.normalize();
			quaternion.push_back(quat_temp_new);
		}
	}

	void Mahony_Attitude::Read_SensorData(Vector_9 measurement)
	{
		Vector_3 gyro_mea, acc_mea, mag_mea;
		gyro_mea = measurement.block<3, 1>(3, 0);
		acc_mea = measurement.block<3, 1>(0, 0);
		mag_mea = measurement.block<3, 1>(6, 0);

		acc_mea.normalize();
		mag_mea.normalize();

		cur_measurement.block<3, 1>(0, 0) = gyro_mea;
		cur_measurement.block<3, 1>(3, 0) = acc_mea;
		cur_measurement.block<3, 1>(6, 0) = mag_mea;
	}

	Eigen::Quaterniond Mahony_Attitude::Run(Vector_9 measurement)
	{
		while (true)
		{
            Read_SensorData(measurement);
			Mahony_Estimate();

			if (Stop())
			{
				while (isStopped())
				{
					#ifdef _WIN32
					Sleep(3000);
					#else
					usleep(3000);
					#endif
				}
			}
            return quaternion.back();
		}
	}

	void Mahony_Attitude::RequestStop()
	{
		unique_lock<mutex> lock(mMutexStop);
		mbStopRequested = true;


	}

	void Mahony_Attitude::RequestStart()
	{
		unique_lock<mutex> lock(mMutexStop);
		if (mbStopped)
		{
			mbStopped = false;
			mbStopRequested = false;
		}

	}

	bool Mahony_Attitude::Stop()
	{
		unique_lock<mutex> lock(mMutexStop);
		if (mbStopRequested)
		{
			mbStopped = true;
			return true;
		}
		return false;
	}

	bool Mahony_Attitude::isStopped()
	{
		unique_lock<mutex> lock(mMutexStop);
		return mbStopped;
	}

	void Mahony_Attitude::Release()
	{
		unique_lock<mutex> lock(mMutexStop);
		mbStopped = false;
		mbStopRequested = false;
		
		Integ_angular = Vector_3::Zero();
		quaternion.clear();
		cout << "Mahony attitude release " << endl;
	}
}