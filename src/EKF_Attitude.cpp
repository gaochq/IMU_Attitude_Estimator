#include "EKF_Attitude.h"
#include "Convert.h"

using namespace std;

namespace IMU
{

EKF_Attitude::EKF_Attitude(bool approx_prediction, double dt):
        mbStopped(false), mbStopRequested(false), deltaT(dt)
{
	cout << "System initialization!" << endl;

	/* Set the primary value of state vector and covariance matrix */

	if (state_X_pro.size() == 0)
	{
		Vector_12 state_X_tmp;
		/*
		state_X_tmp <<	0.0018, -0.0003, 0.0015,
						0.0351, -0.0567, -0.0364,
						0.0093, -0.0012, -0.9998,
						0.0089, 0.4784, 0.8781;
		*/
		state_X_tmp <<	0.0, -0.0, 0.0,
						0.0, -0.0, -0.0,
						0.0, -0.0, -0.9998,
						0.0, 0.0, 1;
		state_X_pro.push_back(state_X_tmp);
	}

	if (CovrMatrix_P_pro.size()==0)
	{
//		CovrMatrix_P_pro.push_back(Eigen::MatrixXd::Ones(12, 12) * 200);
		Vector_3 V1(0, 0, 0.0001);
		Vector_3 V2(0, 0.0001, -0.0001);
		Matrix_3 M1, M2;
		Vect_to_SkewMat(V1, M1);
		Vect_to_SkewMat(V2, M2);

		Eigen::Matrix<double, 12, 12> CovrMatrix_tmp;
		CovrMatrix_tmp <<	0.004*Identity_Matrix3, 0.056*Identity_Matrix3, Zeros_Matrix3, Zeros_Matrix3,
							0.056*Identity_Matrix3, 0.2971*Identity_Matrix3, M1, M2,
							Zeros_Matrix3, -M1, 2.9956*Identity_Matrix3, Zeros_Matrix3,
							Zeros_Matrix3, -M2, Zeros_Matrix3, 0.7046*Identity_Matrix3;
		CovrMatrix_P_pro.push_back(CovrMatrix_tmp);
	}

	using_2ndOrder = approx_prediction;

	// set the process noise as the diagnoal of elements
	Vector_12 Pro_Nosiecovr;
	Vector_9 Mea_Nosiecovr;
	Pro_Nosiecovr <<	1e-4, 1e-4, 1e-4,
						0.08, 0.08, 0.08,
						0.009, 0.009, 0.009,
						0.005, 0.005, 0.005;
	Mea_Nosiecovr <<	0.0008, 0.0008, 0.0008,
						1000, 1000, 1000,
						100, 100, 100;

	Q_noise = Pro_Nosiecovr.asDiagonal();
	R_noise = Mea_Nosiecovr.asDiagonal();

}

void EKF_Attitude::Param_Change(Vector_12 Pro_Nosiecovr, Vector_9 Mea_Nosiecovr)
{
	if (isStopped())
	{
		Q_noise = Pro_Nosiecovr.asDiagonal();
		R_noise = Mea_Nosiecovr.asDiagonal();
	}
}

void EKF_Attitude::Cal_TransMatrix()
{
	Matrix_3 r_acc, r_mag, w_angular, w_angular_T;
	Vector_3 r_acc_v, r_mag_v, w_angular_v;
	Vector_12 state_X_tmp;
	state_X_tmp = state_X_pro.back();

	w_angular_v = state_X_tmp.block<3, 1>(0, 0);
	r_acc_v = state_X_tmp.block<3, 1>(6, 0);
	r_mag_v = state_X_tmp.block<3, 1>(9, 0);

	Vect_to_SkewMat(w_angular_v, w_angular_T);
	w_angular = w_angular_T.transpose();
	Vect_to_SkewMat(r_acc_v, r_acc);
	Vect_to_SkewMat(r_mag_v, r_mag);

	Alin << Zeros_Matrix3, Identity_Matrix3, Zeros_Matrix3, Zeros_Matrix3,
            Zeros_Matrix3, Zeros_Matrix3, Zeros_Matrix3, Zeros_Matrix3,
            r_acc, Zeros_Matrix3, w_angular, Zeros_Matrix3,
            -r_mag, Zeros_Matrix3, Zeros_Matrix3, w_angular;

	Alin = Eigen::MatrixXd::Identity(12, 12) + Alin*deltaT;
}

void EKF_Attitude::Prior_Predict()
{
	Vector_12 state_X_tmp;
	Vector_3  angular_vel, angular_acc, acc_state, mag_state;
	Matrix_3 w_angular, w_angular_T;

	state_X_tmp = state_X_pro.back();
	angular_vel = state_X_tmp.block<3, 1>(0, 0);
	angular_acc = state_X_tmp.block<3, 1>(3, 0);

	Vect_to_SkewMat(angular_vel, w_angular_T);
	w_angular = w_angular_T.transpose();
	angular_vel = angular_vel + angular_acc*deltaT;

	// calculate the non-liner system, and decide wto use the number of taylor expansion 
	if (using_2ndOrder)
	{
		Matrix_3 Temp_Mat = Identity_Matrix3 + w_angular*deltaT + deltaT*deltaT/2*w_angular*w_angular;

        acc_state = state_X_tmp.block<3, 1>(6, 0);
		acc_state = Temp_Mat*acc_state;

		mag_state = state_X_tmp.block<3, 1>(9, 0);
		mag_state = Temp_Mat*mag_state;
	}
	else
	{
		Matrix_3 Temp_Mat;
		acc_state = state_X_tmp.block<3, 1>(6, 0);
		Temp_Mat = Identity_Matrix3 + w_angular*deltaT;
		acc_state = Temp_Mat*acc_state;

		mag_state = state_X_tmp.block<3, 1>(9, 0);
		mag_state = Temp_Mat*mag_state;
	}

	// refer to the 4.20 and 4.21 formula
	Eigen::Matrix<double, 12, 12> CovrMatrix_tmp;

	state_X_tmp << angular_vel, angular_acc, acc_state, mag_state;
	state_X_pro.push_back(state_X_tmp);

	CovrMatrix_tmp = CovrMatrix_P_pro.back();
	CovrMatrix_tmp = Alin*CovrMatrix_tmp*Alin.transpose() + Q_noise;
	CovrMatrix_P_pro.push_back(CovrMatrix_tmp);
}

void EKF_Attitude::Post_Correct()
{

	Vector_12 state_X_tmp;
	Eigen::Matrix<double, 12, 12> CovrMatrix_tmp;

	state_X_tmp = state_X_pro.back();
	state_X_pro.pop_back();
	CovrMatrix_tmp = CovrMatrix_P_pro.back();
	CovrMatrix_P_pro.pop_back();

	// initialize the observe matrix H
	Eigen::Matrix<double, 9, 12> Observe_Matrix;
	Observe_Matrix <<	Identity_Matrix3, Zeros_Matrix3, Zeros_Matrix3, Zeros_Matrix3,
						Zeros_Matrix3, Zeros_Matrix3, Identity_Matrix3, Zeros_Matrix3,
						Zeros_Matrix3, Zeros_Matrix3, Zeros_Matrix3, Identity_Matrix3;

	// calculate the Kalman gain
	Eigen::Matrix<double, 9, 9> Kal_Matrix;
	Eigen::Matrix<double, 12, 9> Kalman_gain;
	Kal_Matrix = Observe_Matrix*CovrMatrix_tmp*Observe_Matrix.transpose() + R_noise;
	Kalman_gain = CovrMatrix_tmp*Observe_Matrix.transpose()*Kal_Matrix.inverse();

	// update the state vector
	Vector_9 Mea_residual;
	Mea_residual = cur_measurement - Observe_Matrix*state_X_tmp;
	state_X_tmp = state_X_tmp + Kalman_gain*Mea_residual;
	state_X_pro.push_back(state_X_tmp);

	// update the covariance Matrix
	CovrMatrix_tmp = (Eigen::MatrixXd::Identity(12, 12) - Kalman_gain*Observe_Matrix)*CovrMatrix_tmp;
	CovrMatrix_P_pro.push_back(CovrMatrix_tmp);

}

void EKF_Attitude::Cal_Quaternion()
{
	//if (quaternion.size() >= 358)
	//	cout << " " << endl;
	// extract the true state of three sensors
	Vector_3 acc_state, mag_state, angular_state;
	Vector_12 state_X_tmp;
	state_X_tmp = state_X_pro.back();

	acc_state = state_X_tmp.block<3, 1>(6, 0);
	mag_state = state_X_tmp.block<3, 1>(9, 0);

	Vector_3 x_state, y_state, z_state;
	acc_state.normalize();
	mag_state.normalize();

	z_state = -acc_state;

	y_state = z_state.cross(mag_state);
	y_state.normalize();

	x_state = y_state.cross(z_state);
	x_state.normalize();

	// the rotation order is X-Y-Z,and 
	Matrix_3 Rotation_matrix;
	Rotation_matrix << x_state, y_state, z_state;
	Eigen::Quaterniond quat_temp(Rotation_matrix.transpose());
//	Eigen::Quaterniond quat_temp = Rotation_to_Quater(Rotation_matrix.transpose());

	Vector_3 Euler;
	Euler = Quaternion_to_Euler(quat_temp);

	
//	cout << Euler << endl;
	quaternion.push_back(quat_temp);
}

void EKF_Attitude::Read_SensorData(Vector_9 measurement)
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

Eigen::Quaterniond EKF_Attitude::Run(Vector_9 measurement)
{
	while (true)
	{
		Read_SensorData(measurement);

		Cal_TransMatrix();

		Prior_Predict();

		Post_Correct();

		Cal_Quaternion();

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

void EKF_Attitude::RequestStop()
{
	unique_lock<mutex> lock(mMutexStop);
	mbStopRequested = true;
}

void EKF_Attitude::RequestStart()
{
	unique_lock<mutex> lock(mMutexStop);
	if (mbStopped)
	{
		mbStopped = false;
		mbStopRequested = false;
	}

}

bool EKF_Attitude::Stop()
{
	unique_lock<mutex> lock(mMutexStop);
	if (mbStopRequested)
	{
		mbStopped = true;
		return true;
	}
	return false;
}

bool EKF_Attitude::isStopped()
{
	unique_lock<mutex> lock(mMutexStop);
	return mbStopped;
}

void EKF_Attitude::Release()
{
	unique_lock<mutex> lock(mMutexStop);
	mbStopped = false;
	mbStopRequested = false;
	state_X_pro.clear();
	CovrMatrix_P_pro.clear();
	quaternion.clear();

	cout << "EKF attitude release " << endl;
}

}