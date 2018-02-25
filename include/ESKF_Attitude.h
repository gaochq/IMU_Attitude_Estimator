#ifndef ESKF_ATTITUDE_H
#define ESKF_ATTITUDE_H

#include <iostream>
#include <vector>
#include <mutex>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "TypeDefs.h"

#ifdef _WIN32
#include "windows.h"
#else
#include "unistd.h"
#endif

using namespace std;

namespace IMU
{
class ESKF_Attitude
{

public:
    ESKF_Attitude(Vector_12 Covar_Mat, double dt);
    // Initialize the true state of the estimator
    // including the nomial state and the error state,
    // and the covariances matrices
    void Init_Estimator();

    // change the parametres of the estimator
    void Param_Change(Vector_12 Covar_Mat);

    // Predict the nomial and error state
    void NominaState_Predict();
    void ErrorState_Predict();

    // Update the filter parametres
    void Update_Filter();

    // Uptate the nominal state
    void Update_NomianState();

    // Reset the error state
    void Reset_ErrorState();

    // read the sensors data and normalize the accelerometer and magnetometer
    void Read_SensorData(Vector_9 measurement);

    Eigen::Quaterniond Run(Vector_9 measurement);
    void Release();
    void RequestStop();
    void RequestStart();
    bool Stop();
    bool isStopped();

private:
    // the Gaussian noise for the covariances matrices Q and R
    Vector_3 DetAng_noise;
    Vector_3 DetAngVel_noise;
    Vector_3 Acc_noise;
    Vector_3 Mag_noise;
    Eigen::Matrix<double, 6, 6> CovarMat_Q;
    Eigen::Matrix<double, 6, 6> CovarMat_R;


    struct IMU_State
    {
    public:

        //! normal state
        Eigen::Quaterniond Nominal_quat;        //! quaterion state
        Vector_3 Nominal_AngVel;                //! Gyro bias state

        //! Error state
        Vector_3 Error_theta;
        Vector_3 Error_AngVel;
        Eigen::Matrix<double, 6, 6> Error_Convar;

        // calculate the observe matrix and the correction residual
        Eigen::Matrix<double, 6, 6> Cal_ObserveMat(Vector_9 measurenment, Vector_6 &residual);
    };

    vector< IMU_State > State_Vector;
    vector< Eigen::Quaterniond > quaternion;
    // sample time
    double deltaT;

    // Current and last time measurement (gyro;acc;mag)
    Vector_9 Cur_Measurement;
    Vector_9 Last_Measurement;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;
};

}


#endif