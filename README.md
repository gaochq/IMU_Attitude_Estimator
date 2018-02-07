# IMU_Attitude_Estimator
---
This project is aimed at estimating the attitude of Attitude Heading and Reference System(AHRS).This library contains 3 popular attitude estimator algorithms.
- Mahony's algorithm
- Extend Kalman Filter(EKF)
- Error State Kalman Filter(ESKF)


&emsp;&emsp;[1] [Mahony R, Hamel T, Pflimlin J M. Nonlinear complementary filters on the special orthogonal group[J]. IEEE Transactions on automatic control, 2008, 53(5): 1203-1218.](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4608934)

&emsp;&emsp;[2] [Pixhawk state estimation](https://pixhawk.org/_media/firmware/apps/attitude_estimator_ekf/ekf_excerptmasterthesis.pdf)

&emsp;&emsp;[3] [Solà, Joan. Quaternion kinematics for the error-state Kalman filter[J]. 2017.](http://219.216.82.193/cache/4/03/www.iri.upc.edu/bbcd603c764cd75e76df0968d16bc022/kinematics.pdf)

## 1. Dependencies
- Eigen3.2.0
- glog
```
sudo apt-get install libgoogle-glog-dev
```

