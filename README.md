# IMU_Attitude_Estimator
---
This project is aimed at estimating the attitude of Attitude Heading and Reference System(AHRS).This library contains 3 popular attitude estimator algorithms.
- Mahony's algorithm
- Extend Kalman Filter(EKF)
- Error State Kalman Filter(ESKF)


&emsp;&emsp;[1] [Euston M, Coote P, Mahony R, et al. A complementary filter for attitude estimation of a fixed-wing UAV[C]// Ieee/rsj International Conference on Intelligent Robots and Systems. IEEE, 2008:340-345.](http://xueshu.baidu.com/s?wd=paperuri%3A%28cdfdf7c8bcba54639385fd106ecf617a%29&filter=sc_long_sign&tn=SE_xueshusource_2kduw22v&sc_vurl=http%3A%2F%2Fieeexplore.ieee.org%2Fxpls%2Fabs_all.jsp%3Farnumber%3D4650766&ie=utf-8&sc_us=10281205102711940977)

&emsp;&emsp;[2] [Pixhawk state estimation](https://pixhawk.org/_media/firmware/apps/attitude_estimator_ekf/ekf_excerptmasterthesis.pdf)

&emsp;&emsp;[3] [Solà, Joan. Quaternion kinematics for the error-state Kalman filter[J]. 2017.](http://219.216.82.193/cache/4/03/www.iri.upc.edu/bbcd603c764cd75e76df0968d16bc022/kinematics.pdf)

## 1. Dependencies
- Eigen3.2.0
- glog
```
sudo apt-get install libgoogle-glog-dev
```

