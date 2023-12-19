cmake_minimum_required(VERSION 3.20.0)
project(IMU_Attitude_Estimator)

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(PythonLibs 2.7)
include_directories(${PYTHON_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)


add_library(${PROJECT_NAME} SHARED
        src/Convert.cpp
        src/EKF_Attitude.cpp
        src/ESKF_Attitude.cpp
        src/Mahony_Attitude.cpp)

target_link_libraries(${PROJECT_NAME}
        ${GLOG_LIBRARIES}
        Eigen3::Eigen
        ${PYTHON_LIBRARIES}) 

add_executable(Demo demo/Test.cpp)
target_link_libraries(Demo ${PROJECT_NAME})