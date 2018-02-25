cmake_minimum_required(VERSION 2.8)
project(IMU_Attitude_Estimator)

#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -o3")


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY  ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY  ${PROJECT_SOURCE_DIR}/lib)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

include_directories("/usr/include/eigen3")

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
        ${EIGEN3_LIBS}
        ${PYTHON_LIBRARIES})

add_executable(Demo demo/Test.cpp)
target_link_libraries(Demo ${PROJECT_NAME})