//
// Created by buyi on 18-1-28.
//

#ifndef IMU_ATTITUDE_ESTIMATOR_TIC_TOC_H_H
#define IMU_ATTITUDE_ESTIMATOR_TIC_TOC_H_H

#include <ctime>
#include <cstdlib>
#include <chrono>

namespace IMU
{

class TicToc
{
public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()        //ms
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count()*1000;
    }

private:
    std::chrono::time_point <std::chrono::system_clock> start, end;
};


} //namesapce IMU

#endif //IMU_ATTITUDE_ESTIMATOR_TIC_TOC_H_H
