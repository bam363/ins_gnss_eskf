//
// Created by Barin A. Moghimi on 8/10/26.
//

#ifndef ESTIMATION_LAB_ESTIMATORRUNNER_H
#define ESTIMATION_LAB_ESTIMATORRUNNER_H

#include "KalmanFilter.hpp"
#include "../include/Simulator.h"
#include <Eigen/Dense>

struct EstimatorRunnerData {
    std::vector<Eigen::Vector3d> xh;
    std::vector<Eigen::Matrix3d> Ph;
    std::vector<double> v;
    std::vector<double> S;
};

class EstimatorRunner {

public:
    EstimatorRunner() = default;

    static EstimatorRunnerData runEstimator(KalmanFilter& filter, const Simulator::SimulationData& data);

private:

};
#endif //ESTIMATION_LAB_ESTIMATORRUNNER_H
