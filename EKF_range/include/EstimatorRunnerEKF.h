//
// Created by Barin A. Moghimi on 8/10/26.
//

#ifndef ESTIMATION_LAB_ESTIMATORRUNNER_H
#define ESTIMATION_LAB_ESTIMATORRUNNER_H

#include "../include/ExtendedKalmanFilter.h"
#include "SimulatorEKF.h"
#include <Eigen/Dense>

class EstimatorRunnerEKF {

public:
    EstimatorRunnerEKF() = default;

    struct EstimatorRunnerData {
        std::vector<Eigen::Vector4d> xh;
        std::vector<Eigen::Matrix4d> Ph;
        std::vector<double> v;
        std::vector<double> S;
    };

    static EstimatorRunnerData runEstimator(ExtendedKalmanFilter& filter,
                                            const SimulatorEKF::SimulationData& data);

private:

};
#endif //ESTIMATION_LAB_ESTIMATORRUNNER_H
