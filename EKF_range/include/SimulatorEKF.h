//
// Created by Barin A. Moghimi on 9/7/26.
//

#ifndef ESTIMATION_LAB_SIMULATOREKF_H
#define ESTIMATION_LAB_SIMULATOREKF_H

#include <Eigen/Dense>
#include <random>
#include "ModelConfigEKF.h"
#include "../../KF_linear/include/SimConfig.h"
#include "ICConfigEKF.h"

class SimulatorEKF
{
public:
    SimulatorEKF(const SimConfig& simConfig, const ModelConfigEKF& modelConfig, const ICConfigEKF& icConfig);

    struct SimulationData {
        std::vector<Eigen::Vector4d> truthState;
        std::vector<double> positionMeasurements1;
        std::vector<double> positionMeasurements2;
        std::size_t N; // length of simulation
    };

    // void propagate();

    double measureRange(const Eigen::Vector2d& anchor, const Eigen::Vector4d& x);

    SimulationData runSimulator();

private:
    ModelConfigEKF modelConfig_;
    SimConfig simConfig_;
    ICConfigEKF icConfig_;
    Eigen::Vector4d x0_;
    std::mt19937 generator_;
    std::normal_distribution<double> rangeNoise_;
};

#endif //ESTIMATION_LAB_SIMULATOREKF_H
