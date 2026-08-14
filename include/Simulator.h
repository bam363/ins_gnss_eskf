//
// Created by Barin A. Moghimi on 8/10/26.
//

#ifndef ESTIMATION_LAB_SIMULATION_H
#define ESTIMATION_LAB_SIMULATION_H
#include <random>
#include <vector>
#include <Eigen/dense>

#include "ICConfig.h"
#include "ModelConfig.h"
#include "SimConfig.h"

struct SimulationData {
    std::vector<Eigen::Vector3d> truthState;
    std::vector<double> positionMeasurements;
    std::vector<double> accelMeasurements;
    std::vector<double> biasState;
    std::size_t N; // length of simulation
};


class Simulator {
public:
    Simulator(const ICConfig& icconfig,
              const ModelConfig& modelConfig,
              const SimConfig& simconfig,
              std::mt19937 generator);

    SimulationData runSimulator();
    double accel_true(int i, double dt);

private:
    // double dt_;
    // double sigma_z_;
    // double simTime_;
    Eigen::Vector3d x0_;
    std::mt19937 generator_;

    ModelConfig modelConfig_;
    SimConfig simConfig_;
    ICConfig icConfig_;
    // Eigen::Matrix3d F_;
    // Eigen::RowVector3d H_;
    // Eigen::RowVector3d B_;
    // Eigen::Matrix3d Q_;
    // Eigen::Matrix3d L_;
    // double R_;
};

#endif //ESTIMATION_LAB_SIMULATION_H
