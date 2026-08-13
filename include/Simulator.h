//
// Created by Barin A. Moghimi on 8/10/26.
//

#ifndef ESTIMATION_LAB_SIMULATION_H
#define ESTIMATION_LAB_SIMULATION_H
#include <random>
#include <vector>
#include <Eigen/dense>
#include "ModelConfig.h"

struct SimulationData {
    std::vector<Eigen::Vector2d> truth;
    std::vector<double> measurements;
    unsigned int N; // length of simulation
};


class Simulator {
public:
    Simulator(const ModelConfig& config,
              Eigen::Vector2d x0,
              double dt,
              double sigma_z,
              double simTime,
              std::mt19937 generator);

    SimulationData runSimulator();

private:
    double dt_;
    double sigma_z_;
    double simTime_;
    std::mt19937 generator_;

    Eigen::Vector2d x0_;
    Eigen::Matrix2d F_;
    Eigen::RowVector2d H_;
    Eigen::Matrix2d Q_;
    Eigen::Matrix2d L_;
    double R_;
};

#endif //ESTIMATION_LAB_SIMULATION_H
