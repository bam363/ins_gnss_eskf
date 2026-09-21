//
// Created by Barin A. Moghimi on 9/7/26.
//

#include "../include/SimulatorEKF.h"


SimulatorEKF::SimulatorEKF(const SimConfig& simConfig, const ModelConfigEKF& modelConfig, const ICConfigEKF& icConfig) :
                           rangeNoise_(0.0, modelConfig.sigmaRange), modelConfig_(modelConfig),
                           simConfig_(simConfig), icConfig_(icConfig)
{}

double SimulatorEKF::measureRange(const Eigen::Vector2d& anchor, const Eigen::Vector4d& x) {
    const auto px = x[0];
    const auto py = x[1];
    const auto lx = anchor[0];
    const auto ly = anchor[1];
    const double noise = rangeNoise_(generator_);
    const double rho = std::sqrt((px - lx) * (px - lx) + (py - ly) * (py - ly));

    return rho + noise;

}

SimulatorEKF::SimulationData SimulatorEKF::runSimulator() {
    SimulationData data;

    // Extract Model Configuration Parameters
    double sigmaRange = modelConfig_.sigmaRange;
    Eigen::Matrix4d F = modelConfig_.F;

    // Extract Simulation Configuration Parameters
    auto dt = simConfig_.dt;
    auto simTime = simConfig_.simTime;

    // Extract Initial Conditions
    auto anchor1 = icConfig_.anchor1;
    auto anchor2 = icConfig_.anchor2;

    std::vector<Eigen::Vector4d> x;
    std::vector<double> z1;
    std::vector<double> z2;

    // Generate truth + measurements
    const int N = static_cast<int>(simTime / dt);

    x.reserve(N);
    z1.reserve(N);
    z2.reserve(N);

    // Extract Initial Condition Parameters
    auto xt = icConfig_.x0;

    for (int i = 0; i < N; i++) {

        // GENERATE MEASUREMENTS
        z1.push_back(measureRange(anchor1, xt));
        z2.push_back(measureRange(anchor2, xt));

        // GENERATE TRUE STATE
        x.push_back(xt);
        xt = F * xt;
    };

    data.truthState = x;
    data.positionMeasurements1 = z1;
    data.positionMeasurements2 = z2;
    data.N = N;

    return data;
}


