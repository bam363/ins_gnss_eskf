//
// Created by Barin A. Moghimi on 8/10/26.
//

#include "Simulator.h"

Simulator::Simulator(const ICConfig& icConfig,
                     const ModelConfig& modelConfig,
                     const SimConfig& simConfig,
                     std::mt19937 generator)
                     : icConfig_(icConfig), modelConfig_(modelConfig),
                       simConfig_(simConfig), generator_(generator)
{}

SimulationData Simulator::runSimulator() {

    SimulationData data;

    // Extract Model Configuration Parameters
    double sigma_z = modelConfig_.sigma_z;
    double sigma_accel = modelConfig_.sigma_accel;
    double sigma_bias = modelConfig_.sigma_bias;
    // auto Q = modelConfig_.Q;
    auto H = modelConfig_.H;
    auto B = modelConfig_.B;
    auto F = modelConfig_.F;

    // Extract Simulation Configuration Parameters
    auto dt = simConfig_.dt;
    auto simTime = simConfig_.simTime;

    // Eigen::LLT<Eigen::Matrix3d> llt(Q);
    // Eigen::Matrix3d L;
    // if (llt.info() == Eigen::Success) {
    //     L = llt.matrixL();
    // } else {
    //     throw std::runtime_error("llt.matrixL() failed");
    // }

    // std::normal_distribution<double> std_noise(0.0, 1.0);
    std::normal_distribution<double> measurement_noise(0.0, sigma_z);
    std::normal_distribution<double> accel_noise(0.0, sigma_accel);
    std::normal_distribution<double> bias_noise(0.0, sigma_bias);

    // Generate truth + measurements
    const int N = static_cast<int>(simTime / dt);

    std::vector<Eigen::Vector3d> x;
    std::vector<double> z;
    std::vector<double> am;

    x.reserve(N);
    z.reserve(N);
    am.reserve(N);

    // Extract Initial Condition Parameters
    auto xt = icConfig_.x0;

    for (int i = 0; i < N; i++) {
        // Eigen::Vector3d Zeta;
        // Zeta << std_noise(generator_), std_noise(generator_), std_noise(generator_);
        // auto w = L * Zeta;

        // GENERATE MEASUREMENTS
        // (position)
        const double v = measurement_noise(generator_);
        z.push_back((H * xt)(0) + v);
        // (acceleration)
        const double n_a = accel_noise(generator_);
        double at = accel_true(i, dt);
        auto b = xt[2];
        am.push_back(at + b + n_a);

        // GENERATE TRUE STATE
        x.push_back(xt);
        // xt = F_ * xt + w;
        xt = F * xt + B * at;
        const double n_b = bias_noise(generator_);
        xt[2] = xt[2] + n_b;
    };

    data.truthState = x;
    data.positionMeasurements = z;
    data.accelMeasurements = am;
    data.N = N;

    return data;
}

double Simulator::accel_true(int i, double dt) {

        if (i < static_cast<int>(2.0 / dt)) {
            return 0.0;
        } else if (i < static_cast<int>(6.0 / dt)) {
            return 1.0;
        } else if (i < static_cast<int>(10.0 / dt)) {
            return 0.0;
        } else if (i < static_cast<int>(14.0 / dt)) {
            return -0.5;
        } else if (i < static_cast<int>(20.0 / dt)) {
            return 0.0;
        } else
            return 0.0;
};