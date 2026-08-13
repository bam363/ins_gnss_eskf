//
// Created by Barin A. Moghimi on 8/10/26.
//

#include "Simulator.h"

Simulator::Simulator(const ModelConfig& config,
                     Eigen::Vector2d x0,
                     double dt,
                     double sigma_z,
                     double simTime,
                     std::mt19937 generator)
                     : F_(config.F), H_(config.H), Q_(config.Q), R_(config.R),
                       x0_(x0), dt_(dt), sigma_z_(sigma_z),
                       simTime_(simTime), generator_(generator)
{}

SimulationData Simulator::runSimulator() {

    SimulationData data;

    Eigen::LLT<Eigen::Matrix2d> llt(Q_);
    Eigen::Matrix2d L;
    if (llt.info() == Eigen::Success) {
        L = llt.matrixL();
    } else {
        throw std::runtime_error("llt.matrixL() failed");
    }

    std::normal_distribution<double> std_noise(0.0, 1.0);
    std::normal_distribution<double> measurement_noise(0.0, sigma_z_);

    // generate truth + measurements
    const int N = static_cast<int>(simTime_ / dt_);

    std::vector<Eigen::Vector2d> x;
    std::vector<double> z;

    x.reserve(N);
    z.reserve(N);

    auto xt = x0_;
    for (int i = 0; i < N; i++) {
        Eigen::Vector2d Zeta;
        Zeta << std_noise(generator_), std_noise(generator_);

        auto w = L * Zeta;
        double v = measurement_noise(generator_);
        z.push_back((H_ * xt)(0) + v);
        x.push_back(xt);

        xt = F_ * xt + w;
    };

    data.truth = x;
    data.measurements = z;
    data.N = N;

    return data;

}