//
// Created by Barin A. Moghimi on 8/10/26.
//

#include "../include/EstimatorRunner.h"

// run estimator
EstimatorRunnerData EstimatorRunner::runEstimator(KalmanFilter& filter, const Simulator::SimulationData& data) {
    EstimatorRunnerData result;
    auto N = data.N;
    auto z_pos = data.positionMeasurements;
    auto z_accel = data.accelMeasurements;
    std::vector<Eigen::Vector3d> xh;
    xh.reserve(N);
    std::vector<Eigen::Matrix3d> Ph;
    Ph.reserve(N);
    std::vector<double> v;
    v.reserve(N);
    std::vector<double> S;
    S.reserve(N);

    for (int j = 0; j < N; j++) {

        // Prediction
        if (j > 0) {
            filter.predict(z_accel[j-1]);
        }

        // Measurement Update
        filter.update(z_pos[j]);

        // Storage
        xh.push_back(filter.state());
        Ph.push_back(filter.covariance());
        v.push_back(filter.innovation());
        S.push_back(filter.innovationCovariance());

    }
    result.xh = xh;
    result.Ph = Ph;
    result.S = S;
    result.v = v;
    return result;
}