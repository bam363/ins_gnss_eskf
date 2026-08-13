//
// Created by Barin A. Moghimi on 8/10/26.
//

#include "EstimatorRunner.h"

// run estimator
EstimatorRunnerData EstimatorRunner::runEstimator(KalmanFilter& filter, const SimulationData& data) {
    EstimatorRunnerData result;
    auto N = data.N;
    auto measurements = data.measurements;
    std::vector<Eigen::Vector2d> xh;
    xh.reserve(N);
    std::vector<Eigen::Matrix2d> Ph;
    Ph.reserve(N);
    std::vector<double> v;
    v.reserve(N);
    std::vector<double> S;
    S.reserve(N);

    for (int j = 0; j < N; j++) {

        // Prediction
        if (j > 0) {
            filter.predict();
        }

        // Measurement Update
        auto z = measurements[j];
        filter.update(z);

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