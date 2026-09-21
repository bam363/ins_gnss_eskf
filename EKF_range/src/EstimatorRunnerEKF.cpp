//
// Created by Barin A. Moghimi on 8/10/26.
//

#include "../include/EstimatorRunnerEKF.h"

// run estimator
EstimatorRunnerEKF::EstimatorRunnerData EstimatorRunnerEKF::runEstimator(ExtendedKalmanFilter& filter,
                                                                         const SimulatorEKF::SimulationData& data) {
    EstimatorRunnerEKF::EstimatorRunnerData result;
    auto anchor1 = filter.anchor1();
    auto anchor2 = filter.anchor2();
    auto N = data.N;
    auto z1 = data.positionMeasurements1;
    auto z2 = data.positionMeasurements2;
    std::vector<Eigen::Vector4d> xh;
    xh.reserve(N);
    std::vector<Eigen::Matrix4d> Ph;
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
        filter.updateRange(z1[j], anchor1);
        filter.updateRange(z2[j], anchor2);


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