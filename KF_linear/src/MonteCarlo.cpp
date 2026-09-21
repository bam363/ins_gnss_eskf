//
// Created by Barin A. Moghimi on 8/10/26.
//

#include "MonteCarlo.h"
#include <random>

#include "ICConfig.h"
#include "ModelConfig.h"
#include "MonteCarloResult.h"
#include "KalmanFilter.hpp"
#include "Simulator.h"
#include "../include/EstimatorRunner.h"
#include "Metrics.h"
#include <numeric>
#include <cassert>

MonteCarlo::MonteCarlo(std::size_t numTrials, unsigned int masterSeed)
                       : numTrials_(numTrials), masterSeed_(masterSeed) {
    if (numTrials_ < 2)
    {
        throw std::invalid_argument("MonteCarlo requires at least 2 trials");
    };
}

MonteCarloResults MonteCarlo::run(const ICConfig& icConfig, const ModelConfig& modelConfig1,
                                  const ModelConfig& modelConfig2, const SimConfig& simConfig) {

    std::mt19937 seedGenerator(masterSeed_);

    auto dt = simConfig.dt;
    auto simTime = simConfig.simTime;

    Metrics metrics;
    std::vector<double> trialMeanNEES;
    trialMeanNEES.reserve(numTrials_);
    std::vector<double> trialMeanNIS;
    trialMeanNIS.reserve(numTrials_);
    trialSeeds_.reserve(numTrials_);

    auto x_true_0 = icConfig.x0;
    auto P = icConfig.P0;
    auto x_est_0 = icConfig.xh0;
    auto sigmaz = modelConfig1.sigma_z;
    auto trueConfig = modelConfig1;
    auto filterConfig = modelConfig2;

    const std::size_t N = static_cast<std::size_t>(simTime / dt);
    std::vector<double> ensembleNEES(N, 0.0);
    std::vector<double> ensemblePosNEES(N, 0.0);
    std::vector<double> ensembleVelNEES(N, 0.0);
    std::vector<double> ensembleBiasNEES(N, 0.0);
    std::vector<std::vector<Eigen::Vector3d>> errorTimeSeries;
    std::vector<Eigen::Matrix3d> meanUncertainty(N, Eigen::Matrix3d::Zero());

    for (std::size_t i = 0; i < numTrials_; i++) {
        unsigned int trialSeed = seedGenerator();
        trialSeeds_.push_back(trialSeed);

        std::mt19937 trialGenerator(trialSeed);

        Eigen::Vector3d err;
        std::normal_distribution<double> pos_uncertainty(0.0, sqrt(P(0,0)));
        std::normal_distribution<double> vel_uncertainty(0.0, sqrt(P(1,1)));
        std::normal_distribution<double> bias_uncertainty(0.0, sqrt(P(2,2)));
        err << pos_uncertainty(trialGenerator), vel_uncertainty(trialGenerator), bias_uncertainty(trialGenerator);
        auto trial_x_est_0 = x_true_0 + err;
        ICConfig trialICConfig = icConfig;
        trialICConfig.xh0 = trial_x_est_0;

        Simulator simulator(trialICConfig, trueConfig, simConfig, trialGenerator);
        Simulator::SimulationData truth_data = simulator.runSimulator();

        KalmanFilter filter(trialICConfig, filterConfig);
        EstimatorRunnerData estimator_runner_data = EstimatorRunner::runEstimator(filter, truth_data);

        // Compute per-trial NEES
        auto x = truth_data.truthState;
        auto xh = estimator_runner_data.xh;
        auto Ph = estimator_runner_data.Ph;

        double nees = metrics.computePerRunNEES(x, xh, Ph);

        trialMeanNEES.push_back(nees);

        // Compute per-sample NEES
        std::vector<double> sampleNEES = metrics.computePerSampleNEES(x, xh, Ph);
        std::vector<double>  samplePosNEES = metrics.computePerSamplePerStateNEES(x, xh, Ph, 0);
        std::vector<double>  sampleVelNEES = metrics.computePerSamplePerStateNEES(x, xh, Ph, 1);
        std::vector<double>  sampleBiasNEES = metrics.computePerSamplePerStateNEES(x, xh, Ph, 2);

        // Accumulate per-sample NEES into ensemble NEES
        for (std::size_t k = 0; k < sampleNEES.size(); k++) {
            ensembleNEES[k] += sampleNEES[k];
            ensemblePosNEES[k] += samplePosNEES[k];
            ensembleVelNEES[k] += sampleVelNEES[k];
            ensembleBiasNEES[k] += sampleBiasNEES[k];
        };

        // Compute NIS
        double nis = metrics.computeNIS(estimator_runner_data.v,
                                        estimator_runner_data.S);
        trialMeanNIS.push_back(nis);

        // Compute and store state error
        auto errorsTimeSeriePerTrial = metrics.computePerSampleError(x, xh);
        errorTimeSeries.push_back(errorsTimeSeriePerTrial);

        // Mean Uncertainty accumulator
        for (std::size_t k = 0; k < N; k++) {
            meanUncertainty[k] += Ph[k];
        };


    }

    // Compute summary statistics
    MonteCarloResults results;
    results.trialSeeds = trialSeeds_;

    // NEES statistics
    results.trialMeanNEES = trialMeanNEES;
    results.meanNEES = metrics.computeMean(trialMeanNEES);
    results.maxNEES = metrics.computeMax(trialMeanNEES);
    results.minNEES = metrics.computeMin(trialMeanNEES);
    results.stdNEES = metrics.computeStd(trialMeanNEES, results.meanNEES);
    for (auto& value : ensembleNEES) {
        value /= numTrials_;
    };
    for (auto& value : ensemblePosNEES) {
        value /= numTrials_;
    };
    for (auto& value : ensembleVelNEES) {
        value /= numTrials_;
    };
    for (auto& value : ensembleBiasNEES) {
        value /= numTrials_;
    };

    results.ensembleMeanNEES = ensembleNEES;
    results.ensemblePosNEES = ensemblePosNEES;
    results.ensembleVelNEES = ensembleVelNEES;
    results.ensembleBiasNEES = ensembleBiasNEES;

    // NIS statistics
    results.trialMeanNIS = trialMeanNIS;
    results.meanNIS = metrics.computeMean(trialMeanNIS);
    results.maxNIS = metrics.computeMax(trialMeanNIS);
    results.minNIS = metrics.computeMin(trialMeanNIS);
    results.stdNIS = metrics.computeStd(trialMeanNIS, results.meanNIS);

    // Empirical Uncertainty
    // Compute mean error time series over all trials
    std::vector<Eigen::Vector3d> meanEps(N, Eigen::Vector3d::Zero());
    for (size_t j = 0; j < N; j++){
        for (size_t i = 0; i < numTrials_; i++)
            meanEps[j] += errorTimeSeries[i][j];
    };
    for (auto& value : meanEps) {
        value /= numTrials_;
    };
    // Compute empirical uncertainty time series
    std::vector<Eigen::Matrix3d> Pemp;
    for (size_t j = 0; j < N; j++) {
        Eigen::Matrix3d outerProd = Eigen::Matrix3d::Zero();
        for (size_t i = 0; i < numTrials_; i++)
            outerProd += (errorTimeSeries[i][j] - meanEps[j]) * (errorTimeSeries[i][j] - meanEps[j]).transpose();
        outerProd /= static_cast<double>(numTrials_ - 1);
        Pemp.push_back(outerProd);
    };
    results.empiricalUncertainty = Pemp;

    // Ensemble Theoretical Uncertainty
    for (auto& value : meanUncertainty) {
        value /= numTrials_;
    }
    results.theoreticalUncertainty = meanUncertainty;

    return results;
}