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
#include "EstimatorRunner.h"
#include "Metrics.h"
#include <numeric>
#include <cassert>

MonteCarlo::MonteCarlo(std::size_t numTrials, unsigned int masterSeed)
                       : numTrials_(numTrials), masterSeed_(masterSeed) { assert(numTrials_ > 1); }

MonteCarloResults MonteCarlo::run(ModelConfig& modelConfig, ICConfig& ICconfig, SimConfig& simConfig) {

    std::mt19937 seedGenerator(masterSeed_);

    auto dt = simConfig.dt;
    auto simTime = simConfig.simTime;

    Metrics metrics;
    std::vector<double> trialMeanNEES;
    trialMeanNEES.reserve(numTrials_);
    std::vector<double> trialMeanNIS;
    trialMeanNIS.reserve(numTrials_);
    trialSeeds_.reserve(numTrials_);

    for (std::size_t i = 0; i < numTrials_; i++) {
        unsigned int trialSeed = seedGenerator();
        trialSeeds_.push_back(trialSeed);

        std::mt19937 trialGenerator(trialSeed);

        auto trueConfig = modelConfig;
        auto x_true_0 = ICconfig.x0;
        auto sigmaz = modelConfig.sigma_z;

        Simulator simulator(trueConfig, x_true_0, dt, sigmaz, simTime, trialGenerator);
        SimulationData truth_data = simulator.runSimulator();

        auto filterConfig = modelConfig;
        auto x_est_0 = ICconfig.xh0;
        auto P = ICconfig.P0;
        KalmanFilter filter(filterConfig, x_est_0, P);
        EstimatorRunnerData estimator_runner_data = EstimatorRunner::runEstimator(filter, truth_data);

        // Compute per-trial NEES
        // auto x = truth_data.truth;
        // auto xh = estimator_runner_data.xh;
        // auto Ph = estimator_runner_data.Ph;

        double nees = metrics.computeNEES(truth_data.truthState,
                                            estimator_runner_data.xh,
                                            estimator_runner_data.Ph);
        trialMeanNEES.push_back(nees);

        double nis = metrics.computeNIS(estimator_runner_data.v,
                                        estimator_runner_data.S);

        trialMeanNIS.push_back(nis);
    }

    // Compute summary statistics
    MonteCarloResults results;
    results.trialSeeds = trialSeeds_;

    // NEES statistics
    results.trialMeanNEES = trialMeanNEES;
    double meanNEES = std::accumulate(trialMeanNEES.begin(),trialMeanNEES.end(),0.0) / trialMeanNEES.size();
    results.meanNEES = meanNEES;
    results.maxNEES = *std::max(trialMeanNEES.begin(), trialMeanNEES.end());
    results.minNEES = *std::min(trialMeanNEES.begin(), trialMeanNEES.end());
    double sumOfSquares = 0.0;
    for (std::size_t i = 0; i < trialMeanNEES.size(); i++) {
        sumOfSquares += (trialMeanNEES[i] - meanNEES) * (trialMeanNEES[i] - meanNEES);
    }
    double stdNEES = std::sqrt(sumOfSquares / (numTrials_ - 1));
    results.stdNEES = stdNEES;

    // NIS statistics
    results.trialMeanNIS = trialMeanNIS;
    double meanNIS = std::accumulate(trialMeanNIS.begin(),trialMeanNIS.end(),0.0) / trialMeanNIS.size();
    results.meanNIS = meanNIS;
    results.maxNIS = *std::max(trialMeanNIS.begin(), trialMeanNIS.end());
    results.minNIS = *std::min(trialMeanNIS.begin(), trialMeanNIS.end());
    sumOfSquares = 0.0;
    for (std::size_t i = 0; i < trialMeanNIS.size(); i++) {
        sumOfSquares += (trialMeanNIS[i] - meanNIS) * (trialMeanNIS[i] - meanNIS);
    }
    double stdNIS = std::sqrt(sumOfSquares / (numTrials_ - 1));
    results.stdNIS = stdNIS;

    return results;
}