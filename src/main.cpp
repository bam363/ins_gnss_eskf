#include <iostream>
#include <Eigen/Dense>
#include <random>

#include "EstimatorRunner.h"
#include "KalmanFilter.hpp"
#include "ModelConfig.h"
#include "Simulator.h"
#include "Metrics.h"
#include "MonteCarlo.h"

// TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or click the <icon src="AllIcons.Actions.Execute"/> icon in the gutter.

int main() {
    // TIP Press <shortcut actionId="RenameElement"/> when your caret is at the <b>lang</b> variable name to see how CLion can help you rename it.

    double dt = 0.1;
    double q = 1.0;
    double sigmaz = 2.0;
    double R = sigmaz * sigmaz;
    double simTime = 20.0;
    // int seed = 500;
    // std::mt19937 generator(seed);

    Eigen::Vector2d x_true_0;
    x_true_0 << 0.0, 10.0;
    Eigen::Vector2d x_est_0;
    x_est_0 << 5.0, 5.0;

    // define F, H, Q, R
    R = sigmaz * sigmaz;
    Eigen::Matrix2d F;
    F << 1.0, dt,
         0.0, 1.0;
    Eigen::RowVector2d H;
    H << 1.0, 0.0;
    Eigen::Matrix2d Q;
    Q << dt * dt * dt / 3.0, dt * dt / 2.0,
         dt * dt / 2.0,      dt;
    Q *= q;
    Eigen::Matrix2d P;
    P << 25.0,  0.0,
          0.0, 25.0;

    // // Generate Truth Model
    ModelConfig trueConfig{F, Q, H, R};
    // Simulator simulator(trueConfig, x_true_0, dt, sigmaz, simTime, generator);
    // SimulationData truth_data = simulator.runSimulator();
    //
    // // Generate Estimation Model
    ModelConfig filterConfig{F, Q, H, R};
    // KalmanFilter filter(filterConfig, x_est_0, P);
    // EstimatorRunnerData estimator_runner_data = EstimatorRunner::runEstimator(filter, truth_data);
    //
    // // Compute Metrics
    // Metrics metrics;
    // double nees = metrics.computeNEES(truth_data.truth,
    //                                     estimator_runner_data.xh,
    //                                     estimator_runner_data.Ph);
    //
    // std::cout << "NEES is :" << nees << std::endl;

    // MONTE CARLO
    std::size_t numTrials = 500;
    int masterSeed = 500;
    MonteCarlo mc(numTrials, masterSeed);
    ModelConfig modelConfig{F, Q, H, R,sigmaz};
    ICConfig ICconfig{x_true_0, P, x_est_0};
    SimConfig simConfig{dt, simTime};
    MonteCarloResults mcResults = mc.run(modelConfig, ICconfig, simConfig);

    std::cout << "Mean NEES: " << mcResults.meanNEES << std::endl;
    std::cout << "Mean NIS: " << mcResults.meanNIS << std::endl;
    return 0;
    // TIP See CLion help at <a href="https://www.jetbrains.com/help/clion/">jetbrains.com/help/clion/</a>. Also, you can try interactive lessons for CLion by selecting 'Help | Learn IDE Features' from the main menu.
}