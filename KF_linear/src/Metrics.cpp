//
// Created by Barin A. Moghimi on 8/10/26.
//
#include "../include/Metrics.h"

#include <numeric>

double Metrics::computePerRunNEES(const std::vector<Eigen::Vector3d>& x,
                         const std::vector<Eigen::Vector3d>& xh,
                         const std::vector<Eigen::Matrix3d>& Ph)
{
    // calculate NEES
    assert(x.size() == xh.size() && xh.size() == Ph.size());
    const std::size_t N = x.size();
    std::vector<double> nees;
    nees.reserve(N);

    for (int k = 0; k < N; ++k) {
        Eigen::Vector3d e = x[k] - xh[k];

        double epsilon =
            e.dot(Ph[k].ldlt().solve(e));

        nees.push_back(epsilon);
    }

    double mean_nees = 0.0;

    for (double value : nees)
        mean_nees += value;

    mean_nees /= nees.size();

    return mean_nees;
}

std::vector<double> Metrics::computePerSampleNEES(const std::vector<Eigen::Vector3d>& x,
                                     const std::vector<Eigen::Vector3d>& xh,
                                     const std::vector<Eigen::Matrix3d>& Ph) {

    std::vector<double> eps;
    eps.reserve(x.size());

    for (std::size_t j = 0; j < x.size(); j++) {
        const auto ej = x[j] - xh[j];
        const auto epsj = ej.dot(Ph[j].ldlt().solve(ej));
        eps.push_back(epsj);
    };

    return eps;
}

std::vector<double> Metrics::computePerSamplePerStateNEES(const std::vector<Eigen::Vector3d>& x,
                                     const std::vector<Eigen::Vector3d>& xh,
                                     const std::vector<Eigen::Matrix3d>& Ph, int stateID) {

    std::vector<double> eps;
    eps.reserve(x.size());

    for (std::size_t j = 0; j < x.size(); j++) {
        const auto ej = x[j][stateID] - xh[j][stateID];
        const auto epsj = ej * ej / Ph[j](stateID, stateID);
        eps.push_back(epsj);
    };

    return eps;
}

std::vector<Eigen::Vector3d> Metrics::computePerSampleError(const std::vector<Eigen::Vector3d>& x,
                                                            const std::vector<Eigen::Vector3d>& xh) {

    std::vector<Eigen::Vector3d> eps;
    eps.reserve(x.size());

    for (std::size_t j = 0; j < x.size(); j++) {
        const auto ej = x[j] - xh[j];
        eps.push_back(ej);
    };

    return eps;
}

double Metrics::computeMean(const std::vector<double>& x) {
    return  std::accumulate(x.begin(),x.end(),0.0) / x.size();
}

double Metrics::computeMax(const std::vector<double>& x) {
    return *std::max(x.begin(), x.end());
}

double Metrics::computeMin(const std::vector<double>& x) {
    return *std::min(x.begin(), x.end());
}

double Metrics::computeStd(const std::vector<double>& x, double mu) {
    double sumOfSquares = 0.0;
    for (std::size_t i = 0; i < x.size(); i++) {
        sumOfSquares += (x[i] - mu) * (x[i] - mu);
    }
    double stdNEES = std::sqrt(sumOfSquares / (x.size() - 1));

    return stdNEES;
}

double Metrics::computeNIS(const std::vector<double> &v,
                           const std::vector<double> &S)
{
    // Compute NIS
    assert(v.size() == S.size());
    const std::size_t N = v.size();
    std::vector<double> nis;
    nis.reserve(N);

    for (std::size_t k = 0; k < N; ++k) {
        double epsilon =
            v[k] * v[k] / S[k];

        nis.push_back(epsilon);
    }

    double mean_nis = 0.0;

    for (double value : nis)
        mean_nis += value;

    mean_nis /= nis.size();

    return mean_nis;
}
