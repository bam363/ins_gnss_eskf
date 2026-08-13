//
// Created by Barin A. Moghimi on 8/10/26.
//
#include "Metrics.h"

double Metrics::computeNEES(const std::vector<Eigen::Vector2d>& x,
                            const std::vector<Eigen::Vector2d>& xh,
                            const std::vector<Eigen::Matrix2d>& Ph)
{
    // calculate NEES
    assert(x.size() == xh.size() && xh.size() == Ph.size());
    const std::size_t N = x.size();
    std::vector<double> nees;
    nees.reserve(N);

    for (int k = 0; k < N; ++k) {
        Eigen::Vector2d e = x[k] - xh[k];

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
