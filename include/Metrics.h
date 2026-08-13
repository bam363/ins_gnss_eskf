//
// Created by Barin A. Moghimi on 8/10/26.
//

#ifndef ESTIMATION_LAB_METRICS_H
#define ESTIMATION_LAB_METRICS_H
#include <Eigen/Dense>
#include <vector>

class Metrics {
public:
    Metrics() = default;

    double computeNEES(const std::vector<Eigen::Vector2d>& x,
                       const std::vector<Eigen::Vector2d>& xh,
                       const std::vector<Eigen::Matrix2d>& Ph);

    double computeNIS(const std::vector<double>& v,
                      const std::vector<double>& S);

};


// calculate errors
// std::vector<double> ep;
// std::vector<double> ev;
// std::vector<double> three_sigma_p;
// std::vector<double> three_sigma_v;
//
// ep.reserve(N);
// ev.reserve(N);
// three_sigma_p.reserve(N);
// three_sigma_v.reserve(N);

// for (int k = 0; k < N; ++k) {
//
//     Eigen::Vector2d e = x[k] - xh[k];
//
//     ep.push_back(e(0));
//     ev.push_back(e(1));
//
//     three_sigma_p.push_back(3.0 * std::sqrt(Ph[k](0,0)));
//     three_sigma_v.push_back(3.0 * std::sqrt(Ph[k](1,1)));
// }

// int p_violations = 0;
// int v_violations = 0;
//
// for (int k = 0; k < N; ++k) {
//
//     if (std::abs(ep[k]) > three_sigma_p[k])
//         ++p_violations;
//
//     if (std::abs(ev[k]) > three_sigma_v[k])
//         ++v_violations;
// }

#endif //ESTIMATION_LAB_METRICS_H
