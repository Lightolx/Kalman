//
// Created by lightol on 2019/9/3.
//
#include <iostream>
#include <utility>

#include "KalmanFilter.h"

using std::cout;
using std::endl;

KalmanFilter::KalmanFilter(Eigen::Matrix3d _A,
                           Eigen::Matrix2d _B0,
                           Eigen::Matrix3d _H,
                           Eigen::Matrix3d _Q,
                           Eigen::Matrix3d _P,
                           Eigen::Matrix3d _R,
                           double _dt) : A(std::move(_A)), B0(std::move(_B0)), H(std::move(_H)), Q(std::move(_Q)), P(std::move(_P)), R(std::move(_R)), dt(_dt) {
    cout << "in initialize, P is\n" << P << endl;

}

void KalmanFilter::Initilize(const Eigen::Vector3d &initial_state) {
    x = initial_state;
    mbInitialized = true;
}

void KalmanFilter::Update(const Eigen::Vector3d &measurement, const Eigen::Vector2d &u) {
    Eigen::Matrix<double, 3, 2> B = Eigen::Matrix<double, 3, 2>::Zero();
    B(0, 0) = std::cos(x.z());
    B(1, 0) = std::sin(x.z());
    B(2, 1) = 1;

    Eigen::Vector3d x_prior = A*x + B*B0*u;                // 先验状态预测
//    cout << "x_prior = " << x_prior.transpose() << endl;
//    cout << "P_k-1.posterior = \n " << P << endl;
    Eigen::Matrix3d P_prior = A*P*A.transpose() + Q;    // 先验状态协方差矩阵预测
    cout << "P_prior = \n" << P_prior << endl;
//    cout << "H = \n" << H << endl;
//    cout << "R = \n" << R << endl;
//    cout << "(H * P_prior * H.transpose() + R) = \n" << (H * P_prior * H.transpose() + R) << endl;
//    cout << "(H * P_prior * H.transpose() + R).inverse() = \n" << (H * P_prior * H.transpose() + R).inverse() << endl;
    // 计算卡尔曼增益
    const Eigen::Matrix3d K = P_prior * H.transpose() * (H * P_prior * H.transpose() + R).inverse();
//    cout << "K = \n" << K << endl;
    // 求解当前后验状态
    x = x_prior + K * (measurement - H * x_prior);
//    cout << "x_posterior = " << x.transpose() << endl;
    // 求解当前后验概率分布
    P = (I - K * H) * P_prior;
    cout << "P_posterior = \n" << P << endl << endl;
}

Eigen::Vector3d KalmanFilter::State() const {
    return x;
}
