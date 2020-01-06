//
// Created by lightol on 2019/9/3.
//
#include <iostream>
#include <utility>

#include <glog/logging.h>

#include "KalmanFilter.h"

using std::cout;
using std::endl;

KalmanFilter::KalmanFilter(Eigen::Matrix3d _A,
                           Eigen::Matrix3d _H,
                           Eigen::Matrix2d _Q,
                           Eigen::Matrix3d _R,
                           Eigen::Matrix3d _P) : A(std::move(_A)), H(std::move(_H)), Q(std::move(_Q)), R(std::move(_R)), P(std::move(_P)) {
//    LOG(INFO) << "in initialize, P is\n" << P << endl;

}

void KalmanFilter::Initilize(const Eigen::Vector3d &initial_state) {
    x = initial_state;
}

void KalmanFilter::Update(const Eigen::Vector3d &measurement, const Eigen::Vector2d &u) {
    Eigen::Matrix<double, 3, 2> B = Eigen::Matrix<double, 3, 2>::Zero();
    B(0, 0) = std::cos(x.z());
    B(1, 0) = std::sin(x.z());
    B(2, 1) = 1;

    Eigen::Vector3d x_prior = A*x + B*u;                // 先验状态预测
    // x对x的雅各比
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = -u[0] * std::sin(x.z());
    F(1, 2) = -u[0] * std::cos(x.z());
    // x对u的雅各比, 也就是B

    Eigen::Matrix3d P_prior = F*P*F.transpose() + B*Q*B.transpose();    // 先验状态协方差矩阵预测
    // 计算卡尔曼增益
    const Eigen::Matrix3d K = P_prior * H.transpose() * (H * P_prior * H.transpose() + R).inverse();
    // 求解当前后验状态
//    double theta = x_prior.z();
//    if (theta > 0) {  // 在pi附近的一个数
//        if (fabs(measurement.z() - theta) < fabs(measurement.z() - (theta - 2 * M_PI))) {
//            // do nothing, 可能情况是 theta = 1.02*pi, measurement.z() = 0.99pi
//            x_prior.z() = theta;
//        } else {
//            // 可能情况是 theta = 1.02*pi, measurement.z() = -0.99pi
//            x_prior.z() = theta - 2 * M_PI;
//        }
//    } else if (theta < 0) {
//        if (fabs(measurement.z() - theta) < fabs(measurement.z() - (theta + 2 * M_PI))) {
//            // do nothing, 可能情况是 theta = -1.02*pi, measurement.z() = -0.99pi
//            x_prior.z() = theta;
//        } else {
//            // 可能情况是 theta = -1.02*pi, measurement.z() = 0.99pi
//            x_prior.z() = theta + 2 * M_PI;
//        }
//    }

    x = x_prior + K * (measurement - H * x_prior);
//    LOG(INFO) << "K = " << K;
//    LOG(INFO) << "measurement is " << measurement;
//    LOG(INFO) << "x_prior = " << x_prior.transpose();
//    LOG(INFO) << "x_post = " << x.transpose();
    // 求解当前后验概率分布
    P = (I - K * H) * P_prior;

    // todo::这里注意x_prior是累加，角度可能超过2pi，而measurement是一次测量，所以角度不会超过2pi.
    //  因此x_prior和measurement之间会有2Pi的偏差，所以每次滤波完成都把theta归一化到[-pi, pi]之间
//    theta = x.z();
//    double alpha = theta + M_PI;  // theta归一化到[-pi, pi],等价于alpha = theta+pi归一化到[0, 2*pi]之间
//    int k = std::floor(alpha / (2*M_PI));
//    alpha -= k * (2*M_PI);
//    theta = alpha - M_PI;
//    x.z() = theta;

//    cout << "P_posterior = \n" << P << endl << endl;
}

Eigen::Vector3d KalmanFilter::State() const {
    return x;
}

void KalmanFilter::Accumulate(const Eigen::Vector2d &u) {
    Eigen::Matrix<double, 3, 2> B = Eigen::Matrix<double, 3, 2>::Zero();
    B(0, 0) = std::cos(x.z());
    B(1, 0) = std::sin(x.z());
    B(2, 1) = 1;

    Eigen::Vector3d x_prior = A*x + B*u;                // 先验状态预测
    // x对x的雅各比
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = -u[0] * std::sin(x.z());
    F(1, 2) = -u[0] * std::cos(x.z());
    // x对u的雅各比, 也就是B
    Eigen::Matrix3d P_prior = F*P*F.transpose() + B*Q*B.transpose();    // 先验状态协方差矩阵预测

    x = x_prior;    // 因为没有后端，所以当前帧的状态只取决于前端的预测
    P = P_prior;

    double theta = x.z();
    int k = std::floor(theta / (2*M_PI));
    x.z() += k * (2*M_PI);
}
