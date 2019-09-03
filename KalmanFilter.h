//
// Created by lightol on 2019/9/3.
//

#ifndef KALMAN_KALMANFILTER_H
#define KALMAN_KALMANFILTER_H


#include <eigen3/Eigen/Eigen>

class KalmanFilter {
public:
    KalmanFilter(Eigen::Matrix3d A,
                 Eigen::Matrix2d B,
                 Eigen::Matrix3d H,
                 Eigen::Matrix3d Q,
                 Eigen::Matrix3d P,
                 Eigen::Matrix3d R,
                 double t);

    void Initilize(const Eigen::Vector3d &initial_state);

    Eigen::Vector3d State() const;

    void Update(const Eigen::Vector3d &measurement, const Eigen::Vector2d &u);

private:
    const Eigen::Matrix3d A;
    const Eigen::Matrix2d B0;
    const Eigen::Matrix3d H;
    const Eigen::Matrix3d Q;
    const Eigen::Matrix3d R;

    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    Eigen::Vector3d x;  // 当前的状态
    Eigen::Matrix3d P;  // 当前状态的协方差矩阵

    // discrete time step, 相当于做MapMatching的频率
    const double dt;

    // Is the filter initialized?
    bool mbInitialized = false;
};


#endif //KALMAN_KALMANFILTER_H
