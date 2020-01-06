//
// Created by lightol on 2019/9/3.
//

#ifndef KALMAN_KALMANFILTER_H
#define KALMAN_KALMANFILTER_H


#include <eigen3/Eigen/Eigen>

class KalmanFilter {
public:
    KalmanFilter(Eigen::Matrix3d A,
                 Eigen::Matrix3d H,
                 Eigen::Matrix2d Q,
                 Eigen::Matrix3d P,
                 Eigen::Matrix3d R);

    void Initilize(const Eigen::Vector3d &initial_state);

    Eigen::Vector3d State() const;

    void Update(const Eigen::Vector3d &measurement, const Eigen::Vector2d &u);

    // 如果当前没有后端观测，那么只能对前端做积分
    void Accumulate(const Eigen::Vector2d &u);

private:
    const Eigen::Matrix3d A;
    const Eigen::Matrix3d H;
    const Eigen::Matrix2d Q;
    const Eigen::Matrix3d R;

    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    Eigen::Vector3d x;  // 当前的状态
    Eigen::Matrix3d P;  // 当前状态的协方差矩阵
};


#endif //KALMAN_KALMANFILTER_H
