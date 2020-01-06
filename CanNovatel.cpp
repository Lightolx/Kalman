//
// Created by lightol on 2019/12/30.
//

// 对Can + Novatel做卡尔曼滤波，为了简便起见，只对car进行3自由度的估计

#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <iomanip>

#include "Common.h"
#include "KalmanFilter.h"

using std::cout;
using std::endl;

int main() {
    int n = 3;  // Number of states,        (x, y, theta),      car在平面上，所以只有3自由度
    int m = 3;  // Number of measurements   (x, y, theta),      mapmatching可以直接给出对这3个量的测量
    int l = 3;  // Number of controls       (dx, dy, dtheta),   轮速计可以给出对线速度及角速度的测量

    // System dynamic matrix,  状态更新方程，也就是预测模型的前一项(后一项是控制输入), X = A*X + B*u + noise, 这才是完整的预测模型
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();     // 这里假设车本身是不会动的，是轮子拖着它在动,所以是单位阵

    // Control dynamic matrix, 控制量转移方程，也就是控制模型，目的是为了把控制量转化为状态量，就像观测模型把观测量转化为状态量
    // 这里的控制量是线速度与角速度，很明显，只需要乘以dt就能把速度转化为位移，也就是当前的position与heading，
    // 当然这只是第一步，是在car坐标系下的状态变化量，还需要另一个状态转换方程变换到世界坐标系下
    Eigen::Matrix<double, 3, 2> B = Eigen::Matrix<double, 3, 2>::Zero();

    // Output matrix, 状态转移方程，也就是观测模型，最典型的就是针孔相机模型
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();  // 因为MapMatching直接输出对3个自由度的观测，所以是单位阵

    // State noise covariance, 预测模型引入的高斯噪声的协方差矩阵,状态转移和控制输入两个一起引入的噪声,但是这里只有控制输入会引入噪声,所以是2维的,
    // 也就是对线速度与角速度的噪声评估
    Eigen::Matrix2d Q = Eigen::Matrix2d::Identity();

    // Measurement noise covariance, 观测模型引入的高斯噪声协方差矩阵
    Eigen::Matrix3d R = 25 * Eigen::Matrix3d::Identity();
//    R(2, 2) = 0.1 * R(2, 2);

    // Estimate error covariance, 当前状态的协方差矩阵
    Eigen::Matrix3d P = Eigen::Matrix3d::Identity();
//    P(2, 2) = 0.1 * P(2, 2);

    KalmanFilter kf(A, H, Q, R, P);

    Eigen::Vector3d initial_state = Eigen::Vector3d::Zero();
    kf.Initilize(initial_state);

    std::vector<Eigen::Vector3d> vPredictions = LoadWOdata("../data/wo.txt");
    int numPredictions = vPredictions.size();   // 轮速计的频率会高很多
    std::queue<Eigen::Vector4d> qMeasurements = LoadNovatelData("../data/novatel.txt");

    // 把CAN和novatel的初始时间对齐，找出第一个有效的novatel的输出
    // 这里假设Novatel先打开
    double CAN1_Ts = vPredictions[1].z();  // 第2帧CAN的时间戳，因为第1帧在坐标原点，无需融合矫正
    Eigen::Vector4d curMeasure = Eigen::Vector4d::Zero();
    double curNovatelTs = 0;
    while (1) {
        curMeasure = qMeasurements.front();
        curNovatelTs = curMeasure.x();

//        LOG(INFO) << curNovatelTs << ", " << CAN1_Ts;
//        LOG(INFO) << fabs(curNovatelTs - CAN1_Ts);
        if (fabs(curNovatelTs - CAN1_Ts) <= 0.005) {
//            LOG(INFO) << std::fixed << std::setprecision(3) << curNovatelTs;
            break;
        }

        qMeasurements.pop();
    }


    // Feed measurements into filter, output estimated states
    std::ofstream fout("../data/can_pose_filter.txt");
//    LOG(INFO) << "numPre = " << numPredictions;
    for (int i = 1; i < numPredictions; ++i) {
        Eigen::Vector3d prediction = vPredictions[i];
        double v = prediction.x();
        double w = prediction.y();
        double t0 = vPredictions[i-1].z();
        v = (v + 0.0) * 0.956211;
        w = (w + 0.000852) * 1.090923;
        double t1 = vPredictions[i].z();
        Eigen::Vector2d u = Eigen::Vector2d(v, w) * (t1 - t0);  // 最简单的线性积分，速度成时间

        // 如果当前有novatel输出的观测值就做前后端融合，否则只是单纯地轮速计积分
//        LOG(INFO) << fabs(t1 - curNovatelTs);
        while(t1 - curNovatelTs > 0.005) {  // 理论上轮速计是滞后的，如果轮速计反而超前了，就让novatel空跑一会
            qMeasurements.pop();
            curMeasure = qMeasurements.front();
            curNovatelTs = curMeasure.x();
        }

        if (fabs(t1 - curNovatelTs) <= 0.005) {  // 5ms以内
//            if(0) {
//            LOG(INFO) << i;
//            LOG(INFO) << std::fixed << std::setprecision(3) << t1 << ", " << curNovatelTs;
            Eigen::Vector3d measure = curMeasure.bottomRows(3);
            Eigen::Vector3d noise = 0.2*Eigen::Vector3d::Random();
            noise.z() *= 0.4;
            measure += noise;
            kf.Update(measure, u);  // 假设novatel的数据是实时传输给CAN的，所以不需要传递时间戳

            // 更新最新的novatel的输入数据
            qMeasurements.pop();
            curMeasure = qMeasurements.front();
            curNovatelTs = curMeasure.x();



        } else {
            kf.Accumulate(u);
        }

        Eigen::Vector3d X = kf.State();
        fout << X.x() << " " << X.y() << endl;

    }
    fout.close();
}