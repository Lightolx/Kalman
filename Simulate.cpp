#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Eigen>

#include "KalmanFilter.h"

using std::cout;
using std::endl;

int main() {
    /*
    int n = 3;  // Number of states,        (x, y, theta),      car在平面上，所以只有3自由度
    int m = 3;  // Number of measurements   (x, y, theta),      mapmatching可以直接给出对这3个量的测量
    int l = 3;  // Number of controls       (dx, dy, dtheta),   轮速计可以给出对线速度及角速度的测量

    double dt = 1.0;   // 假设轮速计的输入频率是1Hz

    // 首先做一个简易版本，假设是匀速运动模型（匀角速度匀线速度），在绕弯
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();        // System dynamic matrix,  状态更新方程，也就是预测模型
    Eigen::Matrix2d B0 = Eigen::Matrix2d::Identity()*dt;    // Control dynamic matrix, 控制量转移方程，也就是控制模型，目的是为了把控制量转化为状态量，就像观测模型把观测量转化为状态量
    // 这里的控制量是线速度与角速度，很明显，只需要乘以dt就能把速度转化为位移，也就是当前的position与heading。当然这只是第一步，是在car坐标系下的状态变化量，还需要另一个状态转换方程变换到世界坐标系下
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();        // Output matrix,          状态转移方程，也就是观测模型，最典型的就是针孔相机模型
    Eigen::Matrix3d Q = 0.6*Eigen::Matrix3d::Identity();    // State noise covariance, 预测模型引入的高斯噪声的协方差矩阵
    Eigen::Matrix3d R = 0.8*Eigen::Matrix3d::Identity();    // Measurement noise covariance, 观测模型引入的高斯噪声协方差矩阵
    R(2, 2) = 0.1 * R(2, 2);
    Eigen::Matrix3d P = 10*Eigen::Matrix3d::Identity();     // Estimate error covariance, 当前状态的协方差矩阵
    P(2, 2) = 0.1 * P(2, 2);

//    Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
//    R << .05, .05, .0, .05, .05, .0, .0, .0, .0;
//    P << .1, .1, .1, .1, 100, 10, .1, 10, 100;

//    cout << "A:\n" << A << endl;
//    cout << "B:\n" << B0 << endl;
//    cout << "H:\n" << H << endl;
//    cout << "Q:\n" << Q << endl;
//    cout << "R:\n" << R << endl;
//    cout << "P:\n" << P << endl;


    KalmanFilter kf(A, B0, H, Q, P, R, dt);

    int N = 100;
    std::vector<Eigen::Vector2d> Us(N);
    for (int i = 0; i < N; ++i) {
        Us[i] = Eigen::Vector2d(7.2, 0.04);   // 假设car以1.2m/s的线速度， 0.01rad/s的角速度匀速运动，实际轨迹是个圆
    }

    Eigen::Vector3d initial_state = Eigen::Vector3d::Zero();
    kf.Initilize(initial_state);

    // 生成观测数据，同时添加高斯白噪声
    double mean = 0.0;  // 均值
    double sigma = 1; // 标准差
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean, sigma);
    std::vector<Eigen::Vector3d> vPredictions(N);
    std::vector<Eigen::Vector3d> vGTs(N);
    std::vector<Eigen::Vector3d> vMeasures(N);


    for (int i = 1; i < N; ++i) {
        // B矩阵从车体坐标系转换到世界坐标系
        Eigen::Matrix<double, 3, 2> B = Eigen::Matrix<double, 3, 2>::Zero();
        double theta_prediction = vPredictions[i-1].z();
        B(0, 0) = std::cos(theta_prediction);
        B(1, 0) = std::sin(theta_prediction);
        B(2, 1) = 1;
        vPredictions[i] = A * vPredictions[i-1] + B * B0 * Us[i];

        double theta_GT = vGTs[i-1].z();
//        double theta_GT = vGTs[i-1].z() + 0.12 * distribution(generator);
        B(0, 0) = std::cos(theta_GT);
        B(1, 0) = std::sin(theta_GT);
        // 注意在最后的那个噪声项是直接加在世界坐标系下的，因为我们假设B*u能够足够说明控制系统的影响，u并没有呈正态分布的误差，状态更新加上控制系统的误差通通反映到最后的一个标准正态误差分布上
        vGTs[i] = A * vGTs[i-1] + B * B0 * Us[i] + 0.6*Eigen::Vector3d(distribution(generator), distribution(generator), 0.1*distribution(generator));

        vMeasures[i] = vGTs[i] + 0.8*Eigen::Vector3d(distribution(generator), distribution(generator), 0.1*distribution(generator));
    }

    // X = A*X + B*u
    // Feed measurements into filter, output estimated states
    double t = 0.0;
    std::ofstream fout_pre("/home/lightol/Programs/MatLab/scripts/kalman/pres.txt");
    std::ofstream fout_gt("/home/lightol/Programs/MatLab/scripts/kalman/gts.txt");
    std::ofstream fout_measure("/home/lightol/Programs/MatLab/scripts/kalman/measurements.txt");
    std::ofstream fout_filter("/home/lightol/Programs/MatLab/scripts/kalman/filters.txt");
    for (int i = 0; i < N; ++i) {
//        t += dt;
        Eigen::Vector3d measure = vMeasures[i];
        kf.Update(measure, Us[i]);
        Eigen::Vector3d X = kf.State();
        fout_filter << X.x() << " " << X.y() << " " << X.z() << endl;
//        cout << "\nt = " << t << endl;
//        cout << "measurements = " << measure.transpose() << endl;
        fout_pre << vPredictions[i].x() << " " << vPredictions[i].y() << " " << vPredictions[i].z() << endl;
        fout_gt << vGTs[i].x() << " " << vGTs[i].y() << " " << vGTs[i].z() << endl;
        fout_measure << vMeasures[i].x() << " " << vMeasures[i].y() << " " << vMeasures[i].z() << endl;
//        cout << "state = " << kf.state().transpose() << endl;
    }
    fout_filter.close();
    fout_pre.close();
    fout_gt.close();
    fout_measure.close();
     */
}