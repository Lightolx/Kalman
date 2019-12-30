//
// Created by lightol on 2019/12/30.
//

#include <iostream>
#include <random>
#include <fstream>
#include <eigen3/Eigen/Eigen>

using std::cout;
using std::endl;

// 读取Novatel输出的绝对轨迹，与WO的积分结果做对比
std::vector<Eigen::Matrix4d> LoadNovatelPosition(const std::string& filename) {
    std::ifstream fin(filename);
    std::string line;
    std::string ptline;
    Eigen::Matrix4d tempTwc = Eigen::Matrix4d::Identity();
    std::vector<Eigen::Matrix4d> vTwcs;
    while (getline(fin, line)) {
        ptline = line.substr(line.find_first_of(" ") + 1);
        std::stringstream ss(ptline);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                ss >> tempTwc(i, j);
            }
        }

        vTwcs.push_back(tempTwc);
    }
    fin.close();

    return vTwcs;
}

// 轮速计的输出数据，包括线速度、角速度与当前时间戳，在这里为了偷懒直接用一个Eigen::Vector3d表示
std::vector<Eigen::Vector3d> LoadWOdata(const std::string& filename) {
    std::ifstream fin(filename);
    std::string line;
    std::vector<Eigen::Vector3d> vMeasurements;
    Eigen::Vector3d measurement = Eigen::Vector3d::Zero();
    while (getline(fin, line)) {
        std::stringstream ss(line);

        for (int i = 0; i < 3; ++i) {
            ss >> measurement(i);
        }

        vMeasurements.push_back(measurement);
    }
    fin.close();

    return vMeasurements;
}

int main() {
    std::vector<Eigen::Vector3d> vMeasurements = LoadWOdata("../data/wo.txt");

    // System dynamic matrix,  状态更新方程，也就是预测模型，我们假设A是单位阵，也就是车本身是没动的，是轮子拖着它在走，也就是有控制输入
    Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
    // Control dynamic matrix, 控制量转移方程，也就是控制模型，目的是为了把控制量转化为状态量，就像观测模型把观测量转化为状态量
    // 这里对轮速的积分只选用一个线性模型，所以就是一个单位阵
    Eigen::Matrix2d B0 = Eigen::Matrix2d::Identity();

    int numMeasurements = vMeasurements.size();
    // 记录在每次CAN数据输出时车的3自由度pose
    std::vector<Eigen::Vector3d> vPositions(numMeasurements, Eigen::Vector3d::Zero());
    vPositions[0] = Eigen::Vector3d::Zero();    // 把第一帧设为坐标原点
    for (int i = 1; i < numMeasurements; ++i) {
        double v = vMeasurements[i-1].x();  // 上一次测量时的线速度
        double w = vMeasurements[i-1].y();  // 上一次测量时的角速度
        // 从上一次测量到当前帧的时间间隔
        double dt = vMeasurements[i].z() - vMeasurements[i-1].z();

        Eigen::Matrix<double, 3, 2> B = Eigen::Matrix<double, 3, 2>::Zero();
        Eigen::Vector3d pose = vPositions[i-1];  // 上一次测量时的car pose
        double theta = pose.z();                 // 上一次测量时车的heading角
        B(0, 0) = std::cos(theta);
        B(1, 0) = std::sin(theta);
        B(2, 1) = 1;

        vPositions[i] = A * pose + B * B0 * Eigen::Vector2d(v, w) * dt;
    }

    std::ofstream fout("/home/lightol/Programs/MATLAB/scripts/kalman/wo_pose.txt");
    for (int i = 0; i < numMeasurements; ++i) {
        Eigen::Vector3d pose = vPositions[i];
        fout << pose.x() << " " << pose.y() << " " << pose.z() << endl;
    }
    fout.close();

    std::vector<Eigen::Matrix4d> vPositionsGt = LoadNovatelPosition("../data/novatel.txt");
    Eigen::Matrix4d Twc0 = vPositionsGt[0];
    Eigen::Matrix4d Tcw0 = Twc0.inverse();
    fout.open("/home/lightol/Programs/MATLAB/scripts/kalman/gt_pose.txt");
    Eigen::Matrix4d R_T = Eigen::Matrix4d::Identity();
    R_T.topLeftCorner(3, 1) = -Eigen::Vector3d::UnitY();
    R_T.col(1).topRows(3) = Eigen::Vector3d::UnitX();
    for (const auto &Twc: vPositionsGt) {
        Eigen::Matrix4d Twc1 = Tcw0 * Twc;  // 从世界系转到第一帧的IMU坐标系
        Eigen::Matrix4d Twc2 = R_T * Twc1;  // imu坐标系定义的是右前上，转到轮速计定义的前左上
        Eigen::Vector3d Oc = Twc2.topRightCorner(3, 1);

        fout << Oc.x() << " " << Oc.y() << " " << Oc.z() << endl;
    }
}