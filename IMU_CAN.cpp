//
// Created by lightol on 2019/12/31.
//

// 把Novatel输出的imu的pose通过外参转化成CAN的pose
#include <eigen3/Eigen/Eigen>
#include <iomanip>
#include "Common.h"

int main() {
    std::vector<std::pair<double, Eigen::Matrix4d> > vpTsPose = LoadNovatelTsPosition("../data/NovAtel-position.txt");

    // Step1: 通过IMU在世界系下的pose得到CAN在世界系下的pose
    Eigen::Matrix4d Tcar_can = Eigen::Matrix4d::Identity();
    Tcar_can << 1.0, 0  , 0  , -1.425,
                0  , 1.0, 0  , 0,
                0  , 0  , 1.0, 0,
                0  , 0  , 0  , 1.0;

    Eigen::Matrix4d Tcar_imu = Eigen::Matrix4d::Identity();
    Tcar_imu << 1.7453292519057202e-05, 0.9999999998476913    , 0.0 , -1.7,
                -0.9999999998476913   , 1.7453292519057202e-05, 0.0 , 0   ,
                0.0                   , 0.0                   , 1.0 , 0   ,
                0                     , 0                     , 0   , 1;
    Eigen::Matrix4d Timu_car = Tcar_imu.inverse();

    std::vector<Eigen::Matrix4d> vPoseIMU = LoadNovatelPosition("../data/NovAtel-position.txt");
    int numPoses = vPoseIMU.size();
    std::vector<Eigen::Matrix4d> vPoseCAN(numPoses, Eigen::Matrix4d::Identity());
    for (int i = 0; i < numPoses; ++i) {
        vPoseCAN[i] = vPoseIMU[i] * Timu_car * Tcar_can;
    }

    // Step2: 把CAN的pose转到第一帧的坐标系下
    Eigen::Matrix4d Twc0 = vPoseCAN[0];
    Eigen::Matrix4d Tcw0 = Twc0.inverse();
    std::ofstream fout("/home/lightol/Programs/MATLAB/scripts/kalman/can_pose_gt.txt");
    for (int i = 0; i < numPoses; ++i) {
        const auto &Twc = vPoseCAN[i];
        Eigen::Matrix4d Twc1 = Tcw0 * Twc;  // 从世界系转到第一帧的CAN坐标系
        // 当前坐标系的原点在第一帧坐标系下的position
        Eigen::Vector3d Oc = Twc1.topRightCorner(3, 1);
        // 当前坐标系的x轴在第一帧坐标系下的方向向量
        Eigen::Vector3d x_axis = Twc1.topLeftCorner(3, 1);
        double theta = std::asin(Eigen::Vector3d::UnitX().cross(x_axis).dot(Eigen::Vector3d::UnitZ()));
        double ts = vpTsPose[i].first;
        fout << std::fixed << std::setprecision(6) << ts << " " << Oc.x() << " " << Oc.y() << " " << theta << std::endl;
    }


    fout.close();

}