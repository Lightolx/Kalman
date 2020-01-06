//
// Created by lightol on 2019/12/30.
//
#include <glog/logging.h>
#include <iomanip>

#include "Common.h"

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

std::vector<std::pair<double, Eigen::Matrix4d> > LoadNovatelTsPosition(const std::string &filename) {
    std::ifstream fin(filename);
    std::string line;
    std::string ptline;
    Eigen::Matrix4d tempTwc = Eigen::Matrix4d::Identity();
    std::vector<std::pair<double, Eigen::Matrix4d> > vpTsTwcs;
    while (getline(fin, line)) {
        // 读取pose
        ptline = line.substr(line.find_first_of(" ") + 1);
        std::stringstream ss(ptline);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                ss >> tempTwc(i, j);
            }
        }

        ptline = line.substr(0, line.find_first_of(" "));
//        LOG(INFO) << ptline;
//        LOG(INFO) << STDTimer::FormatSTDTimeString(ptline);
        long long gpstimestamp = 0;
        STDTimer::timeString2timestamp(STDTimer::FormatSTDTimeString(ptline),
                                       gpstimestamp);
        double ts = gpstimestamp / 1000.0 ;


        vpTsTwcs.emplace_back(ts, tempTwc);
    }
    fin.close();

    return vpTsTwcs;
}

std::queue<Eigen::Vector4d> LoadNovatelData(const std::string& filename) {
    std::ifstream fin(filename);
    std::string line;
    std::queue<Eigen::Vector4d> qMeasurements;
    Eigen::Vector4d measurement = Eigen::Vector4d::Zero();
    while (getline(fin, line)) {
        std::stringstream ss(line);

        for (int i = 0; i < 4; ++i) {
            ss >> measurement(i);
        }

        qMeasurements.push(measurement);
    }
    fin.close();

    return qMeasurements;
}