//
// Created by lightol on 2019/12/30.
//

#ifndef KALMAN_COMMON_H
#define KALMAN_COMMON_H

#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <queue>

#include "timer.h"

std::vector<Eigen::Matrix4d> LoadNovatelPosition(const std::string& filename);

// 同时读取pose和时间戳
std::vector<std::pair<double, Eigen::Matrix4d> > LoadNovatelTsPosition(const std::string &filename);

std::vector<Eigen::Vector3d> LoadWOdata(const std::string& filename);

std::queue<Eigen::Vector4d> LoadNovatelData(const std::string& filename);


#endif //KALMAN_COMMON_H
