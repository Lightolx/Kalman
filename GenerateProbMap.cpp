//
// Created by lightol on 2019/9/9.
//

#include <iostream>
#include <random>
#include <fstream>

using std::cout;
using std::endl;

int main() {
    double arr[100][100];
    for (int i = 0; i < 100; ++i) {
        for (int j = 0; j < 100; ++j) {
            arr[i][j] = 0;
        }
    }

    double mean = 0.5;  // 均值
    double sigma = 1; // 标准差
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(mean, sigma);
    for (int i = 20; i < 100; ++i) {
        for (int j = 5; j < 15; ++j) {
            arr[i][j] = 1.0 - 0.01*(rand() % 10 * std::fabs(j - 10)) ;
        }
    }

    for (int i = 30; i < 50; ++i) {
        for (int j = 40; j < 45; ++j) {
            arr[i][j] = 1.0 - 0.02*(rand() % 10 * std::fabs(j - 42)) ;
        }
    }

    for (int i = 60; i < 64; ++i) {
        for (int j = 70; j < 72; ++j) {
            arr[i][j] = 1.0 - 0.05*(rand() % 10 * std::fabs(j - 71)) ;
        }
    }

    std::ofstream fout("/home/lightol/Programs/MatLab/scripts/SemanticMap/pts.txt");
    for (int i = 0; i < 100; ++i) {
        for (int j = 0; j < 100; ++j) {
            fout << arr[i][j] << " ";
        }
        fout << endl;
    }
    fout.close();
}