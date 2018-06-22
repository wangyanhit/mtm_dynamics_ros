//
// Created by yan on 6/13/18.
//

#ifndef PROJECT_IMU_H
#define PROJECT_IMU_H

#include <Eigen/Core>
#include <Eigen/Geometry>

// This class gets gyro, acceleration and attitude from Gazabo with some Gaussian noise, and then outputs raw data of
// the gyro and accelerometer with bias in gyro and acceleration and gravity terms in acceleration to simulate the real
// IMU.

class IMU
{
public:
    IMU(): acc_raw(Eigen::Vector3d::Zero()), acc_linear(Eigen::Vector3d::Zero()), gyro_raw(Eigen::Vector3d::Zero()) {};
    void set_acc(double x, double y, double z);
    void set_gyro(double x, double y, double z);
    void set_atti(double x, double y, double z, double w);
    void update();

    void get_acc_raw(Eigen::Vector3d &acc_raw_out);
    void get_gyro_raw(Eigen::Vector3d &gyro_raw_out);

private:
    Eigen::Vector3d acc_raw;
    Eigen::Vector3d gyro_raw;
    Eigen::Vector3d acc_linear;
    const Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.8);
    Eigen::Quaterniond atti_q = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);// w x y z

    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
};
#endif //PROJECT_IMU_H
