//
// Created by yan on 6/13/18.
//

#include "imu.h"

void IMU::set_acc(double x, double y, double z)
{
    acc_linear(0) = x;
    acc_linear(1) = y;
    acc_linear(2) = z;

    acc_linear += acc_bias;
}

void IMU::set_gyro(double x, double y, double z)
{
    gyro_raw(0) = x;
    gyro_raw(1) = y;
    gyro_raw(2) = z;

    gyro_raw += gyro_bias;
}

void IMU::set_atti(double x, double y, double z, double w)
{
    atti_q.x() = x;
    atti_q.y() = y;
    atti_q.z() = z;
    atti_q.w() = w;
}

void IMU::update()
{
    Eigen::Matrix3d R = atti_q.inverse().toRotationMatrix();
    acc_raw = acc_linear + R*gravity;
}

void IMU::get_acc_raw(Eigen::Vector3d &acc_raw_out)
{
    acc_raw_out = acc_raw;
}

void IMU::get_gyro_raw(Eigen::Vector3d &gyro_raw_out)
{
    gyro_raw_out = gyro_raw;
}