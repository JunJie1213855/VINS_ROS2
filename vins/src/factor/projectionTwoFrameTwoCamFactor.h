/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once

#include <rcpputils/asserts.hpp>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../estimator/parameters.h"

/**
 * 两帧双目重投影残差因子：左相机 i 帧到右相机 j 帧的跨帧跨相机重投影
 *
 * 2 维残差＝右图像平面上的重投影误差 (pixel)
 * 7 维输入 Pose_i   — 第 i 帧 IMU 位姿
 * 7 维输入 Pose_j   — 第 j 帧 IMU 位姿
 * 7 维输入 外参 cam0 — IMU 到左相机 T_ic0
 * 7 维输入 外参 cam1 — IMU 到右相机 T_ic1
 * 1 维输入 inv_depth — 特征点逆深度
 * 1 维输入 td        — 相机-IMU 时间偏移
 *
 * 投影路径: 左相机(i) → IMU(i) → world → IMU(j) → 右相机(j)
 */
class ProjectionTwoFrameTwoCamFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 7, 1, 1>
{
  public:
    ProjectionTwoFrameTwoCamFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
    							   const Eigen::Vector2d &_velocity_i, const Eigen::Vector2d &_velocity_j,
    				   			   const double _td_i, const double _td_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Vector3d velocity_i, velocity_j;
    double td_i, td_j;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};
