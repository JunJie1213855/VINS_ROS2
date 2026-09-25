/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <rcpputils/asserts.hpp>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../estimator/parameters.h"

/**
 * 视觉重投影残差因子：一个特征点在两帧相机之间的重投影误差
 *
 * 2  维残差＝图像平面上的重投影误差 (pixel)
 * 7  维输入 Pose_i   [x, y, z, qx, qy, qz, qw]  — 第 i 帧 IMU 位姿
 * 7  维输入 Pose_j   [x, y, z, qx, qy, qz, qw]  — 第 j 帧 IMU 位姿
 * 7  维输入 外参      [tx, ty, tz, qx, qy, qz, qw] — IMU 到相机的外参 T_ic
 * 1  维输入 inv_depth — 特征点在 i 帧中的逆深度
 *
 * 链式投影: pts_camera_i → pts_imu_i → pts_w → pts_imu_j → pts_camera_j
 */
class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1>
{
  public:
    ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};
