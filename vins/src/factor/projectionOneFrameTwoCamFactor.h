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
 * 单帧双目重投影残差因子：同一帧内从左相机到右相机的重投影
 *
 * 2 维残差＝右图像平面上的重投影误差 (pixel)
 * 7 维输入 外参 cam0  [tx, ty, tz, qx, qy, qz, qw] — IMU 到左相机
 * 7 维输入 外参 cam1  [tx, ty, tz, qx, qy, qz, qw] — IMU 到右相机
 * 1 维输入 inv_depth — 特征点在左相机帧中的逆深度
 * 1 维输入 td        — 相机-IMU 时间偏移 (用于卷帘快门补偿)
 *
 * 不依赖帧间的 IMU 位姿，仅依赖双目外参和深度
 */
class ProjectionOneFrameTwoCamFactor : public ceres::SizedCostFunction<2, 7, 7, 1, 1>
{
  public:
    ProjectionOneFrameTwoCamFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j,
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
