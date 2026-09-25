/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "utility.h"
// 重力对齐
Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;
    Eigen::Vector3d ng1 = g.normalized(); // 归一化重力向量
    Eigen::Vector3d ng2{0, 0, 1.0};  // z 轴单位基向量
    // R0：将向量 ng1 旋转到 ng2 平行，也就是将重力旋转到和z轴平行
    // 这一步引入了 Yaw 轴偏移
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    // 将 R0 从 rotation Matrix 转换为 yaw、pitch、roll，同时提取 yaw 角
    double yaw = Utility::R2ypr(R0).x();
    // 将 yaw 轴的角度反方向消除，等价于用同一个反方向角度值的 rotation matrix 的左乘
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    return R0;
}
