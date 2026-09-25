/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

/**
 * 位姿局部参数化 (SE(3) 切空间)：将 7 维过参数化 [x,y,z,qx,qy,qz,qw] 映射到 6 维切空间
 *
 * GlobalSize = 7, LocalSize = 6 (δp(3), δθ(3))
 *
 * Plus: [P, Q] ⊕ [δp, δθ] = [P+δp, Q⊗Exp(δθ)]
 *   位置用加法更新，旋转用四元数右乘 (切空间指数映射)
 */
class PoseLocalParameterization : public ceres::LocalParameterization
{
public:
    bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
    bool ComputeJacobian(const double *x, double *jacobian) const override;
    int GlobalSize() const override { return 7; }
    int LocalSize() const override { return 6; }
};
