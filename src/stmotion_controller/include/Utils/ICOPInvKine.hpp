/*
***********************************************************************************************************************************************************************
Copyright notice for IP Docket # 2022-013, The CFS Library in C++ for Robot Arm Trajectory Planning 2020 Carnegie Mellon University. 
All rights reserved.
National Robotics Engineering Center, Carnegie Mellon University www.nrec.ri.cmu.edu
Confidential and Proprietary - Do not distribute without prior written permission.

Rui Chen      ruic3@andrew.cmu.edu
Ruixuan Liu   ruixuanl@andrew.cmu.edu
Weiye Zhao    weiyezha@andrew.cmu.edu
Changliu Liu  cliu6@andrew.cmu.edu

A023793. Sponsor is provided a non-exclusive, world-wide license to the Licensed Technology in the field of robotic painting and arcwelding.

This notice must appear in all copies of this file and its derivatives.
***********************************************************************************************************************************************************************
*/
#pragma once
#include "Utils/Math.hpp"
#include "QuadProg++.hh"

namespace stmotion_controller
{
namespace math
{

Eigen::MatrixXd icop_cartesian2joint(
    const Eigen::MatrixXd& c_arr, const stmotion_controller::math::Vector6d& c0,
    const stmotion_controller::math::VectorJd& theta0, const Eigen::MatrixXd& DH,
    const double& thres, const int& max_iter);

}
}