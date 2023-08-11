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

class CartesianLQR2ndOrder
{

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:

        // specified
        Eigen::MatrixXd A;
        Eigen::MatrixXd B;

        Eigen::VectorXd bu;
        Eigen::VectorXd bv;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd S;
        const double Qs;
        Eigen::MatrixXd R;
        double dt;
        int N;

        const int ud, xd, sd, cartd;

        // composed
        Eigen::MatrixXd Bbar;
        Eigen::MatrixXd Qbar;
        Eigen::MatrixXd Rbar;
        Eigen::MatrixXd Afbar;
        Eigen::MatrixXd Lubar;
        Eigen::MatrixXd bubar;
        Eigen::MatrixXd IRbar;
        Eigen::MatrixXd Ivbar;
        Eigen::MatrixXd bvbar;

        quadprogpp::Matrix<double> G, CE, CI;

    /* -------------------------------------------------------------------------- */
    /*                                  functions                                 */
    /* -------------------------------------------------------------------------- */
    private:

        void Setup();

    public:
        
        CartesianLQR2ndOrder(
            const Eigen::VectorXd& bu,
            const Eigen::VectorXd& bv,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& S,
            const double& Qslack,
            const Eigen::MatrixXd& R,
            const double& dt, const int& N);
        ~CartesianLQR2ndOrder(){};

        void Config(
            const Eigen::VectorXd& bu,
            const Eigen::VectorXd& bv,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& S,
            const Eigen::MatrixXd& R,
            const double& dt,
            const int& N);
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> Solve(const Eigen::VectorXd& xk);

};

}

}