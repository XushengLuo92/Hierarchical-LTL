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
#include "Utils/CartLQR.hpp"

namespace stmotion_controller
{
namespace math
{

CartesianLQR2ndOrder::CartesianLQR2ndOrder(
    const Eigen::VectorXd& bu,
    const Eigen::VectorXd& bv,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& S,
    const double& Qslack,
    const Eigen::MatrixXd& R,
    const double& dt, const int& N
): Qs{Qslack},
    ud{CARTESIAN_DIMS}, xd{2*CARTESIAN_DIMS}, sd{ORIENTATION_DIMS}, cartd{CARTESIAN_DIMS}{
    
    this->Config(bu, bv, Q, S, R, dt, N);

    assert(this->Q.rows() == this->Q.cols() && this->Q.rows() == this->xd);
    assert(this->S.rows() == this->S.cols() && this->S.rows() == this->xd);
    assert(this->R.rows() == this->R.cols() && this->R.rows() == this->ud);

    #ifdef DEBUG_PRINT
        std::cout << " ---- Initialize CartesianLQR2ndOrder ---- " << "\n";
        std::cout << " ------------------- A ----------------- \n" << this->A << "\n";
        std::cout << " ------------------- B ----------------- \n" << this->B << "\n";
    #endif
}

void CartesianLQR2ndOrder::Config(
    const Eigen::VectorXd& bu,
    const Eigen::VectorXd& bv,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& S,
    const Eigen::MatrixXd& R,
    const double& dt,
    const int& N)
{
    this->bu = bu;
    this->bv = bv;
    this->Q = Q;
    this->S = S;
    this->R = R;
    this->dt = dt;
    this->N = N;

    this->A = Eigen::MatrixXd::Identity(this->xd, this->xd);
    this->A.block(0, cartd, cartd, cartd) = this->dt*Eigen::MatrixXd::Identity(cartd, cartd);
    this->B = Eigen::MatrixXd::Zero(this->xd, this->ud);
    this->B.block(0, 0, cartd, ud) = 0.5*(this->dt)*(this->dt)*Eigen::MatrixXd::Identity(cartd, ud);
    this->B.block(cartd, 0, cartd, ud) = (this->dt)*Eigen::MatrixXd::Identity(cartd, ud);

    this->Setup();
}

void CartesianLQR2ndOrder::Setup()
{
    #ifdef DEBUG_PRINT
        std::cout << "" << "\n";
        std::cout << " -------- Setup lifted MPC QP problem -------- " << "\n";
    #endif

    Bbar = Eigen::MatrixXd::Zero((N+1)*xd, N*ud);
    for (int i=0; i<N; ++i)
    {
        Bbar.block((i+1)*xd, 0, xd, N*ud) = A*Bbar.block(i*xd, 0, xd, N*ud);
        Bbar.block((i+1)*xd, i*ud, xd, ud) = B;
    }

    Qbar = Eigen::MatrixXd::Zero((N+1)*xd, (N+1)*xd);
    for (int i=0; i<N; ++i)
    {
        Qbar.block(i*xd, i*xd, xd, xd) = Q;
    }
    Qbar.block(N*xd, N*xd, xd, xd) = S;

    Rbar = Eigen::MatrixXd::Zero(N*ud, N*ud);
    for (int i=0; i<N; ++i)
    {
        Rbar.block(i*ud, i*ud, ud, ud) = R;
    }

    Afbar = Eigen::MatrixXd::Zero((N+1)*xd, xd);
    Afbar.block(0, 0, xd, xd) = Eigen::MatrixXd::Identity(xd, xd);
    for (int i=0; i<N; ++i)
    {
        Afbar.block((i+1)*xd, 0, xd, xd) = A*Afbar.block(i*xd, 0, xd, xd);
    }

    // constant QP matrices
    Eigen::MatrixXd G_eigen = Eigen::MatrixXd::Zero(N*ud+N*sd, N*ud+N*sd);
    G_eigen.block(0, 0, N*ud, N*ud) = Bbar.transpose()*Qbar*Bbar+Rbar;
    G_eigen.block(N*ud, N*ud, N*sd, N*sd) = Qs*Eigen::MatrixXd::Identity(N*sd, N*sd);
    stmotion_controller::math::SetQuadMatFromEigen(G, G_eigen);

    // Aeq*X = beq
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(xd, (N+1)*xd);
    a.block(0, N*xd, xd, xd) = Eigen::MatrixXd::Identity(xd, xd);
    Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(xd, N*ud+N*sd);
    Aeq << a*Bbar, Eigen::MatrixXd::Zero(xd, N*sd);
    stmotion_controller::math::SetQuadMatFromEigen(CE, -Aeq.transpose());

    // Aineq*X <= bineq
    Eigen::MatrixXd Aineq = Eigen::MatrixXd::Zero(0, 0);

    IRbar = Eigen::MatrixXd::Zero(N*sd, (N+1)*xd);
    for (int i=0; i<N; ++i)
    {
        IRbar.block(i*sd, (i+1)*xd+3, sd, sd) = Eigen::MatrixXd::Identity(sd, sd);
    }

    Eigen::MatrixXd Aineq1(N*sd, N*ud+N*sd);
    Aineq1 << IRbar*Bbar, -Eigen::MatrixXd::Identity(N*sd, N*sd);
    Aineq = stmotion_controller::math::EigenVcat(Aineq, Aineq1);

    Eigen::MatrixXd Aineq2(N*sd, N*ud+N*sd);
    Aineq2 << -IRbar*Bbar, -Eigen::MatrixXd::Identity(N*sd, N*sd);
    Aineq = stmotion_controller::math::EigenVcat(Aineq, Aineq2);

    Lubar = Eigen::MatrixXd::Identity(N*ud, N*ud);
    bubar = bu.replicate(N, 1);
    Eigen::MatrixXd Aineq3(2*N*ud+N*sd, N*ud+N*sd);
    Aineq3 << Lubar, Eigen::MatrixXd::Zero(N*ud, N*sd),
             -Lubar, Eigen::MatrixXd::Zero(N*ud, N*sd),
              Eigen::MatrixXd::Zero(N*sd, N*ud), -Eigen::MatrixXd::Identity(N*sd, N*sd);
    Aineq = stmotion_controller::math::EigenVcat(Aineq, Aineq3);

    bvbar = bv.replicate(N, 1);
    Ivbar = Eigen::MatrixXd::Zero(N*cartd, (N+1)*xd);
    for (int i=0; i<N; ++i)
    {
        Ivbar.block(i*cartd, (i+1)*xd+cartd, cartd, cartd) = Eigen::MatrixXd::Identity(cartd, cartd);
    }

    Eigen::MatrixXd Aineq4(N*cartd, N*ud+N*sd);
    Aineq4 << Ivbar*Bbar, Eigen::MatrixXd::Zero(N*cartd, N*sd);
    Aineq = stmotion_controller::math::EigenVcat(Aineq, Aineq4);

    Eigen::MatrixXd Aineq5(N*cartd, N*ud+N*sd);
    Aineq5 << -Ivbar*Bbar, Eigen::MatrixXd::Zero(N*cartd, N*sd);
    Aineq = stmotion_controller::math::EigenVcat(Aineq, Aineq5);

    stmotion_controller::math::SetQuadMatFromEigen(CI, -Aineq.transpose());

    #ifdef DEBUG_PRINT
        std::cout << " ------------------- Bbar ----------------- \n" << Bbar << "\n";
        std::cout << " ------------------- Qbar ----------------- \n" << Qbar << "\n";
        std::cout << " ------------------- Rbar ----------------- \n" << Rbar << "\n";
        std::cout << " ------------------- Afbar ----------------- \n" << Afbar << "\n";
        std::cout << " ------------------- Lubar ----------------- \n" << Lubar << "\n";
        std::cout << " ------------------- bubar ----------------- \n" << bubar << "\n";
        std::cout << " ------------------- IRbar ----------------- \n" << IRbar << "\n";
        std::cout << " ------------------- Ivbar ----------------- \n" << Ivbar << "\n";
        std::cout << " ------------------- bvbar ----------------- \n" << bvbar << "\n";
        std::cout << " ------------------- G ----------------- \n" << G_eigen << "\n";
        std::cout << " ------------------- Aeq ----------------- \n" << Aeq << "\n";
        std::cout << " ------------------- Aineq ----------------- \n" << Aineq << "\n";
    #endif
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> CartesianLQR2ndOrder::Solve(const Eigen::VectorXd& xk)
{
    
    /* 
    * Given current state xk, generate uk..uk+N-1 and xk...nk+N
    * by solving a regulation MPC problem with horizion N, costs Q, S, R
    * and first-order integration dynamics
    */
    #ifdef DEBUG_PRINT
        std::cout << " -------- Solving Cartesian Trajectory -------- " << "\n";
    #endif

    Eigen::VectorXd fbar = Afbar*xk;

    // solve MPC

    // objective
    quadprogpp::Vector<double> g0;
    Eigen::MatrixXd g0_eigen = Eigen::MatrixXd::Zero(1, N*ud+N*sd);
    g0_eigen.block(0, 0, 1, N*ud) = fbar.transpose()*Qbar*Bbar;
    stmotion_controller::math::SetQuadVecFromEigen(g0, g0_eigen.transpose());
    #ifdef DEBUG_PRINT
        std::cout << " ------------------- g0_eigen ----------------- \n" << g0_eigen.transpose() << "\n";
    #endif

    // eq
    quadprogpp::Vector<double> ce0;
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(xd, (N+1)*xd);
    a.block(0, N*xd, xd, xd) = Eigen::MatrixXd::Identity(xd, xd);
    Eigen::MatrixXd beq = -a*fbar;
    stmotion_controller::math::SetQuadVecFromEigen(ce0, beq);
    #ifdef DEBUG_PRINT
        std::cout << " ------------------- beq ----------------- \n" << beq << "\n";
    #endif

    // ineq
    quadprogpp::Vector<double> ci0;
    Eigen::MatrixXd bineq = Eigen::MatrixXd::Zero(0, 0);
    Eigen::MatrixXd bineq1(N*sd, 1);
    bineq1 << -IRbar*fbar;
    bineq = stmotion_controller::math::EigenVcat(bineq, bineq1);

    Eigen::MatrixXd bineq2(N*sd, 1);
    bineq2 << IRbar*fbar;
    bineq = stmotion_controller::math::EigenVcat(bineq, bineq2);

    Eigen::MatrixXd bineq3(2*N*ud+N*sd, 1);
    bineq3 << bubar,
              bubar,
              Eigen::MatrixXd::Zero(N*sd, 1);
    bineq = stmotion_controller::math::EigenVcat(bineq, bineq3);

    Eigen::MatrixXd bineq4(N*cartd, 1);
    bineq4 = bvbar - Ivbar*fbar;
    bineq = stmotion_controller::math::EigenVcat(bineq, bineq4);

    Eigen::MatrixXd bineq5(N*cartd, 1);
    bineq5 = bvbar + Ivbar*fbar;
    bineq = stmotion_controller::math::EigenVcat(bineq, bineq5);

    stmotion_controller::math::SetQuadVecFromEigen(ci0, bineq);
    #ifdef DEBUG_PRINT
        std::cout << " ------------------- bineq ----------------- \n" << bineq << "\n";
    #endif

    // decision variable
    quadprogpp::Vector<double> var;
    stmotion_controller::math::SetQuadVecFromEigen(var, Eigen::MatrixXd::Zero(N*ud+N*sd, 1));

    quadprogpp::Matrix<double> G_copy = G;
    quadprogpp::Matrix<double> CE_copy = CE;
    quadprogpp::Matrix<double> CI_copy = CI;
    double cost = quadprogpp::solve_quadprog(G_copy, g0, CE_copy, ce0, CI_copy, ci0, var);

    Eigen::VectorXd result;
    stmotion_controller::math::SetEigenMatFromQuad(result, var);
    Eigen::VectorXd Uk = result.head(N*ud);
    Eigen::VectorXd slack = result.tail(N*sd);

    Eigen::VectorXd Xk;
    Xk = fbar + Bbar*Uk;

    #ifdef DEBUG_PRINT
        std::cout << " ------------------- Uk ----------------- \n" << Uk << "\n";
        std::cout << " ------------------- Xk ----------------- \n" << Xk << "\n";
    #endif

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> Ukmat(Uk.data(), ud, N);
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> Xkmat(Xk.data(), xd, N+1);
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> slackmat(slack.data(), sd, N);

    #ifdef DEBUG_PRINT
        std::cout << " ------------------- Ukmat ----------------- \n" << Ukmat << "\n";
        std::cout << " ------------------- Xkmat ----------------- \n" << Xkmat << "\n";
        std::cout << " ------------------- slackmat ----------------- \n" << slackmat << "\n";
    #endif

    assert(Ukmat.rows() == ud);
    assert(Ukmat.cols() == N);
    assert(Xkmat.rows() == xd);
    assert(Xkmat.cols() == N+1);

    return std::make_pair(Ukmat, Xkmat);
   
}

std::pair<Eigen::MatrixXd, Eigen::MatrixXd> CartesianTrajectoryMPC(
    const Eigen::VectorXd& xk,
    const Eigen::VectorXd& bu, const int& N, const double& dt,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& S, const Eigen::MatrixXd& R)
{

    /* 
    * Given current state xk, generate uk..uk+N-1 and xk...nk+N
    * by solving a regulation MPC problem with horizion N, costs Q, S, R
    * and first-order integration dynamics
    */

    const int ud{CARTESIAN_DIMS}, xd{CARTESIAN_DIMS};
    assert(Q.rows() == Q.cols() && Q.rows() == xd);
    assert(S.rows() == S.cols() && S.rows() == xd);
    assert(R.rows() == R.cols() && R.rows() == ud);

    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(xd, xd);
    Eigen::MatrixXd B = Eigen::MatrixXd::Identity(xd, ud)*dt;

    #ifdef DEBUG_PRINT
        std::cout << " ------------------- A ----------------- \n" << A << "\n";
        std::cout << " ------------------- B ----------------- \n" << B << "\n";
    #endif

    Eigen::MatrixXd Bbar((N+1)*xd, N*ud);
    Bbar = Eigen::MatrixXd::Zero((N+1)*xd, N*ud);
    for (int i=0; i<N; ++i)
    {
        Bbar.block((i+1)*xd, 0, xd, N*ud) = A*Bbar.block(i*xd, 0, xd, N*ud);
        Bbar.block((i+1)*xd, i*ud, xd, ud) = B;
    }
    #ifdef DEBUG_PRINT
        std::cout << " ------------------- Bbar ----------------- \n" << Bbar << "\n";
    #endif

    Eigen::MatrixXd Qbar((N+1)*xd, (N+1)*xd);
    Qbar = Eigen::MatrixXd::Zero((N+1)*xd, (N+1)*xd);
    for (int i=0; i<N; ++i)
    {
        Qbar.block(i*xd, i*xd, xd, xd) = Q;
    }
    Qbar.block(N*xd, N*xd, xd, xd) = S;
    #ifdef DEBUG_PRINT
        std::cout << " ------------------- Qbar ----------------- \n" << Qbar << "\n";
    #endif

    Eigen::MatrixXd Rbar(N*ud, N*ud);
    Rbar = Eigen::MatrixXd::Zero(N*ud, N*ud);
    for (int i=0; i<N; ++i)
    {
        Rbar.block(i*ud, i*ud, ud, ud) = R;
    }
    #ifdef DEBUG_PRINT
        std::cout << " ------------------- Rbar ----------------- \n" << Rbar << "\n";
    #endif

    Eigen::MatrixXd Afbar((N+1)*xd, xd);
    Afbar = Eigen::MatrixXd::Zero((N+1)*xd, xd);
    Afbar.block(0, 0, xd, xd) = Eigen::MatrixXd::Identity(xd, xd);
    for (int i=0; i<N; ++i)
    {
        Afbar.block((i+1)*xd, 0, xd, xd) = A*Afbar.block(i*xd, 0, xd, xd);
    }
    #ifdef DEBUG_PRINT
        std::cout << " ------------------- Afbar ----------------- \n" << Afbar << "\n";
    #endif

    Eigen::MatrixXd Lubar = Eigen::MatrixXd::Identity(N*ud, N*ud);
    Eigen::MatrixXd bubar = bu.replicate(N, 1);

    #ifdef DEBUG_PRINT
        std::cout << " ------------------- Lubar ----------------- \n" << Lubar << "\n";
        std::cout << " ------------------- bubar ----------------- \n" << bubar << "\n";
    #endif

    Eigen::VectorXd fbar = Afbar*xk;

    // solve MPC
    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, Uk_quad;

    stmotion_controller::math::SetQuadMatFromEigen(G, Bbar.transpose()*Qbar*Bbar+Rbar);
    stmotion_controller::math::SetQuadVecFromEigen(g0, (fbar.transpose()*Qbar*Bbar).transpose());

    CE.resize(N*ud, 0);
    ce0.resize(0);

    stmotion_controller::math::SetQuadMatFromEigen(CI, stmotion_controller::math::EigenVcat(-Lubar, Lubar).transpose());
    stmotion_controller::math::SetQuadVecFromEigen(ci0, stmotion_controller::math::EigenVcat(bubar, bubar));

    stmotion_controller::math::SetQuadVecFromEigen(Uk_quad, Eigen::MatrixXd::Zero(N*ud, 1));

    double cost = quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, Uk_quad);

    Eigen::VectorXd Uk;
    stmotion_controller::math::SetEigenMatFromQuad(Uk, Uk_quad);

    Eigen::VectorXd Xk;
    Xk = fbar + Bbar*Uk;

    #ifdef DEBUG_PRINT
        std::cout << " ------------------- Uk ----------------- \n" << Uk << "\n";
        std::cout << " ------------------- Xk ----------------- \n" << Xk << "\n";
    #endif

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> Ukmat(Uk.data(), ud, N);
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>> Xkmat(Xk.data(), xd, N+1);

    #ifdef DEBUG_PRINT
        std::cout << " ------------------- Ukmat ----------------- \n" << Ukmat << "\n";
        std::cout << " ------------------- Xkmat ----------------- \n" << Xkmat.row(2) << "\n";
    #endif

    assert(Ukmat.rows() == ud);
    assert(Ukmat.cols() == N);
    assert(Xkmat.rows() == xd);
    assert(Xkmat.cols() == N+1);

    return std::make_pair(Ukmat, Xkmat);

}
}
}