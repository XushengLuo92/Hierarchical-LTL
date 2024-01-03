#pragma once
#include "Utils/Math.hpp"
#include "Utils/FileIO.hpp"
#include <cmath>

namespace stmotion_controller
{
namespace robot
{
class Robot
{
    /* -------------------------------------------------------------------------- */
    /*                                   pointer                                  */
    /* -------------------------------------------------------------------------- */
    public:
        typedef std::shared_ptr<Robot> Ptr;
        typedef std::shared_ptr<Robot const> ConstPtr;

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:
        int njoints_ = 6;
        double margin_ = 0.05; // m
        double delta_t_ = 0.008; // ms. Traj frequency
        std::string robot_name_ = "FANUC LRMate200iD7L";
        Eigen::MatrixXd q_;
        Eigen::MatrixXd qd_;
        Eigen::MatrixXd qdd_;

        Eigen::MatrixXd thetamax_; // njoints_ x 2
        Eigen::MatrixXd thetadotmax_; // njoints_ x 2
        double pos_epsilon_ = 5;
        double vel_epsilon_ = 0.000001;
        double acc_epsilon_ = 0.000001;
        math::VectorJd jerk_max_ = Eigen::MatrixXd::Constant(6, 1, 4000.0);
        Eigen::MatrixXd DH_; // size = [n_joint + n_ee, 4]
        Eigen::Matrix4d ee_inv_, tool_inv_, tool_assemble_inv_, tool_disassemble_inv_;
        Eigen::MatrixXd DH_tool_;
        Eigen::MatrixXd DH_tool_assemble_;
        Eigen::MatrixXd DH_tool_disassemble_;
        Eigen::MatrixXd base_frame_;
        Eigen::Matrix4d T_base_inv_;
        Eigen::MatrixXd Am_;
        Eigen::MatrixXd Bm_;
        Eigen::MatrixXd Ac_;
        Eigen::MatrixXd Bc_;
        stmotion_controller::math::Capsule cap_[6];
        stmotion_controller::math::Capsule cap_cur_[6];
        Eigen::MatrixXd M_[8];
        double critical_dist_ = 10000.0;
        Eigen::MatrixXd critical_pts_;
        Eigen::MatrixXd critical_pts_tmp_;
        int critical_link1_; // Link ID on robot
        int critical_link2_; // Link ID on human/environment
        Eigen::MatrixXd J_;
        Eigen::MatrixXd J_full_[6];
        Eigen::MatrixXd Jd_;
        Eigen::MatrixXd Jd_full_[6];
        Eigen::MatrixXd Jdd_;
        Eigen::MatrixXd state_;
        Eigen::MatrixXd vet_;
        double phi_safe_;
        double thres_;
        Eigen::MatrixXd P1_;
        Eigen::MatrixXd P2_;
        Eigen::MatrixXd P3_;
        Eigen::MatrixXd P4_;
        Eigen::MatrixXd Q_;
        Eigen::MatrixXd Adt_;
        Eigen::MatrixXd Bdt_;
        double jssa_k1_ = 0.1;
        double jssa_k2_ = 0.001;
        math::VectorJd pos_err_sum_;
        math::VectorJd vel_err_sum_;
        math::VectorJd acc_err_sum_;
        math::VectorJd last_pos_err_;
        math::VectorJd last_vel_err_;
        math::VectorJd last_acc_err_;
        Eigen::MatrixXd pid_threshold_step_;
        int pid_thresholding_len_ = 60;
        Eigen::MatrixXd pid_thresholding_ubuffer_;
        Eigen::MatrixXd pid_A_;
        Eigen::MatrixXd pid_B_;
        Eigen::MatrixXd pid_Binv_;
        int ssa_on_ = 0;

        int nlink_human_ = 6;
        stmotion_controller::math::Capsule human_cap_[6];

        // Capsules size = 1 (base) + n_joint + 1 (ee)
        std::vector<stmotion_controller::math::Capsule> capsules_;
         

    /* -------------------------------------------------------------------------- */
    /*                                  functions                                 */
    /* -------------------------------------------------------------------------- */
    private:
        void computeJacobiDiff(const Eigen::MatrixXd& DH_cur, const Eigen::MatrixXd& qd, const Eigen::MatrixXd& qdd, 
                               const int& robot_critical_link_id, const Eigen::MatrixXd& critical_pt);
        void calculateSafetyIdx(const Eigen::MatrixXd& D, const Eigen::MatrixXd& BJ, const double& margin, const Eigen::MatrixXd& u);
        void resetPID(int i);

    public:
        Robot();
        ~Robot(){}

        // setter
        void set_base_frame(const Eigen::MatrixXd& base) {base_frame_ = base;}
        void set_robot_q(const math::VectorJd& q) {q_ = q;}
        void set_robot_qd(const math::VectorJd& qd) {qd_ = qd;}
        void set_robot_qdd(const math::VectorJd& qdd) {qdd_ = qdd;}
        void set_DH_tool(const std::string fname);
        void set_DH_tool_assemble(const std::string fname);
        void set_DH_tool_disassemble(const std::string fname);
        void set_JPC_speed(const double& t);
        
        // getter
        void print_robot_property();
        int robot_dof() {return njoints_;};
        Eigen::MatrixXd robot_q() {return q_;};
        Eigen::MatrixXd robot_qd() {return qd_;};
        Eigen::MatrixXd robot_qdd() {return qdd_;};
        Eigen::MatrixXd robot_DH() {return DH_;};
        Eigen::MatrixXd robot_DH_tool() {return DH_tool_;};
        Eigen::MatrixXd robot_DH_tool_assemble() {return DH_tool_assemble_;};
        Eigen::MatrixXd robot_DH_tool_disassemble() {return DH_tool_disassemble_;};
        Eigen::MatrixXd robot_base() {return base_frame_;};
        Eigen::Matrix4d robot_base_inv() {return T_base_inv_;};
        Eigen::Matrix4d robot_ee_inv() {return ee_inv_;};
        Eigen::Matrix4d robot_tool_inv() {return tool_inv_;};
        Eigen::Matrix4d robot_tool_assemble_inv() {return tool_assemble_inv_;};
        Eigen::Matrix4d robot_tool_disassemble_inv() {return tool_disassemble_inv_;};

        // Operations
        void Setup(const std::string& DH_fname, const std::string& base_fname);
        math::VectorJd pid(const math::VectorJd& goal);
        math::VectorJd jpc(const math::VectorJd& goal);
        math::VectorJd JSSA(const math::VectorJd& jerk_ref);
        math::VectorJd step(const math::VectorJd& jerk, const math::VectorJd& goal);
        bool is_static();
        bool reached_goal(math::VectorJd goal);
        int ssa_status() {return ssa_on_;};

};
}
}