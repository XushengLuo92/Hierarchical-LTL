#include "UDP_Interface.hpp"
#include "Robot.hpp"
#include <chrono>
using namespace std::chrono;


int main(int argc, char **argv)
{
    try
    {
        stmotion_controller::math::VectorJd q;
        stmotion_controller::robot::Robot::Ptr robot = std::make_shared<stmotion_controller::robot::Robot>();
        robot->Setup("config/robot_properties/FANUC_DH.txt",
                     "config/robot_properties/FANUC_base.txt");
        robot->print_robot_property();
        Eigen::Matrix<double, 3, 1> cart_goal = Eigen::Matrix<double, 3, 1>::Zero();
        Eigen::Matrix3d rot_goal = Eigen::Matrix3d::Identity();
        cart_goal << 0.5, 0, 0.6;
        q = robot->robot_q();
        q << 10, 20, 30, 40, 50, 60;
        uint max_iter = 20000;
        double step_size = 0.1;
        Eigen::MatrixXd error_list;

        auto start = high_resolution_clock::now();
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
       
        start = high_resolution_clock::now();
        stmotion_controller::math::VectorJd q_IK = stmotion_controller::math::IK(q, cart_goal, rot_goal, robot->robot_DH(), 
                                                                                 robot->robot_base(), 0, max_iter, step_size);
        
        stop = high_resolution_clock::now();
        duration = duration_cast<microseconds>(stop - start);
        Eigen::MatrixXd trans_mtx = stmotion_controller::math::FK(q_IK, robot->robot_DH(), robot->robot_base(), 0);
        std::cout << "IK Solution: \n" << trans_mtx << "\nTime Cost: " << duration.count() << std::endl;
        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}