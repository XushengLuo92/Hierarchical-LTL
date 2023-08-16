#include "UDP_Interface.hpp"
#include "Robot.hpp"
#include <chrono>
using namespace std::chrono;


int main(int argc, char **argv)
{
    try
    {
        bool use_robot = true;
        bool use_pid = true;
        
        std::string fname{"./src/stmotion_controller/py_scripts/sample_jerk_traj.txt"};
        Eigen::MatrixXd traj = stmotion_controller::io::LoadMatFromFile(fname);
        stmotion_controller::udp::UDP_Interface::Ptr robot_connection = std::make_shared<stmotion_controller::udp::UDP_Interface>();
        stmotion_controller::math::VectorJd q;
        stmotion_controller::robot::Robot::Ptr robot = std::make_shared<stmotion_controller::robot::Robot>();
        robot->Setup("./src/stmotion_controller/config/robot_properties/FANUC_DH.txt",
                     "./src/stmotion_controller/config/robot_properties/FANUC_base.txt");
        robot->print_robot_property();
        stmotion_controller::math::VectorJd jerk_ref = Eigen::MatrixXd::Zero(6, 1);
        stmotion_controller::math::VectorJd jerk_safe = Eigen::MatrixXd::Zero(6, 1);
        stmotion_controller::udp::recv_pack recv_packet;
        Eigen::MatrixXd q_profile = Eigen::MatrixXd::Zero(500, 9);
        stmotion_controller::math::VectorJd goal = Eigen::MatrixXd::Zero(6, 1);
        goal << 0, 30, -25, 0, 0, 0;

        if(!use_pid)
        {
            jerk_ref = traj.row(0).transpose();
        }
        else{   
            jerk_ref = robot->pid(goal);
        }
        
        if(use_robot)
        {    
            robot_connection->Setup();
            robot_connection->SendInitPack();
            recv_packet = robot_connection->Recv();
        }       
        
        auto start = high_resolution_clock::now();
        auto stop = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        q = robot->step(jerk_ref, goal);
        int i = 0;
        while(1)//!robot->reached_goal(goal) || !robot->is_static())
        {
            if(i >= q_profile.rows())
            {
                break;
            }
            if(use_pid)
            {
                start = high_resolution_clock::now();
                jerk_ref = robot->pid(goal);
                stop = high_resolution_clock::now();
                duration = duration_cast<microseconds>(stop - start);
                std::cout << "PID calc time: " << duration.count() << std::endl;
            }
            else
            {
                jerk_ref = traj.row(i).transpose();
            }
            if(use_robot)
            {
                robot_connection->Send(q, recv_packet.seq_no, 0, 1);
                jerk_safe = robot->JSSA(jerk_ref);
                q = robot->step(jerk_safe, goal);
                recv_packet = robot_connection->Recv();
            }
            else{
                start = high_resolution_clock::now();
                jerk_safe = robot->JSSA(jerk_ref);
                q = robot->step(jerk_safe, goal);
                stop = high_resolution_clock::now();
                duration = duration_cast<microseconds>(stop - start);
                std::cout << "JSSA calc time: " << duration.count() << std::endl;
            }
            q_profile.row(i) << (robot->robot_q()).coeff(1, 0), (robot->robot_q()).coeff(2, 0), 
                                (robot->robot_qd()).coeff(1, 0), (robot->robot_qd()).coeff(2, 0),
                                (robot->robot_qdd()).coeff(1, 0),(robot->robot_qdd()).coeff(2, 0), 
                                jerk_safe(1), jerk_safe(2), robot->ssa_status();
            i += 1;

            std::cout << "Send: " << q(0) << " " << q(1) << " " << q(2) << " " << q(3) << " " << q(4) << " " << q(5) << " " << i << std::endl;
            std::cout << "Vel: " << (robot->robot_qd()).coeff(1, 0) << " " << (robot->robot_qd()).coeff(2, 0) << std::endl;
            std::cout << "Acc: " << (robot->robot_qdd()).coeff(1, 0) << " " << (robot->robot_qdd()).coeff(2, 0) << std::endl;
            std::cout << "Jerk: " << jerk_safe(0) << " " << jerk_safe(1) << " " << jerk_safe(2) << " " 
                                  << jerk_safe(3) << " " << jerk_safe(4) << " " << jerk_safe(5) << std::endl;
        }
        if(use_robot)
        {
            robot_connection->Send(q, recv_packet.seq_no, 1, 1);
            recv_packet = robot_connection->Recv();
            robot_connection->SendEndPack();
            robot_connection->Shutdown();
        }
        std::cout << "Execution Done!" << "\n";
        std::string q_fname = "../analysis/q_profile.txt";
        stmotion_controller::io::SaveMatToFile(q_profile.block(0,0,i,9), q_fname);
        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}