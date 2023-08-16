#include "UDP_Interface.hpp"
#include "Robot.hpp"
#include <chrono>
using namespace std::chrono;


int main(int argc, char **argv)
{
    try
    {
        bool use_robot = true;
        Eigen::MatrixXd q_tasks = Eigen::MatrixXd::Zero(6, 28);
        q_tasks.col(0) << 0.445, 27.224, -44.181+27.224, 2.789, -49.112, -4.930; // Home
        q_tasks.col(1) << -12.745, 68.813, -44.296+68.813, 1.512, -46.689, 10.292; // Pick up
        q_tasks.col(2) << -12.776, 70.904, -45.541+70.904, 1.545, -45.444, 10.276; // pick down
        q_tasks.col(3) << -13.260, 66.652, -50.950+66.652, -3.225, -27.699, 14.672; // Pick twist
        q_tasks.col(4) << 0.445, 27.224, -44.181+27.224, 2.789, -49.112, -4.930; // Home
        q_tasks.col(5) << 10.154, 58.939, -58.411+58.939, 3.656, -35.707, -104.607; // Drop up
        q_tasks.col(6) << 10.100, 60.699, -59.754+60.699, 3.787, -34.362, -104.712; // Drop down
        q_tasks.col(7) << 10.154, 58.939, -58.411+58.939, 3.656, -35.707, -104.607; // Drop twist
        q_tasks.col(8) << 10.154, 58.939, -58.411+58.939, 3.656, -35.707, -104.607; // Drop up
        q_tasks.col(9) << 0.445, 27.224, -44.181+27.224, 2.789, -49.112, -4.930; // Home

        q_tasks.col(10) << -4.357, 62.556, -52.464+62.556, 5.023, -39.302, 1.387;
        q_tasks.col(11) << -4.460, 64.891, -54.029+64.891, 5.200, -37.739, 1.259;
        q_tasks.col(12) << -4.762, 61.425, -58.760+61.425, 5.405, -20.921, 1.159;
        q_tasks.col(13) << 0.445, 27.224, -44.181+27.224, 2.789, -49.112, -4.930; // Home
        q_tasks.col(14) << 13.748, 60.852, -55.093+60.852, 0.386, -36.148, -104.169;
        q_tasks.col(15) << 13.741, 63.165, -56.635+63.165, 0.401, -34.606, -104.179;
        q_tasks.col(16) << 13.748, 60.852, -55.093+60.852, 0.386, -36.148, -104.169;
        q_tasks.col(17) << 13.748, 60.852, -55.093+60.852, 0.386, -36.148, -104.169;
        q_tasks.col(18) << 0.445, 27.224, -44.181+27.224, 2.789, -49.112, -4.930; // Home

        q_tasks.col(19) << 2.888, 67.695, -44.579+67.695, 3.990, -47.541, -4.672;
        q_tasks.col(20) << 2.779, 70.419, -46.356+70.419, 4.114, -45.761, -4.737;
        q_tasks.col(21) << 2.884, 66.118, -51.885+66.118, 6.998, -28.119, -7.480;
        q_tasks.col(22) << 0.445, 27.224, -44.181+27.224, 2.789, -49.112, -4.930; // Home
        q_tasks.col(23) << 9.806, 59.236, -57.301+59.236, 2.913, -34.174, -104.195;
        q_tasks.col(24) << 9.760, 61.290, -58.713+61.290, 3.025, -32.762, -104.285;
        q_tasks.col(25) << 9.806, 59.236, -57.301+59.236, 2.913, -34.174, -104.195;
        q_tasks.col(26) << 9.806, 59.236, -57.301+59.236, 2.913, -34.174, -104.195;
        q_tasks.col(27) << 0.445, 27.224, -44.181+27.224, 2.789, -49.112, -4.930; // Home

        q_tasks.row(2) = q_tasks.row(2) - q_tasks.row(1);

        stmotion_controller::udp::UDP_Interface::Ptr robot_connection = std::make_shared<stmotion_controller::udp::UDP_Interface>();
        stmotion_controller::math::VectorJd q;
        stmotion_controller::robot::Robot::Ptr robot = std::make_shared<stmotion_controller::robot::Robot>();
        robot->Setup("config/robot_properties/FANUC_DH.txt",
                     "config/robot_properties/FANUC_base.txt");
        robot->print_robot_property();
        stmotion_controller::math::VectorJd jerk_ref = Eigen::MatrixXd::Zero(6, 1);
        stmotion_controller::math::VectorJd jerk_safe = Eigen::MatrixXd::Zero(6, 1);
        stmotion_controller::udp::recv_pack recv_packet;
        Eigen::MatrixXd q_profile = Eigen::MatrixXd::Zero(50000, 9);
        
        int goal_idx = 0;
        stmotion_controller::math::VectorJd goal = q_tasks.col(goal_idx);
        jerk_ref = robot->pid(goal);
        
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
        while(goal_idx < q_tasks.cols())//!robot->reached_goal(goal) || !robot->is_static())
        {
            if(robot->reached_goal(goal) && robot->is_static())
            {
                goal_idx ++;
                if(goal_idx < q_tasks.cols())
                {
                    goal = q_tasks.col(goal_idx);
                }
            }
            start = high_resolution_clock::now();
            jerk_ref = robot->pid(goal);
            stop = high_resolution_clock::now();
            duration = duration_cast<microseconds>(stop - start);
            std::cout << "PID calc time: " << duration.count() << std::endl;
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

            std::cout << "Send: " << q(0) << " " << q(1) << " " << q(2) << " " << q(3) << " " << q(4) << " " << q(5) << " " << goal_idx << " " << i << std::endl;
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
        // std::string q_fname = "../analysis/q_profile.txt";
        // stmotion_controller::io::SaveMatToFile(q_profile.block(0,0,i,9), q_fname);
        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}