#include "Utils/FileIO.hpp"
#include "Utils/Math.hpp"
#include "Robot.hpp"
#include "Lego.hpp"
#include <chrono>
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float32MultiArray.h"
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <fstream>
#include <unistd.h>
using namespace std::chrono;


stmotion_controller::math::VectorJd robot_q = Eigen::MatrixXd::Zero(6, 1);
stmotion_controller::math::VectorJd robot_qd = Eigen::MatrixXd::Zero(6, 1);
stmotion_controller::math::VectorJd robot_qdd = Eigen::MatrixXd::Zero(6, 1);


void robotStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    robot_q << msg->data[0], msg->data[3], msg->data[6], msg->data[9], msg->data[12], msg->data[15];
    robot_qd << msg->data[1], msg->data[4], msg->data[7], msg->data[10], msg->data[13], msg->data[16];
    robot_qdd << msg->data[2], msg->data[5], msg->data[8], msg->data[11], msg->data[14], msg->data[17];
}


int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "task_planning_node");
        ros::NodeHandle nh("~");
        ROS_INFO_STREAM("namespace of nh = " << nh.getNamespace());

        std::string config_fname, root_pwd, DH_fname, robot_base_fname;
        bool use_robot;
        nh.getParam("config_fname", config_fname);
        nh.getParam("root_pwd", root_pwd);

        std::ifstream config_file(config_fname, std::ifstream::binary);
        Json::Value config;
        config_file >> config;
        DH_fname = root_pwd + config["DH_tool_fname"].asString();
        robot_base_fname = root_pwd + config["robot_base_fname"].asString();
        use_robot = config["Use_Robot"].asBool();

        ros::Rate loop_rate(150);
      
        int robot_dof = 6;
        stmotion_controller::robot::Robot::Ptr robot = std::make_shared<stmotion_controller::robot::Robot>();
        robot->Setup(DH_fname, robot_base_fname);
        robot->print_robot_property();
        Eigen::Matrix<double, 3, 1> cart_goal = Eigen::Matrix<double, 3, 1>::Zero();
        Eigen::Matrix3d rot_goal = Eigen::Matrix3d::Identity();
        uint max_iter = 20000;
        double step_size = 0.1;
        Eigen::MatrixXd twist_R(3, 3);
        Eigen::MatrixXd twist_T(4, 4);
        Eigen::MatrixXd cart_T(4, 4);
        int twist_deg = 15;
        double incremental_deg = 1;
        int twist_idx = 0;
        int twist_num = twist_deg / incremental_deg;
       
        ros::Publisher goal_pub = nh.advertise<std_msgs::Float32MultiArray>("goal", robot_dof);
        ros::Subscriber robot_state_sub = nh.subscribe("robot_state", robot_dof * 3, robotStateCallback);
        std_msgs::Float32MultiArray goal_msg;
        
        int num_tasks = 1;
        Eigen::MatrixXd home_q(robot_dof, 1);
        home_q.col(0) << 0.445, 27.224, -44.181+27.224, 2.789, -49.112, -4.930; // Home

        Eigen::MatrixXd pick_up_q = Eigen::MatrixXd::Zero(robot_dof, num_tasks);
        pick_up_q.col(0) << -13.483, 71.114, 27.267, 0.968, -49.220, 12.840; // Pick up  767.195,-184.994, -203.167

        Eigen::MatrixXd pick_down_q = Eigen::MatrixXd::Zero(robot_dof, num_tasks);
        pick_down_q.col(0) << -13.483, 71.765, 27.579, 0.972, -48.876, 12.834; // 767.195,-183.254, -209.878

        Eigen::MatrixXd drop_up_q = Eigen::MatrixXd::Zero(robot_dof, num_tasks);
        drop_up_q.col(0) << 9.572, 59.110, -1.607, -0.003, -29.316, -99.569; //654.264,110.329,-203.318

        Eigen::MatrixXd drop_down_q = Eigen::MatrixXd::Zero(robot_dof, num_tasks);
        drop_down_q.col(0) << 9.572, 59.706, -1.391, -0.003, -28.933, -99.568; // 654.279,110.331,-208.816

        int mode = 0; // 0: home, 1: pick up, 2, pick down 3: pick twist 4: pick twist up 5: home
                      // 6: drop up 7: drop down 8: drop twist 9: drop twist up 10: done
        int task_idx = 0;
        stmotion_controller::math::VectorJd cur_goal = home_q;
        
        while(ros::ok)
        {
            robot->set_robot_q(robot_q);
            robot->set_robot_qd(robot_qd);
            robot->set_robot_qdd(robot_qdd);

            if(robot->reached_goal(cur_goal) && robot->is_static())
            {
                if(mode == 9){
                    mode = 0;
                    task_idx ++;
                    if(task_idx == num_tasks)
                    {
                        mode = 10;
                    }
                }
                else if(mode == 10) // Task Done
                {
                    break;
                }
                else if(mode == 3 || mode == 8)
                {
                    if(twist_idx == twist_num)
                    {
                        mode ++;
                        twist_idx = 0;
                    }
                    else
                    {
                        twist_idx ++;
                    }

                }
                else{
                    mode ++;
                }
                ROS_INFO_STREAM("Mode: " << mode << " Task: " << task_idx);

                if(mode == 0 || mode == 5 || mode == 10)
                {
                    cur_goal = home_q;
                }
                else if(mode == 1)
                {
                    cur_goal = pick_up_q.col(task_idx);
                }
                else if(mode == 2)
                {
                    cur_goal = pick_down_q.col(task_idx);
                }
                else if(mode == 3)
                {
                    double twist_rad = incremental_deg / 180.0 * PI;
                    twist_R << cos(twist_rad), 0, sin(twist_rad), 
                               0, 1, 0, 
                               -sin(twist_rad), 0, cos(twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;
                    cart_T = stmotion_controller::math::FK(robot_q, robot->robot_DH(), robot->robot_base(), false);
                    cart_T = cart_T * twist_T;
                    cur_goal = stmotion_controller::math::IK(robot_q, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3), 
                                                             robot->robot_DH(), robot->robot_base(), 0, max_iter, step_size);
                }
                else if(mode == 4 || mode == 9)
                {
                    cart_T = stmotion_controller::math::FK(robot_q, robot->robot_DH(), robot->robot_base(), false);
                    cart_T.col(3) << cart_T.coeff(0, 3), cart_T.coeff(1, 3), cart_T.coeff(2, 3) + 0.01, 1;
                    cur_goal = stmotion_controller::math::IK(robot_q, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3), 
                                                             robot->robot_DH(), robot->robot_base(), 0, max_iter, step_size);
                }
                else if(mode == 6)
                {
                    cur_goal = drop_up_q.col(task_idx);
                }
                else if(mode == 7)
                {
                    cur_goal = drop_down_q.col(task_idx);
                }
                else if(mode == 8)
                {
                    double twist_rad = incremental_deg / 180.0 * PI;
                    twist_R << cos(-twist_rad), 0, sin(-twist_rad), 
                               0, 1, 0, 
                               -sin(-twist_rad), 0, cos(-twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;
                    cart_T = stmotion_controller::math::FK(robot_q, robot->robot_DH(), robot->robot_base(), false);
                    cart_T = cart_T * twist_T;
                    cur_goal = cur_goal;//stmotion_controller::math::IK(robot_q, cart_T.block(0, 3, 3, 1), cart_T.block(0, 0, 3, 3), 
                                                            //  robot->robot_DH(), robot->robot_base(), 0, max_iter, step_size);
                }
            }
            goal_msg.data.clear();
            for(int j=0; j<robot_dof; j++)
            {
                goal_msg.data.push_back(cur_goal(j));
            }
            goal_pub.publish(goal_msg);
            ros::spinOnce();
        }
        ROS_INFO_STREAM("Task Execution Done!");
        ros::shutdown();
        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}



