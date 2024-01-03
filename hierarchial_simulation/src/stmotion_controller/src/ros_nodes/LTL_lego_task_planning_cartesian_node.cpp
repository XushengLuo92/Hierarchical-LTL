#include "Utils/FileIO.hpp"
#include "Utils/Math.hpp"
#include "Robot.hpp"
#include "Lego.hpp"
#include <chrono>
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float32MultiArray.h"
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

void LTL_robot_mode_update(const std_msgs::Float32MultiArray::ConstPtr& msg){

}

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "task_planning_node");
        ros::NodeHandle nh("~");
        ROS_INFO_STREAM("namespace of nh = " << nh.getNamespace());
        std::string config_fname, root_pwd, task_fname, DH_fname, DH_tool_fname, DH_tool_assemble_fname, DH_tool_disassemble_fname, 
                    robot_base_fname, gazebo_env_setup_fname;
        bool use_robot;
        bool IK_status;
        nh.getParam("config_fname", config_fname);
        nh.getParam("root_pwd", root_pwd);

        std::ifstream config_file(config_fname, std::ifstream::binary);
        Json::Value config;
        config_file >> config;
        DH_fname = root_pwd + config["DH_fname"].asString();
        DH_tool_fname = root_pwd + config["DH_tool_fname"].asString();
        DH_tool_assemble_fname = root_pwd + config["DH_tool_assemble_fname"].asString();
        DH_tool_disassemble_fname = root_pwd + config["DH_tool_disassemble_fname"].asString();
        robot_base_fname = root_pwd + config["robot_base_fname"].asString();
        gazebo_env_setup_fname = root_pwd + config["env_setup_fname"].asString();
        task_fname = root_pwd + config["task_graph_fname"].asString();
        use_robot = config["Use_Robot"].asBool();
        bool infinite_tasks = config["Infinite_tasks"].asBool();
        bool assemble = config["Start_with_assemble"].asBool();

        std::ifstream task_file(task_fname, std::ifstream::binary);
        Json::Value task_json;
        task_file >> task_json;
        ros::Rate loop_rate(150);

        stmotion_controller::lego::Lego_Gazebo::Ptr lego_gazebo_ptr = std::make_shared<stmotion_controller::lego::Lego_Gazebo>();
        lego_gazebo_ptr->setup(gazebo_env_setup_fname, assemble, task_json);
      
        int robot_dof = 6;
        stmotion_controller::robot::Robot::Ptr robot = std::make_shared<stmotion_controller::robot::Robot>();
        robot->Setup(DH_fname, robot_base_fname);
        robot->set_DH_tool(DH_tool_fname);
        robot->set_DH_tool_assemble(DH_tool_assemble_fname);
        robot->set_DH_tool_disassemble(DH_tool_disassemble_fname);
        robot->print_robot_property();
        Eigen::Matrix<double, 3, 1> cart_goal = Eigen::Matrix<double, 3, 1>::Zero();
        Eigen::Matrix3d rot_goal = Eigen::Matrix3d::Identity();
        Eigen::MatrixXd twist_R(3, 3);
        Eigen::MatrixXd twist_T(4, 4);
        Eigen::MatrixXd cart_T(4, 4);
        int twist_deg = 14;
        double incremental_deg = 7;
        int twist_idx = 0;
        int twist_num = twist_deg / incremental_deg;
       
        ros::Publisher goal_pub = nh.advertise<std_msgs::Float32MultiArray>("goal", robot_dof);
        ros::Subscriber robot_state_sub = nh.subscribe("robot_state", robot_dof * 3, robotStateCallback);
        ros::Subscriber robot_mode_LTL=nh.subscribe("/LTL/robot_mode",1,LTL_robot_mode_update);
        std_msgs::Float32MultiArray goal_msg;
        
        int num_tasks = task_json.size();
        Eigen::MatrixXd home_q(robot_dof, 1);
        home_q.col(0) << 0.445, 27.224, -44.181+27.224, 2.789, -49.112, -4.930; // Home

        int mode = 0; // 0: home, 1: pick up, 2, pick down 3: pick twist 4: pick twist up 5: home
                      // 6: drop up 7: drop down 8: drop twist 9: drop twist up 10: done
        int task_idx;
        if(assemble)
        {
            task_idx = 1;
        }
        else
        {
            task_idx = num_tasks;
        }
        
        int pre_mode = -1;
        stmotion_controller::math::VectorJd cur_goal = home_q;
        std::string brick_name;
        
        while(ros::ok)
        {
            auto loop_start = high_resolution_clock::now();
            robot->set_robot_q(robot_q);
            robot->set_robot_qd(robot_qd);
            robot->set_robot_qdd(robot_qdd);
            if(mode >= 3 && mode <= 7 && !use_robot)
            {
                if(pre_mode != mode)
                {
                    lego_gazebo_ptr->update_bricks(robot_q, robot->robot_DH_tool(), robot->robot_base(), false, mode + twist_idx, brick_name);
                }
                else
                {
                    lego_gazebo_ptr->update_bricks(robot_q, robot->robot_DH_tool(), robot->robot_base(), false, 0, brick_name);
                }
            }
            pre_mode = mode;
            if(robot->reached_goal(cur_goal) && robot->is_static())
            {
                if(mode == 10){
                    if(task_idx >= num_tasks && !infinite_tasks && assemble)
                    {
                        break;
                    }
                    else if(task_idx <= 1 && !infinite_tasks && !assemble)
                    {
                        break;
                    }
                    else if(task_idx >= num_tasks && assemble)
                    {
                        task_idx ++;
                        assemble = !assemble;
                    }
                    else if(task_idx <= 1 && !assemble)
                    {
                        task_idx --;
                        assemble = !assemble;
                    }
                    mode = 0;
                    if(assemble)
                    {
                        task_idx ++;
                    }
                    else
                    {
                        task_idx --;
                    }
                }
                else if(mode == 3 || mode == 8)
                {
                    twist_idx ++;
                    if(twist_idx == twist_num)
                    {
                        mode ++;
                        twist_idx = 0;
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
                    auto cur_graph_node = task_json[std::to_string(task_idx)];
                    if(assemble)
                    {
                        brick_name = lego_gazebo_ptr->get_brick_name_by_id(cur_graph_node["id"].asInt(), assemble);
                        lego_gazebo_ptr->calc_brick_storage_pose(brick_name, cart_T, assemble);
                    }
                    else
                    {
                        brick_name = lego_gazebo_ptr->get_brick_name_by_id(cur_graph_node["id"].asInt(), assemble);
                        lego_gazebo_ptr->calc_brick_assemble_pose(brick_name, 
                            cur_graph_node["x"].asInt(), 
                            cur_graph_node["y"].asInt(), 
                            cur_graph_node["z"].asInt(),
                            cur_graph_node["ori"].asInt(), cart_T);
                    }
                    Eigen::Matrix4d up_T = cart_T;
                    up_T(2, 3) = up_T(2, 3) + 0.015;
                    cur_goal =  stmotion_controller::math::IK_closed_form(cur_goal, up_T, robot->robot_DH_tool(), 
                        robot->robot_base_inv(), robot->robot_tool_inv(), 0, IK_status);
                }
                else if(mode == 2)
                {
                    // cart_T(2, 3) = cart_T(2, 3) - 0.002;
                    cur_goal =  stmotion_controller::math::IK_closed_form(cur_goal, cart_T, robot->robot_DH_tool(), 
                        robot->robot_base_inv(), robot->robot_tool_inv(), 0, IK_status);
                }
                else if(mode == 3)
                {
                    double twist_rad = incremental_deg / 180.0 * PI;
                    twist_R << cos(twist_rad), 0, sin(twist_rad), 
                               0, 1, 0, 
                               -sin(twist_rad), 0, cos(twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;
                    cart_T = stmotion_controller::math::FK(cur_goal, robot->robot_DH_tool_disassemble(), robot->robot_base(), false);
                    cart_T = cart_T * twist_T;
                    cur_goal =  stmotion_controller::math::IK_closed_form(cur_goal, cart_T, robot->robot_DH_tool_disassemble(), 
                        robot->robot_base_inv(), robot->robot_tool_disassemble_inv(),0,IK_status);
                }
                else if(mode == 4 || mode == 9)
                {
                    cart_T = stmotion_controller::math::FK(cur_goal, robot->robot_DH_tool_assemble(), robot->robot_base(), false);
                    cart_T(2, 3) = cart_T(2, 3) + 0.015;
                    cur_goal =  stmotion_controller::math::IK_closed_form(cur_goal, cart_T, robot->robot_DH_tool_assemble(), 
                        robot->robot_base_inv(), robot->robot_tool_assemble_inv(),0,IK_status);
                }
                else if(mode == 6)
                {
                    if(assemble)
                    {
                        auto cur_graph_node = task_json[std::to_string(task_idx)];
                        lego_gazebo_ptr->calc_brick_assemble_pose(brick_name, 
                            cur_graph_node["x"].asInt(), 
                            cur_graph_node["y"].asInt(), 
                            cur_graph_node["z"].asInt(),
                            cur_graph_node["ori"].asInt(), cart_T);
                    }
                    else
                    {
                        lego_gazebo_ptr->calc_brick_storage_pose(brick_name, cart_T, assemble);
                    }
                    Eigen::Matrix4d up_T = cart_T;
                    up_T(2, 3) = up_T(2, 3) + 0.015;
                    cur_goal =  stmotion_controller::math::IK_closed_form(cur_goal, up_T, robot->robot_DH_tool(), 
                        robot->robot_base_inv(), robot->robot_tool_inv(),0,IK_status);
                }
                else if(mode == 7)
                {
                    // cart_T(2, 3) = cart_T(2, 3) - 0.0035;
                    cur_goal =  stmotion_controller::math::IK_closed_form(cur_goal, cart_T, robot->robot_DH_tool(), 
                        robot->robot_base_inv(), robot->robot_tool_inv(),0,IK_status);
                }
                else if(mode == 8)
                {
                    double twist_rad = incremental_deg / 180.0 * PI;
                    twist_R << cos(-twist_rad), 0, sin(-twist_rad), 
                               0, 1, 0, 
                               -sin(-twist_rad), 0, cos(-twist_rad);
                    twist_T = Eigen::MatrixXd::Identity(4, 4);
                    twist_T.block(0, 0, 3, 3) << twist_R;
                    cart_T = stmotion_controller::math::FK(cur_goal, robot->robot_DH_tool_assemble(), robot->robot_base(), false);
                    cart_T = cart_T * twist_T;
                    cur_goal =  stmotion_controller::math::IK_closed_form(cur_goal, cart_T, robot->robot_DH_tool_assemble(), 
                        robot->robot_base_inv(), robot->robot_tool_assemble_inv(),0,IK_status);
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



