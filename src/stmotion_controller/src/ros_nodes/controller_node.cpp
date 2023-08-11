#include "UDP_Interface.hpp"
#include "Robot.hpp"
#include <chrono>
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <fstream>
#include<unistd.h>
using namespace std::chrono;


stmotion_controller::math::VectorJd controller_goal = Eigen::MatrixXd::Zero(6, 1);


void goalCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    controller_goal << msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5];
}


int main(int argc, char **argv)
{
    try
    {
        bool use_robot, ssa_enb;
        double jpc_travel_time = 0.0;
        ros::init(argc, argv, "controller_node");
        ros::NodeHandle nh("~");
        ROS_INFO_STREAM("namespace of nh = " << nh.getNamespace());
        std::string config_fname, root_pwd, DH_fname, robot_base_fname;
        nh.getParam("config_fname", config_fname);
        nh.getParam("root_pwd", root_pwd);

        std::ifstream config_file(config_fname, std::ifstream::binary);
        Json::Value config;
        config_file >> config;
        DH_fname = root_pwd + config["DH_fname"].asString();
        robot_base_fname = root_pwd + config["robot_base_fname"].asString();
        ssa_enb = config["SSA_Enable"].asBool();
        use_robot = config["Use_Robot"].asBool();
        jpc_travel_time = config["JPC_travel_time"].asDouble();

        Eigen::MatrixXd cur_q, cur_qd, cur_qdd;
        ROS_INFO_STREAM("SSA_Enable: " << ssa_enb);
        ros::Rate loop_rate(150);
        
        unsigned int microsecond = 1000;

        stmotion_controller::udp::UDP_Interface::Ptr robot_connection = std::make_shared<stmotion_controller::udp::UDP_Interface>();
        stmotion_controller::math::VectorJd q;
        stmotion_controller::robot::Robot::Ptr robot = std::make_shared<stmotion_controller::robot::Robot>();
        robot->Setup(DH_fname, robot_base_fname);
        robot->set_JPC_speed(jpc_travel_time);
        robot->print_robot_property();
        stmotion_controller::math::VectorJd jerk_ref = Eigen::MatrixXd::Zero(6, 1);
        stmotion_controller::math::VectorJd jerk_safe = Eigen::MatrixXd::Zero(6, 1);
        stmotion_controller::udp::recv_pack recv_packet;
        ros::Publisher robot_state_pub = nh.advertise<std_msgs::Float32MultiArray>("robot_state", robot->robot_dof() * 3);
        ros::Subscriber goal_sub = nh.subscribe("robot_goal", robot->robot_dof(), goalCallback);
        ros::Publisher j1_pub = nh.advertise<std_msgs::Float64>("j1_pub", 1);
        ros::Publisher j2_pub = nh.advertise<std_msgs::Float64>("j2_pub", 1);
        ros::Publisher j3_pub = nh.advertise<std_msgs::Float64>("j3_pub", 1);
        ros::Publisher j4_pub = nh.advertise<std_msgs::Float64>("j4_pub", 1);
        ros::Publisher j5_pub = nh.advertise<std_msgs::Float64>("j5_pub", 1);
        ros::Publisher j6_pub = nh.advertise<std_msgs::Float64>("j6_pub", 1);
        std_msgs::Float32MultiArray robot_state_msg;
        std_msgs::Float64 j1_msg;
        std_msgs::Float64 j2_msg;
        std_msgs::Float64 j3_msg;
        std_msgs::Float64 j4_msg;
        std_msgs::Float64 j5_msg;
        std_msgs::Float64 j6_msg; 
        
        if(use_robot)
        {    
            robot_connection->Setup();
            robot_connection->SendInitPack();
            recv_packet = robot_connection->Recv();
        }     
        if(ssa_enb)
        {
            jerk_ref = robot->pid(controller_goal);
        }  
        else
        {
            jerk_ref = robot->jpc(controller_goal);
        }
        
        q = robot->step(jerk_ref, controller_goal);
        uint i = 0;
        while(ros::ok)
        {
            robot_state_msg.data.clear();
            cur_q = robot->robot_q();
            cur_qd = robot->robot_qd();
            cur_qdd = robot->robot_qdd();
            
            for(int j=0; j<robot->robot_dof(); j++)
            {
                robot_state_msg.data.push_back(cur_q(j));
                robot_state_msg.data.push_back(cur_qd(j));
                robot_state_msg.data.push_back(cur_qdd(j));
            }
            robot_state_pub.publish(robot_state_msg);
            if(use_robot)
            {
                robot_connection->Send(q, recv_packet.seq_no, 0, 1);
                if(ssa_enb)
                {
                    jerk_ref = robot->pid(controller_goal);
                    jerk_safe = robot->JSSA(jerk_ref);
                }
                else
                {
                    jerk_safe = robot->jpc(controller_goal);
                }
                q = robot->step(jerk_safe, controller_goal);
                recv_packet = robot_connection->Recv();
            }
            else{
                j1_msg.data = cur_q(0) / 180 * PI;
                j2_msg.data = cur_q(1) / 180 * PI;
                j3_msg.data = cur_q(2) / 180 * PI;
                j4_msg.data = cur_q(3) / 180 * PI;
                j5_msg.data = cur_q(4) / 180 * PI;
                j6_msg.data = cur_q(5) / 180 * PI;
                j1_pub.publish(j1_msg);
                j2_pub.publish(j2_msg);
                j3_pub.publish(j3_msg);
                j4_pub.publish(j4_msg);
                j5_pub.publish(j5_msg);
                j6_pub.publish(j6_msg);
                
                if(ssa_enb)
                {
                    jerk_ref = robot->pid(controller_goal);
                    jerk_safe = robot->JSSA(jerk_ref);
                }
                else
                {
                    jerk_safe = robot->jpc(controller_goal);
                }
                q = robot->step(jerk_safe, controller_goal);
            }
            if(!use_robot)
            {
                usleep(7 * microsecond); // Pause <8ms to simulate the robot controller
            }
            ros::spinOnce();
        }
        if(use_robot)
        {
            robot_connection->Send(q, recv_packet.seq_no, 1, 1);
            recv_packet = robot_connection->Recv();
            robot_connection->SendEndPack();
            robot_connection->Shutdown();
        }
        ROS_INFO_STREAM("Controller exit!");
        ros::shutdown();
        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}



