#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h>
#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Float64.h"
#include "FTS/rq_int.hpp"
#include "FTS/rq_sensor_com.hpp"
#include "FTS/rq_sensor_state.hpp"


static void wait_for_other_connection(){
	INT_8 ret;
	struct timespec tim;
	tim.tv_sec = 1;
	tim.tv_nsec = 0L;

	while(1){
		nanosleep(&tim, (struct timespec *)NULL);
		ret = rq_sensor_state();
		if(ret == 0){
			break;
		}
	}
}

/**
 * \fn void get_data(void)
 * \brief Function to retrieve the power applied to the sensor
 * \param chr_return String to return forces applied
 */
static void get_data(INT_8 * chr_return){
	INT_8 i;
	INT_8 floatData[50];
	for(i = 0; i < 6; i++){
		sprintf(floatData, "%f", rq_state_get_received_data(i));
		if(i == 0){
			strcpy(chr_return, "( ");
			strcat(chr_return, floatData);
		}
		else{
			strcat(chr_return," , ");
			strcat(chr_return,floatData);
		}
		if(i == 5){
			strcat(chr_return, " )");
		}
	}
}


int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "fts_node");
        ros::NodeHandle nh("~");
        ROS_INFO_STREAM("namespace of nh = " << nh.getNamespace());
        ros::Rate loop_rate(10);
        
        
        ros::Publisher j1_pub = nh.advertise<std_msgs::Float64>("j1_pub", 1);
        ros::Publisher j2_pub = nh.advertise<std_msgs::Float64>("j2_pub", 1);
        ros::Publisher j3_pub = nh.advertise<std_msgs::Float64>("j3_pub", 1);
        ros::Publisher j4_pub = nh.advertise<std_msgs::Float64>("j4_pub", 1);
        ros::Publisher j5_pub = nh.advertise<std_msgs::Float64>("j5_pub", 1);
        ros::Publisher j6_pub = nh.advertise<std_msgs::Float64>("j6_pub", 1);
        std_msgs::Float64 j1_msg;
        std_msgs::Float64 j2_msg;
        std_msgs::Float64 j3_msg;
        std_msgs::Float64 j4_msg;
        std_msgs::Float64 j5_msg;
        std_msgs::Float64 j6_msg; 
        

        INT_8 ret = rq_sensor_state();
        if(ret == -1){
            wait_for_other_connection();
        }

        //Read high-level informations
        ret = rq_sensor_state();
        if(ret == -1){
            wait_for_other_connection();
        }

        //Initialize connection with the client
        ret = rq_sensor_state();
        if(ret == -1){
            wait_for_other_connection();
        }


        char bufStream[512];

        while(ros::ok)
        {
            ret = rq_sensor_state();
            if(ret == -1){
                wait_for_other_connection();
            }
            if(rq_sensor_get_current_state() == RQ_STATE_RUN){
                strcpy(bufStream,"");
                get_data(bufStream);
                // printf("%s\n", bufStream);
                /*
                * Here comes the code to send the data to the application.
                */
            }
            ROS_INFO_STREAM(bufStream);
            // std::cout<<data<<std::endl;
            // img_pub.publish(msg);
            ros::spinOnce();
        }
        ROS_INFO_STREAM("FTS exit!");
        ros::shutdown();
        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}



