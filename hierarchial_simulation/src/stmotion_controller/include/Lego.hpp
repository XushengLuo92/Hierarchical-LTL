#pragma once
#include "Utils/Math.hpp"
#include "Utils/FileIO.hpp"
#include <map>
#include "ros/ros.h"
#include <ros/package.h>
#include "gazebo_msgs/SetModelState.h"
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <fstream>

namespace stmotion_controller
{
namespace lego
{
struct lego_brick{
    std::string brick_name;
    float x;
    float y;
    float z;
    float quat_x;
    float quat_y;
    float quat_z;
    float quat_w;
    bool in_stock;
};

class Lego_Gazebo
{
    /* -------------------------------------------------------------------------- */
    /*                                   pointer                                  */
    /* -------------------------------------------------------------------------- */
    public:
        typedef std::shared_ptr<Lego_Gazebo> Ptr;
        typedef std::shared_ptr<Lego_Gazebo const> ConstPtr;

    /* -------------------------------------------------------------------------- */
    /*                                  variables                                 */
    /* -------------------------------------------------------------------------- */
    private:
        int num_bricks = 0;
        std::map<std::string, lego_brick> brick_map;
        ros::ServiceClient client; 
        gazebo_msgs::SetModelState setmodelstate;
        double brick_height_m = 0.0096;
        double P_len = 0.008; // width = size*P-0.0002
        double knob_height = 0.0017;
        std::string cur_brick_name = "";
        void update(const std::string& brick_name, const Eigen::Matrix4d& T);
        double storage_plate_x = 0.0;
        double storage_plate_y = 0.0;
        double storage_plate_z = 0.0;
        int storage_plate_width = 0;
        int storage_plate_height = 0;
        double assemble_plate_x = 0.0;
        double assemble_plate_y = 0.0;
        double assemble_plate_z = 0.0;
        int assemble_plate_width = 0;
        int assemble_plate_height = 0;

    public:
        Lego_Gazebo();
        ~Lego_Gazebo(){}
                
        // getter
        int get_num_bricks() {return num_bricks;}

        // Operations
        void setup(const std::string& env_setup_fname, const bool& assemble, const Json::Value& task_json);
        void update_bricks(const math::VectorJd& robot_q, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, 
                           const bool& joint_rad, const int& task_mode, const std::string& brick_name);

        void calc_brick_loc(const std::string name, const double& ref_x, const double& ref_y, const int& orientation,
                            const int& plate_width, const int& plate_height, const int& brick_loc_x, const int& brick_loc_y, 
                            double& out_x, double& out_y);
        
        void calc_brick_storage_pose(const std::string& name, Eigen::MatrixXd& T, const bool& grab);
        void calc_brick_assemble_pose(const std::string name, const int& brick_x, const int& brick_y, const int& brick_z, 
                                      const int& orientation, Eigen::MatrixXd& T);
        std::string get_brick_name_by_id(const int& id, const bool& assemble);

};
}
}