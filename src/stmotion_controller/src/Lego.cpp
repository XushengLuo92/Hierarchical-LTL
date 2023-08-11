#include "Lego.hpp"

namespace stmotion_controller
{
namespace lego
{
Lego_Gazebo::Lego_Gazebo()
{
}
        
void Lego_Gazebo::setup(const std::string& env_setup_fname, const bool& assemble, const Json::Value& task_json)
{
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "lego_update_node");
    ros::NodeHandle nh("~");
    ROS_INFO_STREAM("namespace of nh = " << nh.getNamespace());
    client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    gazebo_msgs::ModelState brick_pose;
    std::ifstream config_file(env_setup_fname, std::ifstream::binary);
    Json::Value config;
    double x, y, z;
    Eigen::Quaterniond quat(Eigen::Matrix3d::Identity(3, 3));
    brick_pose.pose.orientation.x = quat.x();
    brick_pose.pose.orientation.y = quat.y();
    brick_pose.pose.orientation.z = quat.z();
    brick_pose.pose.orientation.w = quat.w();
    
    config_file >> config;
    storage_plate_x = config["storage_plate"]["x"].asDouble();
    storage_plate_y = config["storage_plate"]["y"].asDouble();
    storage_plate_z = config["storage_plate"]["z"].asDouble();
    storage_plate_width = config["storage_plate"]["width"].asInt();
    storage_plate_height = config["storage_plate"]["height"].asInt();
    assemble_plate_x = config["assemble_plate"]["x"].asDouble();
    assemble_plate_y = config["assemble_plate"]["y"].asDouble();
    assemble_plate_z = config["assemble_plate"]["z"].asDouble();
    assemble_plate_width = config["assemble_plate"]["width"].asInt();
    assemble_plate_height = config["assemble_plate"]["height"].asInt();

    for(auto brick = config.begin(); brick != config.end(); brick++)
    {
        brick_pose.model_name = brick.name();
        
        if(brick.name().compare("storage_plate") == 0)
        {
            storage_plate_x = (*brick)["x"].asDouble();
            storage_plate_y = (*brick)["y"].asDouble();
            storage_plate_z = (*brick)["z"].asDouble();
            storage_plate_width = (*brick)["width"].asInt();
            storage_plate_height = (*brick)["height"].asInt();
            x = storage_plate_x;
            y = storage_plate_y;
            z = storage_plate_z;
        }
        else if(brick.name().compare("assemble_plate") == 0)
        {
            assemble_plate_x = (*brick)["x"].asDouble();
            assemble_plate_y = (*brick)["y"].asDouble();
            assemble_plate_z = (*brick)["z"].asDouble();
            assemble_plate_width = (*brick)["width"].asInt();
            assemble_plate_height = (*brick)["height"].asInt();
            x = assemble_plate_x;
            y = assemble_plate_y;
            z = assemble_plate_z;
        }
        else if(brick.name()[0] == 'b')
        {
            calc_brick_loc(brick.name(), storage_plate_x, storage_plate_y, 0, storage_plate_width, storage_plate_height, 
                           (*brick)["x"].asInt(), (*brick)["y"].asInt(), x, y);
            z = storage_plate_z + (*brick)["z"].asInt() * brick_height_m;
            lego_brick l_brick;
            l_brick.brick_name = brick.name();
            l_brick.x = x;
            l_brick.y = y;
            l_brick.z = z;
            l_brick.in_stock = true;
            Eigen::Matrix3d rot_mtx = Eigen::Matrix3d::Identity(3, 3);
            Eigen::Quaterniond quat(rot_mtx);
            l_brick.quat_x = quat.x();
            l_brick.quat_y = quat.y();
            l_brick.quat_z = quat.z();
            l_brick.quat_w = quat.w();
            brick_map[brick.name()] = l_brick;
        }
        else
        {
            ROS_INFO_STREAM("Unknown brick type!");
            continue;
        }
        brick_pose.pose.position.x = x;
        brick_pose.pose.position.y = y;
        brick_pose.pose.position.z = z;
        setmodelstate.request.model_state = brick_pose;
        client.call(setmodelstate);
    }
    if(!assemble)
    {
        std::string brick_name;
        double brick_x_m, brick_y_m, brick_z_m;
        Eigen::Matrix3d z_90;
        z_90 << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        Eigen::Matrix3d rot_mtx = Eigen::Matrix3d::Identity(3, 3);
        Eigen::Matrix3d rot_mtx1 = rot_mtx * z_90;
        Eigen::Quaterniond quat;
        for(int i=1; i<=task_json.size(); i++)
        {
            auto cur_graph_node = task_json[std::to_string(i)];
            brick_name = get_brick_name_by_id(cur_graph_node["id"].asInt(), 1);
            calc_brick_loc(brick_name, assemble_plate_x, assemble_plate_y, cur_graph_node["ori"].asInt(), assemble_plate_width, assemble_plate_height, 
                           cur_graph_node["x"].asInt(), cur_graph_node["y"].asInt(), brick_x_m, brick_y_m);
            brick_z_m = assemble_plate_z + cur_graph_node["z"].asInt() * brick_height_m;

            brick_map[brick_name].in_stock = assemble;
            if(cur_graph_node["ori"].asInt() == 1)
            {
                quat = rot_mtx1;
            }
            else
            {
                quat = rot_mtx;
            }
            brick_pose.model_name = brick_name;
            brick_pose.pose.position.x = brick_x_m;
            brick_pose.pose.position.y = brick_y_m;
            brick_pose.pose.position.z = brick_z_m;
            brick_pose.pose.orientation.x = quat.x();
            brick_pose.pose.orientation.y = quat.y();
            brick_pose.pose.orientation.z = quat.z();
            brick_pose.pose.orientation.w = quat.w();
            setmodelstate.request.model_state = brick_pose;
            client.call(setmodelstate);
        }
    }
    usleep(1000 * 1000); 
}


void Lego_Gazebo::calc_brick_loc(const std::string name, const double& ref_x, const double& ref_y, const int& orientation,
                                 const int& plate_width, const int& plate_height, const int& brick_loc_x, const int& brick_loc_y, 
                                 double& out_x, double& out_y)
{
    int brick_height = std::stoi(name.substr(1, 1));
    int brick_width = std::stoi(name.substr(2, 1));
    double brick_offset_x_m = brick_loc_x * P_len - 0.0002;
    double brick_offset_y_m = brick_loc_y * P_len - 0.0002;

    double brick_topleft_x_m = std::max(brick_offset_x_m, 0.0) + (ref_x - (plate_width * P_len - 0.0002) / 2.0);
    double brick_topleft_y_m = std::max(brick_offset_y_m, 0.0) + (ref_y - (plate_height * P_len - 0.0002) / 2.0);

    if(orientation == 0)
    {
        out_x = brick_topleft_x_m + (brick_height * P_len - 0.0002) / 2.0;
        out_y = brick_topleft_y_m + (brick_width * P_len - 0.0002) / 2.0;
    }
    else
    {
        out_x = brick_topleft_x_m + (brick_width * P_len - 0.0002) / 2.0;
        out_y = brick_topleft_y_m + (brick_height * P_len - 0.0002) / 2.0;
    }
}


void Lego_Gazebo::calc_brick_storage_pose(const std::string& name, Eigen::MatrixXd& T, const bool& grab)
{
    int brick_height = std::stoi(name.substr(1, 1));
    int brick_width = std::stoi(name.substr(2, 1));
    lego_brick l_brick = brick_map[name];
    brick_map[name].in_stock = !grab;
    Eigen::Quaterniond quat;
    quat.x() = l_brick.quat_x;
    quat.y() = l_brick.quat_y;
    quat.z() = l_brick.quat_z;
    quat.w() = l_brick.quat_w;
    double x, y, z;
    x = l_brick.x - (brick_height * P_len - 0.0002) / 2.0;
    y = l_brick.y;
    z = l_brick.z;

    Eigen::Matrix3d y_180, z_180;
    y_180 << -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0;
    z_180 << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;

    T.block(0, 0, 3, 3) << quat.normalized().toRotationMatrix() * y_180 * z_180;
    T.block(0, 3, 3, 1) << x, y, z;
}


void Lego_Gazebo::calc_brick_assemble_pose(const std::string name, const int& brick_x, const int& brick_y, const int& brick_z, 
                                           const int& orientation, Eigen::MatrixXd& T)
{
    double brick_x_m, brick_y_m, brick_z_m;
    int brick_height = std::stoi(name.substr(1, 1));
    int brick_width = std::stoi(name.substr(2, 1));
    calc_brick_loc(name, assemble_plate_x, assemble_plate_y, orientation, assemble_plate_width, assemble_plate_height, 
                   brick_x, brick_y, brick_x_m, brick_y_m);
    brick_z_m = assemble_plate_z + brick_z * brick_height_m;
    Eigen::Matrix3d rot_mtx = Eigen::Matrix3d::Identity(3, 3);
    Eigen::Matrix3d z_180;
    Eigen::Matrix3d y_180, z_90;
    y_180 << -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0;
    z_180 << -1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0;
    rot_mtx = rot_mtx * y_180 * z_180;
    z_90 << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    
    if(orientation == 0)
    {
        brick_x_m = brick_x_m - (brick_height * P_len - 0.0002) / 2.0;
        brick_y_m = brick_y_m;
    }
    else
    {
        brick_x_m = brick_x_m;
        brick_y_m = brick_y_m + (brick_height * P_len - 0.0002) / 2.0;
        rot_mtx = rot_mtx * z_90;
    }
    T.block(0, 0, 3, 3) << rot_mtx;
    T.block(0, 3, 3, 1) << brick_x_m, brick_y_m, brick_z_m;
}


std::string Lego_Gazebo::get_brick_name_by_id(const int& id, const bool& assemble)
{
    int brick_height = 0;
    int brick_width = 0;
    if(id == 2)
    {
        brick_height = 2;
        brick_width = 4;
    }
    else if(id == 3)
    {
        brick_height = 2;
        brick_width = 6;
    }
    else if(id == 4)
    {
        brick_height = 1;
        brick_width = 8;
    }
    else if(id == 5 || id == 7 || id == 8)
    {
        brick_height = 1;
        brick_width = 4;
    }
    else if(id == 6)
    {
        brick_height = 1;
        brick_width = 6;
    }
    else if(id == 9 || id == 11)
    {
        brick_height = 1;
        brick_width = 2;
    }
    else if(id == 10)
    {
        brick_height = 1;
        brick_width = 1;
    }
    else if(id == 12)
    {
        brick_height = 2;
        brick_width = 2;
    }
    if(assemble)
    {
        for(auto brick:brick_map)
        {
            std::string name = brick.second.brick_name;
            if(brick.second.in_stock == assemble && std::stoi(name.substr(1, 1)) == brick_height && std::stoi(name.substr(2, 1)) == brick_width)
            {
                return name;
            }
        }
    }
    else
    {
        for(auto brick = brick_map.rbegin(); brick != brick_map.rend(); brick ++)
        {
            std::string name = brick->second.brick_name;
            if(brick->second.in_stock == assemble && std::stoi(name.substr(1, 1)) == brick_height && std::stoi(name.substr(2, 1)) == brick_width)
            {
                return name;
            }
        }
    }
    std::cout << "No available brick!" << std::endl;
    return "";
}

void Lego_Gazebo::update(const std::string& brick_name, const Eigen::Matrix4d& T_init)
{
    int brick_height = std::stoi(brick_name.substr(1, 1));
    int brick_width = std::stoi(brick_name.substr(2, 1));
    Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity(4, 4);
    tmp.col(3) << (brick_height * P_len - 0.0002) / 2.0, 0, 0, 1;
    Eigen::Matrix4d T = T_init * tmp;
    
    Eigen::Matrix3d rot_mtx = T.block(0, 0, 3, 3);
    Eigen::Matrix3d y_180;
    y_180 << -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0;
    rot_mtx = rot_mtx * y_180;
    Eigen::Quaterniond quat(rot_mtx);

    gazebo_msgs::ModelState new_pose;
    new_pose.model_name = brick_name;
    new_pose.pose.position.x = T.coeff(0, 3);
    new_pose.pose.position.y = T.coeff(1, 3);
    new_pose.pose.position.z = T.coeff(2, 3);
    new_pose.pose.orientation.x = quat.x();
    new_pose.pose.orientation.y = quat.y();
    new_pose.pose.orientation.z = quat.z();
    new_pose.pose.orientation.w = quat.w();
    
    setmodelstate.request.model_state = new_pose;
    client.call(setmodelstate);
}

void Lego_Gazebo::update_bricks(const math::VectorJd& robot_q, const Eigen::MatrixXd& DH, const Eigen::MatrixXd& base_frame, 
                                const bool& joint_rad, const int& task_mode, const std::string& brick_name)
{
    Eigen::Matrix4d T = math::FK(robot_q, DH, base_frame, joint_rad);
    Eigen::Matrix4d tmp;
    std::string closest_brick_name = brick_name;
    cur_brick_name = brick_name;
    update(cur_brick_name, T);
}


}
}
