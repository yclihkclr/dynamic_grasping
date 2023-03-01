#include "franka_motion/RobotServer.hpp" 

const std::string cart_motion_time_srv_id = "/franka_motion/cart_motion_time";
const std::string joint_motion_vel_srv_id = "/franka_motion/joint_motion_vel";
const std::string pose_path_srv_id = "/franka_motion/pose_path";
const std::string franka_fci_ip = "192.168.1.20";

RobotServer::RobotServer(std::shared_ptr<rclcpp::Node> node) {
    node_ = node;

    // Initialize scaling velocity and acceleration service
    cartMotionTimeService_ = node->create_service<franka_interfaces::srv::CartMotionTime>(cart_motion_time_srv_id, std::bind(&RobotServer::cartMotionTimeSrvCb, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Advertising service %s", cart_motion_time_srv_id.c_str());

    jointMotionVelService_ = node->create_service<franka_interfaces::srv::JointMotionVel>(joint_motion_vel_srv_id, std::bind(&RobotServer::jointMotionVelSrvCb, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Advertising service %s", joint_motion_vel_srv_id.c_str());

    posePathService_ = node->create_service<franka_interfaces::srv::PosePath>(pose_path_srv_id, std::bind(&RobotServer::posePathBezierCb, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Advertising service %s", pose_path_srv_id.c_str());

    robot_ = std::make_shared<orl::Robot>(franka_fci_ip);

    // init pose
    std::array<double, 7> q_goal = {{-0.00771968,-0.599246,-0.0852301,-2.62207,-0.0888355,2.13279,0.693285}};
    robot_->joint_motion(q_goal, 0.3); 
}


bool RobotServer::cartMotionTimeSrvCb(const std::shared_ptr<franka_interfaces::srv::CartMotionTime::Request> req,
           std::shared_ptr<franka_interfaces::srv::CartMotionTime::Response> res)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose [%f,%f,%f,%f,%f,%f]",req->pose[0],req->pose[1],req->pose[2],req->pose[3],req->pose[4],req->pose[5]);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Duration %f",req->duration);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "cartMotionTimeSrvCb");

    orl::Pose dest_pose({req->pose[0],req->pose[1],req->pose[2]},{req->pose[3],req->pose[4],req->pose[5]});
    robot_->cart_motion(dest_pose, req->duration);

    bool ok = !robot_->hasErrors();
    res->success = ok;
    if (ok) {
      return true;
    } else {
      return false;
    }
}

bool RobotServer::jointMotionVelSrvCb(const std::shared_ptr<franka_interfaces::srv::JointMotionVel::Request> req,
           std::shared_ptr<franka_interfaces::srv::JointMotionVel::Response> res)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint [%f,%f,%f,%f,%f,%f,%f]",req->joints[0],req->joints[1],req->joints[2],req->joints[3],req->joints[4],req->joints[5],req->joints[6]);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Velocity Scaling %f",req->velscale);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "jointMotionVelSrvCb");
    std::array<double, 7> q_goal = {{req->joints[0],req->joints[1],req->joints[2],req->joints[3],req->joints[4],req->joints[5],req->joints[6]}};
    robot_->joint_motion(q_goal, req->velscale); 

    bool ok = !robot_->hasErrors();
    res->success = ok;
    if (ok) {
      return true;
    } else {
      return false;
    }
}

bool RobotServer::posePathBezierCb(const std::shared_ptr<franka_interfaces::srv::PosePath::Request> req,
           std::shared_ptr<franka_interfaces::srv::PosePath::Response> res)
{   
    std::vector<orl::Position> way_points;
    for(int i=0;i<(sizeof(req)/3-1);i++)
    {
      orl::Position pose(req->poses[0+i*3],req->poses[1+i*3],req->poses[2+i*3]);
      way_points.push_back(pose);
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waypoint %d with Pose [%f,%f,%f]",i+1,req->poses[0+i*3],req->poses[1+i*3],req->poses[2+i*3]);
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Duration %f",req->duration);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "posePathSrvCb");

    auto bezier_movement = orl::PoseGenerators::BezierMotion(way_points);
    orl::apply_speed_profile(bezier_movement, orl::SpeedProfiles::QuinticPolynomialProfile());
    robot_->move_cartesian(bezier_movement, req->duration);

    bool ok = !robot_->hasErrors();
    res->success = ok;
    if (ok) {
      return true;
    } else {
      return false;
    }
}

