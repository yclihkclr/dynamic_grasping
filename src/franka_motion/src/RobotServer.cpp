#include "franka_motion/RobotServer.hpp" 

const std::string cart_motion_time_srv_id = "/franka_motion/cart_motion_time";
const std::string joint_motion_vel_srv_id = "/franka_motion/joint_motion_vel";
const std::string pose_path_srv_id = "/franka_motion/pose_path";
<<<<<<< HEAD
const std::string franka_hand_srv_id = "/franka_motion/franka_hand";
const std::string franka_fci_ip = "192.168.1.20";

RobotServer::RobotServer(std::shared_ptr<rclcpp::Node> node, bool use_franka_hand) {
=======
const std::string franka_fci_ip = "192.168.1.20";

RobotServer::RobotServer(std::shared_ptr<rclcpp::Node> node) {
>>>>>>> 33994516c9e18c65880689f1e064c03fd2d2679e
    node_ = node;

    // Initialize scaling velocity and acceleration service
    cartMotionTimeService_ = node->create_service<franka_interfaces::srv::CartMotionTime>(cart_motion_time_srv_id, std::bind(&RobotServer::cartMotionTimeSrvCb, this, std::placeholders::_1, std::placeholders::_2));
<<<<<<< HEAD
    RCLCPP_INFO(rclcpp::get_logger("robot_server"), "Advertising service %s", cart_motion_time_srv_id.c_str());

    jointMotionVelService_ = node->create_service<franka_interfaces::srv::JointMotionVel>(joint_motion_vel_srv_id, std::bind(&RobotServer::jointMotionVelSrvCb, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("robot_server"), "Advertising service %s", joint_motion_vel_srv_id.c_str());

    posePathService_ = node->create_service<franka_interfaces::srv::PosePath>(pose_path_srv_id, std::bind(&RobotServer::posePathBezierCb, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("robot_server"), "Advertising service %s", pose_path_srv_id.c_str());

    frankaHandService_ = node->create_service<franka_interfaces::srv::FrankaHand>(franka_hand_srv_id, std::bind(&RobotServer::frankaHandSrvCb, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(rclcpp::get_logger("robot_server"), "Advertising service %s", franka_hand_srv_id.c_str());

    robot_ = std::make_shared<orl::Robot>(franka_fci_ip,use_franka_hand);

    // init pose
    std::array<double, 7> q_goal = {{0.058432293025024944, -0.7970709721832945, -0.19613238666559518, -2.7730435522313703, -0.14883224842143097, 2.034597985161675, 0.7228324700781736}};
    robot_->joint_motion(q_goal, 0.2); 
}


bool RobotServer::controlGripper(bool enable, double target_width, double speed, double force,double epsilon_inner, double epsilon_outer){
    if (enable) {
        target_width = 0.0;
    }

    franka::GripperState state = robot_->get_franka_gripper().readOnce();
    if (target_width > state.max_width || target_width < 0.0) {
      RCLCPP_ERROR(rclcpp::get_logger("robot_server_gripper"), "GripperServer: Commanding out of range width! max_width = '%f' command = '%f'" , state.max_width, target_width);
      return false;
    }
    constexpr double kSamePositionThreshold = 1e-4;
    if (std::abs(target_width - state.width) < kSamePositionThreshold) {
      return true;
    }
    if (target_width >= state.width) {
      return robot_->get_franka_gripper().move(target_width, speed);
    }

    return robot_->get_franka_gripper().grasp(target_width, speed , force, epsilon_inner, epsilon_outer);

}

bool RobotServer::frankaHandSrvCb(const std::shared_ptr<franka_interfaces::srv::FrankaHand::Request> req,
    std::shared_ptr<franka_interfaces::srv::FrankaHand::Response> res)
{
    bool result;
    RCLCPP_INFO(rclcpp::get_logger("robot_server_gripper"), "get request to set franka hand status %d", req->enable);
    RCLCPP_INFO(rclcpp::get_logger("robot_server_gripper"), "gripper configuration are target_width:%f,speed:%f,force:%f,epsilon_inner:%f,epsilon_outer:%f", req->target_width, req->speed, req->force, req->epsilon_inner, req->epsilon_outer);
    result = controlGripper(req->enable, req->target_width, req->speed, req->force, req->epsilon_inner, req->epsilon_outer);

    res->success = result;
    return true;
}
=======
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

>>>>>>> 33994516c9e18c65880689f1e064c03fd2d2679e

bool RobotServer::cartMotionTimeSrvCb(const std::shared_ptr<franka_interfaces::srv::CartMotionTime::Request> req,
           std::shared_ptr<franka_interfaces::srv::CartMotionTime::Response> res)
{
<<<<<<< HEAD
    RCLCPP_INFO(rclcpp::get_logger("robot_server"), "Pose [%f,%f,%f,%f,%f,%f]",req->pose[0],req->pose[1],req->pose[2],req->pose[3],req->pose[4],req->pose[5]);
    RCLCPP_INFO(rclcpp::get_logger("robot_server"), "Duration %f",req->duration);
    RCLCPP_INFO(rclcpp::get_logger("robot_server"), "cartMotionTimeSrvCb");
=======
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose [%f,%f,%f,%f,%f,%f]",req->pose[0],req->pose[1],req->pose[2],req->pose[3],req->pose[4],req->pose[5]);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Duration %f",req->duration);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "cartMotionTimeSrvCb");
>>>>>>> 33994516c9e18c65880689f1e064c03fd2d2679e

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
<<<<<<< HEAD
    RCLCPP_INFO(rclcpp::get_logger("robot_server"), "Joint [%f,%f,%f,%f,%f,%f,%f]",req->joints[0],req->joints[1],req->joints[2],req->joints[3],req->joints[4],req->joints[5],req->joints[6]);
    RCLCPP_INFO(rclcpp::get_logger("robot_server"), "Velocity Scaling %f",req->velscale);
    RCLCPP_INFO(rclcpp::get_logger("robot_server"), "jointMotionVelSrvCb");
=======
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint [%f,%f,%f,%f,%f,%f,%f]",req->joints[0],req->joints[1],req->joints[2],req->joints[3],req->joints[4],req->joints[5],req->joints[6]);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Velocity Scaling %f",req->velscale);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "jointMotionVelSrvCb");
>>>>>>> 33994516c9e18c65880689f1e064c03fd2d2679e
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
<<<<<<< HEAD
      RCLCPP_INFO(rclcpp::get_logger("robot_server"), "Waypoint %d with Pose [%f,%f,%f]",i+1,req->poses[0+i*3],req->poses[1+i*3],req->poses[2+i*3]);
    }
    RCLCPP_INFO(rclcpp::get_logger("robot_server"), "Duration %f",req->duration);
    RCLCPP_INFO(rclcpp::get_logger("robot_server"), "posePathSrvCb");
=======
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waypoint %d with Pose [%f,%f,%f]",i+1,req->poses[0+i*3],req->poses[1+i*3],req->poses[2+i*3]);
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Duration %f",req->duration);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "posePathSrvCb");
>>>>>>> 33994516c9e18c65880689f1e064c03fd2d2679e

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

