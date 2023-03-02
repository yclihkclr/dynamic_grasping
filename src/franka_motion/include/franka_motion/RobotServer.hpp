#include "rclcpp/rclcpp.hpp"
#include <franka_motion/liborl.h>
#include "franka_interfaces/srv/cart_motion_time.hpp"
#include "franka_interfaces/srv/joint_motion_vel.hpp"
#include "franka_interfaces/srv/pose_path.hpp"
<<<<<<< HEAD
#include "franka_interfaces/srv/franka_hand.hpp"

class RobotServer {
public:
    RobotServer(std::shared_ptr<rclcpp::Node> node, bool use_franka_hand=true);
=======

class RobotServer {
public:
    RobotServer(std::shared_ptr<rclcpp::Node> node);
>>>>>>> 33994516c9e18c65880689f1e064c03fd2d2679e

    bool cartMotionTimeSrvCb(const std::shared_ptr<franka_interfaces::srv::CartMotionTime::Request> req,
           std::shared_ptr<franka_interfaces::srv::CartMotionTime::Response> res);

    bool jointMotionVelSrvCb(const std::shared_ptr<franka_interfaces::srv::JointMotionVel::Request> req,
           std::shared_ptr<franka_interfaces::srv::JointMotionVel::Response> res);

    bool posePathBezierCb(const std::shared_ptr<franka_interfaces::srv::PosePath::Request> req,
           std::shared_ptr<franka_interfaces::srv::PosePath::Response> res);

<<<<<<< HEAD
    bool frankaHandSrvCb(const std::shared_ptr<franka_interfaces::srv::FrankaHand::Request> req,
       std::shared_ptr<franka_interfaces::srv::FrankaHand::Response> res);

    bool controlGripper(bool enable, double target_width=0.01, double speed=0.15, double force=0.2, double epsilon_inner=0.005, double epsilon_outer=0.1);

=======
>>>>>>> 33994516c9e18c65880689f1e064c03fd2d2679e
private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<orl::Robot> robot_;

    rclcpp::Service<franka_interfaces::srv::CartMotionTime>::SharedPtr cartMotionTimeService_;
    rclcpp::Service<franka_interfaces::srv::JointMotionVel>::SharedPtr jointMotionVelService_;
    rclcpp::Service<franka_interfaces::srv::PosePath>::SharedPtr posePathService_;
<<<<<<< HEAD
    rclcpp::Service<franka_interfaces::srv::FrankaHand>::SharedPtr frankaHandService_;
=======
>>>>>>> 33994516c9e18c65880689f1e064c03fd2d2679e
    // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatesPublisher_;
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stopMotionSubscriber_;
};