#include "rclcpp/rclcpp.hpp"
#include <franka_motion/liborl.h>
#include "franka_interfaces/srv/cart_motion_time.hpp"
#include "franka_interfaces/srv/joint_motion_vel.hpp"
#include "franka_interfaces/srv/pose_path.hpp"
#include "franka_interfaces/srv/franka_hand.hpp"

class RobotServer {
public:
    RobotServer(std::shared_ptr<rclcpp::Node> node, bool use_franka_hand=true);

    bool cartMotionTimeSrvCb(const std::shared_ptr<franka_interfaces::srv::CartMotionTime::Request> req,
           std::shared_ptr<franka_interfaces::srv::CartMotionTime::Response> res);

    bool jointMotionVelSrvCb(const std::shared_ptr<franka_interfaces::srv::JointMotionVel::Request> req,
           std::shared_ptr<franka_interfaces::srv::JointMotionVel::Response> res);

    bool posePathBezierCb(const std::shared_ptr<franka_interfaces::srv::PosePath::Request> req,
           std::shared_ptr<franka_interfaces::srv::PosePath::Response> res);

    bool frankaHandSrvCb(const std::shared_ptr<franka_interfaces::srv::FrankaHand::Request> req,
       std::shared_ptr<franka_interfaces::srv::FrankaHand::Response> res);

    bool controlGripper(bool enable, double target_width=0.01, double speed=0.15, double force=0.2, double epsilon_inner=0.005, double epsilon_outer=0.1);

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<orl::Robot> robot_;

    rclcpp::Service<franka_interfaces::srv::CartMotionTime>::SharedPtr cartMotionTimeService_;
    rclcpp::Service<franka_interfaces::srv::JointMotionVel>::SharedPtr jointMotionVelService_;
    rclcpp::Service<franka_interfaces::srv::PosePath>::SharedPtr posePathService_;
    rclcpp::Service<franka_interfaces::srv::FrankaHand>::SharedPtr frankaHandService_;
    // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatesPublisher_;
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stopMotionSubscriber_;
};