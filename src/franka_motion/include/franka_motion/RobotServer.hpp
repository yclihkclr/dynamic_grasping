#include "rclcpp/rclcpp.hpp"
#include <franka_motion/liborl.h>
#include "franka_interfaces/srv/cart_motion_time.hpp"
#include "franka_interfaces/srv/joint_motion_vel.hpp"
#include "franka_interfaces/srv/pose_path.hpp"

class RobotServer {
public:
    RobotServer(std::shared_ptr<rclcpp::Node> node);

    bool cartMotionTimeSrvCb(const std::shared_ptr<franka_interfaces::srv::CartMotionTime::Request> req,
           std::shared_ptr<franka_interfaces::srv::CartMotionTime::Response> res);

    bool jointMotionVelSrvCb(const std::shared_ptr<franka_interfaces::srv::JointMotionVel::Request> req,
           std::shared_ptr<franka_interfaces::srv::JointMotionVel::Response> res);

    bool posePathBezierCb(const std::shared_ptr<franka_interfaces::srv::PosePath::Request> req,
           std::shared_ptr<franka_interfaces::srv::PosePath::Response> res);

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<orl::Robot> robot_;

    rclcpp::Service<franka_interfaces::srv::CartMotionTime>::SharedPtr cartMotionTimeService_;
    rclcpp::Service<franka_interfaces::srv::JointMotionVel>::SharedPtr jointMotionVelService_;
    rclcpp::Service<franka_interfaces::srv::PosePath>::SharedPtr posePathService_;
    // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatesPublisher_;
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stopMotionSubscriber_;
};