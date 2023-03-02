#include <rclcpp/rclcpp.hpp>

#include "franka_motion/RobotServer.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("franka_server_node");

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
    exe->spin();
}

int main(int argc, char** argv) {

<<<<<<< HEAD
    //set use_franka_hand or not
    bool use_franka_hand = true;
=======
>>>>>>> 33994516c9e18c65880689f1e064c03fd2d2679e

    // Init ROS node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("franka_server_node");
<<<<<<< HEAD

    node->declare_parameter<bool>("hand", 1);
    node->get_parameter<bool>("hand", use_franka_hand);
    RCLCPP_INFO(node->get_logger(), "use_franka_hand = %d", use_franka_hand);

    auto server = std::make_shared<RobotServer>(node,use_franka_hand);
=======
    auto server = std::make_shared<RobotServer>(node);
>>>>>>> 33994516c9e18c65880689f1e064c03fd2d2679e

    rclcpp::executor::ExecutorArgs arg;
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(arg, 2);
    executor->add_node(node);
    auto future_handle = std::async(std::launch::async, spin, executor);



    // Set spin rate
    rclcpp::Rate rate(100);
    while (rclcpp::ok())
    {
        // my_controller->publishJointStates();
        rate.sleep();
    }
    // rclcpp::shutdown();
}

