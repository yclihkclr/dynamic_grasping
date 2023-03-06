#include <rclcpp/rclcpp.hpp>

#include "franka_motion/RobotServer.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("franka_server_node");

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
    exe->spin();
}

int main(int argc, char** argv) {

    //set use_franka_hand or not
    bool use_franka_hand = true;

    // Init ROS node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("franka_server_node");

    node->declare_parameter<bool>("hand", 1);
    node->get_parameter<bool>("hand", use_franka_hand);
    RCLCPP_INFO(node->get_logger(), "use_franka_hand = %d", use_franka_hand);

    auto server = std::make_shared<RobotServer>(node,use_franka_hand);

    rclcpp::executor::ExecutorArgs arg;
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(arg, 2);
    executor->add_node(node);
    auto future_handle = std::async(std::launch::async, spin, executor);



    // Set spin rate
    rclcpp::Rate rate(50);
    while (rclcpp::ok())
    {
        if(!server->publishRobotStates()){return 0;}
        rate.sleep();
    }
    rclcpp::shutdown();
}

