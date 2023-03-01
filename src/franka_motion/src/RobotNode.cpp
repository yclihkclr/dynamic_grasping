#include <rclcpp/rclcpp.hpp>

#include "franka_motion/RobotServer.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("franka_server_node");

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
    exe->spin();
}

int main(int argc, char** argv) {


    // Init ROS node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("franka_server_node");
    auto server = std::make_shared<RobotServer>(node);

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

