import sys

from franka_interfaces.srv import CartMotionTime
from franka_interfaces.srv import JointMotionVel
from franka_interfaces.srv import PosePath
import rclpy
from rclpy.node import Node
import time


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cart_cli = self.create_client(CartMotionTime, '/franka_motion/cart_motion_time')
        while not self.cart_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.joint_cli = self.create_client(JointMotionVel, '/franka_motion/joint_motion_vel')
        while not self.joint_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.posePath_cli = self.create_client(PosePath, '/franka_motion/pose_path')
        while not self.posePath_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_cart_request(self, a, b):
        req = CartMotionTime.Request()
        req.pose = a
        req.duration = b
        self.future = self.cart_cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_joint_request(self, a, b):
        req = JointMotionVel.Request()
        req.joints = a
        req.velscale = b
        self.future = self.joint_cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_posePath_request(self, a, b):
        req = PosePath.Request()
        req.poses = a
        req.duration = b
        self.future = self.posePath_cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    pose1 = [0.495613, -0.039523, 0.0672314, 3.09829, -0.00640984, 0.0291367]
    # pose2 = [0.395613, 0.0, 0.2072314, 3.09829, -0.00640984, 0.0291367]
    duration1 = 1.0
    # duration2 = 2.0
    joint1 = [-0.00771968,-0.599246,-0.0852301,-2.62207,-0.0888355,2.13279,0.693285]
    velscale1 = 0.3

    poses = []
    pose_a = [pose1[0], pose1[1], pose1[2]+0.2]
    poses.extend(pose_a)
    pose_b = [pose_a[0]-0.2, pose_a[1]-0.2, pose_a[2]]
    poses.extend(pose_b)
    pose_c = [pose_a[0]-0.4, pose_a[1]-0.3, pose_a[2]]
    poses.extend(pose_c)
    pose_d = [pose_a[0]-0.4, pose_a[1]-0.3, pose1[2]]
    poses.extend(pose_d)

    print(poses)
    duration2 = 5.0

    # for i in range(10):
    response_cart = minimal_client.send_cart_request(pose1, duration1)
    minimal_client.get_logger().info(
        'Result for pose [%f,%f,%f,%f,%f,%f] with %fs time executed with status %d' %
        (pose1[0],pose1[1],pose1[2],pose1[3],pose1[4],pose1[5], duration1, response_cart.success))
        # time.sleep(2)
        
    response_posePath = minimal_client.send_posePath_request(poses, duration2)
    minimal_client.get_logger().info(
        'Result for posePath executed with status %d' %
        (response_posePath.success))

    response_joint = minimal_client.send_joint_request(joint1, velscale1)
    minimal_client.get_logger().info(
        'Result for joint [%f,%f,%f,%f,%f,%f,%f] with %f speed factor with status %d' %
        (joint1[0],joint1[1],joint1[2],joint1[3],joint1[4],joint1[5],joint1[6], velscale1, response_joint.success))

        
        


    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()