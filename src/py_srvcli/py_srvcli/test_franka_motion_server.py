import sys

from franka_interfaces.srv import CartMotionTime
from franka_interfaces.srv import JointMotionVel
from franka_interfaces.srv import PosePath
from franka_interfaces.srv import FrankaHand
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

        self.frankaHand_cli = self.create_client(FrankaHand, '/franka_motion/franka_hand')
        while not self.frankaHand_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def gripper_request(self,enable,target_width=0.08,speed=0.5,force=0.2,epsilon_inner=0.005,epsilon_outer=0.1):
        req = FrankaHand.Request()
        req.enable = enable
        req.target_width = target_width
        req.speed = speed
        req.force = force
        req.epsilon_inner = epsilon_inner
        req.epsilon_outer = epsilon_outer
        self.future = self.frankaHand_cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_cart_request(self, pose, duration):
        req = CartMotionTime.Request()
        req.pose = pose
        req.duration = duration
        self.future = self.cart_cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_joint_request(self, joints, velscale):
        req = JointMotionVel.Request()
        req.joints = joints
        req.velscale = velscale
        self.future = self.joint_cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_posePath_request(self, poses, duration):
        req = PosePath.Request()
        req.poses = poses
        req.duration = duration
        self.future = self.posePath_cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    # pose1 = [0.512888, -0.024801, 0.130688, 0.009026, -0.000185, -0.009763]
    pose1 = [0.495613, -0.039523, 0.1372314, 3.09829, -0.00640984, 0.0291367]
    duration1 = 1.0
    joint1 = [0.06625625610839737, -0.516543949110693, -0.18081555820766249, -2.7310316540481128, -0.09787116237481434, 2.2255856248657646, 0.7259974131426877]
    # joint1 = [0.06576316667978721, -0.33619443085737394, -0.1804275372450417, -2.6659560173770838, -0.015912948752442996, 2.3126263302689836, 0.6636788695500128]
    velscale1 = 0.6

    poses = []
    pose_a = [pose1[0], pose1[1], pose1[2]+0.2]
    poses.extend(pose_a)
    pose_b = [pose_a[0]-0.2, pose_a[1]-0.2, pose_a[2]]
    poses.extend(pose_b)
    pose_c = [pose_a[0]-0.4, pose_a[1]-0.3, pose_a[2]]
    poses.extend(pose_c)
    pose_d = [pose_a[0]-0.4, pose_a[1]-0.4, pose_a[2]+0.02]
    poses.extend(pose_d)

    print(poses)
    duration2 = 3.0

    response_joint = minimal_client.send_joint_request(joint1, velscale1)
    minimal_client.get_logger().info(
        'Result for joint [%f,%f,%f,%f,%f,%f,%f] with %f speed factor with status %d' %
        (joint1[0],joint1[1],joint1[2],joint1[3],joint1[4],joint1[5],joint1[6], velscale1, response_joint.success))

    response_gripper = minimal_client.gripper_request(False)
    minimal_client.get_logger().info(
        'Result for control franka hand executed with status %d' %
        (response_gripper.success))
    
    response_cart = minimal_client.send_cart_request(pose1, duration1)
    minimal_client.get_logger().info(
        'Result for pose [%f,%f,%f,%f,%f,%f] with %fs time executed with status %d' %
        (pose1[0],pose1[1],pose1[2],pose1[3],pose1[4],pose1[5], duration1, response_cart.success))
      

    response_gripper = minimal_client.gripper_request(True)
    minimal_client.get_logger().info(
        'Result for control franka hand executed with status %d' %
        (response_gripper.success))


    response_posePath = minimal_client.send_posePath_request(poses, duration2)
    minimal_client.get_logger().info(
        'Result for posePath executed with status %d' %
        (response_posePath.success))

    response_gripper = minimal_client.gripper_request(False)
    minimal_client.get_logger().info(
        'Result for control franka hand executed with status %d' %
        (response_gripper.success))

    response_joint = minimal_client.send_joint_request(joint1, velscale1)
    minimal_client.get_logger().info(
        'Result for joint [%f,%f,%f,%f,%f,%f,%f] with %f speed factor with status %d' %
        (joint1[0],joint1[1],joint1[2],joint1[3],joint1[4],joint1[5],joint1[6], velscale1, response_joint.success))

        

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()