import sys

from franka_interfaces.srv import CartMotionTime
from franka_interfaces.srv import JointMotionVel
from franka_interfaces.srv import PosePath
from franka_interfaces.srv import FrankaHand
from franka_interfaces.msg import RobotState
import rclpy
from rclpy.node import Node
import transform
from srt_serial import PythonSerialDriver

import random
from utils.torch_utils import select_device, load_classifier, time_sync
from utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords,
    xyxy2xywh, strip_optimizer, set_logging)
from utils.datasets import LoadStreams, LoadImages, letterbox
from models.experimental import attempt_load
import torch.backends.cudnn as cudnn
import torch

import pyrealsense2 as rs
import math
import yaml
import argparse
import os
import time
import numpy as np
import sys
import cv2

pipeline = rs.pipeline()  # 定义流程pipeline
config = rs.config()  # 定义配置config
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)  # 流程开始
align_to = rs.stream.color  # 与color流对齐
align = rs.align(align_to)


def get_aligned_images():
    frames = pipeline.wait_for_frames()  # 等待获取图像帧
    aligned_frames = align.process(frames)  # 获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的color帧

    ############### 相机参数的获取 #######################
    intr = color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile(
    ).intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
    '''camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                         'ppx': intr.ppx, 'ppy': intr.ppy,
                         'height': intr.height, 'width': intr.width,
                         'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                         }'''

    # 保存内参到本地
    # with open('./intrinsics.json', 'w') as fp:
    #json.dump(camera_parameters, fp)
    #######################################################

    depth_image = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 深度图（8位）
    depth_image_3d = np.dstack(
        (depth_image_8bit, depth_image_8bit, depth_image_8bit))  # 3通道深度图
    color_image = np.asanyarray(color_frame.get_data())  # RGB图

    # 返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame

class YoloV5:
    def __init__(self, yolov5_yaml_path='config/yolov5s.yaml'):
        '''初始化'''
        # 载入配置文件
        with open(yolov5_yaml_path, 'r', encoding='utf-8') as f:
            self.yolov5 = yaml.load(f.read(), Loader=yaml.SafeLoader)
        # 随机生成每个类别的颜色
        self.colors = [[np.random.randint(0, 255) for _ in range(
            3)] for class_id in range(self.yolov5['class_num'])]
        # 模型初始化
        self.init_model()

    @torch.no_grad()
    def init_model(self):
        '''模型初始化'''
        # 设置日志输出
        set_logging()
        # 选择计算设备
        device = select_device(self.yolov5['device'])
        # 如果是GPU则使用半精度浮点数 F16
        is_half = device.type != 'cpu'
        # 载入模型
        model = attempt_load(
            self.yolov5['weight'], map_location=device)  # 载入全精度浮点数的模型
        input_size = check_img_size(
            self.yolov5['input_size'], s=model.stride.max())  # 检查模型的尺寸
        if is_half:
            model.half()  # 将模型转换为半精度
        # 设置BenchMark，加速固定图像的尺寸的推理
        cudnn.benchmark = True  # set True to speed up constant image size inference
        # 图像缓冲区初始化
        img_torch = torch.zeros(
            (1, 3, self.yolov5['input_size'], self.yolov5['input_size']), device=device)  # init img
        # 创建模型
        # run once
        _ = model(img_torch.half()
                  if is_half else img) if device.type != 'cpu' else None
        self.is_half = is_half  # 是否开启半精度
        self.device = device  # 计算设备
        self.model = model  # Yolov5模型
        self.img_torch = img_torch  # 图像缓冲区

    def preprocessing(self, img):
        '''图像预处理'''
        # 图像缩放
        # 注: auto一定要设置为False -> 图像的宽高不同
        img_resize = letterbox(img, new_shape=(
            self.yolov5['input_size'], self.yolov5['input_size']), auto=False)[0]
        # print("img resize shape: {}".format(img_resize.shape))
        # 增加一个维度
        img_arr = np.stack([img_resize], 0)
        # 图像转换 (Convert) BGR格式转换为RGB
        # 转换为 bs x 3 x 416 x
        # 0(图像i), 1(row行), 2(列), 3(RGB三通道)
        # ---> 0, 3, 1, 2
        # BGR to RGB, to bsx3x416x416
        img_arr = img_arr[:, :, :, ::-1].transpose(0, 3, 1, 2)
        # 数值归一化
        # img_arr =  img_arr.astype(np.float32) / 255.0
        # 将数组在内存的存放地址变成连续的(一维)， 行优先
        # 将一个内存不连续存储的数组转换为内存连续存储的数组，使得运行速度更快
        # https://zhuanlan.zhihu.com/p/59767914
        img_arr = np.ascontiguousarray(img_arr)
        return img_arr

    @torch.no_grad()
    def detect(self, img, canvas=None, view_img=True):
        '''模型预测'''
        # 图像预处理
        img_resize = self.preprocessing(img)  # 图像缩放
        self.img_torch = torch.from_numpy(img_resize).to(self.device)  # 图像格式转换
        self.img_torch = self.img_torch.half(
        ) if self.is_half else self.img_torch.float()  # 格式转换 uint8-> 浮点数
        self.img_torch /= 255.0  # 图像归一化
        if self.img_torch.ndimension() == 3:
            self.img_torch = self.img_torch.unsqueeze(0)
        # 模型推理
        t1 = time_sync()
        pred = self.model(self.img_torch, augment=False)[0]
        # pred = self.model_trt(self.img_torch, augment=False)[0]
        # NMS 非极大值抑制
        pred = non_max_suppression(pred, self.yolov5['threshold']['confidence'],
                                   self.yolov5['threshold']['iou'], classes=None, agnostic=False)
        t2 = time_sync()
        # print("推理时间: inference period = {}".format(t2 - t1))
        # 获取检测结果
        det = pred[0]
        gain_whwh = torch.tensor(img.shape)[[1, 0, 1, 0]]  # [w, h, w, h]

        if view_img and canvas is None:
            canvas = np.copy(img)
        xyxy_list = []
        conf_list = []
        class_id_list = []
        if det is not None and len(det):
            # 画面中存在目标对象
            # 将坐标信息恢复到原始图像的尺寸
            det[:, :4] = scale_coords(
                img_resize.shape[2:], det[:, :4], img.shape).round()
            for *xyxy, conf, class_id in reversed(det):
                class_id = int(class_id)
                xyxy_list.append(xyxy)
                conf_list.append(conf)
                class_id_list.append(class_id)
                if view_img:
                    # 绘制矩形框与标签
                    label = '%s %.2f' % (
                        self.yolov5['class_name'][class_id], conf)
                    self.plot_one_box(
                        xyxy, canvas, label=label, color=self.colors[class_id], line_thickness=3)
        return canvas, class_id_list, xyxy_list, conf_list

    def plot_one_box(self, x, img, color=None, label=None, line_thickness=None):
        ''''绘制矩形框+标签'''
        tl = line_thickness or round(
            0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
        color = color or [random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
        if label:
            tf = max(tl - 1, 1)  # font thickness
            t_size = cv2.getTextSize(
                label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
            cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
            cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3,
                        [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

class PredictionClientAsync(Node):

    def __init__(self):
        super().__init__('prediction_client_async')
        self.hand_eye= [[-0.00818685,  0.991227, -0.131914, 0.621423],
                       [0.999931, 0.00699775, -0.0094753 ,  -0.0256352],
                       [-0.00846908 ,  -0.131982,  -0.991216,  0.591267],
                       [ 0.        ,  0.        ,  0.        ,  1.        ]]
        self.srt = PythonSerialDriver("/dev/ttyUSB0")

        self.model = YoloV5(yolov5_yaml_path='src/prediction_client/prediction_client/yolov5_detect/config/yolov5s.yaml')

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

    #     self.robotState_sub = self.create_subscription(RobotState,'/franka_motion/robot_states',self.robotState_cb,10)
    
    # def robotState_cb(self,msg):
    #     print(msg)

    def pos_srt(self, pressure = 60):
        self.srt.move3Fingers(True, pressure)

    def neg_srt(self, pressure = 20):
        self.srt.move3Fingers(False, pressure)        

    def zero_srt(self, pressure = 0):
        self.srt.move3Fingers(True, pressure)

    def close_gripper(self,is_franka_hand=False):
        if is_franka_hand:
            response_gripper = self.gripper_request(False)
            self.get_logger().info(
            'Result for control franka hand executed with status %d' %
            (response_gripper.success))
        else:
            self.pos_srt()
            self.get_logger().info(
            'srt positve pressure to close gripper')

    def open_gripper(self,is_franka_hand=False):
        if is_franka_hand:
            response_gripper = self.gripper_request(True, 0.03)
            self.get_logger().info(
            'Result for control franka hand executed with status %d' %
            (response_gripper.success))
        else:
            self.neg_srt()
            self.get_logger().info(
            'srt negtive pressure to open gripper')
            # time.sleep(0.2)
            # self.pos_srt(0)

    def move_to_joints(self,joints,velscale):
        response_joint = self.send_joint_request(joints, velscale)
        self.get_logger().info(
        'Result for joint [%f,%f,%f,%f,%f,%f,%f] with %f speed factor with status %d' %
        (joints[0],joints[1],joints[2],joints[3],joints[4],joints[5],joints[6], velscale, response_joint.success))

    def cart_pose_time(self,pose,duration):
        response_cart = self.send_cart_request(pose, duration)
        self.get_logger().info(
        'Result for pose [%f,%f,%f,%f,%f,%f] with %fs time executed with status %d' %
        (pose[0],pose[1],pose[2],pose[3],pose[4],pose[5], duration, response_cart.success))

    def cart_path_time(self,poses,duration):
        response_posePath = self.send_posePath_request(poses, duration)
        self.get_logger().info(
        'Result for posePath executed with status %d' %
        (response_posePath.success))

    def detect_once(self,show_result = True):
    # Wait for a coherent pair of frames: depth and color
        intr, depth_intrin, color_image, depth_image, aligned_depth_frame = get_aligned_images()  # 获取对齐的图像与相机内参
        while not depth_image.any() or not color_image.any():
            intr, depth_intrin, color_image, depth_image, aligned_depth_frame = get_aligned_images()
        # Convert images to numpy arrays
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
            depth_image, alpha=0.03), cv2.COLORMAP_JET)
        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))
        
        # Show images

        t_start = time.time()  # 开始计时
        # YoloV5 目标检测
        canvas, class_id_list, xyxy_list, conf_list = self.model.detect(
            color_image)

        t_end = time.time()  # 结束计时\
        #canvas = np.hstack((canvas, depth_colormap))
        #print(class_id_list)

        camera_xyz_list=[]
        if xyxy_list:
            for i in range(len(xyxy_list)):
                ux = int((xyxy_list[i][0]+xyxy_list[i][2])/2)  # 计算像素坐标系的x
                uy = int((xyxy_list[i][1]+xyxy_list[i][3])/2)  # 计算像素坐标系的y
                dis = aligned_depth_frame.get_distance(ux, uy)
                camera_xyz = rs.rs2_deproject_pixel_to_point(
                    depth_intrin, (ux, uy), dis)  # 计算相机坐标系的xyz
                camera_xyz = np.round(np.array(camera_xyz), 3)  # 转成3位小数
                camera_xyz = camera_xyz.tolist()
                cv2.circle(canvas, (ux,uy), 4, (255, 255, 255), 5)#标出中心点
                cv2.putText(canvas, str(camera_xyz), (ux+20, uy+10), 0, 1,
                            [225, 255, 255], thickness=2, lineType=cv2.LINE_AA)#标出坐标
                camera_xyz_list.append(camera_xyz)
                print(camera_xyz_list)
                                # 添加fps显示
                if show_result:
                    fps = int(1.0 / (t_end - t_start))
                    cv2.putText(canvas, text="FPS: {}".format(fps), org=(50, 50),
                                fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, thickness=2,
                                lineType=cv2.LINE_AA, color=(0, 0, 0))
                    cv2.namedWindow('detection', flags=cv2.WINDOW_NORMAL |
                                    cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
                    cv2.imshow('detection', canvas)
                    k = cv2.waitKey(0)
                    if k == 27:  # wait for ESC key to exit
                        cv2.destroyAllWindows()
        return camera_xyz_list

    def hand_eye_transform(self, camera_xyz):
        bTc = self.hand_eye
        cTo = [[1.,  0, 0, camera_xyz[0]],
               [0, 1., 0 ,  camera_xyz[1]],
               [0 ,  0,  1., camera_xyz[2]],
               [ 0.  ,  0.  ,  0.  ,  1.   ]]

        bTo = np.dot(bTc,cTo)
        xyz = transform.translation_from_matrix(bTo)
        return xyz

    def soft_gripper_eef_transform(self,base_xyzrpy):
        bTo =transform.euler_matrix(base_xyzrpy[3],base_xyzrpy[4],base_xyzrpy[5])
        bTo[0,3]=base_xyzrpy[0]
        bTo[1,3]=base_xyzrpy[1]
        bTo[2,3]=base_xyzrpy[2]
        eTg = [[math.cos(math.pi/12),  -math.sin(math.pi/12), 0, 0],
               [math.sin(math.pi/12),   math.cos(math.pi/12),  0 ,  0],
               [0 ,  0,  1.,                                0.02],
               [ 0.  ,  0.  ,  0.  ,  1.   ]]
        
        bTe = np.dot(bTo,np.linalg.inv(eTg))
        xyz = transform.translation_from_matrix(bTe)
        rpy = transform.euler_from_matrix(bTe)
        return [xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2]]

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

    def add_waypoint_to_path(self,path,pose):
        path.extend(pose)

    def pose_offset(self,pose,x_offset,y_offset,z_offset):
        return [pose[0]+x_offset, pose[1]+y_offset, pose[2]+z_offset]

def main(args=None):
    rclpy.init(args=args)

    prediction_client = PredictionClientAsync()

    prediction_client.open_gripper()
    
    init_joint = [0.16312787685894567, -0.9984653897620572, -0.13817682679145038, -2.6634519014816442, -0.1601847994128863, 1.7038212698830497, 0.6833146796069212]
    velscale = 0.3
    prediction_client.move_to_joints(init_joint,velscale)
    time.sleep(0.1)
    pre_pick_joint = [0.13821194475575496, -0.4434162515090197, -0.12757366092581499, -2.4691517728457257, -0.09563205587863921, 2.0746510965419294, 0.6008980268862696]
    prediction_client.move_to_joints(pre_pick_joint,velscale)
    
    camera_xyz_list = prediction_client.detect_once()
    while len(camera_xyz_list) <1:
        camera_xyz_list = prediction_client.detect_once()
    base_xyz = prediction_client.hand_eye_transform(camera_xyz_list[0])
    print("base_xyz is ",base_xyz)

    target_pose = [base_xyz[0], base_xyz[1], base_xyz[2], 3.1415926, 0.0, 0.0]
    # target_pose = [0.512888, -0.024801, 0.170688, 3.1415926, 0.0, 0.0]
    target_pose = prediction_client.soft_gripper_eef_transform(target_pose)
    print("target_pose is:", target_pose)
    duration1 = 1.0



    poses = []
    pose_a = prediction_client.pose_offset(target_pose,0,0,0.2)
    prediction_client.add_waypoint_to_path(poses,pose_a)

    pose_b = prediction_client.pose_offset(pose_a,-0.2,-0.2,0)
    prediction_client.add_waypoint_to_path(poses,pose_b)

    pose_c = prediction_client.pose_offset(pose_a,-0.4,-0.3,0)
    prediction_client.add_waypoint_to_path(poses,pose_c)

    pose_d = prediction_client.pose_offset(pose_a,-0.4,-0.4,0.02)
    prediction_client.add_waypoint_to_path(poses,pose_d)

    print(poses)
    duration2 = 4.0



    prediction_client.cart_pose_time(target_pose,duration1)
    
    prediction_client.close_gripper()

    prediction_client.cart_path_time(poses,duration2)

    prediction_client.open_gripper()

    prediction_client.move_to_joints(pre_pick_joint,velscale)


    prediction_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()