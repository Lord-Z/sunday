#coding=utf-8


import cv2

import rospy
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import sys
import moveit_commander
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes
import random

class graspDemo:
    def __init__(self):
        # 机械臂规划参考系
        self.reference_frame = 'world'
        # 初始化moveit控制器
        moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.robot.RobotCommander()
        # 初始化manipulator group
        self.arm = moveit_commander.move_group.MoveGroupCommander("manipulator")
        self.end_effector_link = self.arm.get_end_effector_link()
        # 设置容忍误差
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.05)
        self.arm.allow_replanning(True)
        self.arm.set_pose_reference_frame(self.reference_frame)
        # 吸盘吸取服务
        rospy.wait_for_service('/sunday/vacuum_gripper/off')
        rospy.wait_for_service('/sunday/vacuum_gripper/on')
        self.vacuumOn=rospy.ServiceProxy('/sunday/vacuum_gripper/on', Empty)
        self.vacuumOff=rospy.ServiceProxy('/sunday/vacuum_gripper/off', Empty)

    def move(self, pose, orientation):
        # 机械臂移动控制 pose 位置 orientation 方向
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = pose[0]
        target_pose.pose.position.y = pose[1]
        target_pose.pose.position.z = pose[2]
        target_pose.pose.orientation.x = orientation[0]
        target_pose.pose.orientation.y = orientation[1]
        target_pose.pose.orientation.z = orientation[2]
        target_pose.pose.orientation.w = orientation[3]
        # 设置当前坐标为起始坐标
        self.arm.set_start_state_to_current_state()
        # 设置位置目标
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        self.arm.go(wait=True)

    def go_named(self, place = 'home'):
        # 移动至命名的pose
        self.arm.set_named_target(place)
        self.arm.go(wait=True)
        
    @property
    def poseNow(self):
        # 返回当前坐标
        return self.arm.get_current_pose(self.end_effector_link)
    
    def move_by_posNow(self, dx, dy, dz):
        # 在当前坐标的基础上设置目标
        self.arm.set_start_state_to_current_state()
        
        pos = self.poseNow.pose
        pos.position.x -= dx
        pos.position.y -= dy
        pos.position.z -= dz
        
        self.arm.set_pose_target(pos, self.end_effector_link)
        self.arm.go(wait=True)
        
    def move_by_joints(self,joints):
        # 设置关节坐标
        self.arm.set_joint_value_target(joints)
        state = self.arm.go(wait=True)

        return state
     
    def move_cartesian(self, x, y, z):    
        # 笛卡尔规划
        waypoints = []
        pos = self.poseNow.pose
        pos.position.x = x
        pos.position.y = y
        pos.position.z = z
        waypoints.append(pos)
        fraction = 0.0 
        maxtries = 100
        attempts = 0
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.set_start_state_to_current_state()
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.1,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
            attempts += 1
            
        if fraction == 1.0:
            self.arm.execute(plan)
            return True
        else:
            pass
        return False

    def move_cartesian_byNow(self, x, y, z):    
        # 笛卡尔规划（从当前坐标设置目标）
        waypoints = []
        pos = self.poseNow.pose
    #        waypoints.append(pos)
        pos.position.x -= x
        pos.position.y -= y
        pos.position.z -= z
        # pos.orientation.x = preState['down'][0]
        # pos.orientation.y = preState['down'][1]
        # pos.orientation.z = preState['down'][2]
        # pos.orientation.w = preState['down'][3]
        waypoints.append(pos)
        fraction = 0.0 
        maxtries = 100
        attempts = 0
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.arm.set_start_state_to_current_state()
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.1,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
            attempts += 1
            
        if fraction == 1.0:
            self.arm.execute(plan)
            return True
        else:
            pass
        return False
                
    def shutDowm(self):
        # moveit关闭
        moveit_commander.roscpp_initializer.roscpp_shutdown()
        rospy.loginfo('grasp complete!!!')

def get_rotation_matrix(q):
    # in TF, it is xyzw
    # xyzw方向转旋转矩阵
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]
    rot = [[1-2*y*y-2*z*z, 2*x*y+2*w*z, 2*x*z-2*w*y], 
            [2*x*y-2*w*z, 1-2*x*x-2*z*z, 2*y*z+2*w*x],
            [2*x*z+2*w*y, 2*y*z-2*w*x, 1-2*x*x-2*y*y]]
    return rot

def get_CameraFrame_Pos(u, v, depthValue):
    # 图像系转相机系（u、v图像坐标，depthValue对应坐标的深度值）
    # fx fy cx cy为相机内参
    fx = 1043.99267578125
    fy = 1043.99267578125
    cx = 960
    cy = 540

    z = float(depthValue)
    x = float((u - cx) * z) / fx
    y = float((v - cy) * z) / fy

    return [x, y, z, 1]

def get_RT_matrix(base_frame, reference_frame):
    # 获取base_frame到reference_frame旋转平移矩阵，通过tf变换获取
    listener = tf.TransformListener()
    i = 3 # 尝试3次，三次未获取到则获取失败
    while i!=0:
        try:
            listener.waitForTransform(base_frame, reference_frame,rospy.Time.now(), rospy.Duration(3.0))
            camera2World = listener.lookupTransform(base_frame, reference_frame, rospy.Time(0))
            break
        except:           
            pass
        i = i - 1
    
    T = camera2World[0]
    R = get_rotation_matrix(camera2World[1])
    R[0].append(0)
    R[1].append(0)
    R[2].append(0)
    R.append([0.0,0.0,0.0,1.0])
    R = np.mat(R) 
    return [R,T]

def coordinate_transform(cameraFrame_pos, R, T):
    # 相机系转世界坐标系，先旋转再平移
    worldFrame_pos = R.I * np.mat(cameraFrame_pos).T 
    worldFrame_pos[0,0] = worldFrame_pos[0,0] + T[0]
    worldFrame_pos[1,0] = worldFrame_pos[1,0] + T[1]
    worldFrame_pos[2,0] = worldFrame_pos[2,0] + T[2]
    worldFrame_pos = [worldFrame_pos[0,0], worldFrame_pos[1,0], worldFrame_pos[2,0]]
    return worldFrame_pos

findObject = False
depthImg = np.array(0)
depthOK = False

def BoundingBoxCallBack(data):
    # yolo检测的回调函数
    global findObject, u, v, graspObject

    if not findObject:
        # 待抓取目标为空，则请求输入抓取目标
        if graspObject == '':
            graspObject = raw_input('object detected, please input the object you want to grasp:(cube, trangle, star)\n')
        objectGrasp = []
        for dat in data.bounding_boxes:
            # 遍历所有目标，种类与待抓取目标相同则保存目标中心位置
            if graspObject == dat.Class:
                objectGrasp.append([dat.Class, (dat.xmin + dat.xmax)/2, (dat.ymin + dat.ymax)/2])
        if objectGrasp != []:
            # 如果待抓取目标存在，则在目标列表随机选择一个返回
            rospy.loginfo('{} found, begin grasp!!!'.format(graspObject))
            _, u, v = random.choice(objectGrasp)
            findObject = True
        else:
            rospy.loginfo('The object you want to grasp is absent!!!')

def depthCallback(data):
    # 深度图像回调函数
    global depthImg,depthOK
    depthOK = False
    depthImg = CvBridge().imgmsg_to_cv2(data, data.encoding)
    depthOK = True
# 吸盘向下对应的xyzw坐标
endDown = [0.02529503620768233, 0.7063632122071156, 0.025274909157321696, 0.7069457918049917]

def grasp_test(grasp_demo):
    # 定点抓取测试
    print(grasp_demo.poseNow)
    grasp_demo.move([0.2,0.15,1.05],endDown)
    grasp_demo.vacuumOn()
    rospy.sleep(0.5)
    grasp_demo.move([-0.2,0.15,1.05],endDown)
    grasp_demo.vacuumOff()
    grasp_demo.go_named()

if __name__ == "__main__":
    # 深度图像和检测框对应topic名称
    depth_topic = 'camera/depth/image_raw'
    BoundingBox_topic = '/yolov5/BoundingBoxes'
    # 初始化ros节点
    rospy.init_node('grasp')
    # 实例化抓取控制类
    grasp_demo = graspDemo()
    # 机械臂移动到该位置
    grasp_demo.go_named('scan_food') # move to place that can found food
    # 初始化图像系坐标及待抓取物体
    u = 960
    v = 540
    graspObject = ''
    # 订阅深度图像和检测框对应topic
    rospy.Subscriber(BoundingBox_topic, BoundingBoxes, BoundingBoxCallBack, queue_size=1) 
    rospy.Subscriber(depth_topic, Image, depthCallback, queue_size=1) 

    while not rospy.is_shutdown():
        # 如果检测到待抓取物体且获取到了深度图
        if findObject and depthOK:
            # 获取相机系到世界系的旋转平移矩阵
            [R, T] = get_RT_matrix('world','camera_color_optical_frame')
            u, v = int(u), int(v) # 防止检测的坐标越界
            # 图像坐标转相机系坐标
            cameraFrame_pos = get_CameraFrame_Pos(u, v, depthImg[v,u]/1000.0)
            # 相机系坐标转世界系坐标
            worldFrame_pos = coordinate_transform(cameraFrame_pos, R, T)
            print(graspObject + ' pose is: ', worldFrame_pos)
            
            # 移动到抓取物体的上方
            grasp_demo.move([worldFrame_pos[0], worldFrame_pos[1], worldFrame_pos[2]+0.015],endDown)
            # 开启吸盘
            grasp_demo.vacuumOn()
            rospy.sleep(0.5) # wait for grasp
            # 向上移动仿真防止直接移动吸到其他物体
            grasp_demo.move_cartesian_byNow(0, 0 , -0.1) # avoid grasp other object by mistake
            # 移动到放置位置
            grasp_demo.move([worldFrame_pos[0], worldFrame_pos[1], worldFrame_pos[2]+0.015],endDown)
            # 关闭吸盘
            grasp_demo.vacuumOff()
            # 回到检测的位置
            grasp_demo.go_named('scan_food')            
            # 重置参数
            findObject = False
            graspObject = ''