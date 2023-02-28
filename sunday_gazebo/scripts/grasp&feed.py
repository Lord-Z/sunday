#coding=utf-8

# changement
# line 51 graspDemo.move
# line 177 get_RT_matrix
# line 207
# line 230 mouthDetectCallBack
# line 292--end
# line 316 is the decrease distance every step, you can change it to whatever you like, but too large may casue big bias

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
    
        self.reference_frame = 'world'
        
        moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.robot.RobotCommander()

        self.arm = moveit_commander.move_group.MoveGroupCommander("manipulator")
        self.end_effector_link = self.arm.get_end_effector_link()
        
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.05)
        self.arm.allow_replanning(True)
        self.arm.set_pose_reference_frame(self.reference_frame)

        rospy.wait_for_service('/sunday/vacuum_gripper/off')
        rospy.wait_for_service('/sunday/vacuum_gripper/on')
        self.vacuumOn=rospy.ServiceProxy('/sunday/vacuum_gripper/on', Empty)
        self.vacuumOff=rospy.ServiceProxy('/sunday/vacuum_gripper/off', Empty)

    def move(self, pose, orientation):
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
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        status = self.arm.go()
        
        return status

        # go change to plan and execute to get feedback
        # trajectory = self.arm.plan()

        # if trajectory[0]:
        #     self.arm.execute(trajectory[1])
        #     return True
        # else:
        #     rospy.loginfo('plan to pose failed')
        #     return False

    def go_named(self, place = 'home'):
        # go back home
        self.arm.set_named_target(place)
        self.arm.go(wait=True)
        
    @property
    def poseNow(self):
        return self.arm.get_current_pose(self.end_effector_link)
    
    def move_by_posNow(self, dx, dy, dz):
        self.arm.set_start_state_to_current_state()
        
        pos = self.poseNow.pose
        pos.position.x -= dx
        pos.position.y -= dy
        pos.position.z -= dz
        
        self.arm.set_pose_target(pos, self.end_effector_link)
        self.arm.go(wait=True)
        
    def move_by_joints(self,joints):
        self.arm.set_joint_value_target(joints)
        state = self.arm.go(wait=True)

        return state
     
    def move_cartesian(self, x, y, z):    
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
        moveit_commander.roscpp_initializer.roscpp_shutdown()
        rospy.loginfo('grasp complete!!!')

def get_rotation_matrix(q):
    # in TF, it is xyzw
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]
    rot = [[1-2*y*y-2*z*z, 2*x*y+2*w*z, 2*x*z-2*w*y], 
            [2*x*y-2*w*z, 1-2*x*x-2*z*z, 2*y*z+2*w*x],
            [2*x*z+2*w*y, 2*y*z-2*w*x, 1-2*x*x-2*y*y]]
    return rot

def get_CameraFrame_Pos(u, v, depthValue):
    fx = 1043.99267578125
    fy = 1043.99267578125
    cx = 960
    cy = 540

    z = float(depthValue)
    x = float((u - cx) * z) / fx
    y = float((v - cy) * z) / fy

    return [x, y, z, 1]

def get_RT_matrix(base_frame, reference_frame):
    # may get tf error even after 3 times and cause a error, so return in try--except
    listener = tf.TransformListener()
    i = 3
    while i!=0:
        try:
            listener.waitForTransform(base_frame, reference_frame,rospy.Time.now(), rospy.Duration(3.0))
            camera2World = listener.lookupTransform(base_frame, reference_frame, rospy.Time(0))
            T = camera2World[0]
            R = get_rotation_matrix(camera2World[1])
            R[0].append(0)
            R[1].append(0)
            R[2].append(0)
            R.append([0.0,0.0,0.0,1.0])
            R = np.mat(R) 
            return [R,T]
        except:           
            rospy.loginfo('tf error!!!')
        i = i - 1

def coordinate_transform(cameraFrame_pos, R, T):
    worldFrame_pos = R.I * np.mat(cameraFrame_pos).T 
    worldFrame_pos[0,0] = worldFrame_pos[0,0] + T[0]
    worldFrame_pos[1,0] = worldFrame_pos[1,0] + T[1]
    worldFrame_pos[2,0] = worldFrame_pos[2,0] + T[2]
    worldFrame_pos = [worldFrame_pos[0,0], worldFrame_pos[1,0], worldFrame_pos[2,0]]
    return worldFrame_pos

findObject = False
# add a findPerson Flag
findPerson = False
depthImg = np.array(0)
depthOK = False

def BoundingBoxCallBack(data):
    global findObject, u, v, graspObject

    if not findObject:
        if graspObject == '':
            graspObject = raw_input('object detected, please input the object you want to grasp:(cube, trangle, star)\n')
        objectGrasp = []
        for dat in data.bounding_boxes:
            if graspObject == dat.Class:
                objectGrasp.append([dat.Class, (dat.xmin + dat.xmax)/2, (dat.ymin + dat.ymax)/2])
        if objectGrasp != []:
            rospy.loginfo('{} found, begin grasp!!!'.format(graspObject))
            _, u, v = random.choice(objectGrasp)
            findObject = True
        else:
            rospy.loginfo('The object you want to grasp is absent!!!')

def mouthDetectCallBack(data):
    global findPerson, u, v, feedState

    if feedState and not findPerson: # food ready
        personFound = []
        for dat in data.bounding_boxes:
            if 'open' == dat.Class:
                personFound.append([dat.Class, (dat.xmin + dat.xmax)/2, (dat.ymin + dat.ymax)/2])
        if personFound != []:
            rospy.loginfo('person mouth found, begin feeding!!!')
            _, u, v = random.choice(personFound)
            findPerson = True

def depthCallback(data):
    global depthImg,depthOK
    depthOK = False
    depthImg = CvBridge().imgmsg_to_cv2(data, data.encoding)
    depthOK = True
    
endDown = [0.02529503620768233, 0.7063632122071156, 0.025274909157321696, 0.7069457918049917]
horizon = [0.707135042256876, -4.911317675396038e-05, -1.8686383267952342e-05, 0.7070785170340573]
def grasp_test(grasp_demo):
    # test vacuum grasp
    print(grasp_demo.poseNow)
    grasp_demo.move([0.2,0.15,1.05],endDown)
    grasp_demo.vacuumOn()
    rospy.sleep(0.5)
    grasp_demo.move([-0.2,0.15,1.05],endDown)
    grasp_demo.vacuumOff()
    grasp_demo.go_named()

if __name__ == "__main__":
    depth_topic = 'camera/depth/image_raw'
    BoundingBox_topic = '/yolov5/BoundingBoxes'
    mouthDetectTopic = '/yolov5/mouth/BoundingBoxes'

    rospy.init_node('grasp')
    grasp_demo = graspDemo()
    grasp_demo.go_named('scan_food') # move to place that can found food
    # print(grasp_demo.poseNow) 
    u = 960
    v = 540
    graspObject = ''
    feedState = 0
    rospy.Subscriber(BoundingBox_topic, BoundingBoxes, BoundingBoxCallBack, queue_size=1) 
    rospy.Subscriber(mouthDetectTopic, BoundingBoxes, mouthDetectCallBack, queue_size=1) 
    rospy.Subscriber(depth_topic, Image, depthCallback, queue_size=1) 

    while not rospy.is_shutdown():
        if findObject and depthOK and not feedState:
            # get rotation and translation matrix
            [R, T] = get_RT_matrix('world','camera_color_optical_frame')
            u, v = int(u), int(v)
            cameraFrame_pos = get_CameraFrame_Pos(u, v, depthImg[v,u]/1000.0)
            worldFrame_pos = coordinate_transform(cameraFrame_pos, R, T)
            print(graspObject + ' pose is: ', worldFrame_pos)

            grasp_demo.move([worldFrame_pos[0], worldFrame_pos[1], worldFrame_pos[2]+0.015],endDown)
            grasp_demo.vacuumOn()
            rospy.sleep(0.5) # wait for grasp  

            # food ready and then go to recognize mouth
            grasp_demo.go_named('scan_face2')
            rospy.sleep(3) # wait for detect mouth ready
            rospy.loginfo('ready to detect mouth!!!')
            feedState = 1 # set a flag to show detectMouth ready

        if feedState and findPerson and depthOK: # ready to feed and findperson
            # may cause a error, so use try--except
            try:
                [R, T] = get_RT_matrix('world','camera_color_optical_frame')
            except:
                rospy.loginfo('get tf error!!!')
            # same as before, get mouth pose
            u, v = int(u), int(v)
            cameraFrame_pos = get_CameraFrame_Pos(u, v, depthImg[v,u]/1000.0)
            worldFrame_pos = coordinate_transform(cameraFrame_pos, R, T)
            print('person mouth pose is: ', worldFrame_pos)
            # try to move to mouth position, decrease 0.03 to avoid collision
            status = grasp_demo.move([worldFrame_pos[0]-0.03, worldFrame_pos[1], worldFrame_pos[2]],horizon)
            # if failed to plan, try to decrease x--distance every step
            if not status:
                x = worldFrame_pos[0] - 0.03 # origin x
                while x - worldFrame_pos[0]>=-0.3 and not status:
                    print('plan to {} failed'.format([x, worldFrame_pos[1], worldFrame_pos[2]]))
                    x = x - 0.05 # decrease
                    status = grasp_demo.move([x, worldFrame_pos[1], worldFrame_pos[2]],horizon)
            # release food
            grasp_demo.vacuumOff()
            grasp_demo.go_named('scan_food')
            print('feed complete!!!')

            feedState = 0
            findPerson = False
            findObject = False
            graspObject = ''