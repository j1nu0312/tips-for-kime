#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

####################################################################################################
# Info
#
## Title    : "find_object_2d 로 물체 인식하여 open manipulator로 picking and tracking 하기"
## Blog     : https://cafe.naver.com/openrt/23335
## GitHub   : https://github.com/minwoominwoominwoo7/open_manipulator_find_object_2d
#
####################################################################################################




import rospy        # Python ROS 라이브러리
import roslaunch    # Python ROS Launch 라이브러리
import numpy as np
import subprocess   # linux command 들을 python 내에서 실행시키게 해주는 package
import os           # 디렉터리(폴더)나 경로, 파일 등 활용
import sys          # python 인터프리터가 제공하는 변수와 함수를 직접 제어

from enum import Enum   # 열거형(enumeration) 지원 [class enum.Enum : 열거형 상수를 만들기 위한 베이스 클래스]

# [ref] http://wiki.ros.org/std_msgs
from std_msgs.msg import UInt8, Float32MultiArray   # ROS 표준 메세지 패키지

# [ref] http://docs.ros.org/en/jade/api/tf/html/python/transformations.html
from tf.transformations import *
import tf

from PySide import QtCore, QtGui, QtOpenGL  # 윈도우(화면) 처리를 위한 라이브러리

# [ref] http://wiki.ros.org/sensor_msgs
from sensor_msgs.msg import JointState      # ROS Sensor 관련 메세지
# [ref] http://wiki.ros.org/geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Point  # ROS geometric 관련 메세지

from math import pow, atan2, sqrt

# Manipulator (OpenManipulator 관련 메세지)
# [ref] http://wiki.ros.org/open_manipulator_msgs
# [githib] https://github.com/ROBOTIS-GIT/open_manipulator_msgs
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.msg import KinematicsPose
from open_manipulator_msgs.msg import OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import SetKinematicsPose
from open_manipulator_msgs.srv import GetJointPosition
from open_manipulator_msgs.srv import GetKinematicsPose
from open_manipulator_msgs.srv import SetActuatorState
 
 
 # PickAndPlace 클래스 선언
class PickAndPlace():
    ## PickAndPlace 생성자 (초기화)
    def __init__(self):
        # [ref] https://docs.python.org/3.8/library/enum.html#functional-api
        self.CurrentMode = Enum('CurrentMode', 
                                   'idle \
                                    init \
                                    waitObject \
                                    move_to_pick \
                                    close_object \
                                    move_to_place' )   

        # [ref] http://docs.ros.org/en/diamondback/api/tf/html/c++/classtf_1_1TransformListener.html
        self.listener = tf.TransformListener()

        self.jointStates = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.kinematicsStates = [0.0, 0.0, 0.0]
        self.open_manipulator_moving_state = "STOPPED"
        self.current_mode = self.CurrentMode.init.value
        self.pickObjectPose = PoseStamped()     # A Pose with reference coordinate frame and timestamp
        self.pickTargetPose = PoseStamped()    
        self.placeObjectPose = PoseStamped()
        self.placeTargetPose = PoseStamped()         
        self.is_triggered = False
        self.currentToolPose = Pose()           # A representation of pose in free space, composed of position and orientation.

        ## [example]
        # 1 global_name = rospy.get_param("/global_name")
        # 2 relative_name = rospy.get_param("relative_name")
        # 3 private_param = rospy.get_param('~private_name')
        # 4 default_param = rospy.get_param('default_param', 'default_value')
        self.use_platform = rospy.get_param("~use_platform","true")

        ## 오픈메니퓰레이터에서 제공하는 서비스들
        # [ref] http://wiki.ros.org/open_manipulator_controller
        self.set_joint_position = rospy.ServiceProxy('goal_joint_space_path', SetJointPosition)
        self.set_kinematics_position = rospy.ServiceProxy('goal_task_space_path_position_only', SetKinematicsPose)
        self.set_joint_position_from_present = rospy.ServiceProxy('goal_joint_space_path_from_present', SetJointPosition)
        self.set_actuator_state = rospy.ServiceProxy('set_actuator_state', SetActuatorState)
        self.set_gripper_control = rospy.ServiceProxy('goal_tool_control', SetJointPosition)

        ## 오픈메니퓰레이터에서 제공하는 Published Topics
        self.open_manipulator_joint_states_sub_ = rospy.Subscriber('joint_states', JointState, self.jointStatesCallback)
        self.open_manipulator_kinematics_pose_sub_ = rospy.Subscriber('gripper/kinematics_pose', KinematicsPose, self.kinematicsPoseCallback)
        self.open_manipulator_states_sub = rospy.Subscriber('states', OpenManipulatorState, self.statesCallback)
        
        # 검출한 Object 정보와 관련
        self.object_sub = rospy.Subscriber('objects', Float32MultiArray, self.objectCallback)
        
        rospy.sleep(1)
        # actuator enable 
        self.actuatorTorque(True)   # 오픈 메니퓰레이터 On
        self.setInitPose()          # 초기위치로 이동

        loop_rate = rospy.Rate(10) # 10hz (1초에 10번 주기)
        
        while not rospy.is_shutdown() :
            if self.is_triggered == True:   # 조인트 정보 읽었느지 확인
                self.fnControlNode()        # Pick & Place 상태 제어 루프
                pass
            loop_rate.sleep()

    # 오픈 메니퓰레이터 기구학 좌표 (x,y,z) 업데이트
    def kinematicsPoseCallback(self, msg):
        self.currentToolPose = msg.pose
        rospy.logwarn(' currentToolPose x,y,z %.2f , %.2f, %.2f  ', \
                        self.currentToolPose.position.x, self.currentToolPose.position.y, self.currentToolPose.position.z )

    # Pick & Place 상태 제어 루프
    def fnControlNode(self):
        # lane_following
        if self.current_mode == self.CurrentMode.init.value:
            rospy.loginfo("init mode")
            #if self.setInitPose() :
            if self.setBackwardPose() :
                self.current_mode = self.CurrentMode.waitObject.value
                rospy.loginfo("init pose ok")                  

        elif self.current_mode == self.CurrentMode.waitObject.value:  
            rospy.loginfo("waitObject mode")          
            pick_object_detect_duration = rospy.get_rostime().to_sec() - self.pickObjectPose.header.stamp.to_sec()
            place_object_detect_duration = rospy.get_rostime().to_sec() - self.placeObjectPose.header.stamp.to_sec()
            rospy.logwarn("duration pick %.2f  , place %.2f ", pick_object_detect_duration , place_object_detect_duration )
            if pick_object_detect_duration < 1 and place_object_detect_duration < 1 :
                rospy.loginfo(" detect object ")
                self.pickTargetPose = self.pickObjectPose
                self.placeTargetPose = self.placeObjectPose
                self.current_mode = self.CurrentMode.move_to_pick.value

        elif self.current_mode == self.CurrentMode.move_to_pick.value: 
            rospy.loginfo("move_to_pick mode")  
            if self.moveToObject() :
                self.current_mode = self.CurrentMode.close_object.value
                rospy.loginfo("init pose ok")      

        elif self.current_mode == self.CurrentMode.close_object.value: 
            rospy.loginfo("close_to_pick mode")  
            if self.closeToObject() :
                self.current_mode = self.CurrentMode.move_to_place.value
                rospy.loginfo("closeToObject ok")

        elif self.current_mode == self.CurrentMode.move_to_place.value: 
            rospy.loginfo("move_to_place mode")  
            if self.moveToPlace() :
                self.current_mode = self.CurrentMode.idle.value
                rospy.loginfo("moveToPlace ok")

        elif self.current_mode == self.CurrentMode.idle.value: 
            rospy.loginfo("idle mode") 
            rospy.sleep(3)
            self.current_mode = self.CurrentMode.init.value

    # 물체 인식 관련 함수
    def objectCallback(self,msg):
        # init position
        pickObejectNumList = [100,101,102,103,104,105,106,107,108,109,110] 
        placeObejectNumList = [1,2,3,4,5,6,7,8,9,10]

        #rospy.logwarn( 'objectCallback' )   
        data = msg.data

        for i in range(0,len(data),12):
            id = int(data[i])

            if id in pickObejectNumList : 
                pass 
            else:
                rospy.loginfo("idle mode") 
                continue                

            objectWidth = data[i+1]
            objectHeight = data[i+2]
            qtHomography = QtGui.QTransform(data[i+3], data[i+4], data[i+5],
					               data[i+6], data[i+7], data[i+8],
					               data[i+9], data[i+10], data[i+11])
            qtTopLeft = qtHomography.map(QtCore.QPointF(0,0));     
            qtTopRight = qtHomography.map(QtCore.QPointF(objectWidth,0)); 
            qtBottomLeft = qtHomography.map(QtCore.QPointF(0,objectHeight)); 
            qtBottomRight = qtHomography.map(QtCore.QPointF(objectWidth,objectHeight)); 
            centX = int(( qtTopLeft.x() + qtTopRight.x() + qtBottomLeft.x() + qtBottomRight.x() ) / 4)
            centY = int(( qtTopLeft.y() + qtTopRight.y() + qtBottomLeft.y() + qtBottomRight.y() ) / 4)
            '''rospy.loginfo("Object %d detected, CentX,Y ( %d , %d ) witdh ( %d , %d ),corners point at TopLeft (%.0f,%.0f) TopRight(%.0f,%.0f) BottomLeft(%.0f,%.0f) BottomRight(%.0f,%.0f)\n",
                            id, centX, centY, objectWidth, objectWidth,
                            qtTopLeft.x(), qtTopLeft.y(),
                            qtTopRight.x(), qtTopRight.y(),
                            qtBottomLeft.x(), qtBottomLeft.y(),
                            qtBottomRight.x(), qtBottomRight.y())'''
            try:
                # [ref] http://docs.ros.org/en/diamondback/api/tf/html/c++/classtf_1_1Transformer.html
                (trans,rot) = self.listener.lookupTransform('link1', 'object_'+str(id), rospy.Time(0))
                self.pickObjectPose.header.stamp = rospy.get_rostime()
                self.pickObjectPose.pose.position.x = trans[0]
                self.pickObjectPose.pose.position.y = trans[1]
                self.pickObjectPose.pose.position.z = trans[2]
                #rospy.logwarn('ft of object_%s is  xyz( %.2f, %.2f, %.2f from link1 %.2f ', str(id), trans[0], trans[1], trans[2],self.pickObjectPose.header.stamp.to_sec())   
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn('there is not tf of pickObject_%s ' , str(id) )

        for i in range(0,len(data),12):
            id = int(data[i])

            if id in placeObejectNumList : 
                pass 
            else:
                rospy.loginfo("idle mode") 
                continue                

            objectWidth = data[i+1]
            objectHeight = data[i+2]
            qtHomography = QtGui.QTransform(data[i+3], data[i+4], data[i+5],
					               data[i+6], data[i+7], data[i+8],
					               data[i+9], data[i+10], data[i+11])
            qtTopLeft = qtHomography.map(QtCore.QPointF(0,0));     
            qtTopRight = qtHomography.map(QtCore.QPointF(objectWidth,0)); 
            qtBottomLeft = qtHomography.map(QtCore.QPointF(0,objectHeight)); 
            qtBottomRight = qtHomography.map(QtCore.QPointF(objectWidth,objectHeight)); 
            centX = int(( qtTopLeft.x() + qtTopRight.x() + qtBottomLeft.x() + qtBottomRight.x() ) / 4)
            centY = int(( qtTopLeft.y() + qtTopRight.y() + qtBottomLeft.y() + qtBottomRight.y() ) / 4)
            '''rospy.loginfo("Object %d detected, CentX,Y ( %d , %d ) witdh ( %d , %d ),corners point at TopLeft (%.0f,%.0f) TopRight(%.0f,%.0f) BottomLeft(%.0f,%.0f) BottomRight(%.0f,%.0f)\n",
                            id, centX, centY, objectWidth, objectWidth,
                            qtTopLeft.x(), qtTopLeft.y(),
                            qtTopRight.x(), qtTopRight.y(),
                            qtBottomLeft.x(), qtBottomLeft.y(),
                            qtBottomRight.x(), qtBottomRight.y())'''
            try:
                (trans,rot) = self.listener.lookupTransform('link1', 'object_'+str(id), rospy.Time(0))
                self.placeObjectPose.header.stamp = rospy.get_rostime()
                self.placeObjectPose.pose.position.x = trans[0]
                self.placeObjectPose.pose.position.y = trans[1]
                self.placeObjectPose.pose.position.z = trans[2]
                #rospy.logwarn('ft of object_%s is  xyz( %.2f, %.2f, %.2f from link1 %.2f ', str(id), trans[0], trans[1], trans[2],self.pickObjectPose.header.stamp.to_sec())   
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn('there is not tf of placeObject_%s ' , str(id) )                

    # 오픈메니퓰레이터 물체로 이동
    def moveToObject(self):
        rospy.logwarn("move to object")
        resp = False
        end_effector_name = "gripper" 
        kinematics_pose = KinematicsPose()
        planning_group = "arm"        
        kinematics_pose.pose = self.pickTargetPose.pose
        kinematics_pose.pose.position = self.forwardObjectPosition( kinematics_pose.pose.position, -0.05 )
        kinematics_pose.pose.position.z += 0.03

        moveDistance = math.sqrt((kinematics_pose.pose.position.x - self.currentToolPose.position.x)**2 
                               + (kinematics_pose.pose.position.y - self.currentToolPose.position.y)**2       
                               + (kinematics_pose.pose.position.z - self.currentToolPose.position.z)**2 )

        #distance 0.3 m -> 3 sec operate time 
        #distance 0.1 m -> 1 sec operate time        

        operating_time = moveDistance * 10
        operating_limit_time = operating_time

        if operating_time < 1 :
            operating_limit_time = 1
        elif operating_time > 3 :
            operating_limit_time = 3        

        rospy.logwarn("go xyz %.2f,%.2f,%.2f , moveDistance %.2f, operate time %.2f ( %.2f )" ,\
                       kinematics_pose.pose.position.x, kinematics_pose.pose.position.y, kinematics_pose.pose.position.z, \
                       moveDistance, operating_time , operating_limit_time)       

        try:
            resp = self.set_kinematics_position(planning_group, end_effector_name, kinematics_pose, operating_time)
            print 'kinemetics resp1 {} time '.format(resp.is_planned, operating_time)
            rospy.sleep(operating_time)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

        return resp

    # 오픈메니퓰레이터 물체로 잡기
    def closeToObject(self):
        rospy.logwarn("close to object")
        resp = False
        end_effector_name = "gripper" 
        kinematics_pose = KinematicsPose()
        planning_group = "arm"
        kinematics_pose.pose = self.pickTargetPose.pose
        kinematics_pose.pose.position = self.forwardObjectPosition( kinematics_pose.pose.position, 0.03 )
        if self.use_platform :
            rospy.logwarn("??????1")
            kinematics_pose.pose.position.z += 0.05
        else :
            rospy.logwarn("??????2")
            kinematics_pose.pose.position.z -= 0.05

        kinematics_pose.pose.position.y += 0.005

        moveDistance = math.sqrt((kinematics_pose.pose.position.x - self.currentToolPose.position.x)**2 
                               + (kinematics_pose.pose.position.y - self.currentToolPose.position.y)**2       
                               + (kinematics_pose.pose.position.z - self.currentToolPose.position.z)**2 )

        #distance 0.3 m -> 3 sec operate time 
        #distance 0.1 m -> 1 sec operate time        

        operating_time = moveDistance * 10
        operating_limit_time = operating_time

        if operating_time < 1 :
            operating_limit_time = 1
        elif operating_time > 3 :
            operating_limit_time = 3        

        rospy.logwarn("go xyz %.2f,%.2f,%.2f , moveDistance %.2f, operate time %.2f ( %.2f )" ,\
                       kinematics_pose.pose.position.x, kinematics_pose.pose.position.y, kinematics_pose.pose.position.z, \
                       moveDistance, operating_time , operating_limit_time)       

        try:
            resp = self.set_kinematics_position(planning_group, end_effector_name, kinematics_pose, operating_time)
            print 'kinemetics resp1 {} time '.format(resp.is_planned, operating_time)
            rospy.sleep(operating_time)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False

        return resp        

    # 오픈메니퓰레이터 내려놓기 장소로 이동
    def moveToPlace(self):
        rospy.logwarn("move_to_place")

        # close gripper 
        joint_position = JointPosition()
        joint_position.joint_name = ['gripper']  
        joint_position.position =  [-0.01] #-0.01 0.01
        resp = False
        try:    
            path_time = 1       # ??
            resp = self.set_gripper_control("",joint_position, path_time)
            rospy.sleep(path_time)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e  
        if not resp :
            return False 
        
        # initial pose
        self.setBackwardPose2()    

        # go place position
        rospy.logwarn("go to place")
        resp = False
        end_effector_name = "gripper" 
        kinematics_pose = KinematicsPose()
        planning_group = "arm"
        kinematics_pose.pose = self.placeTargetPose.pose
        kinematics_pose.pose.position = self.forwardObjectPosition( kinematics_pose.pose.position, 0.03 )
        kinematics_pose.pose.position.z += 0.13
        kinematics_pose.pose.position.x += 0.015

        moveDistance = math.sqrt((kinematics_pose.pose.position.x - self.currentToolPose.position.x)**2 
                               + (kinematics_pose.pose.position.y - self.currentToolPose.position.y)**2       
                               + (kinematics_pose.pose.position.z - self.currentToolPose.position.z)**2 )

        #distance 0.3 m -> 3 sec operate time 
        #distance 0.1 m -> 1 sec operate time        

        operating_time = moveDistance * 10
        operating_limit_time = operating_time

        if operating_time < 1 :
            operating_limit_time = 1
        elif operating_time > 3 :
            operating_limit_time = 3        

        rospy.logwarn("go xyz %.2f,%.2f,%.2f , moveDistance %.2f, operate time %.2f ( %.2f )" ,\
                       kinematics_pose.pose.position.x, kinematics_pose.pose.position.y, kinematics_pose.pose.position.z, \
                       moveDistance, operating_time , operating_limit_time)       

        try:
            resp = self.set_kinematics_position(planning_group, end_effector_name, kinematics_pose, operating_time)
            print 'kinemetics resp1 {} time '.format(resp.is_planned, operating_time)
            rospy.sleep(operating_time)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False             

        # open gripper 
        joint_position = JointPosition()
        joint_position.joint_name = ['gripper']  
        joint_position.position =  [0.01] #-0.01 0.01
        resp = False
        try:    
            path_time = 1                    
            resp = self.set_gripper_control("",joint_position, path_time)
            rospy.sleep(path_time)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e  
        if not resp :
            return False 

        return True          


    # 물체 위치 계산 (물체 위치 + 진행 거리)
    def forwardObjectPosition( self, objectPosition, forward_distance ):
        resultPoint = Point()
        if(abs(objectPosition.x) < 0.001) :
            objectPosition.x = 0.001
        radian = math.atan(objectPosition.y/objectPosition.x)
        degree = math.degrees(radian)
        dist = forward_distance
        distX = math.cos(radian)*dist
        distY = math.sin(radian)*dist
        resultPoint.x = objectPosition.x + distX 
        resultPoint.y = objectPosition.y + distY
        resultPoint.z = objectPosition.z 
        rospy.loginfo("%.2f m forward,so objectposition change xyz(%.2f ,%.2f, %.2f) -> xyz(%.2f ,%.2f, %.2f)",
                       forward_distance, objectPosition.x, objectPosition.y, objectPosition.z , 
                       resultPoint.x, resultPoint.y, resultPoint.z)
        return resultPoint

    # 오픈메니퓰레이터 전원 설정 Set Actuator (Torque On/Off)
    def actuatorTorque(self, enable):
        rospy.logwarn("actuatorTorque")
        joint_name = ['joint1','joint2','joint3','joint4','gripper']
        try:                  
            resp = self.set_actuator_state(enable)
            rospy.sleep(1)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e  
            return False
        if not resp :
            rospy.loginfo("set_actuator enable fail")        
        return resp

    # Set 초기위치 이동
    def setInitPose(self):
        rospy.logwarn("setInitPose")
        # init position
        joint_position = JointPosition()
        joint_position.joint_name = ['joint1','joint2','joint3','joint4']  
        joint_position.position =  [0.0, -1.05, 0.35, 0.70]   
        #joint_position.position =  [0.0, -1.791, 0.507, 1.438]     
        resp = False    
        try:    
            path_time = 2                    
            resp = self.set_joint_position("",joint_position, path_time)
            rospy.sleep(path_time)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e  
        if not resp :
            return False    

    # 오픈메니퓰레이터 물러섬 위치 이동 (그리퍼 오픈)
    def setBackwardPose(self):
        rospy.logwarn("setInitPose")
        # init position
        joint_position = JointPosition()
        joint_position.joint_name = ['joint1','joint2','joint3','joint4']  
        #joint_position.position =  [0.0, -1.05, 0.35, 0.70]   
        joint_position.position =  [0.0, -1.791, 0.507, 1.438]     
        resp = False    
        try:    
            path_time = 2                    
            resp = self.set_joint_position("",joint_position, path_time)
            rospy.sleep(path_time)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e  
        if not resp :
            return False               

        # open gripper 
        joint_position = JointPosition()
        joint_position.joint_name = ['gripper']  
        joint_position.position =  [0.01] #-0.01 0.01
        resp = False
        try:    
            path_time = 1        # 그리퍼 ??
            resp = self.set_gripper_control("",joint_position, path_time)
            rospy.sleep(path_time)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e  
        if not resp :
            return False 

        return True         

    # 오픈메니퓰레이터 물러섬 위치2 이동 (그리퍼 X)
    def setBackwardPose2(self):
        rospy.logwarn("setInitPose")
        # init position
        joint_position = JointPosition()
        joint_position.joint_name = ['joint1','joint2','joint3','joint4']  
        #joint_position.position =  [0.0, -1.05, 0.35, 0.70]   
        joint_position.position =  [0.0, -1.791, 0.507, 1.438]     
        resp = False    
        try:    
            path_time = 2                    
            resp = self.set_joint_position("",joint_position, path_time)
            rospy.sleep(path_time)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e  
        if not resp :
            return False               

        return True              

    # 오픈메니퓰레이터 x,y,z 좌표 업데이트
    def kinematicsPoseCallback(self, msg):
        self.kinematicsStates[0] = msg.pose.position.x
        self.kinematicsStates[1] = msg.pose.position.y
        self.kinematicsStates[2] = msg.pose.position.z
        #rospy.logwarn(' kinematicsPoseCallback %.2f , %.2f, %.2f  ', self.kinematicsStates[0], self.kinematicsStates[1], self.kinematicsStates[2] )

    # 오픈메니퓰레이터 조인트 값 업데이트
    def jointStatesCallback(self, msg):
	    #rospy.logwarn('jointStatesCallback %d ', len(msg.position) )
        self.is_triggered = True
        for i, pose in enumerate(msg.position):
            self.jointStates[i] = pose
            #print 'boundingBoxe {} {} '.format(i, pose)            

    # 오픈메니퓰레이터 상태 업데이트
    def statesCallback(self, msg):	
        self.open_manipulator_moving_state = msg.open_manipulator_moving_state

    def main(self):
        rospy.spin()


# Main 함수
if __name__ == '__main__':
    rospy.init_node('pick_node_controller')     # 'pick_node_controller' 이름의 노드 생성
    rospy.loginfo("pick_node_controller")       # 로그 정보 출력
    node = PickAndPlace()                       # PickAndPlace 클래스 인스턴트 생성
    node.main()                                 # ROS Node 루프 (rospy.spin())
