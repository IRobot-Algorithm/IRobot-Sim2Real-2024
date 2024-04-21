#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import numpy as np
from scipy.spatial.transform import Rotation as sciR
from math import sqrt, atan2


import rospy
from geometry_msgs.msg import Twist, Pose, Point
from rmus_solution.srv import graspsignal, graspsignalResponse
from std_msgs.msg import Bool

import tf2_ros
import tf2_geometry_msgs
#from simple_pid import PID

class manipulater:
    def __init__(self) -> None:
        self.arm_gripper_pub = rospy.Publisher("arm_gripper", Point, queue_size=2)
        self.arm_position_pub = rospy.Publisher("arm_position", Pose, queue_size=2)
        self.timeout_pub = rospy.Publisher("/timeout", Bool, queue_size=2)#发布微调是否超时
        self.cmd_vel_puber = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/get_blockinfo", Pose, self.markerPoseCb, queue_size=1)

        self.current_marker_poses = None
        self.image_time_now = 0
        self.observation = np.array([0.0, 0.0])

        self.ros_rate = 30
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        self.service = rospy.Service(
            "/let_manipulater_work", graspsignal, self.trimerworkCallback
        )
        
    def markerPoseCb(self, msg):
        self.current_marker_poses = msg
        self.image_time_now = rospy.get_time()
        self.observation = [msg.position.x, msg.position.z]

    def getTargetPosAndAngleInBaseLinkFrame(self, pose_in_cam):
        if not self.tfBuffer.can_transform(
            "base_link", "camera_aligned_depth_to_color_frame_correct", rospy.Time.now()
        ):
            rospy.logerr(
                "pick_and_place: cannot find transform between base_link and camera_aligned_depth_to_color_frame_correct"
            )
            return None, None
        posestamped_in_cam = tf2_geometry_msgs.PoseStamped()
        posestamped_in_cam.header.stamp = rospy.Time.now()
        posestamped_in_cam.header.frame_id = (
            "camera_aligned_depth_to_color_frame_correct"
        )
        posestamped_in_cam.pose = pose_in_cam
        posestamped_in_base = self.tfBuffer.transform(posestamped_in_cam, "base_link")
        pose_in_base = posestamped_in_base.pose
        pos = np.array(
            [pose_in_base.position.x, pose_in_base.position.y, pose_in_base.position.z]
        )
        quat = np.array(
            [
                pose_in_cam.orientation.x,
                pose_in_cam.orientation.y,
                pose_in_cam.orientation.z,
                pose_in_cam.orientation.w,
            ]
        )
        angle = sciR.from_quat(quat).as_euler("YXZ")[0]
        #print("target_pos:", pos, "target_angle:", angle)
        return pos, angle

    def sendBaseVel(self, vel):
        twist = Twist()
        twist.linear.z = 0.0
        twist.linear.x = vel[0]
        twist.linear.y = vel[1]
        twist.angular.z = vel[2]
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        self.cmd_vel_puber.publish(twist)

    def trimerworkCallback(self, req):
        if req.mode == 0:
            self.reset_arm()
            rospy.sleep(0.2)
            self.open_gripper()
            resp = graspsignalResponse()
            resp.res = True
            resp.response = "reset arm position and open gripper"
            return resp
        elif req.mode == 20:
            rospy.loginfo("temp storage place ...")
            
            #self.sendBaseVel([0.25, 0.0, 0.0])
            #rospy.sleep(0.3)
            #self.sendBaseVel([0.0, 0.0, 0.0])
            #rospy.sleep(0.4)
            pose = Pose()
            pose.position.x = 0.19
            pose.position.y = -0.079
            self.arm_position_pub.publish(pose)
            rospy.sleep(1.9)
            self.open_gripper()
            rospy.sleep(0.3)                
            
            reset_thread = threading.Thread(target=self.reset_arm)
            reset_thread.start()
            
            resp = graspsignalResponse()
            resp.res = True
            resp.response = "Successfully temp Place"
            return resp

        # 此部分内容无用
        #initial_time = rospy.get_time()
        #while rospy.get_time() - self.image_time_now > 0.1:
        #    rospy.loginfo("latency detected!")
        #    if rospy.get_time() - initial_time > 3.0:
        #        self.sendBaseVel([-0.2, 0.0, 0.0])
        #        rospy.sleep(0.5)
        #        self.sendBaseVel([-0.2, 0.0, 0.0])
        #    if rospy.get_time() - initial_time > 6.0:
        #        resp = graspsignalResponse()
        #        resp.res = True
        #        resp.response = "Successfully Grasp fake"
        #        return resp
        #    rospy.sleep(0.1)

        rate = rospy.Rate(self.ros_rate)

        resp = graspsignalResponse()
        current_time1 = rospy.Time.now()

        if req.mode == 1:
            rospy.loginfo("grasp preparing ...")

            self.open_gripper()

            x_threshold = 0.01
            y_threshold = 0.01
            x_dis_tar = 0.360
            angle_threshold = 0.1
            
            # 初始化pid参数,kp,ki,kd根据实际在小车速度控制部分设置
            error = [0.0, 0.0 , 0.0]
            prev_error = [0.0, 0.0 , 0.0]  # 数组依次为linear_x, linear_y, angle.z
            toral_error = [0.0 , 0.0, 0.0]
              
            while not rospy.is_shutdown():
                target_marker_pose = self.current_marker_poses
                if target_marker_pose is None:
                    continue

                target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                    target_marker_pose
                )
                # 追踪停车位置，按照0.5的减速比减速x速度
                cmd_vel = [0.0, 0.0, 0.0]                
                cmd_vel[0]= 0.4*sqrt(pow(target_pos[0],2)+pow(target_pos[1],2))
                cmd_vel[0] = np.clip(cmd_vel[0], 0.1, 0.3) # liner.x超速限制在[0.1,0.3]
                cmd_vel[1] = 8*target_pos[1] # linear.y
                cmd_vel[1] = np.clip(cmd_vel[1], -0.2, 0.2) # 超速限制在速度范围内
                cmd_vel[2] = -4*target_angle  # angle.z
                cmd_vel[2] = np.clip(cmd_vel[2], -0.3, 0.3) # 超速限制在速度范围内
                #cmd_vel[2]= 4 * atan2(target_pos[1],target_pos[0]) # 方向角角速度控制
                #print("target.liner.x=",cmd_vel[0], " target.liner.y=",cmd_vel[1], " yaw angle.z speed=", cmd_vel[2])
                
                # 小车加持pid的积分和微分控制
                #cmd_vel = [0.0, 0.0, 0.0]
                ## linear_x
                #error[0] = target_pos[0]          
                #cmd_vel[0]= 0.4*error[0] + 0.00*toral_error[0] + 5*(error[0] - prev_error[0])
                #toral_error[0] += error[0]
                #prev_error[0] = error[0]
                #cmd_vel[0] = np.clip(cmd_vel[0], 0.1, 0.3) # liner.x超速限制在[0.1,0.3]
                ## linear_y
                #error[1] = target_pos[1] 
                #cmd_vel[1] = 8*error[1] + 0.00*toral_error[1] + 15*(error[1] - prev_error[1])
                #toral_error[1] += error[1]
                #prev_error[1] = error[1]
                #cmd_vel[1] = np.clip(cmd_vel[1], -0.2, 0.2) # 超速限制在速度范围内
                ## angle_z
                #error[2] = target_angle
                #cmd_vel[2] = -(2*error[2] + 0.00*toral_error[2] + 5*(error[2] - prev_error[2])) # angle.z
                #toral_error[2] += error[2]
                #prev_error[2] = error[2]
                #cmd_vel[2] = np.clip(cmd_vel[2], -0.3, 0.3) # 超速限制在速度范围内
                #print("target.liner.x=",cmd_vel[0], " target.liner.y=",cmd_vel[1], " yaw angle.z speed=", cmd_vel[2])

                # 如果距离太近，后退小车
                if (target_pos[0] - x_dis_tar) < -x_threshold:
                    cmd_vel[0] = -0.1
                elif np.abs(target_pos[0] - x_dis_tar) <= x_threshold and (
                    np.abs(target_pos[1]) <= y_threshold
                    and np.abs(target_angle) <= angle_threshold
                ):
                    cmd_vel = [0.0, 0.0, 0.0]
                self.sendBaseVel(cmd_vel)
                if np.abs(target_pos[0] - x_dis_tar) <= x_threshold and (
                    np.abs(target_pos[1]) <= y_threshold
                    and np.abs(target_angle) <= angle_threshold
                ):
                    pose = Pose()
                    pose.position.x = 0.19
                    pose.position.y = -0.07
                    self.arm_position_pub.publish(pose)
                    rospy.sleep(1.5)
                    self.sendBaseVel([0.25, 0.0, 0.0])
                    rospy.sleep(0.3)
                    self.sendBaseVel([0.0, 0.0, 0.0])
                    rospy.sleep(0.5)
                    rospy.loginfo("Place: reach the goal for grasping.")
                    break

                current_time2 = rospy.Time.now()
                if current_time2.secs - current_time1.secs > 9:#如果微调时间超过xx秒
                    rospy.logerr("微调时间过长")
                    self.timeout_pub.publish(True)
                    rospy.sleep(2)
                    return resp
                self.timeout_pub.publish(False)

                rate.sleep()

            target_marker_pose = self.getTargetPosAndAngleInBaseLinkFrame(
                    target_marker_pose
                )
            self.close_gripper()
            rospy.sleep(1.0)

            self.close_gripper()
            rospy.sleep(1.0)
            reset_thread = threading.Thread(target=self.reset_arm)
            reset_thread.start()

            self.sendBaseVel([-0.3, 0.0, 0.0])
            rospy.sleep(0.4)
            self.sendBaseVel([0.0, 0.0, 0.0])

            resp.res = True
            resp.response = str(target_angle)
            return resp
        
        elif req.mode == 11:
            rospy.loginfo("grasp preparing ...")

            self.open_gripper()

            x_threshold = 0.01
            y_threshold = 0.01
            x_dis_tar = 0.365
            angle_threshold = 0.1

            while not rospy.is_shutdown():
                target_marker_pose = self.current_marker_poses
                if target_marker_pose is None:
                    continue

                target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                    target_marker_pose
                )
                # 追踪停车位置，按照0.5的减速比减速x速度
                cmd_vel = [0.0, 0.0, 0.0]                
                cmd_vel[0]= 0.25*sqrt(pow(target_pos[0],2)+pow(target_pos[1],2))
                cmd_vel[0] = np.clip(cmd_vel[0], 0.1, 0.15) # liner.x超速限制在[0.1,0.5]
                cmd_vel[1] = 10*target_pos[1]  # liner.y
                cmd_vel[1] = np.clip(cmd_vel[1], -0.1, 0.1) # 超速限制在速度范围内
                cmd_vel[2] = -4*target_angle  # angle.z
                cmd_vel[2] = np.clip(cmd_vel[2], -0.3, 0.3) # 超速限制在速度范围内
                #cmd_vel[2]= 4 * atan2(target_pos[1],target_pos[0]) # 方向角角速度控制
                print("target.liner.x = ",cmd_vel[0],  "  yaw angle.z speed = ", cmd_vel[2])
                               
                # 如果距离太近，后退小车
                if (target_pos[0] - x_dis_tar) < -x_threshold:
                    cmd_vel = [-0.1, 0.0, 0.0]
                elif np.abs(target_pos[0] - x_dis_tar) <= x_threshold and (
                    np.abs(target_pos[1]) <= y_threshold
                    and np.abs(target_angle) <= angle_threshold
                ):
                    cmd_vel = [0.0, 0.0, 0.0]
                self.sendBaseVel(cmd_vel)
                if np.abs(target_pos[0] - x_dis_tar) <= x_threshold and (
                    np.abs(target_pos[1]) <= y_threshold
                ):
                    pose = Pose()
                    pose.position.x = 0.19
                    pose.position.y = -0.08
                    self.sendBaseVel([0.25, 0.0, 0.0])
                    rospy.sleep(0.3)
                    self.sendBaseVel([0.0, 0.0, 0.0])
                    rospy.sleep(1.0)
                    self.arm_position_pub.publish(pose)
                    rospy.sleep(1.0)
                    rospy.loginfo("Place: reach the goal for grasping.")
                    break

                current_time2 = rospy.Time.now()
                if current_time2.secs - current_time1.secs > 9:#如果微调时间超过xx秒
                    rospy.logerr("微调时间过长")
                    self.timeout_pub.publish(True)
                    rospy.sleep(2)
                    return resp
                self.timeout_pub.publish(False)

                rate.sleep()

            target_marker_pose = self.getTargetPosAndAngleInBaseLinkFrame(
                    target_marker_pose
                )
            self.close_gripper()
            rospy.sleep(1.0)

            self.close_gripper()
            rospy.sleep(1.0)
            reset_thread = threading.Thread(target=self.reset_arm)
            reset_thread.start()

            self.sendBaseVel([-0.3, 0.0, 0.0])
            rospy.sleep(0.4)
            self.sendBaseVel([0.0, 0.0, 0.0])

            resp.res = True
            resp.response = str(target_angle)
            return resp
        
        elif req.mode == 12:
            rospy.loginfo("grasp preparing ...")

            self.open_gripper()

            x_threshold = 0.01
            y_threshold = 0.01
            x_dis_tar = 0.370
            angle_threshold = 0.1

            while not rospy.is_shutdown():
                target_marker_pose = self.current_marker_poses
                if target_marker_pose is None:
                    continue

                target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                    target_marker_pose
                )
                # 追踪停车位置，按照0.5的减速比减速x速度
                cmd_vel = [0.0, 0.0, 0.0]                
                cmd_vel[0]= 0.25*sqrt(pow(target_pos[0],2)+pow(target_pos[1],2))
                cmd_vel[0] = np.clip(cmd_vel[0], 0.1, 0.15) # liner.x超速限制在[0.1,0.5]
                cmd_vel[1] = 10*target_pos[1]  # liner.y
                cmd_vel[1] = np.clip(cmd_vel[1], -0.1, 0.1) # 超速限制在速度范围内
                cmd_vel[2] = -4*target_angle  # angle.z
                cmd_vel[2] = np.clip(cmd_vel[2], -0.3, 0.3) # 超速限制在速度范围内
                #cmd_vel[2]= 4 * atan2(target_pos[1],target_pos[0]) # 方向角角速度控制
                #print("target.liner.x = ",cmd_vel[0],  "  yaw angle.z speed = ", cmd_vel[2])
                               
                # 如果距离太近，后退小车
                if (target_pos[0] - x_dis_tar) < -x_threshold:
                    cmd_vel = [-0.1, 0.0, 0.0]
                elif np.abs(target_pos[0] - x_dis_tar) <= x_threshold and (
                    np.abs(target_pos[1]) <= y_threshold
                    and np.abs(target_angle) <= angle_threshold
                ):
                    cmd_vel = [0.0, 0.0, 0.0]
                self.sendBaseVel(cmd_vel)
                if np.abs(target_pos[0] - x_dis_tar) <= x_threshold and (
                    np.abs(target_pos[1]) <= y_threshold
                ):
                    pose = Pose()
                    pose.position.x = 0.19
                    pose.position.y = -0.08
                    self.sendBaseVel([0.25, 0.0, 0.0])
                    rospy.sleep(0.3)
                    self.sendBaseVel([0.0, 0.0, 0.0])
                    rospy.sleep(1.0)
                    self.arm_position_pub.publish(pose)
                    rospy.sleep(1.0)
                    rospy.loginfo("Place: reach the goal for grasping.")
                    break

                current_time2 = rospy.Time.now()
                if current_time2.secs - current_time1.secs > 9:#如果微调时间超过xx秒
                    rospy.logerr("微调时间过长")
                    self.timeout_pub.publish(True)
                    rospy.sleep(2)
                    return resp
                self.timeout_pub.publish(False)

                rate.sleep()

            target_marker_pose = self.getTargetPosAndAngleInBaseLinkFrame(
                    target_marker_pose
                )
            self.close_gripper()
            rospy.sleep(1.0)

            self.close_gripper()
            rospy.sleep(1.0)
            reset_thread = threading.Thread(target=self.reset_arm)
            reset_thread.start()

            self.sendBaseVel([-0.3, 0.0, 0.0])
            rospy.sleep(0.4)
            self.sendBaseVel([0.0, 0.0, 0.0])

            resp.res = True
            resp.response = str(target_angle)
            return resp

        elif req.mode == 2:
            rospy.loginfo("level 1 preparing ...")
            
            # self.pre()

            x_threshold = 0.01
            y_threshold = 0.01
            #x_dis_tar = 0.395
            x_dis_tar = 0.290
            angle_threshold = 0.1

            while not rospy.is_shutdown():
                target_marker_pose = self.current_marker_poses
                if target_marker_pose is None:
                    continue
                
                target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                    target_marker_pose
                )

                cmd_vel = [0.0, 0.0, 0.0] 
                # 追踪停车位置，按照0.5的减速比减速x速度               
                cmd_vel[0] = 0.4*sqrt(pow(target_pos[0],2)+pow(target_pos[1],2))  # liner.x
                cmd_vel[0] = np.clip(cmd_vel[0], 0.1, 0.2) # 超速限制在速度范围内
                cmd_vel[1] = 10*target_pos[1]  # liner.y
                cmd_vel[1] = np.clip(cmd_vel[1], -0.15, 0.15) # 超速限制在速度范围内
                cmd_vel[2] = -4*target_angle  # angle.z
                cmd_vel[2] = np.clip(cmd_vel[2], -0.3, 0.3) # 超速限制在速度范围内
                #print("liner.x=",cmd_vel[0]," liner.y=",cmd_vel[1]," angle.z=",cmd_vel[2])
                
                # 如果距离太近，后退小车
                if target_pos[0]-x_dis_tar < -x_threshold:
                    cmd_vel[0] = -0.1
                elif (np.abs(target_pos[0]-x_dis_tar) <= x_threshold
                        and np.abs(target_pos[1]) <= y_threshold
                        and np.abs(target_angle) <= angle_threshold
                    ):
                        cmd_vel = [0.0, 0.0, 0.0]
                self.sendBaseVel(cmd_vel)

                if np.abs(target_pos[0]-x_dis_tar) <= x_threshold and (
                    np.abs(target_pos[1]) <= y_threshold
                ):
                    #rospy.loginfo("Trim well in the all dimention, going open loop")
                    #self.sendBaseVel([0.25, 0.0, 0.0])
                    #rospy.sleep(0.3)
                    #self.sendBaseVel([0.25, 0.0, 0.0])
                    #rospy.sleep(0.3)
                    self.sendBaseVel([0.0, 0.0, 0.0])
                    rospy.loginfo("Place: reach the goal for placing.")
                    break
                rate.sleep()

            rospy.loginfo("Trim well in the horizon dimention")

            target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                self.current_marker_poses
            )
            self.sendBaseVel([0.0, 0.0, 0.0])
            rospy.sleep(2.0)
            self.pre()
            rospy.sleep(2.5)
            self.open_gripper()
            rospy.sleep(1.0)
            reset_thread = threading.Thread(target=self.reset_arm)
            reset_thread.start()

            self.sendBaseVel([-0.3, 0.0, 0.0])
            rospy.sleep(0.6)
            self.sendBaseVel([0.0, 0.0, 0.0])

            resp.res = True
            resp.response = "Successfully Place"
            return resp
    
        elif req.mode == 3:
            rospy.loginfo("level 2 preparing ...")
            
            self.pre2()
            
            x_threshold = 0.01
            y_threshold = 0.01
            #x_dis_tar = 0.379
            x_dis_tar = 0.274
            angle_threshold = 0.1
            
            while not rospy.is_shutdown():
                target_marker_pose = self.current_marker_poses
                if target_marker_pose is None:
                    continue
                
                target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                    target_marker_pose
                )

                cmd_vel = [0.0, 0.0, 0.0]                
                # 追踪停车位置，按照0.5的减速比减速x速度               
                cmd_vel[0] = 0.4*sqrt(pow(target_pos[0],2)+pow(target_pos[1],2))  # liner.x
                cmd_vel[0] = np.clip(cmd_vel[0], 0.1, 0.2) # 超速限制在速度范围内
                cmd_vel[1] = 10*target_pos[1]  # liner.y
                cmd_vel[1] = np.clip(cmd_vel[1], -0.15, 0.15) # 超速限制在速度范围内
                cmd_vel[2] = -4*target_angle  # angle.z
                cmd_vel[2] = np.clip(cmd_vel[2], -0.3, 0.3) # 超速限制在速度范围内
                #print("liner.x=",cmd_vel[0]," liner.y=",cmd_vel[1]," angle.z=",cmd_vel[2])
                
                # 如果距离太近，后退小车
                if target_pos[0]-x_dis_tar < -x_threshold:
                    cmd_vel[0] = -0.1
                elif (np.abs(target_pos[0]-x_dis_tar) <= x_threshold
                        and np.abs(target_pos[1]) <= y_threshold
                        and np.abs(target_angle) <= angle_threshold
                    ):
                        cmd_vel = [0.0, 0.0, 0.0]
                self.sendBaseVel(cmd_vel)

                if np.abs(target_pos[0] - x_dis_tar) <= x_threshold and (
                    np.abs(target_pos[1]) <= y_threshold
                ):
                    #rospy.loginfo("Trim well in the all dimention, going open loop")
                    #self.sendBaseVel([0.0, 0.0, 0.0])
                    #rospy.sleep(1.0)
                    #self.sendBaseVel([0.25, 0.0, 0.0])
                    #rospy.sleep(0.3)
                    #self.sendBaseVel([0.25, 0.0, 0.0])
                    #rospy.sleep(0.3)
                    self.sendBaseVel([0.0, 0.0, 0.0])
                    rospy.loginfo("Place: reach the goal for placing.")
                    break
                rate.sleep()
            
            rospy.loginfo("Trim well in the horizon dimention")

            target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                self.current_marker_poses
            )
            self.sendBaseVel([0.0, 0.0, 0.0])
            rospy.sleep(2.0)
            self.open_gripper()
            rospy.sleep(1.0)
            reset_thread = threading.Thread(target=self.reset_arm)
            reset_thread.start()

            self.sendBaseVel([-0.3, 0.0, 0.0])
            rospy.sleep(0.6)
            self.sendBaseVel([0.0, 0.0, 0.0])

            resp.res = True
            resp.response = "Successfully Place"
            return resp
        
        elif req.mode == 4:
            rospy.loginfo("level 3 preparing ...")
            
            self.pre3()
            
            x_threshold = 0.01
            y_threshold = 0.01
            #x_dis_tar = 0.378
            x_dis_tar = 0.273
            angle_threshold = 0.1
            
            while not rospy.is_shutdown():
                target_marker_pose = self.current_marker_poses
                if target_marker_pose is None:
                    continue
                
                target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                    target_marker_pose
                )

                cmd_vel = [0.0, 0.0, 0.0]                
                # 追踪停车位置，按照0.5的减速比减速x速度               
                cmd_vel[0] = 0.4*sqrt(pow(target_pos[0],2)+pow(target_pos[1],2))  # liner.x
                cmd_vel[0] = np.clip(cmd_vel[0], 0.1, 0.2) # 超速限制在速度范围内
                cmd_vel[1] = 10*target_pos[1]  # liner.y
                cmd_vel[1] = np.clip(cmd_vel[1], -0.15, 0.15) # 超速限制在速度范围内
                cmd_vel[2] = -4*target_angle  # angle.z
                cmd_vel[2] = np.clip(cmd_vel[2], -0.3, 0.3) # 超速限制在速度范围内
                #print("liner.x=",cmd_vel[0]," liner.y=",cmd_vel[1]," angle.z=",cmd_vel[2])
                
                # 如果距离太近，后退小车
                if target_pos[0]-x_dis_tar < -x_threshold:
                    cmd_vel[0] = -0.1
                elif (np.abs(target_pos[0]-x_dis_tar) <= x_threshold
                        and np.abs(target_pos[1]) <= y_threshold
                        and np.abs(target_angle) <= angle_threshold
                    ):
                        cmd_vel = [0.0, 0.0, 0.0]
                self.sendBaseVel(cmd_vel)

                if np.abs(target_pos[0] - x_dis_tar) <= x_threshold and (
                    np.abs(target_pos[1]) <= y_threshold
                ):
                    #rospy.loginfo("Trim well in the all dimention, going open loop")
                    #self.sendBaseVel([0.0, 0.0, 0.0])
                    #rospy.sleep(1.0)
                    #self.sendBaseVel([0.25, 0.0, 0.0])
                    #rospy.sleep(0.3)
                    #self.sendBaseVel([0.25, 0.0, 0.0])
                    #rospy.sleep(0.3)
                    self.sendBaseVel([0.0, 0.0, 0.0])
                    rospy.loginfo("Place: reach the goal for placing.")
                    break
                rate.sleep()
            
            rospy.loginfo("Trim well in the horizon dimention")

            target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                self.current_marker_poses
            )
            self.sendBaseVel([0.0, 0.0, 0.0])
            rospy.sleep(2.0)
            self.open_gripper()
            rospy.sleep(1.0)
            reset_thread = threading.Thread(target=self.reset_arm)
            reset_thread.start()

            self.sendBaseVel([-0.3, 0.0, 0.0])
            rospy.sleep(0.6)
            self.sendBaseVel([0.0, 0.0, 0.0])

            resp.res = True
            resp.response = "Successfully Place"
            return resp
        
        elif req.mode == 5:
            rospy.loginfo("level 4 preparing ...")
            
            self.pre4()
            
            x_threshold = 0.01
            y_threshold = 0.01
            #x_dis_tar = 0.377
            x_dis_tar = 0.272
            angle_threshold = 0.1
            
            while not rospy.is_shutdown():
                target_marker_pose = self.current_marker_poses
                if target_marker_pose is None:
                    continue
                
                target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                    target_marker_pose
                )

                cmd_vel = [0.0, 0.0, 0.0]                
                # 追踪停车位置，按照0.5的减速比减速x速度               
                cmd_vel[0] = 0.4*sqrt(pow(target_pos[0],2)+pow(target_pos[1],2))  # liner.x
                cmd_vel[0] = np.clip(cmd_vel[0], 0.1, 0.2) # 超速限制在速度范围内
                cmd_vel[1] = 10*target_pos[1]  # liner.y
                cmd_vel[1] = np.clip(cmd_vel[1], -0.15, 0.15) # 超速限制在速度范围内
                cmd_vel[2] = -4*target_angle  # angle.z
                cmd_vel[2] = np.clip(cmd_vel[2], -0.3, 0.3) # 超速限制在速度范围内
                #print("liner.x=",cmd_vel[0]," liner.y=",cmd_vel[1]," angle.z=",cmd_vel[2])
                
                # 如果距离太近，后退小车
                if target_pos[0]-x_dis_tar < -x_threshold:
                    cmd_vel[0] = -0.1
                elif (np.abs(target_pos[0]-x_dis_tar) <= x_threshold
                        and np.abs(target_pos[1]) <= y_threshold
                        and np.abs(target_angle) <= angle_threshold
                    ):
                        cmd_vel = [0.0, 0.0, 0.0]
                self.sendBaseVel(cmd_vel)

                if np.abs(target_pos[0] - x_dis_tar) <= x_threshold and (
                    np.abs(target_pos[1]) <= y_threshold
                ):
                    #rospy.loginfo("Trim well in the all dimention, going open loop")
                    #self.sendBaseVel([0.0, 0.0, 0.0])
                    #rospy.sleep(1.0)
                    #self.sendBaseVel([0.25, 0.0, 0.0])
                    #rospy.sleep(0.3)
                    #self.sendBaseVel([0.25, 0.0, 0.0])
                    #rospy.sleep(0.3)
                    self.sendBaseVel([0.0, 0.0, 0.0])
                    rospy.loginfo("Place: reach the goal for placing.")
                    break
                rate.sleep()
            
            rospy.loginfo("Trim well in the horizon dimention")

            target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                self.current_marker_poses
            )
            self.sendBaseVel([0.0, 0.0, 0.0])
            rospy.sleep(2.0)
            self.open_gripper()
            rospy.sleep(1.0)
            reset_thread = threading.Thread(target=self.reset_arm)
            reset_thread.start()

            self.sendBaseVel([-0.3, 0.0, 0.0])
            rospy.sleep(0.6)
            self.sendBaseVel([0.0, 0.0, 0.0])

            resp.res = True
            resp.response = "Successfully Place"
            return resp

    def open_gripper(self):
        open_gripper_msg = Point()
        open_gripper_msg.x = 0.0
        open_gripper_msg.y = 0.0
        open_gripper_msg.z = 0.0
        rospy.loginfo("open the gripper")
        self.arm_gripper_pub.publish(open_gripper_msg)

    def close_gripper(self):
        close_gripper_msg = Point()
        close_gripper_msg.x = 1.0
        close_gripper_msg.y = 0.0
        close_gripper_msg.z = 0.0
        rospy.loginfo("close the gripper")
        self.arm_gripper_pub.publish(close_gripper_msg)

    def reset_arm(self):
        reset_arm_msg = Pose()
        reset_arm_msg.position.x = 0.1
        reset_arm_msg.position.y = 0.12
        reset_arm_msg.position.z = 0.0
        reset_arm_msg.orientation.x = 0.0
        reset_arm_msg.orientation.y = 0.0
        reset_arm_msg.orientation.z = 0.0
        reset_arm_msg.orientation.w = 0.0
        rospy.loginfo("reset the arm")
        self.arm_position_pub.publish(reset_arm_msg)

    def pre(self):
        rospy.loginfo("<manipulater>: now prepare to grip")
        pose = Pose()
        pose.position.x = 0.206
        pose.position.y = -0.039
        self.arm_position_pub.publish(pose)
        
    def pre2(self):
        rospy.loginfo("<manipulater>: level 2 place")
        pose = Pose()
        pose.position.x = 0.19
        pose.position.y = 0.012
        self.arm_position_pub.publish(pose)
        
    def pre3(self):
        rospy.loginfo("<manipulater>: level 3 place")
        pose = Pose()
        pose.position.x = 0.19
        pose.position.y = 0.065
        self.arm_position_pub.publish(pose)
    
    def pre4(self):
        rospy.loginfo("<manipulater>: level 4 place")
        pose = Pose()
        pose.position.x = 0.19
        pose.position.y = 0.116
        self.arm_position_pub.publish(pose)


if __name__ == "__main__":
    rospy.init_node("manipulater_node", anonymous=True)
    rter = manipulater()
    rospy.spin()

