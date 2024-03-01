#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import numpy as np
from scipy.spatial.transform import Rotation as sciR



import rospy
from geometry_msgs.msg import Twist, Pose, Point
from rmus_solution.srv import graspsignal, graspsignalResponse
from std_msgs.msg import Bool

import tf2_ros
import tf2_geometry_msgs
from simple_pid import PID

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

        self.kp = 20.0
        self.ki = 0.5
        self.kd = 0.0
        self.x_dis_tar_1 = 0.348 #0.335
        self.x_dis_tar_2 = 0.41 #0.395
        self.x_dis_tar_3 = 0.41 #self.x_dis_tar_3 should equals to self.x_dis_tar_2
        self.x_threshold = 0.005 # 可能需要减小，以提高精度
        self.y_threshold_p = 0.018
        self.y_threshold_n = 0.018
        self.y_threshold = 0.013 # 可能需要减小，以提高精度
        self.y_rough_threshold = 0.025
        self.yaw_threshold = 0.1
        self.adjust_speed_lowwer_limit = -0.3
        self.adjust_speed_upper_limit = 0.3
        self.position_pid = PID(self.kp, self.ki, self.kd, np.array([self.x_dis_tar_1, 0, 0]), None)
        print(self.position_pid)

        self.x_prepared = False
        self.y_prepared = False
        self.y_rough_prepared = False
        self.yaw_prepared = False


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
        print("target_pos:", pos, "target_angle:", angle)
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

        initial_time = rospy.get_time()
        while rospy.get_time() - self.image_time_now > 0.1:
            rospy.loginfo("latency detected!")
            if rospy.get_time() - initial_time > 3.0:
                self.sendBaseVel([-0.2, 0.0, 0.0])
                rospy.sleep(0.5)
                self.sendBaseVel([-0.2, 0.0, 0.0])
            if rospy.get_time() - initial_time > 6.0:
                resp = graspsignalResponse()
                resp.res = True
                resp.response = "Successfully Grasp fake"
                return resp
            rospy.sleep(0.1)

        rate = rospy.Rate(self.ros_rate)

        resp = graspsignalResponse()
        current_time1 = rospy.Time.now()

        if req.mode == 1:
            rospy.loginfo("First trim then grasp")
            rospy.loginfo("Trim to the right place")

            self.open_gripper()
            rospy.sleep(0.1)

            self.x_prepared = False
            self.y_prepared = False
            self.y_rough_prepared = False
            self.position_pid = PID(self.kp, self.ki, self.kd, np.array([self.x_dis_tar_1, 0, 0]), None)

            while not rospy.is_shutdown():
                target_marker_pose = self.current_marker_poses
                if target_marker_pose is None:
                    continue

                target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                    target_marker_pose
                )
                cmd_vel = [0.0, 0.0, 0.0]
                
                cmd_vel = self.position_pid.__call__(np.array([target_pos[0], target_pos[1], target_angle]))
                # print("cmd_vel before clip", cmd_vel)
                cmd_vel = np.clip(cmd_vel, self.adjust_speed_lowwer_limit, self.adjust_speed_upper_limit)
                # print("cmd_vel after  clip", cmd_vel)
                cmd_vel[0] = -cmd_vel[0]
                cmd_vel[1] = -cmd_vel[1]
                # cmd_vel[2] = 0

                # if not self.y_rough_prepared:
                #     cmd_vel[0] = 0
                #     cmd_vel[2] = 0
                #     print("preparing rough y axis...")

                # if not self.yaw_prepared and self.y_rough_prepared:
                #     cmd_vel[0] = 0
                #     cmd_vel[1] = 0                
                #     #print("preparing yaw...")

                # if not self.x_prepared and self.y_rough_prepared and self.yaw_prepared:
                #     cmd_vel[1] = 0
                #     cmd_vel[2] = 0                
                #     print("preparing x axis...")

                # if not self.y_prepared and self.y_rough_prepared and self.yaw_prepared and self.x_prepared:
                #     cmd_vel[0] = 0
                #     cmd_vel[2] = 0
                #     print("preparing y axis...")  

                # if not self.y_rough_prepared:
                #     cmd_vel[0] = 0
                #     cmd_vel[2] = 0
                #     print("preparing rough y axis...")
                # elif not self.yaw_prepared:
                #     cmd_vel[0] = 0
                #     cmd_vel[1] = 0                
                #     #print("preparing yaw...")
                # elif not self.x_prepared:
                #     cmd_vel[1] = 0
                #     cmd_vel[2] = 0                
                #     print("preparing x axis...")
                # elif not self.y_prepared:
                #     cmd_vel[0] = 0
                #     cmd_vel[2] = 0
                #     print("preparing y axis...")  

                if not self.y_prepared:
                    cmd_vel[0] = 0
                    cmd_vel[2] = 0
                    print("preparing y axis...")
                elif not self.yaw_prepared:
                    cmd_vel[0] = 0
                    cmd_vel[1] = 0                
                    print("preparing yaw...")
                elif not self.x_prepared:
                    cmd_vel[1] = 0
                    cmd_vel[2] = 0                
                    print("preparing x axis...")


                if np.abs(target_pos[0] - self.x_dis_tar_1) <= self.x_threshold:
                    self.x_prepared = True
                else :
                    self.x_prepared = False
                
                # if (target_pos[1] - 0.0) <= self.y_threshold_p and (0.0 - target_pos[1]) <= self.y_threshold_n:
                #     self.y_prepared = True
                # else :
                #     self.y_prepared = False

                if np.abs(target_pos[1] - 0.0) <= self.y_threshold:
                    self.y_prepared = True
                    self.y_rough_prepared = True
                elif np.abs(target_pos[1] - 0.0) <= self.y_rough_threshold:
                    self.y_prepared = False
                    self.y_rough_prepared = True
                else:
                    self.y_prepared = False
                    self.y_rough_prepared = False

                if np.abs(target_angle - 0) <= self.yaw_threshold:
                    self.yaw_prepared = True
                else :
                    self.yaw_prepared = False

                self.sendBaseVel(cmd_vel)
                
                if self.x_prepared and self.y_prepared and self.yaw_prepared:
                    cmd_vel = [0.0, 0.0, 0.0]
                    pose = Pose()
                    pose.position.x = 0.19
                    pose.position.y = -0.08
                    self.sendBaseVel([0.25, 0.0, 0.0])
                    rospy.sleep(0.3)
                    self.sendBaseVel([0.0, 0.0, 0.0])
                    rospy.sleep(1.0)
                    self.arm_position_pub.publish(pose)
                    rospy.sleep(1.0)
                    rospy.loginfo("Place: reach the goal for placing.")
                    break

                current_time2 = rospy.Time.now()
                if current_time2.secs - current_time1.secs > 10:#如果微调时间超过xx秒
                    rospy.logerr("微调时间过长")
                    self.timeout_pub.publish(True)
                    rospy.sleep(2)
                    return resp
                self.timeout_pub.publish(False)

                rate.sleep()

            target_marker_pose = self.current_marker_poses
            self.close_gripper()
            rospy.sleep(1.0)

            self.close_gripper()
            rospy.sleep(1.0)
            self.reset_arm()

            self.sendBaseVel([-0.3, 0.0, 0.0])
            rospy.sleep(0.4)
            self.sendBaseVel([0.0, 0.0, 0.0])

            resp.res = True
            resp.response = str(target_angle)
            return resp

        elif req.mode == 2:
            rospy.loginfo("First trim then place")

            self.x_prepared = False
            self.y_prepared = False
            self.y_rough_prepared = False
            self.position_pid = PID(self.kp, self.ki, self.kd, np.array([self.x_dis_tar_2, 0, 0]), None)

            self.pre()
            theta = 0.10

            while not rospy.is_shutdown():
                target_marker_pose = self.current_marker_poses
                if target_marker_pose is None:
                    continue
                
                target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                    target_marker_pose
                )
                cmd_vel = [0.0, 0.0, 0.0]

                cmd_vel = self.position_pid.__call__(np.array([target_pos[0], target_pos[1], target_angle]))
                cmd_vel = np.clip(cmd_vel, self.adjust_speed_lowwer_limit, self.adjust_speed_upper_limit)
                cmd_vel[0] = -cmd_vel[0]
                cmd_vel[1] = -cmd_vel[1]
                # cmd_vel[2] = 0

                if self.y_prepared != True:
                    cmd_vel[0] = 0
                    cmd_vel[2] = 0
                    #print("preparing y axis...")

                if self.yaw_prepared != True and self.y_prepared == True:
                    cmd_vel[0] = 0
                    cmd_vel[1] = 0                
                    #print("preparing yaw...")

                if self.x_prepared != True and self.y_prepared == True and self.yaw_prepared == True:
                    cmd_vel[1] = 0
                    cmd_vel[2] = 0                
                    #print("preparing x axis...")

                if np.abs(target_pos[0] - self.x_dis_tar_2) <= self.x_threshold:
                    self.x_prepared = True
                else :
                    self.x_prepared = False
                
                if (target_pos[1] - 0.0) <= self.y_threshold_p and (0.0 - target_pos[1]) <= self.y_threshold_n:
                    self.y_prepared = True
                else :
                    self.y_prepared = False

                if np.abs(target_angle - 0) <= self.yaw_threshold:
                    self.yaw_prepared = True
                else :
                    self.yaw_prepared = False

                self.sendBaseVel(cmd_vel)

                if self.x_prepared == True and self.y_prepared == True and self.yaw_prepared == True:
                    rospy.loginfo("Trim well in the all dimention, going open loop")
                    self.sendBaseVel([0.0, 0.0, 0.0])
                    rospy.sleep(1.0)
                    self.sendBaseVel([0.25, 0.0, 0.0])
                    rospy.sleep(0.3)
                    self.sendBaseVel([0.25, 0.0, 0.0])
                    rospy.sleep(0.3)
                    self.sendBaseVel([0.0, 0.0, 0.0])
                    rospy.loginfo("Place: reach the goal for placing.")
                    break
                rate.sleep()

            rospy.loginfo("Trim well in the horizon dimention")

            

            target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                self.current_marker_poses
            )
            self.sendBaseVel([0.0, 0.0, 0.0])
            rospy.sleep(1.0)
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
            rospy.loginfo("First trim then place")
            
            self.x_prepared = False
            self.y_prepared = False
            self.y_rough_prepared = False
            self.position_pid = PID(self.kp, self.ki, self.kd, np.array([self.x_dis_tar_3, 0, 0]), None)
            
            self.pre2()

            while not rospy.is_shutdown():
                target_marker_pose = self.current_marker_poses
                if target_marker_pose is None:
                    continue
                
                target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                    target_marker_pose
                )
                cmd_vel = [0.0, 0.0, 0.0]

                cmd_vel = self.position_pid.__call__(np.array([target_pos[0], target_pos[1], target_angle]))
                cmd_vel = np.clip(cmd_vel, self.adjust_speed_lowwer_limit, self.adjust_speed_upper_limit)
                cmd_vel[0] = -cmd_vel[0]
                cmd_vel[1] = -cmd_vel[1]
                # cmd_vel[2] = 0

                if self.y_prepared != True:
                    cmd_vel[0] = 0
                    cmd_vel[2] = 0
                    #print("preparing y axis...")

                if self.yaw_prepared != True and self.y_prepared == True:
                    cmd_vel[0] = 0
                    cmd_vel[1] = 0                
                    #print("preparing yaw...")

                if self.x_prepared != True and self.y_prepared == True and self.yaw_prepared == True:
                    cmd_vel[1] = 0
                    cmd_vel[2] = 0                
                    #print("preparing x axis...")

                if np.abs(target_pos[0] - self.x_dis_tar_2) <= self.x_threshold:
                    self.x_prepared = True
                else :
                    self.x_prepared = False
                
                if (target_pos[1] - 0.0) <= self.y_threshold_p and (0.0 - target_pos[1]) <= self.y_threshold_n:
                    self.y_prepared = True
                else :
                    self.y_prepared = False

                if np.abs(target_angle - 0) <= self.yaw_threshold:
                    self.yaw_prepared = True
                else :
                    self.yaw_prepared = False

                self.sendBaseVel(cmd_vel)

                if self.x_prepared == True and self.y_prepared == True and self.yaw_prepared == True:
                    rospy.loginfo("Trim well in the all dimention, going open loop")
                    self.sendBaseVel([0.0, 0.0, 0.0])
                    rospy.sleep(1.0)
                    self.sendBaseVel([0.25, 0.0, 0.0])
                    rospy.sleep(0.3)
                    self.sendBaseVel([0.25, 0.0, 0.0])
                    rospy.sleep(0.3)
                    self.sendBaseVel([0.0, 0.0, 0.0])
                    rospy.loginfo("Place: reach the goal for placing.")
                    break
                rate.sleep()
            
            rospy.loginfo("Trim well in the horizon dimention")

            

            target_pos, target_angle = self.getTargetPosAndAngleInBaseLinkFrame(
                self.current_marker_poses
            )
            self.sendBaseVel([0.0, 0.0, 0.0])
            rospy.sleep(1.0)
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
        pose.position.x = 0.21
        pose.position.y = -0.036
        self.arm_position_pub.publish(pose)
        
    def pre2(self):
        rospy.loginfo("<manipulater>: level 2 place")
        pose = Pose()
        pose.position.x = 0.21
        pose.position.y = 0.014
        self.arm_position_pub.publish(pose)
        
    def pre3(self):
        rospy.loginfo("<manipulater>: level 3 place")
        pose = Pose()
        pose.position.x = 0.21
        pose.position.y = 0.065
        self.arm_position_pub.publish(pose)
    
    def pre4(self):
        rospy.loginfo("<manipulater>: level 4 place")
        pose = Pose()
        pose.position.x = 0.21
        pose.position.y = 0.115
        self.arm_position_pub.publish(pose)


if __name__ == "__main__":
    rospy.init_node("manipulater_node", anonymous=True)
    rter = manipulater()
    rospy.spin()

