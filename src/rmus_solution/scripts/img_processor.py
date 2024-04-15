#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import Int8
from sensor_msgs.msg import Image, CameraInfo
from rmus_solution.srv import switch, switchResponse

import cv2
import numpy as np
from threading import Thread
from scipy.spatial.transform import Rotation as R

from detect import marker_detection


def pose_aruco_2_ros(rvec, tvec):
    aruco_pose_msg = Pose()
    aruco_pose_msg.position.x = tvec[0]
    aruco_pose_msg.position.y = tvec[1]
    aruco_pose_msg.position.z = tvec[2]
    rotation_matrix = cv2.Rodrigues(rvec)
    r_quat = R.from_matrix(rotation_matrix[0]).as_quat()
    aruco_pose_msg.orientation.x = r_quat[0]
    aruco_pose_msg.orientation.y = r_quat[1]
    aruco_pose_msg.orientation.z = r_quat[2]
    aruco_pose_msg.orientation.w = r_quat[3]
    return aruco_pose_msg


class Processor:
    def __init__(self, initial_mode=0, verbose=True) -> None:
        self.current_mode = initial_mode
        self.collapsed = False
        self.verbose = verbose
        self.start_time = int(rospy.get_time() - 3)
        self.bridge = CvBridge()
        self.current_visualization_image = None
        self.all_detectd_ID = []
        self.dst_ID = 0
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5.0)
                rospy.loginfo("Get topic /camera/color/image_raw.")
                break
            except:
                rospy.logwarn("Waiting for message /camera/color/image_raw.")
                continue

        while not rospy.is_shutdown():
            try:
                camerainfo = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=5.0)
                rospy.loginfo("Get topic /camera/color/camera_info.")
                self.camera_matrix = np.array(camerainfo.K, "double").reshape((3,3))
                rospy.loginfo("camera_matrix :\n {}".format(self.camera_matrix))                
                break
            except:
                rospy.logwarn("Waiting for message /camera/color/camera_info.")
                continue

        try:
            if self.verbose:
                self.vis_thread = Thread(target=self.visualization)
                self.vis_thread.start()
        except:
            while True:
                self.collapsed = False
                rospy.logerr("The visualization window has collapsed!")
        rospy.Subscriber(
            "/camera/color/image_raw", Image, self.imageCallback, queue_size=1
        )
        rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw",
            Image,
            self.depthCallback,
            queue_size=1,
        )
        rospy.Service("/image_processor_switch_mode", switch, self.modeCallBack)
        self.pub_p = rospy.Publisher("/get_gameinfo", UInt8MultiArray, queue_size=1)
        self.pub_b = rospy.Publisher("/get_blockinfo", Pose, queue_size=1)
        self.pub_all_ID = rospy.Publisher("/all_detect_ID", UInt8MultiArray, queue_size=1)
        self.pub_dst_ID = rospy.Publisher("/dst_ID", Int8, queue_size=1)
        self.detected_gameinfo = [-1, -1, -1]
        self.uint32data = [None] * 9
        
    def imageCallback(self, image):
        self.image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.this_image_time_ms = int(image.header.stamp.nsecs / 1e6) + int(
            1000 * (image.header.stamp.secs - self.start_time)
        )
        locked_current_mode = self.current_mode
        #print("detect mode recieved:", locked_current_mode)
        if locked_current_mode == 10: # 获得兑换站上方3位目标数字
            detected_gameinfo = self.get_gameinfo(self.image)
            if detected_gameinfo is None:
                pass
            else:
                if (
                    detected_gameinfo[0] == self.detected_gameinfo[0]
                    and detected_gameinfo[1] == self.detected_gameinfo[1]
                    and detected_gameinfo[2] == self.detected_gameinfo[2]
                ):
                    self.pub_p.publish(UInt8MultiArray(data=self.detected_gameinfo))
                    #rospy.loginfo("Second gameinfo get, publishing")
               # else:
                    # rospy.loginfo("First gameinfo get")
                    # rospy.loginfo(detected_gameinfo)

                self.detected_gameinfo = detected_gameinfo
            self.current_visualization_image = self.image
        elif locked_current_mode <= 9 and locked_current_mode >= 1:
            self.all_detectd_ID = []
            t_begin = rospy.Time.now().to_nsec()
            self.update_uint32_data(locked_current_mode)
            if self.uint32data[locked_current_mode - 1] is None:
                pass
            else:
                self.pub_b.publish(self.latest_pose)
                # self.latest_pose.position = None
                # self.latest_pose.orientation = None
            if self.all_detectd_ID != []:
                print("self.all_detectd_ID = " +str(self.all_detectd_ID))
            self.pub_all_ID.publish(UInt8MultiArray(data=self.all_detectd_ID))
            #print("publish pose once used: ", int((rospy.Time.now().to_nsec() - t_begin)/1e6), "ms")
        elif locked_current_mode == 0:
            self.current_visualization_image = self.image
        elif locked_current_mode == 11:
            self.all_detectd_ID = []
            t_begin = rospy.Time.now().to_nsec()
            self.update_uint32_data(locked_current_mode)
            # if self.uint32data[locked_current_mode - 1] is None:
            #     pass
            # else:
            #     self.pub_b.publish(self.latest_pose)
                # self.latest_pose.position = None
                # self.latest_pose.orientation = None
            
            if self.all_detectd_ID != []:
                print("self.all_detectd_ID = " +str(self.all_detectd_ID))
            self.pub_all_ID.publish(UInt8MultiArray(data=self.all_detectd_ID))
        else:
            assert False

    def depthCallback(self, image):
        self.depth_img = self.bridge.imgmsg_to_cv2(image, "32FC1")

    def modeCallBack(self, req):
        if 0 <= req.mode <= 11:
            self.current_mode = req.mode
            return switchResponse(self.current_mode)
        else:
            return switchResponse(req.mode)

    def get_gameinfo(self, image):
        gameinfo = [0, 0, 0]
        id_list, quads_list, area_list, tvec_list, rvec_list, _, _ = marker_detection(
            image, camera_matrix=self.camera_matrix, area_filter_size=150, verbose=self.verbose, exchange_station=2
        )
        number_dict = {}
        for id, quads in zip(id_list, quads_list):
            if id <= 6 and id >= 1:
                number_dict[id] = (
                    quads[0][0][0] + quads[1][0][0] + quads[2][0][0] + quads[3][0][0]
                ) / 4.0
        if len(number_dict) == 3:
            min_ = np.min(list(number_dict.values())) + 0.5
            max_ = np.max(list(number_dict.values())) - 0.5
            for k, v in number_dict.items():
                if v < min_:
                    gameinfo[0] = k
            for k, v in number_dict.items():
                if v > max_:
                    gameinfo[2] = k
            for k, v in number_dict.items():
                if v > min_ and v < max_:
                    gameinfo[1] = k
            return gameinfo
        else:
            return None

    def update_uint32_data(self, blockid):
        all_ID = []
        if blockid <= 9 and blockid >= 1:
            last_info = self.uint32data[blockid - 1]
        if blockid <= 6 and blockid >= 1:
            (
                id_list,
                quads_list,
                area_list,
                tvec_list,
                rvec_list,
                all_ID,
                _,
            ) = marker_detection(
                self.image,
                camera_matrix=self.camera_matrix,
                template_ids=[blockid],
                area_filter_size=1200,
                verbose=self.verbose,
                height_range=(0.0, 0.06),
                exchange_station=0
            )
        elif blockid <= 9 and blockid >= 7:
            (
                id_list,
                quads_list,
                area_list,
                tvec_list,
                rvec_list,
                all_ID,
                minareas_list,
            ) = marker_detection(
                self.image,
                camera_matrix=self.camera_matrix,
                #template_ids=[7, 8, 9],
                template_ids=[blockid],
                area_filter_size=1200,
                verbose=self.verbose,
                height_range=(0.00, 0.06),
                exchange_station=1
            )
        
        elif blockid == 11:
            self.dst_ID = 0
            (
                id_list,
                quads_list,
                area_list,
                tvec_list,
                rvec_list,
                all_ID,
                minareas_list,
            ) = marker_detection(
                self.image,
                camera_matrix=self.camera_matrix,
                template_ids=[1, 2, 3, 4, 5, 6],
                area_filter_size=1200,
                verbose=self.verbose,
                height_range=(0.00, 0.06),
            )
            self.all_detectd_ID = list(set(all_ID))
            if id_list != [] and tvec_list != []:
                try:
                    # 计算每个平移向量的范数
                    norms = [np.linalg.norm(tvec) for tvec in tvec_list]
                    self.dst_ID = id_list[np.argmin(norms)]
                    print("Nearest ID is------>", self.dst_ID)
                    self.pub_dst_ID.publish(self.dst_ID)
                except:
                    print(id_list)
                    print(tvec_list)
            else:
                self.dst_ID = -1
                self.pub_dst_ID.publish(self.dst_ID)
            return

        self.all_detectd_ID = list(set(all_ID))

        pose_list = [pose_aruco_2_ros(r, t) for t, r in zip(tvec_list, rvec_list)]

        if id_list != []:
            best_id = area_list.index(max(area_list))
        else:
            pose_list.clear()
            self.latest_pose = Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0033,y=6.9877,z=0.0108,w=0.9999)
            )
            self.uint32data[blockid - 1] = [-1]
            return
        try:
            quads = quads_list[best_id]
        except:
            assert False
        depth = self.get_current_depth(quads)
        blockinfo = [
            blockid,
            quads[0, 0, 0],
            quads[0, 0, 1],
            quads[1, 0, 0],
            quads[1, 0, 1],
            quads[2, 0, 0],
            quads[2, 0, 1],
            quads[3, 0, 0],
            quads[3, 0, 1],
            int(area_list[best_id]),
            int(depth),
            self.this_image_time_ms,
        ]
        if self.verbose:
            cv2.putText(
                self.current_visualization_image,
                "x: " + str(pose_list[best_id].position.x),
                (0, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                self.current_visualization_image,
                "y: " + str(pose_list[best_id].position.y),
                (0, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                self.current_visualization_image,
                "z: " + str(pose_list[best_id].position.z),
                (0, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
            )
        self.latest_pose = pose_list[best_id] # 发送的pose
        self.uint32data[blockid - 1] = blockinfo
        return

    def get_current_depth(self, quads):
        locked_depth = self.depth_img
        new_contour = quads.copy()
        new_contour[0, 0, 0], new_contour[2, 0, 0] = int(
            quads[0, 0, 0] * 0.75 + 0.25 * quads[2, 0, 0]
        ), int(quads[0, 0, 0] * 0.25 + 0.75 * quads[2, 0, 0])
        new_contour[0, 0, 1], new_contour[2, 0, 1] = int(
            quads[0, 0, 1] * 0.75 + 0.25 * quads[2, 0, 1]
        ), int(quads[0, 0, 1] * 0.25 + 0.75 * quads[2, 0, 1])
        new_contour[1, 0, 0], new_contour[3, 0, 0] = int(
            quads[1, 0, 0] * 0.75 + 0.25 * quads[3, 0, 0]
        ), int(quads[1, 0, 0] * 0.25 + 0.75 * quads[3, 0, 0])
        new_contour[1, 0, 1], new_contour[3, 0, 1] = int(
            quads[1, 0, 1] * 0.75 + 0.25 * quads[3, 0, 1]
        ), int(quads[1, 0, 1] * 0.25 + 0.75 * quads[3, 0, 1])

        mask = np.zeros(locked_depth.shape, np.uint8)
        cv2.drawContours(mask, [new_contour], -1, 255, -1)
        depth = int(cv2.mean(locked_depth, mask=mask)[0] * 1000)
        if self.verbose:
            cv2.putText(
                mask,
                str(depth) + "mm",
                (new_contour[0, 0, 0], new_contour[0, 0, 1] - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
                1,
            )
            self.current_visualization_image = (
                np.expand_dims(mask, axis=-1) * 0.5 + self.image * 0.5
            ).astype(np.uint8)
        return depth

    def visualization(self):
        while not rospy.is_shutdown():
            try:
                if self.current_visualization_image is not None:
                    cv2.imshow("frame", self.current_visualization_image)
                    cv2.waitKey(1)
            except:
                self.collapsed = False
                rospy.logerr("The visualization window has collapsed!")


if __name__ == "__main__":
    rospy.init_node("image_node", anonymous=True)
    rter = Processor(initial_mode=0, verbose=False)
    rospy.loginfo("Image thread started")
    rospy.spin()