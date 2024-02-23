#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt8MultiArray
from rmus_solution.srv import switch, setgoal, graspsignal
from geometry_msgs.msg import Pose

def get_boxid_blockid_inorder(gameinfo):
    return [0,1,2], [gameinfo[0], gameinfo[1], gameinfo[2]]

def compare_poses(pose1, pose2):
    # Compare position
    rospy.loginfo("Comparing position~~~~~~~~~~~")
    pos1 = pose1.pose.position
    pos2 = pose2.pose.position
    rospy.loginfo("Comparing position!!!!!!!!!!!!!!!!!!!!")
    if pos1.x != pos2.x or pos1.y != pos2.y or pos1.z != pos2.z:
        return False

    # Compare orientation
    ori1 = pose1.pose.orientation
    ori2 = pose2.pose.orientation
    if ori1.x != ori2.x or ori1.y != ori2.y or ori1.z != ori2.z or ori1.w != ori2.w:
        return False
    
    return True

def check_topic_messages(topic_name):
    try:
        rospy.wait_for_message(topic_name, Pose, timeout=1)
    except rospy.exceptions.ROSException:
        return False
    return True;


def grip(gameinfo , is_here, response):
    for i, target in enumerate(gameinfo.data):
        response = img_switch_mode(target)
        if check_topic_messages("/get_blockinfo") and is_here == [0]:#判断是否识别到了目标方块
            is_here = [1]#有目标方块
            print("这里有目标方块"+ str(target))
            rospy.sleep(0.5)#应该是留给微调的时间,之后可以改为接收到消息再进行下一步
            trimer_response = trimer(1,"")
            try:
                rospy.sleep(0.5)
                blockinfo = rospy.wait_for_message("/get_blockinfo", Pose, timeout=1)
                rospy.sleep(0.5)
                blockinfo2 = rospy.wait_for_message("/get_blockinfo", Pose, timeout=1)

                if compare_poses(blockinfo, blockinfo2) == True:
                    rospy.logerr("抓取失败")
                    rospy.sleep(3.0)
                    trimer_response = trimer(1,"")
                    while trimer_response.res == False:#这边是原来就有判断夹取是否成功么？
                        trimer_response = trimer(1,"")
                    rospy.logerr("另一次抓取成功了！！")
                else:
                    rospy.logerr("抓取成功")
            except:
                rospy.logerr("未接收到方块的位姿信息")
        
    return is_here,response


if __name__ == '__main__':
    rospy.init_node("gamecore_node")

    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service("/set_navigation_goal", 1.0)
            break
        except:
            rospy.logwarn("Waiting for set_navigation_goal Service")
            rospy.sleep(0.5)

    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service("/let_manipulater_work", 1.0)
            break
        except:
            rospy.logwarn("Waiting for let_manipulater_work Service")
            rospy.sleep(0.5)

    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service("/image_processor_switch_mode", 1.0)
            break
        except:
            rospy.logwarn("Waiting for image_processor_switch_mode Service")
            rospy.sleep(0.5)

    rospy.loginfo("Get all rospy sevice!")
    navigation = rospy.ServiceProxy("/set_navigation_goal", setgoal)
    trimer = rospy.ServiceProxy("/let_manipulater_work", graspsignal)
    img_switch_mode = rospy.ServiceProxy("/image_processor_switch_mode", switch)
    rospy.sleep(2)

    trim_res = trimer(0, "")
    response = img_switch_mode(10)
    navigation_result = navigation(9, "")

    gameinfo = rospy.wait_for_message("/get_gameinfo", UInt8MultiArray, timeout=7)
    while gameinfo.data[0] == 0 or gameinfo.data[1] == 0 or gameinfo.data[2] == 0:
       gameinfo = rospy.wait_for_message("/get_gameinfo", UInt8MultiArray, timeout=7)
       rospy.logwarn("Waiting for gameinfo detection.")
       rospy.sleep(0.5)

    response = img_switch_mode(0)
    for i in range(0,3):
        is_here = [0]#判断这里有没有目标方块

        navigation_result = navigation(2, "")
        is_here, response = grip(gameinfo , is_here, response)
        if is_here == [0]:
            navigation_result = navigation(4, "")
            is_here, response = grip(gameinfo , is_here, response)
        if is_here == [0]:
            navigation_result = navigation(1, "")
            is_here, response = grip(gameinfo , is_here, response)
        navigation_result = navigation(6+i, "")#暂时没写放到哪个上面
        print("         到达目的地"+str(6+i))


        response = img_switch_mode(7+i)
        rospy.sleep(5)
        trimer_response = trimer(2,"")
        response = img_switch_mode(0)


    navigation_result = navigation(11, "")
    
    response = img_switch_mode(0)