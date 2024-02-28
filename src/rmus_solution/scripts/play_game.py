#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt8MultiArray
from rmus_solution.srv import switch, setgoal, graspsignal
from geometry_msgs.msg import Pose

from std_msgs.msg import Bool


timeout = False

def set_timeout(data):
    global timeout 
    timeout = data
    

def catch():
    trimer_response = trimer(1,"")
    global timeout
    if timeout.data == True:
        rospy.logerr("微调超时,放弃此次抓取")
        return True   #True代表超时了
    return False

def grip(gameinfo , is_here, response):
    for i, target in enumerate(gameinfo.data):
        block_num = i
        response = img_switch_mode(target)#切换为识别对应目标
        rospy.sleep(0.5)
        blockinfo = rospy.wait_for_message("/all_detect_ID", UInt8MultiArray, timeout=1)
        # output_str = "blockinfo.data = {}".format(blockinfo.data)
        # 不知为什么输出不了这个列表,问问曾阳
        # print(output_str)
        if target in blockinfo.data and is_here != 1:#判断是否识别到了目标方块
            print("这里有目标方块"+ str(target))
            if catch(): 
                is_here = 2#表示微调超时了
                return is_here,response, block_num
            rospy.sleep(2)#给抓取好方块留出时间

            blockinfo = rospy.wait_for_message("/all_detect_ID", UInt8MultiArray, timeout=1)
            if target in blockinfo.data :
                times = 0#尝试重新抓取的次数
                while target in blockinfo.data and times<5:#若抓不上会尝试抓20次
                    times = times + 1
                    rospy.logerr("抓取失败,将进行第"+str(times+1)+"次抓取")
                    if catch():
                        is_here = 2
                        return is_here,response, block_num
                    
                    blockinfo = rospy.wait_for_message("/all_detect_ID", UInt8MultiArray, timeout=1)

                if target not in blockinfo.data:
                    rospy.loginfo("抓取终于成功了！！")
                    is_here = 1#抓到目标方块
                if target in blockinfo.data:
                    rospy.logerr("多次抓取失败，前往下一个目标点")
                    
            else:
                rospy.loginfo("抓取成功")
                is_here = 1#抓到目标方块

            if is_here == 1:
                break;
    if  is_here == 0:
        rospy.loginfo("这里没有目标方块 或 抓取失败")
    return is_here, response ,block_num

def go_to_another_side(location ,gameinfo, is_here, response):
    rospy.loginfo("准备去此矿区另一侧")
    if location == 1:
        navigation_result = navigation(12, "")
    if location == 2:
        navigation_result = navigation(22, "")
    if location == 3:
        navigation_result = navigation(32, "")
    is_here, response, block_num = grip(gameinfo , is_here, response)
    return is_here, response ,block_num


def put(block_num):
        address_name = ['B','O','X']
        navigation_result = navigation(6+block_num, "")#暂时没写放到哪个上面
        rospy.loginfo("到达兑换站"+address_name[block_num])#输出到达了兑换站 
        response = img_switch_mode(7+block_num)
        rospy.sleep(0.5)
        trimer_response = trimer(2,"")

if __name__ == '__main__':
    rospy.init_node("gamecore_node")
    game_begin_time = rospy.Time.now().to_sec()
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

    rospy.Subscriber("/timeout", Bool, set_timeout)

    trim_res = trimer(0, "")
    response = img_switch_mode(10)



    navigation_result = navigation(9, "")

    gameinfo = rospy.wait_for_message("/get_gameinfo", UInt8MultiArray, timeout=7)

    while gameinfo.data[0] == 0 or gameinfo.data[1] == 0 or gameinfo.data[2] == 0:
       gameinfo = rospy.wait_for_message("/get_gameinfo", UInt8MultiArray, timeout=7)
       rospy.logwarn("Waiting for gameinfo detection.")
       rospy.sleep(0.5)

    rest_block = 3#剩余的方块

    response = img_switch_mode(0)
    while rest_block > 0:
        is_here = 0#判断这里有没有目标方块

        navigation_result = navigation(11, "")
        location = 1#在矿区1
        is_here, response ,block_num= grip(gameinfo , is_here, response)


        if is_here != 1:
            #去此矿区的另一侧
            is_here, response ,block_num= go_to_another_side(location, gameinfo, is_here, response)
        print("一="+str(is_here))
        if is_here == 2:
            is_here = 0

        if is_here == 0:
            navigation_result = navigation(21, "")
            location = 2#在矿区2
            is_here, response ,block_num= grip(gameinfo , is_here, response)
            if is_here != 1:
            #去此矿区的另一侧
                is_here, response ,block_num= go_to_another_side(location, gameinfo, is_here, response)
        print("二="+str(is_here))
        if is_here == 2:
            is_here = 0

        if is_here == 0:
            navigation_result = navigation(31, "")
            location = 3#在矿区3
            is_here, response, block_num = grip(gameinfo , is_here, response)
            if is_here != 1:
                #去此矿区的另一侧
                is_here, response, block_num = go_to_another_side(location, gameinfo, is_here, response)
        print("三="+str(is_here))
        if is_here != 1:
            rospy.logerr("什么都没抓到QAQ")
            continue
        
        if is_here ==1:
            put(block_num)
            rest_block = rest_block -1
            rospy.loginfo("还剩"+str(rest_block)+"个方块")



        response = img_switch_mode(0)



    navigation_result = navigation(5, "")
    game_total_time = rospy.Time.now().to_sec()-game_begin_time
    print("game_total_time:", game_total_time)
    response = img_switch_mode(0)
