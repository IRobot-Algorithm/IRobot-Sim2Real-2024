#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt8MultiArray
from rmus_solution.srv import switch, setgoal, graspsignal
from geometry_msgs.msg import Pose

from std_msgs.msg import Bool


timeout = False

class block:
    def __init__(self, ID,  placement , location = -1, mode = -1):#矿石ID、放到哪里,哪个矿区(-1为不知道 ),放置模式
        self.ID = ID
        self.location = location
        self.mode = mode
        self.placement = placement
        self.is_set = 0#是否已经被放好了

class robot:
    def __init__(self):
        self.block = 0#矿车含有的矿石,0表示无矿石
        self.location = 0#在哪个地方


def set_timeout(data):
    global timeout 
    timeout = data
    

def catch():
    trimer(1,"")
    global timeout
    if timeout.data == True:
        rospy.logerr("微调超时,放弃此次抓取")
        return True   #True代表超时了
    return False

def tell_is_here(block_id):#判断此方块在没在这里
    blockinfo = rospy.wait_for_message("/all_detect_ID", UInt8MultiArray, timeout=1)
    return block_id in blockinfo.data

def grip():
    for i in range(1,7):
        if blocks[i].mode == 0 and tell_is_here(i):
            print("这里有目标方块"+ str(i))
            img_switch_mode(i)
            if catch():
                return False
            rospy.sleep(1)
            times = 1
            while tell_is_here(i) and times < 1 :# 尝试五次抓取
                times = times + 1
                rospy.logerr("抓取失败，将进行第"+str(times)+"抓取")
                if catch(): 
                    return False
                rospy.sleep(2)#给抓取好方块留出时间
                
                
            if tell_is_here(i) == False:
                rospy.loginfo("恭喜！抓取成功！！")
                remove_block_information(i)
                my_robot.block = i
                return True
    return False
def go_to_another_side(location):#去当前矿区的另一侧
    rospy.loginfo("准备去此矿区另一侧")
    if location == 11:
        go_to(12)
    if location == 21:
        go_to(22)
    if location == 31:
        go_to(32)
    add_area_information(location//10)

def put():
        address_name = ['B','O','X']
        navigation(6+blocks[my_robot.block].placement, "")
        rospy.loginfo("到达兑换站"+address_name[blocks[my_robot.block].placement])#输出到达了兑换站 
        img_switch_mode(7+blocks[my_robot.block].placement)
        rospy.sleep(0.5)
        trimer(2,"")
        blocks[my_robot.block].is_set = 1

def update_block_location(area, block_place_information):#更新矿石的位置信息
    for i in range(1,7):
        if i in block_place_information and blocks[i].is_set == 0:
            blocks[i].location = area
            #print("blocks["+str(i)+"].location="+str(blocks[i].location))

def remove_block_information(block_id):
    blocks[block_id].location = -1#已抓取成功

def add_area_information(area):#添加当前画面识别到的所有矿石
    img_switch_mode(11)
    area = area - 1
    blockinfo = rospy.wait_for_message("/all_detect_ID", UInt8MultiArray, timeout=1)
    block_place_information[area].update(blockinfo.data)
    update_block_location(area+1, block_place_information[area])


def go_to(location):#仅仅是去那里
    navigation(location, "")
    my_robot.location = location
    
def detect_area(area):#对指定矿区进行一次抓取、放置的全过程。若抓取失败，则返回false
    location = area*10 + 1
    go_to(location)
    img_switch_mode(11)#此模式，便于识别ID以加入到集合里 
    add_area_information(area)
    result = grip()
    if result == False and (my_robot.location == 11 or my_robot.location == 21 or my_robot.location == 31):
        go_to_another_side(my_robot.location)
        if grip():
            put()
            return True
    elif result == True:
        put()
        return True
    else:
        return False
    

def grip_specified_block(block):
    #print("112321321  block.location*10+1="+str(block.location*10+1))
    go_to(block.location*10+1)
    img_switch_mode(11)#此模式，便于识别ID以加入到集合里 

    add_area_information(block.location)

    result = grip()
    if result == False and (my_robot.location == 11 or my_robot.location == 21 or my_robot.location == 31):
        go_to_another_side(block.location*10+1)
        if grip():
            put()
            return True
    elif result == True:
        put()
        return True
    else:
        return False
    
def update_rest_block():
    sum = 0
    for i in range(1,7):
        if blocks[i].mode == 0 and blocks[i].is_set == 0:
            sum+=1
    return sum
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
    img_switch_mode(10)




    navigation_result = navigation(9, "")

    gameinfo = rospy.wait_for_message("/get_gameinfo", UInt8MultiArray, timeout=7)

    blocks = [block(0,0), block(0,0), block(0,0), block(0,0), block(0,0), block(0,0), block(0,0)]
    print(gameinfo.data)
    my_robot = robot()
    for i,ID in enumerate(gameinfo.data):
        blocks[ID].ID = ID
        blocks[ID].placement = i
        blocks[ID].mode = 0


    rest_block = 3#剩余的方块

    response = img_switch_mode(0)

    block_place_information = [{0},{0},{0},{0}]#初始化，此变量用于储存一个矿区有哪些方块 


    is_here = 0#判断这里有没有目标方块

    while rest_block > 0:
        for i in range(1, 7):
            #print("wow  blocks["+str(i)+"].location="+str(blocks[i].location))
            if blocks[i].mode == 0 and blocks[i].location != -1:
                grip_specified_block(blocks[i])

        rest_block = update_rest_block()

        if rest_block > 0:
            detect_area(1)
        rest_block = update_rest_block()
  
        if rest_block > 0:
            detect_area(2)
        rest_block = update_rest_block()

        if rest_block > 0:
            detect_area(3)
        rest_block = update_rest_block()



    navigation_result = navigation(5, "")
    

    game_total_time = rospy.Time.now().to_sec()-game_begin_time
    print("game_total_time:", game_total_time)
    response = img_switch_mode(0)
