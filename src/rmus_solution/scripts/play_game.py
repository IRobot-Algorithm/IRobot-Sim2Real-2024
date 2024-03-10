#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt8MultiArray
from rmus_solution.srv import switch, setgoal, graspsignal
from geometry_msgs.msg import Pose

from std_msgs.msg import Bool

import tf2_ros
import tf2_geometry_msgs

import random


timeout = False

class block:
    def __init__(self, ID,  placement , area = -1, mode = -1):#矿石ID、放到哪里,哪个矿区(-1为不知道 ),放置模式
        self.ID = ID
        self.area = area
        self.mode = 0
        self.placement = placement
        self.is_set = 0#是否已经被放好了
        self.is_put = 0#是否被放到暂存区了
        self.pos_in_map = None
        self.pos_in_cam = None
class robot:
    def __init__(self):
        self.block = 0#矿车含有的矿石,0表示无矿石
        self.location = 0#在哪个地方



def set_timeout(data):
    global timeout 
    timeout = data

def tell_go_park_hard():
    # if rospy.Time.now().to_sec() - game_begin_time < 20:
    if 300 - (rospy.Time.now().to_sec() - game_begin_time) < 20:#记得改为20
        print("距离游戏结束还有:",300-(rospy.Time.now().to_sec() - game_begin_time),"将回到初始区")
        go_to(5)
        print("total time:",rospy.Time.now().to_sec() - game_begin_time,"将回到初始区")
        rospy.sleep(2000)

def catch():
    tell_go_park_hard()
    trimer(1,"")
    global timeout
    if timeout.data == True:
        rospy.logerr("微调超时,放弃此次抓取")
        return True   #True代表超时了
    return False

def is_obstructed_by_wall(block_id):
    get_block_pos_in_cam(block_id)
    get_block_pos_in_map(block_id)
    print("blocks[",block_id,"]",blocks[block_id].pos_in_map)
    print("my_robot.location =",my_robot.location)
    if blocks[block_id].pos_in_map == None:#若TF崩了，则不进行卡墙判断
        return False
    
    half_robot_width = 0.15
    if my_robot.location == 12:
        if -0.3 + half_robot_width >= blocks[block_id].pos_in_map.position.x:
            rospy.logerr("抓此方块会被墙挡住，放弃此角度")
            return True
    if my_robot.location==21:
        if 2.9 - half_robot_width <= blocks[block_id].pos_in_map.position.x:
            rospy.logerr("抓此方块会被墙挡住，放弃此角度")
            return True
    # if my_robot.location == 31:
    #     if 3.6 - half_robot_width <= blocks[block_id].pos_in_map.position.y:
    #         rospy.logerr("抓此方块会被墙挡住，放弃此角度")
    #         return True
    return False

def tell_is_here(block_id):#判断此方块在没在这里
    blockinfo = rospy.wait_for_message("/all_detect_ID", UInt8MultiArray, timeout=1)
    return block_id in blockinfo.data

def grip(block = -1):
    if block == -1:
        tell_go_park_hard()
        for i in range(1,7):
            if blocks[i].mode != -1 and tell_is_here(i):
                print("这里有目标方块"+ str(i))
                img_switch_mode(i)
                if is_obstructed_by_wall(i):
                    return False
                if catch():
                    return False
                rospy.sleep(1)
                times = 1
                while tell_is_here(i) and times < 5 :# 尝试五次抓取
                    times = times + 1
                    rospy.logerr("抓取失败，将进行第"+str(times)+"抓取")
                    if catch(): 
                        return False
                    rospy.sleep(1)#给抓取好方块留出时间
                    
                
                if tell_is_here(i) == False:
                    rospy.loginfo("恭喜！抓取成功！！")
                    add_area_information(blocks[i].area)
                    remove_block_information(i)
                    my_robot.block = i
                    return True
        return False
    else: #即如果是抓特定方块的话
        print("这里有目标方块"+ str(block))
        img_switch_mode(block)
        if catch():
            return False
        rospy.sleep(1)
        times = 1
        while tell_is_here(block) and times < 5 :# 尝试五次抓取
            times = times + 1
            rospy.logerr("抓取失败，将进行第"+str(times)+"抓取")
            if catch(): 
                return False
            rospy.sleep(1)#给抓取好方块留出时间
            
        
        if tell_is_here(block) == False:
            rospy.loginfo("恭喜！抓取成功！！")
            my_robot.block = block
            return True

def go_to_another_side(location):#去当前矿区的另一侧
    rospy.loginfo("准备去此矿区另一侧")
    if location == 11:
        go_to(12)
    if location == 21:
        go_to(22)

    # if location == 32:
    #     go_to(33)

    if location == 31:
        go_to(32)

    add_area_information(location//10)

def put():
    global rest_block
    rest_block = rest_block - 1

    if blocks[my_robot.block].mode == 1:
        address_name = ['B','O','X']
        go_to(6+blocks[my_robot.block].placement)
        rospy.loginfo("到达兑换站"+address_name[blocks[my_robot.block].placement])#输出到达了兑换站 
        img_switch_mode(7+blocks[my_robot.block].placement)
        rospy.sleep(0.5)
        trimer(2,"")
        blocks[my_robot.block].is_set = 1
        
    elif blocks[my_robot.block].mode == 2:
        go_to(6)
        rospy.loginfo("到达兑换站X")#输出到达了兑换站 
        img_switch_mode(7)
        rospy.sleep(0.5)
        trimer(3,"")
        blocks[my_robot.block].is_set = 1    

    elif blocks[my_robot.block].mode == 3:
        go_to(6)
        rospy.loginfo("到达兑换站X")#输出到达了兑换站 
        img_switch_mode(7)
        rospy.sleep(0.5)
        trimer(4,"")

    elif blocks[my_robot.block].mode == 4:
        go_to(6)
        rospy.loginfo("到达兑换站B")#输出到达了兑换站 
        img_switch_mode(7)
        rospy.sleep(0.5)
        trimer(5,"")

    else:
        put_on_floor()

def put_on_floor():
    global temporary_storage_num
    temporary_storage_num += 1

    print("前往矿石暂存点",temporary_storage_num)
    go_to(40+temporary_storage_num)#41 42 43为相应坐标点
    trimer(0,"")#让张天晓给我写一个无脑放置
    blocks[my_robot.block].is_put = 1
    blocks[my_robot.block].mode = 1 + temporary_storage_num
    temporary_storage_info[temporary_storage_num] = my_robot.block

def update_block_location(area, block_place_information):#更新矿石的位置信息
    for i in range(1,7):
        if i in block_place_information and blocks[i].is_set == 0:
            blocks[i].area = area
            #print("blocks["+str(i)+"].location="+str(blocks[i].location))

def remove_block_information(block_id):
    remove_area_information(block_id)
    blocks[block_id].area = -1#已抓取成功

def add_area_information(area):#添加当前画面识别到的所有矿石
    img_switch_mode(11)
    blockinfo = rospy.wait_for_message("/all_detect_ID", UInt8MultiArray, timeout=1)
    block_place_information[area].update(blockinfo.data)

    update_block_location(area, block_place_information[area])

def remove_area_information(block_id):
    block_place_information[blocks[block_id].area].discard(block_id)
    if 0 in block_place_information[blocks[block_id].area]:
        block_place_information[blocks[block_id].area].discard(0)
        print("已移除掉0")
        print("block_place_information[",blocks[block_id].area,"]",block_place_information[blocks[block_id].area])
        if len(block_place_information[blocks[block_id].area]) == 0:
            go_to_another_side(my_robot.location)
            
    print("block_place_information[",blocks[block_id].area,"]=",block_place_information[blocks[block_id].area])

def go_to(location):#仅仅是去那里
    if location != 5:
        tell_go_park_hard()
    navigation(location, "")
    my_robot.location = location
    
def there_are_blocks(area):#判断此处有没有方块
    print("block_place_information[",area,"]=",block_place_information[area])
    if len(block_place_information[area]) == 0:
        return False
    return True

def detect_area(area):#对指定矿区进行一次抓取、放置的全过程。若抓取失败，则返回false
    if there_are_blocks(area) == False:
        return False
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
    result = grip(block)
    return result
    
def init_this_node():
    rospy.init_node("gamecore_node")
    # game_begin_time = rospy.Time.now().to_sec()
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

def get_block_pos_in_cam(block_id):
    print("get_block_pos_in_cam")
    blockinfo = rospy.wait_for_message("/get_blockinfo", Pose, timeout=1)
    
    blocks[block_id].pos_in_cam = blockinfo
    
def get_block_pos_in_map(block_id):
    posestamped_in_cam = tf2_geometry_msgs.PoseStamped()
    posestamped_in_cam.header.stamp = rospy.Time.now()
    posestamped_in_cam.header.frame_id = (
        "camera_aligned_depth_to_color_frame_correct"
    )

    posestamped_in_cam.pose = blocks[block_id].pos_in_cam
    try:
        posestamped_in_base = tfBuffer.transform(posestamped_in_cam, "map", rospy.Duration(1.0))
        blocks[block_id].pos_in_map = posestamped_in_base.pose
    
        pub.publish(posestamped_in_base)        
    except:
        rospy.logerr("无法进行TF转换")
        blocks[block_id].pos_in_map = None
def RunEverywhere():#在地图里到处跑
    img_switch_mode(11)
    while True:
        target = [11,12,13,21,22,23,31,32,9,5,7]
        num = random.randint(0, 10)
        go_to(target[num])
if __name__ == '__main__':
    init_this_node()
    game_begin_time = rospy.Time.now().to_sec()

    pub = rospy.Publisher('/block_pos_in_map', tf2_geometry_msgs.PoseStamped, queue_size=10)#用于测试

    tfBuffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tfBuffer)


    navigation = rospy.ServiceProxy("/set_navigation_goal", setgoal)
    trimer = rospy.ServiceProxy("/let_manipulater_work", graspsignal)
    img_switch_mode = rospy.ServiceProxy("/image_processor_switch_mode", switch)
    
    rospy.sleep(2)
    rospy.Subscriber("/timeout", Bool, set_timeout)
    
    trim_res = trimer(0, "")
    img_switch_mode(10)

    my_robot = robot()

    temporary_storage_num = 0#此变量用于记录暂存点的矿石数量

    go_to(9)

    gameinfo = rospy.wait_for_message("/get_gameinfo", UInt8MultiArray, timeout=7)

    blocks = [block(0,0), block(0,0), block(0,0), block(0,0), block(0,0), block(0,0), block(0,0)]
    print(gameinfo.data)
    my_robot = robot()
    for i,ID in enumerate(gameinfo.data):
        blocks[ID].ID = ID
        blocks[ID].placement = i
        blocks[ID].mode = 1


    rest_block = 6#剩余的方块

    response = img_switch_mode(0)

    block_place_information = [{0},{0},{0},{0}]#初始化，此变量用于储存一个矿区有哪些方块 
    temporary_storage_info = [0, 0, 0, 0]#储存暂存区的方块ID

    is_here = 0#判断这里有没有目标方块
    while rest_block > 0:
        if rest_block == 6 and rospy.Time.now().to_sec()-game_begin_time > 100:
            RunEverywhere()
        if rest_block > 0:
            detect_area(1)

        if rest_block > 0:
            detect_area(2)

        if rest_block > 0:
            detect_area(3)
    game_rest_time = rospy.Time.now().to_sec()-game_begin_time
    print("目前耗时:", game_rest_time,"秒")
    for i in range(1,4):
        while True:
            go_to(40+i)
            img_switch_mode(temporary_storage_info[i])
            if grip_specified_block(temporary_storage_info[i]):
                break
        put()

    navigation_result = go_to(5)

    game_total_time = rospy.Time.now().to_sec()-game_begin_time
    print("game_total_time:", game_total_time)
    response = img_switch_mode(0)
