#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 保存在rviz里面设置的导航点 在../save/goal_list/goal_list.txt（工作空间的上层目录下的路径）

import time
import os
import numpy as np
import cv2
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap,GetMapRequest,GetMapResponse
from geometry_msgs.msg import PoseStamped
import SaveOrRead


class Map(object):
    '''
    地图类
    '''
    def __init__(self):
        self.delta = rospy.get_param("/slam_gmapping/delta", 0.05)
        print ("delta : %s"%self.delta)
        self.map_goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback )
        
        #创建保存目录
        SaveOrRead.mkdir("../save/map")
        SaveOrRead.mkdir("../save/goal_list")

        #创建文件
        SaveOrRead.touch("../save/goal_list/goal_list.txt")

        #文件打开
        self.fo = open("../save/goal_list/goal_list.txt", "a")

        #监听者
        self.listener = tf.TransformListener()

        #建立一个列表，存放地图点
        self.pose_list = []
        
        #
        rospy.spin()


    def __del__(self):
        self.fo.close()

    def call(self):
        try:
            rospy.wait_for_service('/dynamic_map', 2.0)
            mapClient = rospy.ServiceProxy('/dynamic_map', GetMap())
            #此处使用的是委托服务
            res = mapClient()
            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def callback(self, msg): 
        print("回调函数")

        try:
            #关键坐标变换函数!!!!!
            (trans,rot) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            position = trans
            orientation = rot
            self.pose_list.append([trans[0], trans[1], trans[2], orientation[0], orientation[1], orientation[2], orientation[3]])
            self.saveGoal(position, orientation)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Can't transform from /map to /base_footprint")


        
    
    def saveGoal(self, position, orientation):
        print("SaveGoal"),
        print(position),
        print(orientation)


        # = Pose(Point(0.500, 0.000, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))



        ws = "locations['x']= Pose(Point("
        for i in range(0 , len(position)):
            ws = ws + str(position[i])
            if i < len(position) - 1 :
                ws = ws + ", "
        ws = ws + "), Quaternion("
        for i in range(0 , len(orientation)):
            ws = ws + str(orientation[i])
            if i < len(orientation) - 1 :
                ws = ws + ", "
        ws = ws + "))\n"

        
        self.fo.write(ws)
        print ( "Real Goal : %s"%position)
        try:
            res = self.call()
            mapArray = np.array(res.map.data)
            cols = res.map.info.width
            rows = res.map.info.height
            mapArray =  mapArray.reshape((rows, cols))
            #print mapArray
            #print mapArray.shape
            #print type(mapArray)
            map = np.empty((rows,cols,3), dtype = np.uint8)
            #BGR
            map.fill(127)
            #数组中 未知区域的值为  -1，标记为灰色(默认色[127, 127, 127])
            #      无障碍区域的值为  0，标记为白色([255, 255, 255])
            #      有障碍区域的值为  100，标记为黑色([0, 0, 0])2
            grey = np.array([127, 127, 127])
            whilt = np.array([255, 255, 255])
            black = np.array([0, 0, 0])
            red = np.array([0, 0, 255])
            
        except Exception,e:
            print (e)
            rospy.logerr("Can't connect service name : /dynamic_map")
            return

        for i in range(0, rows):
            for j in range(0, cols):
                if (mapArray[i,j] == 100):
                    map[rows-i-1,j] = black
                if (mapArray[i,j] == 0):
                    map[rows-i-1,j] = whilt
                    
        #金色标记历史中base_footprint的所到达的点
        for i in range(0, len(self.pose_list)):
            cv2.circle(map, (int((self.pose_list[i])[0]/self.delta) + cols / 2 + 1, rows /2 - 1 - int((self.pose_list[i])[1]/self.delta)), 4, (0, 215, 255),-1)
        s = time.strftime("../save/map/%Y_%m_%d-%H:%M:%S.tiff", time.localtime(time.time()))
        print ("Saving...  "),
        cv2.imwrite(s, map)
        print ("Saved")


if __name__=='__main__':
    rospy.init_node('display_map_with_goal', anonymous=False)
    v = Map()
    rospy.spin()
