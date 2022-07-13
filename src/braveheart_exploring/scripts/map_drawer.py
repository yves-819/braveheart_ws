#!/usr/bin/env python
# -*- coding: utf-8 -*-

#保存地图在../save/saveMaps （工作空间的上层目录下的路径）

import time
import os
import numpy as np
import cv2
import rospy
import tf
import SaveOrRead
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap,GetMapRequest,GetMapResponse
from geometry_msgs.msg import PoseStamped,PointStamped


#给参数  zibar_info
class Map(object):
    '''
    地图类
    '''
    def __init__(self):
        rospy.init_node('braveheart_zibar', anonymous=False)
        self.delta = rospy.get_param("/slam_gmapping/delta", 0.05)
        # print ("delta : %s"%self.delta)
        
        self.a = 0
        self.terminus_flag=0

        #创建保存目录
        SaveOrRead.mkdir("../save/saveMaps")

        #建立一个列表，存放地图点
        self.xlist = {}
        self.ylist = {}
        self.terminusXlist={}
        self.terminusYlist={}

        self.info_list = {}
        self.zuoyouflag = 0
        self.flag1list = {}
        self.testlist = {}


        self.terminus_a=0
        self.terminus_print=0
        self.terminus_count=0
        self.terminus_x = 0
        self.terminus_y = 0

        print("map_drawer")

        #定时保存
        while not rospy.is_shutdown():
            rospy.Subscriber('zbar_result', String, self.callback, queue_size=1)
            rospy.Subscriber('zuo', String, self.callbackzuo, queue_size=1)
            rospy.Subscriber('you', String, self.callbackyou, queue_size=1)
            time.sleep(10)
            self.saveMap()
        
        rospy.spin()

    def __del__(self):
        pass
    
    def callback(self,data):
         #监听坐标树，以方便使用tf来帮助我们计算机器人坐标变换
        
        listener = tf.TransformListener()
        listener.waitForTransform("/map", "/base_link", rospy.Time(0),rospy.Duration(4.0))
        #print(self.zuoyouflag)
        if (self.zuoyouflag==1):
            self.zuoyouflag = 0
            self.info_list[self.a] = data.data
            self.flag1list[self.a] = 1
            laser_point=PointStamped()
            laser_point.header.frame_id = "base_link"
            laser_point.header.stamp =rospy.Time(0)
            laser_point.point.x=0.16           #0.16
            laser_point.point.y=0.11          #0.11
            laser_point.point.z=0.0
            p=listener.transformPoint("map",laser_point)
            self.xlist[self.a] = int(p.point.x)*198
            self.ylist[self.a] = int(p.point.y)*198
            self.a = self.a+1
            print("turtle1_link:({},{},{})---->--->--world_frame:({},{},{})".format(
            laser_point.point.x,
            laser_point.point.y,
            laser_point.point.z, 
            p.point.x, 
            p.point.y,
            p.point.z))
            print(self.info_list)

        if (self.zuoyouflag==2):
            self.zuoyouflag = 0
            self.info_list[self.a] = data.data
            self.flag1list[self.a] = 2
            laser_point2=PointStamped()
            laser_point2.header.frame_id = "base_link"
            laser_point2.header.stamp =rospy.Time(0)
            laser_point2.point.x= 0.16          #0.16
            laser_point2.point.y=-0.11             #-0.11
            laser_point2.point.z=0.0
            p=listener.transformPoint("map",laser_point2)
            self.xlist[self.a] = int(p.point.x)*198
            self.ylist[self.a] = int(p.point.y)*198
            self.a = self.a+1
            print("turtle1_link:({},{},{})---->--->--world_frame:({},{},{})".format(
            laser_point2.point.x,
            laser_point2.point.y,
            laser_point2.point.z,
            p.point.x, 
            p.point.y,
            p.point.z))
            print(self.info_list)
        

        #(tran, rot) = listener.lookupTransform('/base_link', '/imu_frame',  rospy.Time(0.0))

        
        
    

    def callbackzuo(self,data):
        self.zuoyouflag = 1
    def callbackyou(self,data):
        self.zuoyouflag = 2

    def call(self):
        print("call")
        try:
            rospy.wait_for_service('/dynamic_map', 2.0)
            mapClient = rospy.ServiceProxy('/dynamic_map', GetMap())
            #此处使用的是委托服务
            res = mapClient() 

            return res
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    

    # def touch(self,file_name): 
    #     folder = os.path.exists(file_name) 
    #     if folder:
    #         os.remove(file_name)
    #         os.mknod(file_name)
    #         print ("%s文件已存在,重新创建"%(file_name))
    #     else: 
    #         os.mknod(file_name)
    #         print ("创建%s"%(file_name))

    # #写操作
    # def echo(self,file_name, string):
    #     folder = os.path.exists(file_name) 
    #     if folder:
    #         fo = open(file_name, "a")
    #         fo.write(string)
    #         fo.close()
    #     else:
    #         print("路径下不存在此文件 %s"%(file_name))

    
    def saveMap(self):
        try:
            res = self.call()
            mapArray = np.array(res.map.data)
            cols = res.map.info.width
            rows = res.map.info.height
            print("cols",cols,"rows",rows)
            mapArray =  mapArray.reshape((rows, cols))
            #print mapArray
            #print mapArray.shape
            #print type(mapArray)
            map = np.empty((rows,cols,3), dtype = np.uint8)
            #BGR
            map.fill(127)
            #数组中 未知区域的值为  -1，标记为灰色(默认色[127, 127, 127])
            #      无障碍区域的值为  0，标记为白色([255, 255, 255])
            #      有障碍区域的值为  100，标记为黑色([0, 0, 0])
            grey = np.array([127, 127, 127])
            whilt = np.array([255, 255, 255])
            black = np.array([0, 0, 0])
            red = np.array([0, 0, 255])
            blue = np.array([255, 0, 0])

            
            
        except Exception,e:
            print (e)
            rospy.logerr("Can't connect service name : /dynamic_map")
            return
        
        #print(" map get succeess !")

        #self.touch('/home/sz/save/mapdata.txt')

        for i in range(0, rows):
            for j in range(0, cols):
                #self.echo('/home/sz/save/mapdata.txt', str(mapArray[i,j])+',')
                if (mapArray[i,j] == 100):
                    map[rows-i-1,j] = black
                if (mapArray[i,j] == 0):
                    map[rows-i-1,j] = whilt


        for z in range(0,len(self.xlist)):
            print(z)
            map_camera_pointX=self.xlist[z]+rows/2
            map_camera_pointY=self.ylist[z]+cols/2
            #print(map_camera_pointX)
            if (self.flag1list[z] == 1):
                for i in range(1,cols/2):
                    if (mapArray[i,map_camera_pointY]==100):
                        self.terminus_y=map_camera_pointY
                        self.terminus_x=i
                        print(self.terminus_x)
                        self.terminus_flag=1
                        break
            if (self.flag1list[z] == 2):
                for i in range(cols/2,1970):
                    if (mapArray[i, map_camera_pointY]==100):
                        self.terminus_y=map_camera_pointY
                        self.terminus_x=i
                        print(self.terminus_x)
                        self.terminus_flag=1
                        break
            if (self.terminus_flag==1):
                # for i in range(-5,4):
                #     if (mapArray[self.terminus_x,self.terminus_y+i]==100):
                #         self.terminus_count=self.terminus_count+1
                # if(self.terminus_count>=5):
                #     self.terminus_count=0
                self.terminusXlist[z]=self.terminus_x
                self.terminusYlist[z]=self.terminus_y
            if (self.terminus_flag==0):
                self.terminusXlist[z]=map_camera_pointX
                self.terminusYlist[z]=map_camera_pointY
            self.terminus_flag=0

            for k in range(0,len(self.terminusXlist)):
                for r in range(0,8):
                    for c in range(0,8):
                        map[1984-self.terminusXlist[k]-r, self.terminusYlist[k]-c] = red
                        map[1984-self.terminusXlist[k]-r, self.terminusYlist[k]+c] = red
                        map[1984-self.terminusXlist[k]+r, self.terminusYlist[k]-c] = red
                        map[1984-self.terminusXlist[k]+r, self.terminusYlist[k]+c] = red

                    # self.terminus_print=1
        #if (self.terminus_print==0):            
        # for k in range(0,len(self.xlist)):
        #     print('开始')
        #     for i in range(0,8):
        #         for j in range(0,8):
        #             map[1984-(self.xlist[k] + rows / 2)-i, self.ylist[k]-j + cols / 2] = blue
        #             map[1984-(self.xlist[k] + rows / 2)-i, self.ylist[k]+j + cols / 2] = blue
        #             map[1984-(self.xlist[k] + rows / 2)+i, self.ylist[k]-j + cols / 2] = blue
        #             map[1984-(self.xlist[k] + rows / 2)+i, self.ylist[k]+j + cols / 2] = blue

        # for i in range(0,8):
		# 	for j in range(0,8):
		# 		map[0, 0] = blue
		# 		map[0, 0+j] = blue
		# 		map[0+i, 0] = blue
		# 		map[0+i, 0+j] = blue


         #金色目标点标记
        print(" 开始标记 !")
        print("self.info_list"),
        print(self.info_list)
        for i in range( 0 , len(self.terminusXlist)):
            print("标记第"),
            print(i),
            print("个二维码")
            #(int(((self.position_list[i])[0] + cols / 2 + 1) / self.delta) , /2 - 1 -  self.position_list[1]) / self.delta))
            cv2.putText(map, self.info_list[i],(1984-60-self.terminusXlist[i],self.terminusYlist[i]+60)  , cv2.FONT_HERSHEY_COMPLEX, 1, (200, 0, 200), 1)
       
        s = time.strftime("../save/saveMaps/%Y_%m_%d-%H:%M:%S.tiff", time.localtime(time.time()))
        print ("Saving...  "),
        print(s),
        cv2.imwrite(s, map)
        print ("Saved")


if __name__=='__main__':
    v = Map()
    
    rospy.spin()
