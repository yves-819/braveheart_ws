#!/usr/bin/env python 
# -*- coding: utf-8 -*-
# 复制粘贴 设置rviz里用鼠标点的导航点，
import roslib
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from random import sample  
from math import pow, sqrt  
import collections

class NavTest():  
    def __init__(self):  
        rospy.init_node('exploring_slam', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  

        # 在每个目标位置暂停的时间 (单位：s)
        self.rest_time = rospy.get_param("~rest_time", 2)  

        # 是否仿真？  
        self.fake_test = rospy.get_param("~fake_test", True)  

        # 到达目标的状态  
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',   
                       'SUCCEEDED', 'ABORTED', 'REJECTED',  
                       'PREEMPTING', 'RECALLING', 'RECALLED',  
                       'LOST']  
 
        # 设置目标点的位置  
        # 在rviz中点击 2D Nav Goal 按键，然后单击地图中一点  
        # 在终端中就会看到该点的坐标信息  复制到下面即可
        locations = collections.OrderedDict()
        #locations = dict()  

        #locations['1'] = Pose(Point(1.042, 1.008, 0.000), Quaternion(0.000, 0.000, 0.308, 0.951))
        #locations['2'] = Pose(Point(-1.035, 0.013, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))
        #locations['3'] = Pose(Point(-0.247, 0.552, 0.000), Quaternion(0.000, 0.000, 0.693, 0.721))
        #locations['4'] = Pose(Point(1.042, 0.008, 0.000), Quaternion(0.000, 0.000, 0.000, 0.100))
        #locations['5'] = Pose(Point(1.042, 0.008, 0.000), Quaternion(0.000, 0.000, 0.000, 0.100))

        locations['1']= Pose(Point(1.38777878078e-17, 1.73472347598e-18, 0.0), Quaternion(-0.0, -0.0, -0.000363602786508, 0.999522057355))
        locations['2']= Pose(Point(2.43862443693, -0.0253279830589, 0.0), Quaternion(-0.0, -0.0, 0.622744312888, 0.781862937704))
        locations['3']= Pose(Point(2.62691789759, 1.21039538423, 0.0), Quaternion(-0.0, -0.0, 0.0614390258952, 0.997659887254))
        locations['4']= Pose(Point(4.73685775751, 1.22383332489, 0.0), Quaternion(0.0, -0.0, 0.99393171128, 0.105094430854))
        locations['5']= Pose(Point(2.568441092, 1.42905584992, 0.0), Quaternion(-0.0, 0.0, -0.78097037683, 0.623823929395))

        locations['6']= Pose(Point(2.92556405255, -0.9056763117, 0.0), Quaternion(-0.0, 0.0, -0.277463486912, 0.815924052602))
        locations['7']= Pose(Point(2.92556405255, -1.7956763117, 0.0), Quaternion(-0.0, 0.0, -0.577463486912, 0.815924052602))
        locations['8']= Pose(Point(3.35910627154, -3.72552255143, 0.0), Quaternion(-0.0, -0.0, -0.131253648868, 0.99087243057))
        locations['9']= Pose(Point(5.71515469924, -3.86298575146, 0.0), Quaternion(-0.0, -0.0, 0.790725510815, 0.611461100273))
        locations['10']= Pose(Point(5.90973448457, -2.58538155937, 0.0), Quaternion(-0.0, -0.0, 0.064386991295, 0.997483334303))
        # 发布控制机器人的消息  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)  

        # 订阅move_base服务器的消息  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  

        rospy.loginfo("Waiting for move_base action server...")  

        # 60s等待时间限制  
        self.move_base.wait_for_server(rospy.Duration(20))  
        rospy.loginfo("Connected to move base server")  
  
        # 保存机器人的在rviz中的初始位置  
        initial_pose = PoseWithCovarianceStamped()  

        # 保存成功率、运行时间、和距离的变量  
        n_locations = len(locations)  # 获取有多少个目标点
        n_goals = 0  # 实际程序走了多少个目标点
        n_successes = 0  # 成功到达目标点的次数
        i = n_locations  # 用来判断目标点是否走完了
        distance_traveled = 0  
        start_time = rospy.Time.now()  
        running_time = 0  
        location = ""  
        last_location = ""  
        i = 0  
 
        # 确保有初始位置  
        while initial_pose.header.stamp == "":  
            rospy.sleep(1)  

        rospy.loginfo("Starting navigation test")  

        # 开始主循环，随机导航  
        while not rospy.is_shutdown():  
            # 如果已经走完了所有点，再重新开始排序  
            # if i == n_locations:  
            #     i = 0  
            #     #sequence = sample(locations,len(locations))
            #     sequence = list(locations.keys())
            #     # 如果最后一个点和第一个点相同，则跳过  
            #     if sequence[0] == last_location:  
            #         i = 1  
            
            
            sequence = list(locations.keys())
            # 在当前的排序中获取下一个目标点  
            location = sequence[i]  

            # 跟踪行驶距离  
            # 使用更新的初始位置  
            if initial_pose.header.stamp == "":  
                distance = sqrt(pow(locations[location].position.x -   
                                    locations[last_location].position.x, 2) +  
                                pow(locations[location].position.y -   
                                    locations[last_location].position.y, 2))  
            else:  
                rospy.loginfo("Updating current pose.")  
                distance = sqrt(pow(locations[location].position.x -   
                                    initial_pose.pose.pose.position.x, 2) +  
                                pow(locations[location].position.y -   
                                    initial_pose.pose.pose.position.y, 2))  
                initial_pose.header.stamp = ""  

            # 存储上一次的位置，计算距离  
            last_location = location  

            # 计数器加1

            if(i >= 11):
                 rospy.sleep(1000)
            i += 1  
            n_goals += 1  

            # 设定下一个目标点  
            self.goal = MoveBaseGoal()  
            self.goal.target_pose.pose = locations[location]  
            self.goal.target_pose.header.frame_id = 'map'  
            self.goal.target_pose.header.stamp = rospy.Time.now()  

            # 让用户知道下一个位置  
            rospy.loginfo("Going to: " + str(location))  

            # 向下一个位置进发  
            self.move_base.send_goal(self.goal)  

            # 五分钟时间限制  
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))   

            # 查看是否成功到达  
            if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
            else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Goal succeeded!")  
                    n_successes += 1  
                    distance_traveled += distance  
                    rospy.loginfo("State:" + str(state))  
                else:  
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))  

            # 运行所用时间  
            running_time = rospy.Time.now() - start_time  
            running_time = running_time.secs / 60.0  

            # 输出本次导航的所有信息  
            rospy.loginfo("Success so far: " + str(n_successes) + "/" +   
                          str(n_goals) + " = " +   
                          str(100 * n_successes/n_goals) + "%")  

            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +   
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")  

            rospy.sleep(self.rest_time)  
# 主循环结束
    def update_initial_pose(self, initial_pose):  
        self.initial_pose = initial_pose  

    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  
    # 此时类才结束(编辑器收起函数功能有问题)

def trunc(f, n):  
    slen = len('%.*f' % (n, f))  

    return float(str(f)[:slen])  

if __name__ == '__main__':  
    try:  
        NavTest()  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Exploring SLAM finished.")
