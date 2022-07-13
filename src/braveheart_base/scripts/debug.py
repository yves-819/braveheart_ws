#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import numpy as np
import matplotlib.pyplot as plt
import fstream
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String


class Debug(object):
    def __init__(self):
        rospy.init_node('debug_node', anonymous=False)

        #获取路径名
        self.direction = rospy.get_param("~direction", "")
        #精度，此处为pwm数据递增的速度，精度越小，测试需要的时间越长
        self.accuracy = rospy.get_param("~accuracy", 0xff)
        #停留时间，此处为等待电机速度调整至稳态所需的时间(s)，时间越长，测量结果越精确
        self.duration = rospy.get_param("~duration", 1.0)

        #速度命令源
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #接收数据
        self.sub = rospy.Subscriber("/config", String, self.callback )
        #文件名
        self.file = self.direction + "/data.txt"
        
        fstream.mkdir(self.direction)
        fstream.touch(self.file)
        fstream.clear(self.file)
        fstream.echo(self.file, "inputLeft outputLeft inputRight outputRight\n\n")

        #消息，此处速度为真是的占空比pwm 0~0x7fff
        self.cmd = Twist(Vector3(-0x7fff,0,0), Vector3(0,0,0))
        #结束标志
        self.endflag = False

        while not rospy.is_shutdown():
            if self.endflag:
                break
            else:
                self.publish()
            time.sleep(self.duration)
        rospy.spin()
        self.calculate()
        
    def publish(self):
        if abs(self.cmd.linear.x) > 0x7fff:
            self.endflag = True
            self.cmd.linear.x = 0
            print("数据保存完毕。请按ctrl + c退出")
        else:
            self.cmd.linear.x += self.accuracy
        self.pub.publish(self.cmd)


    def callback(self, msg):
        str = msg.data
        strlist = str.split('$')
        print(strlist)
        for i in range(0, len(strlist)-1):
            print("保存数据 %s"%strlist[i])
            fstream.echo(self.file, strlist[i] + " ")
        fstream.echo(self.file, "\n" )

    def calculate(self):
        ofilename = self.direction + "/fuction.txt"
        fstream.touch(ofilename)
        list = fstream.getLineList(self.file, 2, fstream.getLines(self.file))#第二行到最后进行数据加载
        inputLeft = []
        inputRight = []
        outputLeft = []
        outputRight = []
        exceptOutputLeft= []
        exceptOutputRight= []


        for i in range(0, len(list)):
            tmplist = list[i].split(" ")  #加载其中一行
            if abs(int(tmplist[0])) > 20:     #加载左轮数据
                inputLeft.append(int(tmplist[0]))
                outputLeft.append(int(tmplist[1]))
            if abs(int(tmplist[0])) > 20:     #加载左轮数据
                inputRight.append(int(tmplist[2]))
                outputRight.append(int(tmplist[3]))
        
        #数据预处理装载
        x_l = np.array(inputLeft)
        y_l = np.array(outputLeft)
        x_r = np.array(inputRight)
        y_r = np.array(outputRight)

        #用17次多项式拟合
        degree = 17
        f_l = np.polyfit(x_l, y_l, degree)
        f_r = np.polyfit(x_r, y_r, degree)
        p_l = np.poly1d(f_l)
        p_r = np.poly1d(f_r)
        function_string_l = "Left\noutput = "
        function_string_r = "Right\noutput = "
        for i in range(0, len(f_l)):
            degree = len(f_l) - i - 1
            function_string_l += "(" + str(f_l[i]) + ") * pow(input, " + str(degree) + ")"
            if i < len(f_l) - 1:
                function_string_l += "\n+" 
            else:
                function_string_l += ";"
        for i in range(0, len(f_r)):
            degree = len(f_r) - i - 1
            function_string_r += "(" + str(f_r[i]) + ") * pow(input, " + str(degree) + ")"
            if i < len(f_l) - 1:
                function_string_r += "\n+" 
            else:
                function_string_r += ";"
        
        #写入文件
        print (function_string_l)
        fstream.echo(ofilename, function_string_l)
        fstream.echo(ofilename, "\n\n\n")
        print (function_string_r)
        fstream.echo(ofilename, function_string_r)
        
        exceptOutputLeft = p_l(inputLeft)
        exceptOutputRight = p_r(inputRight)
        p0 = plt.figure(figsize=(20,24))

        ax0 = p0.add_subplot(2,1,1)
        plt.plot(inputLeft, outputLeft,'r.', label='original')
        plt.plot(inputLeft, exceptOutputLeft,'c-', label='except')
        plt.xlabel("input")
        plt.ylabel("output")
        plt.title("Left IO")
        
        ax1 = p0.add_subplot(2,1,2)
        plt.plot(inputRight, outputRight,'r.', label='original')
        plt.plot(inputRight, exceptOutputRight,'c-', label='except')
        plt.xlabel("input")
        plt.ylabel("output")
        plt.title("Right IO")
        
        plt.savefig( self.direction + "/输入输出图.png")
        print("数据计算完毕。请按ctrl + c退出")
        print("hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh")
