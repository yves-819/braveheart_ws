#!/usr/bin/env python 
# -*- coding: utf-8 -*-

# 为前面几个python文件提供的库函数

import os


def mkdir(path):
    folder = os.path.exists(path) 
    if not folder:
        os.makedirs(path) 
        print "创建",path
    else: 
        print path,",已经存在此目录" 


def touch(file_name): 
    folder = os.path.exists(file_name) 
    if folder:
        '''
        os.remove(file_name)
        os.mknod(file_name)
        print file_name,",重新创建"
        '''
        fo = open(file_name, "a")
        fo.write("\n\n")
        fo.close()
    else: 
        os.mknod(file_name)
        print "创建",file_name


def loadGoalList( file_name ):
    fo = open( file_name, "r")
    lines_count = len(fo.readlines())
    #print(lines_count)
    fo.close()

    fo = open( file_name, "r")
    goal_list = []
    for i in range(1 , lines_count + 1):
        line = fo.readline()
        strlist = line.split(" ")
        floatlist = []
        if len(strlist) > 1:
            for j in range(1 , len(strlist)):
                floatlist.append( float(strlist[j]) )
            goal_list.append(floatlist)
        else:
            temp_len = len(goal_list)
            for j in range(0, temp_len):
                goal_list.pop(0)
            temp_len = 0
    fo.close()
    return goal_list


def loadFiles():
    pass
