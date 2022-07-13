#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import os

#创建目录
def mkdir(direction):
    folder = os.path.exists(direction) 
    if not folder:
        os.makedirs(direction)
        print ("创建文件夹%s"%(direction))
    else: 
        print ("%s,已经存在此目录" % (direction))

#创建文件
def touch(file_name): 
    folder = os.path.exists(file_name) 
    if folder:
        os.remove(file_name)
        os.mknod(file_name)
        print ("%s文件已存在,重新创建"%(file_name))
    else: 
        os.mknod(file_name)
        print ("创建%s"%(file_name))

#写操作 
def echo(file_name, string):
    folder = os.path.exists(file_name) 
    if folder:
        fo = open(file_name, "a")
        fo.write(string)
        fo.close()
    else:
        print("路径下不存在此文件 %s"%(file_name))

#读操作
def cat(file_name):
    folder = os.path.exists(file_name) 
    if folder:
        fo = open(file_name, "r")
        string = fo.read()
        fo.close()
        return string
    else:
        print("路径下不存在此文件 %s"%(file_name))
        return ""

#清空操作
def clear(file_name):
    folder = os.path.exists(file_name) 
    if folder:
        fo = open(file_name, "w")
        fo.write("")
        fo.close()
    else:
        print("路径下不存在此文件 %s"%(file_name))

#获取行数
def getLines(file_name):
    folder = os.path.exists(file_name)
    lines = 0
    if folder:
        fo = open(file_name, "r")
        lines = len(fo.readlines())  
        fo.close()
    else:
        print("路径下不存在此文件 %s"%(file_name))
    return lines

#获取文件内容
def getLineList(file_name, start, end):
    string = cat(file_name)
    if string is "":
        return []
    else:
        list = string.split("\n")
        return list[start:end]