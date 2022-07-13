#ifndef BASE_COMMON
#define BASE_COMMON

//所以平时需要用到的头文件都放在这里
#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <unistd.h>
#include <typeinfo>
#include <inttypes.h>
#include <sstream>



#include <cmath>
#include <cstring>
#include <signal.h>
#include <vector>
#include "serial/serial.h"


//ros头文件
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"



using namespace std;
using namespace serial;

//常量定义
const double PI = 3.1415926;

//调试模式
//#define DEBUG 1


#endif // BASE_COMMON
