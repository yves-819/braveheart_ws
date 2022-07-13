#include <sstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>

bool flag1 = 0;
void cameraCallback(const std_msgs::String::ConstPtr& msg)  //回调函数 
{		
	flag1 = 1;
	tf::TransformListener listener;
	geometry_msgs::PointStamped  camera_point;
	try
	{
		camera_point.header.frame_id="base_link";  //绑定到base_link下
		camera_point.header.stamp=ros::Time();
		camera_point.point.x=1;     //设置摄像头相对于base_link的原点
		camera_point.point.y=0;
		camera_point.point.z=0;

		geometry_msgs::PointStamped  result_point; 
		
		bool A = listener.waitForTransform("Target_frame","base_link",ros::Time(0),ros::Duration(3.0));
		ROS_INFO("%d",A);
		listener.transformPoint("Target_frame",camera_point,result_point);
		ROS_INFO("base_link:(%2.f,%2.f,%2.f)---->--->--Target_frame:(%2.f,%2.f,%2.f)",
		camera_point.point.x,
		camera_point.point.y,
		camera_point.point.z,
		result_point.point.x,
		result_point.point.y,
		result_point.point.z);
	}
 	catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    //   continue;
	}
}

int main(int argc,char** argv) 
{
	ros::init(argc,argv,"camera");
	ros::NodeHandle node;

	// ros::Rate loop_rate(100); 
	ros::Subscriber camera_sub=node.subscribe("zbar_result",1000, cameraCallback);

	ros::spin();
	return 0;
}
