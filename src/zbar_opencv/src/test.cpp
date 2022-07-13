// #include <sstream>
// #include <ros/ros.h>
// #include <std_msgs/String.h>
// #include <tf/transform_broadcaster.h>

// int main(int argc,char** argv)
// {
//     ros::init(argc, argv, "test1");
//     ros::NodeHandle n;
    
//     ros::Rate loop_rate(100);   //控制节点运行频率

//     tf::TransformBroadcaster broadcaster;   //tf广播

//     tf::Transform base_world; //转换关系
//     tf::Quaternion q;
//     q.setRPY(0,0,0);
//     base_world.setRotation(q);
//     base_world.setOrigin(tf::Vector3(0,1,0));
    
//     while(n.ok())
//     {
//         broadcaster.sendTransform(tf::StampedTransform(base_world,ros::Time::now(),"Target_frame","base_link"));
//         loop_rate.sleep();
//     }
//     return 0;

// }




#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_publisher");//初始化ROS节点与节点名称
    ros::NodeHandle n;                    //创建节点的句柄
    ros::Rate loop_rate(100);             //控制节点运行的频率,与loop.sleep共同使用

    tf::TransformBroadcaster broadcaster; //创建tf广播器
    
    tf::Transform base2laser;
    tf::Quaternion q;
    q.setRPY(0,0,0);
    base2laser.setRotation(q);              //设置旋转坐标
    base2laser.setOrigin(tf::Vector3(1,0,0));//设平移坐标，laser在base的(1,0,0)位置

    while (n.ok())
    {
        //循环发布坐标变换，两种方式
        broadcaster.sendTransform(tf::StampedTransform(base2laser,ros::Time::now(),"Target_frame","base_link"));
        //broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 0), tf::Vector3(1, 0.0, 0)),ros::Time::now(),"base_link", "base_laser"));
        loop_rate.sleep();
    }
    return 0; 
}


