#include "joy_control/TeleopJoy.h"

TeleopJoy::TeleopJoy():
  linear_(1),
  angular_(0)
{
  // 在node中接受launch文件参数，参数名自定义(两边相同)，参数二为设置参数，参数三为没有接收参数时的默认值
  //接受到launch里面param标签里名为axis_linear变量的值赋给第二个参数
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);  //angular_ 等变量在h文件定义
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  nh_.param("Y", increase_linear_, increase_linear_);
  nh_.param("A", decrease_linear_, decrease_linear_);
  nh_.param("X", increase_angular_, increase_linear_);
  nh_.param("B", decrease_angular_, decrease_angular_);

  //把cmd_vel 赋值给joy_cmd_vel 即两者相等 
  nh_.param<std::string>("joy_cmd_vel", joy_cmd_vel_, "/cmd_vel");

  //std::cout << joy_cmd_vel_ << std::endl;

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>(joy_cmd_vel_, 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopJoy::joyCallback, this);

}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if( joy->buttons[increase_linear_] > 0 )
  {
      l_scale_ += 0.1 * l_scale_;  //原始数据为0.1
      std::cout <<  "linear : " << l_scale_ << std::endl;
  }
  if( joy->buttons[decrease_linear_] > 0)
  {
      l_scale_ -= 0.1 * l_scale_;
      std::cout <<  "linear : " << l_scale_ << std::endl;
  }

  if( joy->buttons[increase_angular_] > 0)
  {
      a_scale_ += 0.1 * a_scale_;
      std::cout <<  "angular : " << a_scale_ << std::endl;
  }
  if( joy->buttons[decrease_angular_] > 0)
  {
      a_scale_ -= 0.1 * a_scale_;
      std::cout <<  "angular : " << a_scale_ << std::endl;
  }


  geometry_msgs::Twist twist;
  twist.angular.z =  a_scale_*joy->axes[angular_];
  twist.linear.x = l_scale_*joy->axes[linear_];
  

  vel_pub_.publish(twist);
}
