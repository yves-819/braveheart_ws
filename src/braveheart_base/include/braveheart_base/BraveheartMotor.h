#ifndef BRAVEHEARTMOTOR
#define BRAVEHEARTMOTOR

#include "braveheart_base/MotorProtocol.h"


class BraveheartMotor
{
public:
	BraveheartMotor();
	virtual ~BraveheartMotor();

	//回调函数
	void cmdVelCallBack(const geometry_msgs::Twist & msg);

	//发布者发布函数
	void publishOdom();

	//PID调速返回值为 15位精度的pwm占空比，在此处上位机对底盘进行最后的控制
	int mapping(const char & LorR,const int & input );

private:
#ifdef DEBUG
	//调试
	ros::Publisher pub_config;
	std_msgs::String cfg_msg;
	int last_dleft_encoder, last_dright_encoder;
#endif
	
	//ros交互
	ros::NodeHandle nh;
	ros::Publisher pub_odom;
    ros::Subscriber sub_speed;

	tf::TransformBroadcaster tf_broadcaster;

	//ros消息
	geometry_msgs::TransformStamped tf_transform;
    nav_msgs::Odometry odom;
	

	//ros系统时间
	ros::Time time_current, time_prev;

	//串口交互
	Serial * serialPort;
	string serialport_name;
    int baudrate;

	//编码可读可发送标志位
	bool canWrite;
	bool canRead;

	//通信协议
	MotorProtocol * protocol;

	//里程计和姿态积分   长度单位 m
    //单位时间从下位机读取编码器的数据  数据为 QPR，即PPR*4
    int dleft_encoder, dright_encoder;
    //单位时间下机器人的姿态变化  坐标系为 前一时刻机器人坐标系
    double dx, dy, dtheta;
    //单位时间下机器人中心在x方向上移动的距离，单位时间下机器人的左右轮子移动的距离
    double d, dleft, dright;
	//初始位置下 编码器的数据
	long long left_encoder, right_encoder;
	//初始位置下 机器人的姿态
	long double tf_x, tf_y, tf_theta;
	//初始位置下 机器人中心在x方向上移动的距离，机器人的左右轮子移动的距离
	long double tf_distance, tf_left, tf_right;
	//期望的编码器脉冲数量
    int right_except_pulse, left_except_pulse;
	//占空比15位 符号1位  共16位
	int left_pwm, right_pwm,left2_pwm, right2_pwm;
	

	//物理尺寸					 长度单位 m   
	int reduction;				//电机减速比
	double wheel_radius;			//轮子半径
	double base_width;				//轮子间距

	int PPR;					//编码器线数
	int QPR;					//PPR*4  编码器转一圈获得脉冲数量
};

#endif //BRAVEHEARTMOTOR