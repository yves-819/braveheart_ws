#include "braveheart_base/BraveheartMotor.h"

BraveheartMotor::BraveheartMotor()
{
    //私有参数 获取
    ros::NodeHandle nh_private("~");
    nh_private.param<string>("serialport_name", serialport_name, "/dev/base_serial");
    nh_private.param<int>("baudrate", baudrate, 9600);
    nh_private.param<int>("reduction" ,reduction , 100);
    nh_private.param<double>("wheel_radius" ,wheel_radius , 0.06);
    nh_private.param<double>("base_width" ,base_width , 0.35);
    nh_private.param<int>("PPR" ,PPR , 11 );
    nh_private.param<int>("QPR" ,QPR , PPR * 4);

    //串口初始化
    Timeout timeout = Timeout::simpleTimeout(1000);
    try
    {
        serialPort = new Serial(serialport_name, baudrate, timeout);
    }
    catch (exception e)
    {
        cout << "SerialPort Opened failed" << endl;
        exit(0);
    }
     
    

    //检查串口是否正常
    if(serialPort->isOpen()){cout << "SerialPort isOpen." << endl;}
    else{cout << "SerialPort Opened failed" << endl;return;}

    //通信协议初始化
    protocol = new MotorProtocol();

    //休眠3s,丢弃掉串口中错误的信息
    usleep(3000000);
    serialPort->flush ();
    
#ifdef DEBUG
    //调试
    last_dleft_encoder = 0;
    last_dright_encoder = 0;
    pub_config = nh.advertise<std_msgs::String>("config", 10);
#endif

    //设置一些初始值

    //发布者 接收者 初始化
    pub_odom = nh.advertise<nav_msgs::Odometry>("/old_odom", 10);
    sub_speed = nh.subscribe("/cmd_vel", 1, &BraveheartMotor::cmdVelCallBack, this);
    

    // ros 时钟控制
    ros::Rate loop_rate(30);
    time_prev = ros::Time::now();
    time_current = ros::Time::now();

    //ros消息 坐标系等初始化
    tf_transform.header.frame_id = "odom";
    tf_transform.child_frame_id = "base_footprint";
    odom.header.frame_id = "odom";
    odom.child_frame_id= "base_footprint";


    //里程计和姿态积分   长度单位 m  角度为 弧度角 即 旋转3.14为旋转半圈
    //单位时间从下位机读取编码器的数据  数据为 QPR，即PPR*4
    dleft_encoder = 0; dright_encoder = 0;
    //单位时间下机器人的姿态变化  坐标系为 前一时刻机器人坐标系
    dx = 0; dy = 0; dtheta = 0;
    //单位时间下机器人中心在x方向上移动的距离，单位时间下机
    d = 0; dleft = 0; dright = 0;
	//初始位置下 编码器的数据
	left_encoder = 0; right_encoder = 0;
	//初始位置下 机器人的姿态
	tf_x = 0; tf_y = 0; tf_theta = 0;
	//初始位置下 机器人中心在x方向上移动的距离，机器人的左右轮子移动的距离
	tf_distance = 0; tf_left = 0; tf_right = 0;

    //读串口设置
    //收数据缓存区
    uint8_t *rxBuffer = new uint8_t[protocol->getSumSize() * 2];
    //串口收缓存区长度
    size_t res_len = 0;
    //接收是否可以进行正常的读取
    canRead = false;
    canWrite = false;

    //期望的编码器脉冲数量
    right_except_pulse = 0; left_except_pulse = 0;
	//占空比15位 符号1位  共16位
	left_pwm = 0; right_pwm = 0;left2_pwm = 0; right2_pwm = 0;

    while (nh.ok())
    {
        ///////////////////////////////读串口///////////////////////////////
        //检查串口接收缓存区中数据的大小，判断是否能够使用协议进行解码
        if (serialPort->available() >= protocol->getSumSize())
        {  
            //读取协议封装长度的数据，并返回读取的数据长度
            res_len = serialPort->read(rxBuffer, protocol->getSumSize());

            //如果读取的数据长度多余协议进行解码所需要的最短长度，那就尝试进行解码
            if (res_len >= protocol->getSumSize())
            {
                for (size_t i = 0; i < protocol->getSumSize(); i ++)
                {   
                    //将uint8_t类型的数据保存到协议对象中
                    //查看是否解码已经成功
                    canRead = protocol->decode(rxBuffer[i]);
                    if (canRead)
                    {
                        //读取解码后的具体数据，解析完成后才能发送数据
                        publishOdom();
                        //cout << "receive data  " << protocol->getLeft() << "  " << protocol->getRight() << endl;
                    }
                }
            }
        }
        else
        {
            //cout << "No data get" << endl;
        }
        ///////////////////////////////读串口结束///////////////////////////////

        //写串口
        if (canWrite)
        {
            //检查串口数据输出缓存区状态
            int txnum = serialPort->write(protocol->txBuffer, protocol->getSumtxSize());
            /*
            for (size_t i = 0; i < protocol->getSumtxSize(); i++)
            {
                cout << (int)protocol->txBuffer[i] << " ";
            }
            cout << endl;
            */
            
            //将协议封装好的数据写入串口中
            //cout << "wrote :" << txnum << endl;
            canWrite = false;
            //文件参数
        #ifdef DEBUG
            pub_config.publish(cfg_msg);
        #endif
        }
        
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}

BraveheartMotor::~BraveheartMotor()
{
    //最后给下位机发送停止指令
    //写串口
    //将数据封装到通讯协议里面
    
    protocol->setLeft(0,0);
    protocol->setRight(0,0);
    if (protocol->encode())
    {   
        for (size_t i = 0; i < 10; i++)
        {
            try
            {
                //检查串口数据输出缓存区状态
                int txnum = serialPort->write(protocol->txBuffer, protocol->getSumtxSize());
                //将协议封装好的数据写入串口中
                //cout << "wrote stop  :" << txnum << endl;
                usleep(100000);
            }
            catch (exception e)
            {
                cout << "stop failed at " << i << " time " << endl; 
            }
        }
    }
    serialPort->close();

}


//回调函数
void BraveheartMotor::cmdVelCallBack(const geometry_msgs::Twist & msg)
{
#ifdef DEBUG
    //调试
    stringstream ss;
    ss.clear();
    ss.str("");
    ss << last_dleft_encoder << " " << left_pwm << "$" << last_dright_encoder << " " << right_pwm << "$";
    cfg_msg.data = ss.str();
    pub_config.publish(cfg_msg);
    left_pwm = mapping('L',msg.linear.x);
    right_pwm = mapping('R',msg.linear.x);
#else 
    //根据线速度设置 左右的速度
    double speed_l = msg.linear.x;
    double speed_r = msg.linear.x;

	if(speed_l<0.15&&speed_l > 0) {speed_l = 0.15;}
	else if(speed_l>-0.15&&speed_l < 0){speed_l = -0.15;}

	if(speed_r<0.15&&speed_r > 0) {speed_r = 0.15;}
	else if(speed_r>-0.15&&speed_r < 0){speed_r = -0.15;}

    //加上旋转的速度   线速度 = 角速度 * 半径， 然后分摊到左右两边
	if(msg.angular.z<1.1 && msg.angular.z>0)
	{
		speed_l -= (1.1 * base_width/2);
    	speed_r += (1.1 * base_width/2);
	}
	else if(msg.angular.z>-1.1 && msg.angular.z<0)
	{
		speed_l -= (-1.1 * base_width/2);
    	speed_r += (-1.1 * base_width/2);
	}
	else
	{
		speed_l -= (msg.angular.z * base_width/2);
    	speed_r += (msg.angular.z * base_width/2);
	}
	

    //cout << "Set speed_l " <<speed_l <<  ", speed_r: "<< speed_r << endl;

    //速度转化为目标脉冲数
    //速度 = ((脉冲数 / QPR)/减速比) *PI * 直径
    //脉冲数 = 速度* 减速比 * QPR/(PI*直径)
    left_except_pulse = round( 0.1 * speed_l * reduction * QPR /(2 * PI * wheel_radius)); 
    right_except_pulse = round( 0.1 * speed_r * reduction * QPR /(2 * PI * wheel_radius));
    
    //控制确定PWM占空比
    left_pwm = mapping('L',left_except_pulse);
    right_pwm = mapping('R',right_except_pulse);
    left2_pwm = mapping('l',left_except_pulse);
    right2_pwm = mapping('r',right_except_pulse);
#endif
    //写串口
    //将数据封装到通讯协议里面 
    protocol->setLeft(left_pwm,left2_pwm);
    protocol->setRight(right_pwm,right2_pwm);
    
    //协议编码
    if (protocol->encode())
    {   
        canWrite = true;
        //cout << "encode succeess\n";
    }
    else
    {
        canWrite = false;
        cout << "ENCODE FAILED: left or right value is too large for 16 bit" << endl;
        return;
    }
}//回调函数结束


//发布者发布函数
void BraveheartMotor::publishOdom()
{   
    //里程计和姿态积分   长度单位 m
    //获取协议解码后的数据
    dleft_encoder = protocol->getLeft();
    dright_encoder = protocol->getRight();
#ifdef DEBUG
    last_dleft_encoder = dleft_encoder;
    last_dright_encoder = dright_encoder;

#endif
    cout << "Set speed except_pulse: " << left_except_pulse << ", " << right_except_pulse << endl;//脉冲数(debuge模式下输出为0) 下面第二行相关
    cout << "Get speed measure_pulse: " << dleft_encoder << ", " << dright_encoder << endl;       //单位时间从下位机读取的编码器数据  即为 QPR
//    cout << "Set speed pwm: " <<left_pwm <<  ", "<< right_pwm<<", " <<left2_pwm <<  ", "<< right2_pwm << "\n\n";                 //占空比 debuge下为msg.linear.x
                                                                                               //正常模式 为由速度计算得到的脉冲数再mapping计算得到

    //里程计累加
    left_encoder += dleft_encoder;
    right_encoder += dright_encoder;

    //单位时间下机器人中心在x方向上移动的距离，单位时间下机器人的左右轮子移动的距离
    dleft = 2 * PI * wheel_radius * dleft_encoder / (reduction * QPR);
    dright =  2 * PI * wheel_radius * dright_encoder / (reduction * QPR);
    d = (dleft + dright) / 2;
    
    //轮子运动的距离
    tf_left = 2 * PI * wheel_radius * left_encoder  /(reduction * QPR);
    tf_right = 2 * PI * wheel_radius * right_encoder /(reduction * QPR);
    tf_distance = (tf_left + tf_right)/2;
    //cout<< "d     tf_distance  " << d << "  " << tf_distance << endl;
    //ROS_INFO("d : %f,  tf_distance : %f", d, tf_distance);

    //转角  z轴正方向
    dtheta = (dright - dleft) / base_width;
    tf_theta += dtheta;

    //cout<< "dtheta  tf_theta   " << dtheta << "  " << tf_theta << endl;
    //ROS_INFO("dtheta : %f,  tf_theta : %f", dtheta, tf_theta);
    

    //有数据时候才能进行角度变换
    if (d != 0)
    {
        dx = cos(dtheta) * d;
        dy = sin(dtheta) * d;
        tf_x += dx*cos(tf_theta) - dy*sin(tf_theta);
        tf_y += dx*sin(tf_theta) + dy*cos(tf_theta);
    }
    
    //获取时间
    time_current = ros::Time::now();
    double elapsed = (time_current - time_prev).toSec();
    time_prev = time_current;
    //cout << "elapsed  " << elapsed << endl;

    // 发布 tf 信息  imu融合时应该注释掉
    geometry_msgs::Quaternion odom_quaternion = tf::createQuaternionMsgFromYaw(tf_theta);
    // tf_transform.header.stamp = time_current;
    // tf_transform.transform.translation.x = tf_x;
    // tf_transform.transform.translation.y = tf_y;
    // tf_transform.transform.rotation =odom_quaternion;
    // tf_broadcaster.sendTransform(tf_transform);

    // 发布里程计信息

    //添加协方差（融合时使用）
      double my_ODOM_POSE_COVARIANCE[36] = {0.001, 0, 0, 0, 0, 0,
                                            0, 0.001, 0, 0, 0, 0,
                                            0, 0, 1000000, 0, 0, 0,
                                            0, 0, 0, 1000000, 0, 0,
                                            0, 0, 0, 0, 1000000, 0,
                                       0, 0, 0, 0, 0, 1000};

     double my_ODOM_POSE_COVARIANCE2[36] = {0.000000001, 0, 0, 0, 0, 0,
                                            0, 0.001, 0.000000001, 0, 0, 0,
                                            0, 0, 1000000, 0, 0, 0,
                                            0, 0, 0, 1000000, 0, 0,
                                            0, 0, 0, 0, 1000000, 0,
                                            0, 0, 0, 0, 0, 0.000000001};
    
     double my_ODOM_TWIST_COVARIANCE[36] = {0.001, 0, 0, 0, 0, 0, 
                                            0, 0.001, 0, 0, 0, 0,
                                            0, 0, 1000000, 0, 0, 0,
                                            0, 0, 0, 1000000, 0, 0,
                                            0, 0, 0, 0, 1000000, 0,
                                            0, 0, 0, 0, 0, 1000};

    double my_ODOM_TWIST_COVARIANCE2[36] = {0.000000001, 0, 0, 0, 0, 0, 
                                            0, 0.001, 0.000000001, 0, 0, 0,
                                            0, 0, 1000000, 0, 0, 0,
                                            0, 0, 0, 1000000, 0, 0,
                                            0, 0, 0, 0, 1000000, 0,
                                            0, 0, 0, 0, 0, 0.000000001};

    if (d!=0 && dtheta!=0)
    {
        int i = 0;
        for(;i<36;i++)
        {
            odom.pose.covariance[i] = my_ODOM_POSE_COVARIANCE[i];
            odom.twist.covariance[i] = my_ODOM_TWIST_COVARIANCE[i];    
        }
    }
    
    else
    {
        int i = 0;
        for(;i<36;i++)
        {
            odom.pose.covariance[i] = my_ODOM_POSE_COVARIANCE2[i];
            odom.twist.covariance[i] = my_ODOM_TWIST_COVARIANCE2[i];    
        }
    }
    

    odom.header.stamp = time_current;
    odom.pose.pose.position.x = tf_x;
    odom.pose.pose.position.y = tf_y;
    odom.pose.pose.orientation = odom_quaternion;
    odom.twist.twist.linear.x = d / elapsed;
    odom.twist.twist.angular.z = dtheta / elapsed;

    //cout << "linear.x  angular.z  " << odom.twist.twist.linear.x << " " << odom.twist.twist.angular.z << endl;
    pub_odom.publish(odom);

    //数据清零
    dleft_encoder = 0; dright_encoder = 0;
    dleft = 0; dright = 0; d=0;
    dtheta = 0;
    dx = 0; dy = 0;
    
}//发布者函数结束



//根据脉冲数量进行控制  参数为输入的脉冲期望
int BraveheartMotor::mapping(const char & LorR,const int & input )
{
    int output = 0;

    #ifdef DEBUG
        //调试模式
        output = input;
    #else
        if (LorR == 'L') //左轮映射
        {
            //用户自己编写映射程序，可在不同速度区间进行设置
			output = (1.5664473587513669e-43) * pow(input, 17)
+(-1.3144251226972983e-40) * pow(input, 16)
+(-4.129033819677336e-37) * pow(input, 15)
+(3.586705486944878e-34) * pow(input, 14)
+(4.465429538391345e-31) * pow(input, 13)
+(-4.02895999696567e-28) * pow(input, 12)
+(-2.5687385615398834e-25) * pow(input, 11)
+(2.3968288064633734e-22) * pow(input, 10)
+(8.625112866896923e-20) * pow(input, 9)
+(-8.066735669658716e-17) * pow(input, 8)
+(-1.7813084387609007e-14) * pow(input, 7)
+(1.5106825520839988e-11) * pow(input, 6)
+(2.3757031832521045e-09) * pow(input, 5)
+(-1.4279443585057565e-06) * pow(input, 4)
+(-0.00019554073556534823) * pow(input, 3)
+(0.05496574856330252) * pow(input, 2)
+(17.895686896254) * pow(input, 1)
+(-308.73850199486304) * pow(input, 0);

			
        }
        else if (LorR == 'R') //右轮映射
        {
         output = (5.082452132754643e-45) * pow(input, 17)
+(-4.710807217384896e-41) * pow(input, 16)
+(2.715933913551984e-40) * pow(input, 15)
+(1.2850024862930776e-34) * pow(input, 14)
+(-1.268391964652653e-32) * pow(input, 13)
+(-1.41799754240443e-28) * pow(input, 12)
+(9.811788994863044e-27) * pow(input, 11)
+(8.03533047519833e-23) * pow(input, 10)
+(-5.0443977501043004e-23) * pow(input, 9)
+(-2.4336465928553667e-17) * pow(input, 8)
+(-2.385656370118658e-15) * pow(input, 7)
+(3.632674272817557e-12) * pow(input, 6)
+(9.406625494451261e-10) * pow(input, 5)
+(-1.864993592666654e-07) * pow(input, 4)
+(-0.00013481893803894503) * pow(input, 3)
+(-0.003762790192332664) * pow(input, 2)
+(17.205377241440587) * pow(input, 1)
+(433.3370937057263) * pow(input, 0);
        }
    else if (LorR =='l') //左轮映射
    {
    
   output = (-1.149726882712182e-43) * pow(input, 17)
+(-7.02015760949734e-41) * pow(input, 16)
+(3.742961756830987e-37) * pow(input, 15)
+(2.162009960105794e-34) * pow(input, 14)
+(-4.945388887448681e-31) * pow(input, 13)
+(-2.7153228811480144e-28) * pow(input, 12)
+(3.4059384810077835e-25) * pow(input, 11)
+(1.7822297760847652e-22) * pow(input, 10)
+(-1.2993948223637356e-19) * pow(input, 9)
+(-6.503916982524058e-17) * pow(input, 8)
+(2.6676869183763515e-14) * pow(input, 7)
+(1.2947964204813298e-11) * pow(input, 6)
+(-2.524621642015372e-09) * pow(input, 5)
+(-1.2831187130968024e-06) * pow(input, 4)
+(5.2625345588794423e-05) * pow(input, 3)
+(0.052332948983994966) * pow(input, 2)
+(14.632934650475491) * pow(input, 1)
+(-366.15172058394205) * pow(input, 0);

        
    }
    else if (LorR =='r') //右轮映射
    {
output = (7.892135953273372e-46) * pow(input, 17)
+(-1.057068274015694e-40) * pow(input, 16)
+(2.1082878243052008e-38) * pow(input, 15)
+(3.079474983122742e-34) * pow(input, 14)
+(-5.070897558563398e-32) * pow(input, 13)
+(-3.656835256110547e-28) * pow(input, 12)
+(4.531752100029617e-26) * pow(input, 11)
+(2.269523260090383e-22) * pow(input, 10)
+(-1.8800652303233263e-20) * pow(input, 9)
+(-7.840121210042479e-17) * pow(input, 8)
+(3.3160352347816443e-15) * pow(input, 7)
+(1.4828645949441124e-11) * pow(input, 6)
+(-9.906043461546727e-12) * pow(input, 5)
+(-1.4083762109132244e-06) * pow(input, 4)
+(-6.0303853682245874e-05) * pow(input, 3)
+(0.0556698311054468) * pow(input, 2)
+(15.119058888851416) * pow(input, 1)
+(-447.51450135069194) * pow(input, 0);
    }

    #endif
    
    if(abs(output) > 0x7fff)
    {   
        cout << "Mapping Warn: left or right value is too large for 16 bit"<< endl;
        if(output>0)
        {
            output = 0x7fff;
        }
        else
        {
            output = - 0x7fff;
        }
    }
    output = (int)(output);
    return output;
}
