#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include "std_msgs/String.h"

using namespace std;
using namespace cv;
using namespace zbar;
//static const std::string OPENCV_WINDOW="Image window";
          std_msgs::String  msg,msgflag,panduan;
          bool flag = 0; int flag1 = 0;

void zbarscanner(cv_bridge::CvImagePtr cv_ptr)
{
        // Create a zbar reader
    ImageScanner scanner;

    // Configure the reader
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);


        // Capture an OpenCV frame   捕获一个OpenCV框架
        cv::Mat frame,frame_grayscale;
        frame=cv_ptr->image;
        // Convert to grayscale  转化为灰度
        cvtColor(frame, frame_grayscale, CV_BGR2GRAY);

        // Obtain  image data 获取图像数据
        int width = frame_grayscale.cols;
        int height = frame_grayscale.rows;
        uchar *raw = (uchar *)(frame_grayscale.data);

        // Wrap image data  包装图像数据
        Image image(width, height, "Y800", raw, width * height);

        // Scan the image for barcodes  扫描图像中的条形码
//
        scanner.scan(image);
   
        // Extract results   提取结果
        int counter = 0;
        for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
            
                msg.data = symbol->get_data();

                if(msg.data!="" && msgflag.data != msg.data)
                {
                    flag =1;
                    flag1 = 1;
                }
                msgflag.data = msg.data;

            // Draw  location of the  symbols found   绘制找到的符号的位置
            if (symbol->get_location_size() == 4) {

                line(frame, Point(symbol->get_location_x(0), symbol->get_location_y(0)), Point(symbol->get_location_x(1), symbol->get_location_y(1)), Scalar(0, 255, 0), 2, 8, 0);
                line(frame, Point(symbol->get_location_x(1), symbol->get_location_y(1)), Point(symbol->get_location_x(2), symbol->get_location_y(2)), Scalar(0, 255, 0), 2, 8, 0);
                line(frame, Point(symbol->get_location_x(2), symbol->get_location_y(2)), Point(symbol->get_location_x(3), symbol->get_location_y(3)), Scalar(0, 255, 0), 2, 8, 0);
                line(frame, Point(symbol->get_location_x(3), symbol->get_location_y(3)), Point(symbol->get_location_x(0), symbol->get_location_y(0)), Scalar(0, 255, 0), 2, 8, 0);
            }

            counter++;
        }

}

void zbarscanner2(cv_bridge::CvImagePtr cv_ptr)
{
        // Create a zbar reader
    ImageScanner scanner;

    // Configure the reader
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);


        // Capture an OpenCV frame   捕获一个OpenCV框架
        cv::Mat frame,frame_grayscale;
        frame=cv_ptr->image;
        // Convert to grayscale  转化为灰度
        cvtColor(frame, frame_grayscale, CV_BGR2GRAY);

        // Obtain  image data 获取图像数据
        int width = frame_grayscale.cols;
        int height = frame_grayscale.rows;
        uchar *raw = (uchar *)(frame_grayscale.data);

        // Wrap image data  包装图像数据
        Image image(width, height, "Y800", raw, width * height);

        // Scan the image for barcodes  扫描图像中的条形码
//
        scanner.scan(image);
   
        // Extract results   提取结果
        int counter = 0;
        for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
            
                msg.data = symbol->get_data();

                if(msg.data!="" && msgflag.data != msg.data)
                {
                    flag =1;
                    flag1 = 2;
                }
                msgflag.data = msg.data;

            // Draw  location of the  symbols found   绘制找到的符号的位置
            if (symbol->get_location_size() == 4) {

                line(frame, Point(symbol->get_location_x(0), symbol->get_location_y(0)), Point(symbol->get_location_x(1), symbol->get_location_y(1)), Scalar(0, 255, 0), 2, 8, 0);
                line(frame, Point(symbol->get_location_x(1), symbol->get_location_y(1)), Point(symbol->get_location_x(2), symbol->get_location_y(2)), Scalar(0, 255, 0), 2, 8, 0);
                line(frame, Point(symbol->get_location_x(2), symbol->get_location_y(2)), Point(symbol->get_location_x(3), symbol->get_location_y(3)), Scalar(0, 255, 0), 2, 8, 0);
                line(frame, Point(symbol->get_location_x(3), symbol->get_location_y(3)), Point(symbol->get_location_x(0), symbol->get_location_y(0)), Scalar(0, 255, 0), 2, 8, 0);
            }

            counter++;
        }

}

class ImageConverter
{
    ros::NodeHandle nh;

    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Subscriber image_sub2;
    image_transport::Publisher image_pub;
     

  public:
    ImageConverter():it(nh)
    {
        //使用image_transport订阅图像话题“in” 和 发布图像话题“out”
        image_sub=it.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb,this);
        image_sub2=it.subscribe("/usb_cam2/image_raw",1,&ImageConverter::imageCb2,this);
       // image_pub=it.advertise("zbar_opencv",1);

    }

    ~ImageConverter(){}

    //订阅回调函数
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            //将ROS图像消息转化为适合Opencv的CvImage
            cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s",e.what());
            return;
        }
     
        zbarscanner(cv_ptr);
        //image_pub.publish(cv_ptr->toImageMsg());
    }

    void imageCb2(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            //将ROS图像消息转化为适合Opencv的CvImage
            cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s",e.what());
            return;
        }
     
        zbarscanner2(cv_ptr);
        //image_pub.publish(cv_ptr->toImageMsg());
    }
};


int main(int argc, char **argv) {

    ros::init(argc,argv,"zbar_opencv");
    ros::NodeHandle n;
    ros::Publisher zbar_result_pub = n.advertise<std_msgs::String>("zbar_result", 1);
    ros::Publisher zuo_pub = n.advertise<std_msgs::String>("zuo", 1);
    ros::Publisher you_pub = n.advertise<std_msgs::String>("you", 1);
    ImageConverter ic;
    
    std::stringstream ss,sss;
    ss << "cnm" ;
    msgflag.data = ss.str();
    // 设置循环的频率
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
        //cout<< msg.data << endl;

        		// 循环等待回调函数

         if(flag1 == 1)
       {
           flag1 = 0;
           zuo_pub.publish(panduan);
       }
       else if(flag1 == 2)
       {
           flag1 = 0;
           you_pub.publish(panduan);
       }
       if(flag == 1)
       {
           flag = 0;
           zbar_result_pub.publish(msg);
       }
      
                
          ros::spinOnce();
	
		// 按照循环频率延时
         loop_rate.sleep();
  }

    return 0;
}

