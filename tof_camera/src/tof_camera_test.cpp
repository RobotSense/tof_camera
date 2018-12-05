#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


#define IMAGE_WIDTH    160
#define IMAGE_HEIGHT    60   //图像尺度
cv_bridge::CvImagePtr cv_ptr;  //申明一个CvImagePtr指针
unsigned short disMax,disMin;

void on_mouse(int event, int x, int y, int flags, void *ustc)   
{
	
	switch (event) {
	case CV_EVENT_LBUTTONDOWN://按下左键
	{       
		if ((cv_ptr->image.at<unsigned short>(y,x)<10000)&&(cv_ptr->image.at<unsigned short>(y,x)>10))
                ROS_INFO("distance:%d cm",cv_ptr->image.at<unsigned short>(y/4,x/4)/10);
                else
                ROS_INFO("distance:NA");
	}	break;
        }
} 
/*回调函数*/
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  static unsigned short maxLast=0;
  cv::Mat gray(60,160,CV_8UC1);  //创建灰度图矩阵
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg,"16UC1");//读取消息 深度图为 cv_ptr->image  
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to '16UC1'.", msg->encoding.c_str());//错误信息
  }


  disMax=8000;
  if ((disMax<maxLast)&&((maxLast-disMax)<2000))
      disMax=maxLast;
  maxLast=disMax;
  for(int i=0;i<IMAGE_HEIGHT;i++)                                               //遍历矩阵 转换为灰度图
  {
     for(int j=0;j<IMAGE_WIDTH;j++)
     {  
        if((cv_ptr->image.at<unsigned short>(i,j) < 10000 )&& (cv_ptr->image.at<unsigned short>(i,j) > 10))
        {
          gray.at<unsigned char>(i,j)= (cv_ptr->image.at<unsigned short>(i,j) - 10) *255/ (disMax - 10) ;  //转换到0~255之间
        }
        else
        {
          gray.at<unsigned char>(i,j)=255;//不在范围赋值0
        }
     }
  }
  //转换完成
  cv::namedWindow("view");
  cv::setMouseCallback("view", on_mouse, 0);//调用回调函数

  cv::applyColorMap(gray, gray, cv::COLORMAP_RAINBOW);//伪彩映射
   for(int i=0;i<IMAGE_HEIGHT;i++)                                               //遍历矩阵 转换为灰度图
  {
     for(int j=0;j<IMAGE_WIDTH;j++)
     {  
        if(gray.at<cv::Vec3b>(i, j)[0]==255&&gray.at<cv::Vec3b>(i, j)[2]==170)
             {
               gray.at<cv::Vec3b>(i, j)[0]=0;
	       gray.at<cv::Vec3b>(i, j)[1]=0;
	       gray.at<cv::Vec3b>(i, j)[2]=0;
             }    
     }
  } 

  cv::resize(gray,gray,cv::Size(gray.cols*4,gray.rows*4),0,0,cv::INTER_LINEAR);

  cv::imshow("view", gray);                       //显示彩色图片
  cv::waitKey(1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener"); //初始化节点
  ros::NodeHandle nh;  
  cv::namedWindow("view");                 //图像显示窗口
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("tof/image", 1, imageCallback);//订阅消息
  ros::spin();
  cv::destroyWindow("view");
  return 0;
}
