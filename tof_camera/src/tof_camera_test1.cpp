#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
using namespace std;
#define IMAGE_WIDTH    160
#define IMAGE_HEIGHT    60   //图像尺度
int nframe=0;
double cx=87.1415;
double cy=19.5734;
double fx=113.0965;
double fy=111.6327;
double depthScale=1000.0;
unsigned short disMax,disMin;
cv_bridge::CvImagePtr cv_ptr;  //申明一个CvImagePtr指针
cv::Mat gray_b;

//检测函数
cv::Mat MoveDetect(cv::Mat background,cv::Mat frame,cv::Mat re)
{   
    int xmax=0;
    int xmin=160;
    int ymax=0;
    int ymin=60;
    cv::Point ccs;
    cv::Mat result = re.clone();

    cv::Mat diff;
    cv::absdiff(background, frame, diff);
 
    cv::Mat diff_thresh;
    cv::threshold(diff, diff_thresh, 30, 255, CV_THRESH_BINARY);

    cv::Mat kernel_erode = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat kernel_dilate = getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
    cv::erode(diff_thresh, diff_thresh, kernel_erode);

    cv::dilate(diff_thresh, diff_thresh, kernel_dilate);

    vector< vector<cv::Point> > contours;

    cv::findContours(diff_thresh, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    vector<cv::Moments> contours_moments(contours.size());
    vector<cv::Point2f> centers(contours.size());
   if(contours.size()!=0)
    {

	for (size_t i = 0; i < contours.size(); i++)
	{
		if(contours[i].size()>70)
               {
		contours_moments[i] = moments(contours[i]);
		centers[i] = cv::Point(static_cast<float>(contours_moments[i].m10 / contours_moments[i].m00), static_cast<float>(contours_moments[i].m01 / contours_moments[i].m00));
		//图像中心Center(x0, y0)=(m10/m00,m01/m00)
       cv::circle(result,centers[i], 2,  cv::Scalar(0,255, 0));//绘制质点
       cv::drawContours(result, contours, -1, cv::Scalar(0, 255, 0), 2);//在result上绘制轮廓

        float sum=0;
	float z,x,h;

        for(int v=centers[i].y-1;v<=centers[i].y+1;v++)
            for(int u=centers[i].x-1;u<=centers[i].x+1;u++)
            {
                unsigned int d=cv_ptr->image.ptr<unsigned short>(v)[u];
                double k;
            	k=sqrt(1+(u-cx)*(u-cx)/fx/fx+(v-cy)*(v-cy)/fy/fy);
            	z=double(d)/k/depthScale;
                x=(u-cx)*z/fx;
		h=z*sqrt(2)/2-x*sqrt(2)/2+0.05;
 		sum=sum+h;
	    }
        sum=sum/9;
        cout<<sum<<endl;
        }
	}
    } 
    cv_ptr->image;
    return result;//返回result
}
/*回调函数*/
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  static unsigned short maxLast=0;
  cv::Mat gray(60,160,CV_8UC1);  //创建灰度图矩阵
  cv::Mat result(60,160,CV_8UC3);
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg,"16UC1");//读取消息 深度图为 cv_ptr->image  
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to '16UC1'.", msg->encoding.c_str());//错误信息
  }

  disMax=5000;
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
  
  if(nframe==0)
  {   
      gray_b=gray;
      nframe=1;
  }
  cv::Mat gray_frame=gray;
 

  //转换完成
  cv::namedWindow("view");
  cv::applyColorMap( gray, result, cv::COLORMAP_RAINBOW);//伪彩映射
   for(int i=0;i<IMAGE_HEIGHT;i++)                                               //遍历矩阵 转换为灰度图
  {
     for(int j=0;j<IMAGE_WIDTH;j++)
     {  
        if(result.at<cv::Vec3b>(i, j)[0]==255&&result.at<cv::Vec3b>(i, j)[2]==170)
             {
               result.at<cv::Vec3b>(i, j)[0]=0;
	       result.at<cv::Vec3b>(i, j)[1]=0;
	       result.at<cv::Vec3b>(i, j)[2]=0;
             }    
     }
  } 
  result=MoveDetect(gray_b,gray_frame,result);
  cv::resize(result,result,cv::Size(result.cols*4,result.rows*4),0,0,cv::INTER_LINEAR);
  cv::imshow("view", result);                       //显示彩色图片
  cv::waitKey(1);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener"); //初始化节点
  ros::NodeHandle nh;  

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("tof/image", 1, imageCallback);//订阅消息
  ros::spin();
  cv::destroyWindow("view");
  return 0;
}
