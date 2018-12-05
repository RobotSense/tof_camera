
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#define IMAGE_WIDTH    160
#define IMAGE_HEIGHT    60   //图像尺度
double cx=87.1415;
double cy=19.5734;
double fx=113.0965;
double fy=111.6327;
double depthScale=1000.0;
unsigned short disMax,disMin;
cv_bridge::CvImagePtr cv_ptr;  //申明一个CvImagePtr指针
ros::Publisher pub;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


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
    PointCloud::Ptr pointCloud(new PointCloud);
    PointCloud::Ptr filter(new PointCloud);
    cv::Mat depth=cv::imread("../pcl/depth.png",-1);
    for(int v=0;v<IMAGE_HEIGHT;v++)
        for(int u=10;u<IMAGE_WIDTH;u++)
        {
            unsigned int d=cv_ptr->image.ptr<unsigned short>(v)[u];
            if(d==0) continue;
            PointT p;
            double k;
            k=sqrt(1+(u-cx)*(u-cx)/fx/fx+(v-cy)*(v-cy)/fy/fy);
            p.z=double(d)/k/depthScale;
            p.x=(u-cx)*p.z/fx;
            p.y=(v-cy)*p.z/fy;
            pointCloud->points.push_back(p);
        }
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (pointCloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*filter);
    filter->is_dense=false;
    filter->header.frame_id = "tof/pd";
    pub.publish (filter);
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener"); //初始化节点
  ros::NodeHandle nh;  

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("tof/image", 1, imageCallback);//订阅消息
  pub = nh.advertise<PointCloud> ("pointcloud2", 1);
  ros::spin();
  return 0;
}
