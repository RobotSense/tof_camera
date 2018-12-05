#include <ros/ros.h>
#include <tof_camera/SetTime.h>
#include <tof_camera/SetMode.h>
#include <tof_camera/GetInfo.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <math.h>
#include <unistd.h>


using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作

#define IMAGE_WIDTH               160
#define IMAGE_HEIGHT              60 
#define IMAGE_RESOLUTION          (IMAGE_WIDTH*IMAGE_HEIGHT)
#define IMAGE_INFO_NUM            4
#define IMAGE_INFO_END_FLAG_ONE   3
#define IMAGE_INFO_END_FLAG_TWO   2
#define IMAGE_INFO_FRAME_TYPE     1
#define IMAGE_INFO_TEMPERATURE    0    
        
#define COMMAND_START             2
#define COMMAND_STOP              3
#define COMMAND_DEPTH             5
#define COMMAND_GRAYSCALE         6
#define COMMAND_ENABLE_HDR        7
#define COMMAND_DISABLE_HDR       8


io_service iosev;
serial_port sp(iosev);         //定义传输的串口   根据情况修改为"/dev/ttyACM*"
char modeFlag;
char HDRFlag;
unsigned short ITime;
unsigned short HTime;
bool getinfo(tof_camera::GetInfo::Request &req,tof_camera::GetInfo::Response &res)
{

    if (modeFlag==COMMAND_DEPTH)
    { 
       res.mode="depth";
    }
 
    if(modeFlag==COMMAND_GRAYSCALE)
    {       
       res.mode="gray";
    }
    if (HDRFlag==COMMAND_DISABLE_HDR)
    { 
       res.HDR="disable";
       res.mode="depth";
    }
    else
    { 
       res.HDR="enable";
       res.mode="depth";
       res.HDRTime=HTime;
    }
    res.IntegrationTime=ITime;
    return true;
}
bool setmode(tof_camera::SetMode::Request &req,tof_camera::SetMode::Response &res)
{
    char commandHex[3] = {0x23,0x23,0x00}; // 0x23,0x23 is command flag
    if (req.mode=="depth")
    {
    	commandHex[2]=COMMAND_DEPTH;
    	write(sp, buffer(commandHex, 3));          //设置为深度图格式
        modeFlag=COMMAND_DEPTH;
        ROS_INFO("mode:depth"); 
    }
    else if(req.mode=="gray")
    {
        if (HDRFlag==COMMAND_ENABLE_HDR)
        { 
           commandHex[2]=COMMAND_DISABLE_HDR;
    	   write(sp, buffer(commandHex, 3));       
           HDRFlag=COMMAND_DISABLE_HDR;
        }
        sleep(1);//延迟1秒
        commandHex[2]=COMMAND_GRAYSCALE;
    	write(sp, buffer(commandHex, 3));          //设置为灰度图格式
        modeFlag=COMMAND_GRAYSCALE;
        ROS_INFO("mode:gray");  
    }
    else if(req.mode=="enableHDR")
    {
        commandHex[2]=COMMAND_ENABLE_HDR;
    	write(sp, buffer(commandHex, 3));          //设置为HDR图格式
        HDRFlag=COMMAND_ENABLE_HDR;
        ROS_INFO("enable HDR");
    }
    else if(req.mode=="disableHDR")
    {
        commandHex[2]=COMMAND_DISABLE_HDR;
    	write(sp, buffer(commandHex, 3));          //设置为HDR图格式
        HDRFlag=COMMAND_DISABLE_HDR;
        ROS_INFO("disable HDR");
    }
    else
    {
     res.flag="error";
     return true;
    }
    
    res.flag="ok";
    return true;
}
bool settime(tof_camera::SetTime::Request &req,tof_camera::SetTime::Response &res)
{
    char timeHex[4];
    if(HDRFlag==COMMAND_DISABLE_HDR)
    {
    timeHex[0] = 0x24; // flag
    timeHex[1] = 0x24;
    timeHex[2] = req.time/256;
    timeHex[3] = req.time%256;
    write(sp, buffer(timeHex, 4));
    ITime=req.time;
    ROS_INFO("ITime:%d",ITime);
    }
    else
    {
    timeHex[0] = 0x27; // flag
    timeHex[1] = 0x27;
    timeHex[2] = req.time/256;
    timeHex[3] = req.time%256;
    write(sp, buffer(timeHex, 4));
    HTime=req.time;
    ROS_INFO("HTime:%d",HTime);
    }
    res.flag="ok";
    return true;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tof_camera");                           //初始化节点
    ros::NodeHandle nh;

    ros::ServiceServer service1 = nh.advertiseService("tof/settime", &settime);
    ros::ServiceServer service2 = nh.advertiseService("tof/setmode", &setmode);
    ros::ServiceServer service3 = nh.advertiseService("tof/getinfo", &getinfo);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("tof/image", 1);      //发布话题

    ros::Rate loop_rate(10);


    sp.open("/dev/ttyACM0");
    sp.set_option(serial_port::baud_rate(9600));
    sp.set_option(serial_port::flow_control());
    sp.set_option(serial_port::parity());
    sp.set_option(serial_port::stop_bits());
    sp.set_option(serial_port::character_size(8));

    char commandHex[3] = {0x23,0x23,0x00}; // 0x23,0x23 is command flag

    commandHex[2]=COMMAND_DEPTH;
    write(sp, buffer(commandHex, 3));          //设置为深度图格式
    modeFlag=COMMAND_DEPTH;
    ROS_INFO("mode:depth"); 
    sleep(1);//延迟1秒 

    commandHex[2]=COMMAND_START;
    write(sp, buffer(commandHex, 3));              
    ROS_INFO("start");                          //发送开始命令

    cv::Mat mt(60,160,CV_16UC1);               
    HDRFlag=COMMAND_DISABLE_HDR;
    ITime=300;
    HTime=2000;
    ROS_INFO("time:%d",ITime); 
    try
    {       
 
        while (ros::ok())
        {
            unsigned short buf[IMAGE_RESOLUTION+IMAGE_INFO_NUM];                      //图像缓存
            unsigned short flag[2];                                                   //结束标志缓存
            read (sp,buffer(flag));                                        //读取串口
            if(flag[0]==0x232A && flag[1] == 0x2A23 )                      //找到结束标志
            {
                read (sp,buffer(buf));
                if(buf[IMAGE_RESOLUTION+IMAGE_INFO_END_FLAG_TWO]==0x232A && buf[IMAGE_RESOLUTION+IMAGE_INFO_END_FLAG_ONE] == 0x2A23)    //确认是完整图像
                {
                    for(int i=0;i<IMAGE_HEIGHT;i++)
                    {
                        for(int j=0;j<IMAGE_WIDTH;j++)
                        {   
                            if(buf[i*IMAGE_WIDTH+j]>=30000)
                            	buf[i*IMAGE_WIDTH+j]=0;
                            if (modeFlag==COMMAND_DEPTH||HDRFlag==COMMAND_ENABLE_HDR)
                            { 
                                mt.at<unsigned short>(i,j)=(buf[i*IMAGE_WIDTH+j])/2;
                                
                            }
                            else if(modeFlag==COMMAND_GRAYSCALE)
                            {
                                mt.at<unsigned short>(i,j)=buf[i*IMAGE_WIDTH+j];
                            }
                        }
                    }                                                            //赋值
                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "16UC1", mt).toImageMsg();//转换为可发送的图片
                    pub.publish(msg);                                          //发布 
                    cv::waitKey(1);
                    
                    loop_rate.sleep();
                }
            }
            iosev.run_one();
            ros::spinOnce();
        }
    }
    catch (exception& err)
    {   
        commandHex[2]=COMMAND_STOP;
        write(sp, buffer(commandHex, 3));              //发送停止命令
        ROS_INFO("stop er");
        return 0; 
    }
    commandHex[2]=COMMAND_STOP;
    write(sp, buffer(commandHex, 3));              //发送停止命令
    ROS_INFO("stop"); 

    return 0;
}


