#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <math.h>
//#include<unistd.h>
#include <iostream>
#include <cstdio>
#include <cstring>
#include "string"
//#include "SerialPort.h"
extern "C" {
#include "serial.h"
#include "RobotControl.h"
}
using namespace std;
#define O_PI 3.1415926      //PI
#define WheelDiameter 0.152 //轮子直径
#define WheelWidth 0.625//轮子间距  
#define L_value 0.273  //轮子距中心距离
#define EncoderTicks 260000.0 //编码器线数
#define UpdateTime 0.1       //下mZ机的发送频率
#define FRAME_LENGTH 13 //数据帧的长度
#define DATA_LENGTH 10  //数据长度
#define Linear_Factor 1.08578 //线性矫正参数
#define Angular_Factor 0.985  //旋转校正参数
const double sqrt33=sqrt(3)/3;
const double EncodertoMileage=WheelDiameter*O_PI/EncoderTicks;
double x = 0, y = 0, theta = 0; //小车的位置和角度
double vx = 0, vy = 0, vth = 0; //小车的线速度和角速度
//SerialPort serialport;     	     //串口
string encoderBuffer; //缓冲区
int bufferSize = 0;   //缓冲区大小
int tmp_count = 0;
int tmp_sum = 0;
RobotInfo robotInfo;
double Dis = 0;
double LeftDis = 0;
double RightDis = 0;
//上一次左右轮里程
double lastv1encoder=0;
double lastv2encoder=0;
double lastv3encoder=0;
ros::Time currentTime;
ros::Time lastTime;
/************************
***************************
	    Twist转换为小车轮子速度信息
在此函数中：
把从cmd_vel主题中的Twist类型信息
转换为小车左右两轮的速度信息 单位:count/s
***************************************************/
void twistToWspd(const geometry_msgs::Twist &msg, float &v1spd, float &v2spd,float&v3spd)
{
    geometry_msgs::Twist twist = msg;
    float vel_x = twist.linear.x;
    float vel_theta = twist.angular.z;
    ROS_INFO("send speed msg vel_x:%.3f vel_theta:%.3f", vel_x,vel_theta);
    vel_theta = vel_theta; //* (1 / Angular_Factor);
    float vel_v1 = 0, vel_v2 = 0,vel_v3=0;
    vel_v1=((float)sqrt(3)*vel_x/2)+L_value*vel_theta;
    vel_v2=L_value*vel_theta-((float)sqrt(3)*vel_x/2);
    vel_v3=L_value*vel_theta;
    //   m/s
    v1spd=vel_v1;
    v2spd=vel_v2;
    v3spd=vel_v3;
}
//下位发过来的码盘数据转换为里程计数据
/***************************************************
	       编码器转换为里程计
在此函数中：
把编码器信息转换为里程计数据
包括坐标、角度、速度、角速度
***************************************************/
/*
void encoderToOdometry(int leftEncoderCounts, int rightEncoderCounts)
{
    int deltaLeft = leftEncoderCounts;
    int deltaRight = rightEncoderCounts;
    double deltaLeftdis = (double)deltaLeft / EncoderTicks * WheelDiameter * O_PI;
    double deltaRightdis = (double)deltaRight / EncoderTicks * WheelDiameter * O_PI;
    double deltaDis = 0.5f * (deltaLeftdis + deltaRightdis);
    double deltaTheta = (double)(deltaRightdis - deltaLeftdis) / WheelWidth;
    ROS_INFO("deltaDis: %f", deltaDis);
    //修正参数
    deltaDis = deltaDis * Linear_Factor;
    deltaTheta = deltaTheta * Angular_Factor;
    double deltaX = deltaDis * (double)cos(theta);
    double deltaY = deltaDis * (double)sin(theta);
    x += deltaX;
    y += deltaY;
    theta += deltaTheta;
    vx = deltaX / UpdateTime;
    vy = deltaY / UpdateTime;
    vth = deltaTheta / UpdateTime;
    if (theta > O_PI)
        theta -= 2 * O_PI;
    else if (theta < -O_PI)
        theta += 2 * O_PI;
}
*/
/***************************************************
	        cmd_vel回调函数
在此回调函数：
1.把Twist信息转换为小车两轮速度
2.通过串口把速度信息发给下位机
***************************************************/
void cmdVelCb(const geometry_msgs::Twist &msg)
{
    float v1spd = 0, v2spd = 0,v3spd=0;
    twistToWspd(msg, v1spd, v2spd,v3spd);
    MoveRobot(v2spd, v3spd,v1spd);
    //ROS_INFO("send speed msg v1:%.3f v2:%.3f,v3:%.3f", v1spd, v2spd,v3spd);
}
int getOdomNew()
{
    //从下位机获取的机器人信息
    robotInfo = GetRobotInfo();
    ROS_INFO("dis: %d, %d, %d",robotInfo.encoder1,robotInfo.encoder2,robotInfo.encoder3);
    //新增的三轮距离
    double deltav1dis = (double)robotInfo.encoder1-lastv1encoder;
    double deltav2dis = (double)robotInfo.encoder2-lastv2encoder;
    double deltav3dis = (double)robotInfo.encoder3-lastv3encoder;
    ROS_INFO("d1,d2,d3:%f,%f,%f",deltav1dis,deltav2dis,deltav3dis);
    lastv1encoder = robotInfo.encoder1;
    lastv2encoder = robotInfo.encoder2;
    lastv3encoder = robotInfo.encoder3;
   /*
   double deltaX = EncodertoMileage*(deltav1dis-deltav2dis)/sqrt(3);
        //double deltaX = deltav1dis*sqrt(3)/2-deltav2dis*sqrt(3)/2;
   double deltaY = EncodertoMileage*(deltav3dis*2-deltav2dis-deltav1dis)/3;
        // double deltaY = deltav1dis/2+deltav2dis/2-deltav3dis;
   ROS_INFO("deltaY:%f",deltaY);
   */
    //转角
    double deltaTheta = EncodertoMileage*(deltav1dis+deltav2dis+deltav3dis) / (3*L_value);
    double deltaX=(-2.0/3.0)*sin(theta)*deltav3dis+(-sqrt33*cos(theta)+1.0/3.0*sin(theta))*deltav2dis+(sqrt33*cos(theta)+1.0/3.0*sin(theta))*deltav1dis;
     deltaX*=EncodertoMileage;
    double deltaY=(2.0/3.0)*cos(theta)*deltav3dis+(-sqrt33*sin(theta)-1.0/3.0*cos(theta))*deltav2dis+(sqrt33*sin(theta)-1.0/3.0*cos(theta))*deltav1dis;
     deltaY*=EncodertoMileage;
    //double deltaTheta = (double)(atan(deltaX/deltaY));
    x += deltaX;
    y += deltaY;
  
    theta += deltaTheta;
    ROS_INFO("x,y,theta:%f,%f,%f",x,y,theta);
    currentTime = ros::Time::now();
    double dt = (currentTime - lastTime).toSec();
    vx = deltaX / dt;
    vy = deltaY / dt;
    vth = deltaTheta / dt;
    if (theta > O_PI)
        theta -= 2 * O_PI;
    else if (theta < -O_PI)
        theta += 2 * O_PI;
    //ROS_INFO("theta:,,,,,,,,,%f",theta);
    //ROS_INFO("vth: %f",vth);
}
int main(int argc, char **argv)
{
    unsigned char data[] = {"serialport test"};
    ros::init(argc, argv, "OdometryNode");
    RobotControlInit();
    ros::NodeHandle nodehandle;
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion odom_quat;
    nav_msgs::Odometry odom;
    ros::Publisher odom_pub = nodehandle.advertise<nav_msgs::Odometry>("odom", 10);
    ros::Subscriber sub = nodehandle.subscribe("cmd_vel", 1, &cmdVelCb);
    ros::Rate rate(5);
    //tf坐标
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    //里程计数据
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    while (ros::ok())
    {
        ros::spinOnce();
        lastTime = ros::Time::now();
        getOdomNew();
        //发布TF消息
        ros::Time current_time = ros::Time::now();
        odom_quat = tf::createQuaternionMsgFromYaw(theta);
        odom_trans.header.stamp = current_time;
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);
        //发布里程消息
        odom.header.stamp = current_time;
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        odom_pub.publish(odom);
        rate.sleep();
    }
    return 0;
}
