#ifndef USART_H
#define USART_H
//串口相关的头文件
#include<stdio.h>      /*标准输入输出定义*/
#include<stdlib.h>     /*标准函数库定义*/
#include<unistd.h>     /*Unix 标准函数定义*/
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>      /*文件控制定义*/
#include<termios.h>    /*PPSIX 终端控制定义*/
#include<errno.h>      /*错误号定义*/
#include<string.h>
#include<string>

using namespace std;

#define UART_FALSE  -1
#define UART_TRUE   0

#define GNR_COM  1
#define USB_COM  2
#define COM_TYPE USB_COM


class SerialPort
{
public:
   int fd;              								     //文件描述符  
    int isnormal;
   SerialPort(); 	      		           					     //构造函数
   ~SerialPort(void );			           					     //析构函数
   int Open(int id,int speed);                	   					     //打开串口
   void Close(void );                    	   					     //关闭串口
   int Set(int speed,int flow_ctrl,int databits,int stopbits,char parity);                   //串口设置参数
   int readBuffer(char *rcv_buf,int data_len);                                               //串口接受
   int writeBuffer(unsigned char *send_buf,int data_len);                                             //串口发送
   int sendWheelSpd(float lspd,float rspd);                                      	     //发送轮子速度 
protected:
private:
    	 								     //指示串口是否正常 	
};
unsigned char XorCode(unsigned char *buf,int len);
unsigned char XorCode(std::string buf,int len);

#endif

