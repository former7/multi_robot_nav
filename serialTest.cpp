#include<stdio.h>
#include"ros/ros.h"
#include"unistd.h"
extern "C"{
#include "serial.h"
#include "RobotControl.h"

}
#include<iostream>
using namespace std;
int main(int argc,char ** argv)
{
    ros::init(argc,argv,"serialTest");
    RobotControlInit();
    int input=0;
    cout<<"1 forward 2 back 3 left 4 right 5 stop"<<endl;
/*
while(true){
input++;
MoveRobot(0,0);
if(input>=10){
cout<<"go"<<endl;
MoveRobot(0.2,0.2);
//sleep(1);
}
sleep(1);
//usleep(100000);
}    
*/

while(true)
    {
 
      	cin>>input;
	switch(input)
	{
	case 1:
		{
			cout<<"forward"<<endl;
			MoveRobot(0.3,-0.3,0);
			break;
		}
	case 2:
		{
			MoveRobot(-0.1,-0.1,0);
			break;
		}
	case 3:
		{
			MoveRobot(0.05,-0.05,0);
			break;
		}
	case 4:
		{
			MoveRobot(-0.1,0.1,0);
			break;
		}
	default:
		{
			MoveRobot(0,0,0);
			break;
		}
	}	 
    }
return 0;
		
}
