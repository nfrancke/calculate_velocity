/*****************************************************************************************************************************************
Function: 
This program responds on the receiving of a Float32MultiArray message on topic written in define variable "TOPIC_NAME.
The only data that this program will read is data[4], data[6] and data[7]. This is because serial port 5,7,8 will be written.
When the program can't open a serial port the program will be shut down. 
For debugging it is recommend to set the "log level" of ROS to -> DEBUG. normally it is setted to INFO.

Pre:
A "ros param" that descibes the factor between the incomming array (RPM) and the sending velocity to the motordrivers.
two "ros params" that describe the minimum and the maximum of the velocity that will be send to the motordrivers. NOTE: this is no RPM!!!

Post:
rs422 message on serial bus 5, 6 and 7 that gives the speed in pulses per time sample. The exact message can be found in the outputbuffer.

coordinate system:
shooting system is on the x-axe

		^x
		|
		|
	   zX----->y

Writer 		: Niek Francke
date 		: 17-11-2015
********************************************************************************************************************************************/

#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include "ros/ros.h"
#include <math.h>

//-----settings
#define PUBLISH_TOPIC_NAME 			"mcWheelVelocityMps"
#define SUBSCRIBE_TOPIC_NAME		"motorspeed_set"
#define TOPIC_BUFFER_SIZE			1
#define ANGLE_1						0 	//degrees
#define ANGLE_2						120 //degrees
#define ANGLE_3						240 //degrees
#define THETA						30	//degrees
#define RADIUS_OMNI_WHEEL			0.1016 //meter
#define RADIUS_DRIVING_SYSTEM		0.22 //meter

using namespace std;

/*****************************************************************************************************************************************
Start defining class PublischAndSubscribe
********************************************************************************************************************************************/
class PublishAndSubscribe
{
public:
	
	///////////////////////////////////////////////////////////////////////////////////////////
	//Function: create class and read params
	//pre: 	-
	//post: iConvertFactor will be 0 if there is no param readed.
	///////////////////////////////////////////////////////////////////////////////////////////
	PublishAndSubscribe(ros::NodeHandle nh)
	{
		sub = nh.subscribe(SUBSCRIBE_TOPIC_NAME,TOPIC_BUFFER_SIZE, &PublishAndSubscribe::twistMessageReceived, this);
		pub = nh.advertise<std_msgs::Float32MultiArray>(PUBLISH_TOPIC_NAME,1);
	}
	
	/////////////////////////////////////////////////////////////////////////////
	//function: this function responds on a float32MultiArray message.
	//pre: values for array places 4,6,7, because the serial ports are 5,7 and 8
	//post: -
	/////////////////////////////////////////////////////////////////////////////
	void twistMessageReceived(const geometry_msgs::Twist::ConstPtr& msg)
	{
		float fX = msg->linear.x;
		float fY = msg->linear.y;
		float fOmega = msg->angular.z;

		ROS_DEBUG("x = %f", fX);
		ROS_DEBUG("y = %f", fY);
		ROS_DEBUG("omega = %f", fOmega);

		float fAngle1 = ((float)ANGLE_1/180)*M_PI;
		float fAngle2 = ((float)ANGLE_2/180)*M_PI;
		float fAngle3 = ((float)ANGLE_3/180)*M_PI;
		float fTheta  = ((float)THETA/180)*M_PI;

		ROS_DEBUG("angle1 is %f  pi radians and %i degrees" ,(fAngle1/M_PI), ANGLE_1);
		ROS_DEBUG("angle2 is %f pi radians and %i degrees" , (fAngle2/M_PI), ANGLE_2);
		ROS_DEBUG("angle3 is %f pi radians and %i degrees" , (fAngle3/M_PI), ANGLE_3);
		ROS_DEBUG("theta is %f pi radians and %i degrees" , (fTheta/M_PI), THETA);

		float fSpeedWheel[10];

		//uitleg functie
		//fSpeedWheel[7] = (-sin(fTheta)*cos(fTheta)*fY + (cos(fTheta) * cos(fTheta))*fX + RADIUS_DRIVING_SYSTEM*(-fOmega))/RADIUS_OMNI_WHEEL; 
		//fSpeedWheel[5] = (-sin(fTheta + fAngle2)*cos(fTheta)*fY + (cos(fTheta + fAngle2) * cos(fTheta))*fX + RADIUS_DRIVING_SYSTEM*(-fOmega))/RADIUS_OMNI_WHEEL;
		//fSpeedWheel[8] = (-sin(fTheta + fAngle3)*cos(fTheta)*fY + (cos(fTheta + fAngle3) * cos(fTheta))*fX + RADIUS_DRIVING_SYSTEM*(-fOmega))/RADIUS_OMNI_WHEEL;

		fSpeedWheel[7] = (-sin(fTheta + fAngle1)*fY + (cos(fTheta + fAngle1)*fX + RADIUS_DRIVING_SYSTEM*(-fOmega))/RADIUS_OMNI_WHEEL; 
		fSpeedWheel[5] = (-sin(fTheta + fAngle2)*fY + (cos(fTheta + fAngle2)*fX + RADIUS_DRIVING_SYSTEM*(-fOmega))/RADIUS_OMNI_WHEEL;
		fSpeedWheel[8] = (-sin(fTheta + fAngle3)*fY + (cos(fTheta + fAngle3)*fX + RADIUS_DRIVING_SYSTEM*(-fOmega))/RADIUS_OMNI_WHEEL;

		ROS_DEBUG("speed wheel 1 = %f", fSpeedWheel[7]);
		ROS_DEBUG("speed wheel 2 = %f", fSpeedWheel[5]);
		ROS_DEBUG("speed wheel 3 = %f", fSpeedWheel[8]);

		std_msgs::Float32MultiArray msg_out;

		msg_out.data.clear();

      	msg_out.data.push_back(0);
	    msg_out.data.push_back(0);
	    msg_out.data.push_back(0);
	    msg_out.data.push_back(0);
	    msg_out.data.push_back(fSpeedWheel[5]);
	    msg_out.data.push_back(0);
	    msg_out.data.push_back(fSpeedWheel[7]);
	    msg_out.data.push_back(fSpeedWheel[8]);
	    msg_out.data.push_back(0);
	    msg_out.data.push_back(0);

	    pub.publish(msg_out);

	}

private:
	ros::Subscriber sub;	//define ros subscriber
	ros::Publisher pub;
};
/*****************************************************************************************************************************************
end of defining class Subscribe
********************************************************************************************************************************************/

/*****************************************************************************************************************************************
Start of main
********************************************************************************************************************************************/
int main(int argc, char **argv  )
{
	//initialize ROS
	ros::init(argc, argv, "calculate_velocity_node");
	
	//create nodehandle
	ros::NodeHandle nh;	

	//create class
	PublishAndSubscribe PandSobject(nh);

	//wait until a Float32MulitArray is received and run the callback function
	ros::spin();

	return 0;
}

/*****************************************************************************************************************************************
End of main
********************************************************************************************************************************************/
/*****************************************************************************************************************************************
Functions
********************************************************************************************************************************************/
/*****************************************************************************************************************************************
End of functions
********************************************************************************************************************************************/
