#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <linefolowing/MLS_Measurement.h>
#include <linefolowing/speed_wheel.h> 
#include <agv_msgs/agv_action.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <pthread.h>

#define DEFAULT_DIRECTION "forward" 

#define LCP_NUM_BIT   	       0b00000111
#define MAKER_NUM_BIT 	       0b11111000
#define LINE_IS_GOOD_BIT       0b00000001
#define TRACK_LEVEL_BIT        0b00001110
#define SENSOR_FLIPPED_BIT     0b00010000
#define POLARITY_BIT 	       0b00100000
#define READING_CODE_BIT       0b01000000
#define EVENT_FLAG_BIT         0b10000000

struct lcp
{
	uint8_t lcp_nummber;
	uint8_t marker;
};  lcp lcp;

struct status
{
	bool line_good;
	bool sensor_fipped;
	bool polarity;
	bool reading_code;
	uint8_t track_level;
};  status status;

struct mlse
{
	uint8_t status;
	uint8_t lcp;
	uint8_t error_register;	
	float position[3];
};  mlse mlse;

/* Include funtion for Sick line */
void Gacceleration(float& present_speed, const float step);
void Deceleration(float& present_speed, const float step);

/* CallBack Subcriber */
#define Pi 3.1415926535
#define rad_rpm 9.5492965964254

//Global data
float present_speed;  		 			// percent speed %
float present_speed_setting; 	// percent speed setting
float speed_setting; 					 // speed max when percent speed = 100%  (m/s)
float L; 											   // khoang cach 2 banh
float Lm; 											// Khoang cach  tu tam truc banh den tam do line 
float R; 											  // wheel radius (in meters per radian)
float v_max_wheel; 						// speed maximum of moter behind gear
float v_min_wheel; 						 // speed maximum of moter behind gear
float Lt;									 		 // Dường kính vòng cua
float V;  						  					 // forward velocity (ie meters per second)
float K;  											 // He so chuyen banh răng
int W_l, W_r; 				   	     		   // speed befor gear
int8_t direct ;

ros::Publisher speedwheel;
linefolowing::speed_wheel robot;

uint8_t action_ = 0;
ros::Publisher charger_status_pub;

void chargerActionCallback(const agv_msgs::agv_action& msg)
{
	ROS_INFO("linefolowersick.cpp-80-actionCallback()");
	action_ = msg.action;
	// linefolowing::agv_action status = ActionState(action_);
    switch(action_){
      case 6:  // CHARGING IN
	  	direct = -1;
        break;
      case 7:  // CHARGING OUT
	  	direct = 1;
        break;
      case 8:  // LIFT IN
	  	direct = -1;
        break;
      case 9:  //LIFT OUT
	  	direct = 1;
        break;
      default:
      {}
    }
}//teleop_keyCallback 

void mlsCallback(const linefolowing::MLS_Measurement& msg)
{
	mlse.status = msg.status;
	mlse.lcp = msg.lcp;
	/* 
	* #LCP
	* The following assignment applies (see "Outputof line center points", page 40):
	* 0 => No track found
	* 2 => One track found
	* 3 => Two tracks found: Left diverter
	* 6 => Two tracks found: Left diverter
	* 7 => Three tracks found or 90 °C intersection
	*/
	lcp.lcp_nummber = mlse.lcp & LCP_NUM_BIT;
	/* 
	* Marker 
	* Bit 0 is the introductory character bit
	* Bit 1...4 present code 1...15
	*/
	lcp.marker = mlse.lcp & MAKER_NUM_BIT;
	/* 
	* True is  Sufficiently strong track detected
	* Fasle is  No track or track too weak
	*/
	status.line_good = mlse.status & LINE_IS_GOOD_BIT;
	/*
	* Display of magnetic field strength in accor‐dance
	*/
	status.track_level = mlse.status & TRACK_LEVEL_BIT;
	/*
	* Indicates whether or not the measuring rangehas been inverted
	* False => Negative positions on cable outlet side
	* True => Positive positions on cable outlet side
	*/
	status.sensor_fipped = mlse.status & SENSOR_FLIPPED_BIT;
	/*
	* Indicates whether the upper surface of themagnetic tape is magnetized to the north orsouth pole
	* False => North pole
	* True => South pole
	*/
	status.polarity = mlse.status & POLARITY_BIT;
	/* 
	* False => No code present to read
	* True => Sensor is reading code
	*/
	status.reading_code = mlse.status & READING_CODE_BIT;
	/* Error register */
	mlse.error_register = msg.error;
	
	float angle_error; 	// error angle 
	float W;  					// angular velocity (ie radians per second)
	float v_r; 					// clockwise angular velocity of right wheel (ie radians per second)
	float v_l;				 	// counter-clockwise angular velocity of left wheel (ie radians per second)

	if(status.line_good == true && int8_t(speed_setting/abs(speed_setting)) ==  direct)
	{
		if(msg.position[2] > 0.00)
		{	
			present_speed_setting = 0;
			direct = 0;

			robot.wheel_letf = 0;
			robot.wheel_right = 0;
			speedwheel.publish(robot);

			agv_msgs::agv_action charging_action;
			charging_action.action = action_;
			charging_action.status = 3;
			charger_status_pub.publish(charging_action);
			ROS_INFO("linefolwersick.cpp-170: Publish result");	
			// ROS_INFO("linefolwersick.cpp-168: Vung 3, stop  dir = %d", direct);	
			for(int i = 0; i<0; i++) ROS_INFO(" ");
		}
		else if(msg.position[2] <= 0.00)
		{
			if(msg.position[0] == 0.00)
			{
				present_speed_setting = 1;
				ROS_INFO("linefolwersick.cpp-179: Vung 1, run, dir = %d", direct);	
				for(int i = 0; i<0; i++) ROS_INFO(" ");
			}
			else if(msg.position[0] < 0.00)
			{
				present_speed_setting = 0.32;
				ROS_INFO("linefolwersick.cpp-185: Vung 0, giam toc, dir = %d", direct);	 	
				for(int i = 0; i<0; i++) ROS_INFO(" ");
			}
		}
	}else present_speed_setting = 0;
	
	V = abs(speed_setting*present_speed); // V cai dat 
	angle_error = atan(msg.position[1]/Lm);
	v_l = ((1 - (L*angle_error)/(2*Lt)) * (V/R)) * rad_rpm;
	v_r = ((1 + (L*angle_error)/(2*Lt)) * (V/R)) * rad_rpm;
	
	W_l = v_l*K;
	if(W_l > v_max_wheel) W_l = v_max_wheel;
	if(W_l < v_min_wheel) W_l = 0;

	W_r = v_r*K;
	if(W_r > v_max_wheel) W_r = v_max_wheel;
	if(W_r < v_min_wheel) W_r = 0;
	
} //echo_line_previousCallback

/* MAIN */
int main(int argc, char **argv)
{
	char direction[20]; 
	char str[20];
    char param[70];
	int ucIndex; 					//ucontroller index number
	char topicSubscribe[20];
	char topicPublish[20];

    /***Create Node */
	ros::init(argc, argv, "MLS_Line_Sick_Forward");
	ros::NodeHandle n;	
	ros::Rate loop_rate(20);
	linefolowing::speed_wheel robot;

	// system("rosservice call /SickLocSetPose {\"posex: 2000, posey: 1000, yaw: 30000, uncertainty: 1000\"}");
 	// ROS_INFO("linefolowersick.cpp-222-sudo ip link set can0 type can");	
	// system("sudo ip link set can0 type can");
 	// ROS_INFO("linefolowersick.cpp-225-sudo ip link set can0 up type can bitrate 125000");	
	// system("sudo ip link set can0 up type can bitrate 125000");

	//Open and initialize the serial port to the uController
  	if (argc > 1) 
  	{
	    if(sscanf(argv[1],"%d", &ucIndex)==1) 
	    {
			sprintf(topicSubscribe, "mls%d",ucIndex);
			sprintf(topicPublish, "speedwheel%d",ucIndex);
			
	    }else 
	    	{
			    ROS_ERROR("Not connect mls%d !!",ucIndex);
			    return 1;
	  		}
	}else 
		{
			strcpy(topicSubscribe, "mls0");
			strcpy(topicPublish, "speedwheel0");
		}

  	sprintf(param,"/consept_mls%d/present_speed_setting",ucIndex);
  	n.getParam(param,present_speed_setting);

  	sprintf(param,"/consept_mls%d/L",ucIndex);
  	n.getParam(param,L);

  	sprintf(param,"/consept_mls%d/Lm",ucIndex);
  	n.getParam(param,Lm);

	sprintf(param,"/consept_mls%d/R",ucIndex);
  	n.getParam(param,R);

  	sprintf(param,"/consept_mls%d/Lt",ucIndex);
  	n.getParam(param,Lt);

  	sprintf(param,"/consept_mls%d/v_max_wheel",ucIndex);
  	n.getParam(param,v_max_wheel);

  	sprintf(param,"/consept_mls%d/v_min_wheel",ucIndex);
  	n.getParam(param,v_min_wheel);

	sprintf(param,"/consept_mls%d/Speed",ucIndex);
    n.getParam(param,speed_setting);

	sprintf(param,"/consept_mls%d/K",ucIndex);
  	n.getParam(param,K);

 	ROS_INFO("linefolowersick.cpp-274-MLS line sick %d starting",ucIndex);	

	/* Publisher */
	speedwheel = n.advertise<linefolowing::speed_wheel>("cmd_vel_to_wheel", 20);
	 
	/* Subscriber position line */
	ros::Subscriber action = n.subscribe("/charging_action", 20,chargerActionCallback);	 	
	ros::Subscriber mls = n.subscribe(topicSubscribe, 20,mlsCallback);

  	charger_status_pub = n.advertise<agv_msgs::agv_action>("charging_status", 1000);
	
	/* clock */
	clock_t begin_time = clock();

	while (ros::ok())
	{
			/* This is a message object. You stuff it with data, and then publish it. */
			//if(int8_t(speed_setting/abs(speed_setting)) == dir && direct != 0 && direct == dir )
			if(int8_t(speed_setting/abs(speed_setting)) ==  direct)
			{
				if(float(clock()-begin_time)/CLOCKS_PER_SEC*1000  >= 1)		 // 1 ms
				{
					if(present_speed != present_speed_setting)
					{
						if(present_speed_setting > present_speed) {
							Gacceleration(present_speed, 0.02);
						}else if(present_speed_setting < present_speed) {
							Deceleration(present_speed, 0.02);
						}
					} 
					begin_time = clock();
				}
				if(status.line_good == true)
				{
					robot.wheel_letf  = W_l*direct;
					robot.wheel_right = W_r*direct*(-1);
					if(status.track_level <= 3 && status.track_level > 0) ROS_WARN("linefolowersick.cpp-310-From MLS%d: track too weak!!",ucIndex);
				}else 
					{
						ROS_ERROR("linefolowersick.cpp-313-From MLS%d: no track!!", ucIndex);
						robot.wheel_letf = 0;
						robot.wheel_right = 0;
					}
				ROS_INFO("linefolowersick.cpp-317-Banh trai = %d Banh phai = %d",robot.wheel_letf, robot.wheel_right);
				speedwheel.publish(robot);
			}
			loop_rate.sleep();
			ros::spinOnce();
	}	
	return 0;
}

void Gacceleration(float& present_speed,const float step)
{
	present_speed += step;
}

void Deceleration(float& present_speed, const float step)
{
	present_speed -= step;
}