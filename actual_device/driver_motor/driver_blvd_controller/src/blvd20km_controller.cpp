#include <vector>
#include <string>
#include <ros/ros.h>
#include <sys/stat.h>
#include "mbrtu/modbusrtu.h"
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>
#include <std_msgs/Header.h>
#include <diagnostic_msgs/KeyValue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <driver_blvd_controller/speed_wheel.h>

#define DEFAULT_BAUDRATE 115200
#define DEFAULT_SERIALPORT "/dev/AGV-BLDV20KM"
#define DEFAULT_ID 1

/* Globle value */
int speed[2];
uint16_t alarm_status[2], feedback_speed[2], warning_status[2];
int check_connect;
struct stat sb;
clock_t begin;
int ID;

ros::Subscriber navigation;


//Process ROS receive from navigation message, send to uController
void speedWheelCallback(const driver_blvd_controller::speed_wheel& robot)
{
	speed[0] = robot.wheel_letf;
  	speed[1] = robot.wheel_right;
} //navigationCallback

int main(int argc, char **argv)
{
	char port[30];    //port name
	int baud;     	  //baud rate 
	char topicPublish[30]; // topic name

	if (argc > 1) {
		if(sscanf(argv[1],"%d", &ID)==1) {
			ROS_INFO("blvd20km_controller.cpp-44-ID = %d", ID);
		}
	else{
			ROS_ERROR("blvd20km_controller.cpp-47-ucontroller index parameter invalid");
			return 1;
		}
	}
	else{
		ID = DEFAULT_ID;
	}

	strcpy(port, DEFAULT_SERIALPORT);
	if (argc > 2)
		strcpy(port, argv[2]);

	baud = DEFAULT_BAUDRATE;
	if (argc > 3) {
		if(sscanf(argv[3],"%d", &baud)!=1) {
		  ROS_ERROR("blvd20km_controller.cpp-62-ucontroller baud rate parameter invalid");
		  return 1;
		}
	}

	/*create ros node*/
	ros::init(argc, argv, "Driver_motor");
	ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
	/* Subscriber */
	ros::Subscriber cmd_vel_to_wheel =  nh->subscribe("cmd_vel_to_wheel", 20,speedWheelCallback); 
	/* Publisher */
	ros::Publisher diagnostic_pub = nh->advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 20);
    
	diagnostic_msgs::DiagnosticArray dir_array;
	diagnostic_msgs::DiagnosticStatus Driver;
	diagnostic_msgs::KeyValue getSpeed;
	diagnostic_msgs::KeyValue alarmRecord;
	diagnostic_msgs::KeyValue warningRecord;
	std_msgs::Header timer;
	ros::Time a_little_after_the_beginning(0, 1000000);
	timer.frame_id = "driverID";

	std::string Char_name = "/Slam Agv/Motor driver/Oriental_BLVD20KM: Driver ";
	Driver.name = Char_name + std::to_string(ID);
	Driver.hardware_id = std::to_string(ID);

	ros::Rate loop_rate(20); 
	while(ros::ok())
	{
		/* onpen comport */
		ROS_INFO("blvd20km_controller.cpp-92-connection initializing (%s) at %d baud", port, baud);
		Mb_open_device(port,baud,1,8,1); /*even , 8 bit, 1 stop_bit*/
		sleep(1);
		if(stat(port, &sb) == 0)
		{
				writeResetAlarm(ID); 
				writeSpeedControlMode(ID,BLVD02KM_SPEED_MODE_USE_DIGITALS);
				writeAcceleration(ID,2);
				writeDeceleration(ID,2);
				writeSpeed(ID,BLVD20KM_SPEED_MIN);
				writeStop(ID);
				clearAlarmRecords(ID); 
				clearWarningRecords(ID);
		}
		while(ros::ok())
		{
			if((double)(clock() -  begin)/( CLOCKS_PER_SEC/1000) >= 2)
			{
				check_connect = stat(port, &sb);
				begin = clock();
			}
			
			if(check_connect != 0){
				for(int i= 0; i<4; i++) ROS_INFO("  ");
				ROS_INFO("blvd20km_controller.cpp-116-Disconnected");
				break;
			} 

			if(speed[ID-1] > 0)
			{
				writeForward(ID);

			}else if(speed[ID-1] < 0)
			{
				writeReverse(ID);

			}else writeStop(ID);

			writeSpeed(ID, abs(speed[ID-1]));
			feedbackSpeed(ID, &feedback_speed[ID-1]);
			readAlarm(ID, &alarm_status[ID-1]);
			readWarning(ID, &warning_status[ID-1]);
			
			Driver.values.clear();
			
			if (check_connect != 0)
			{
				Driver.level = diagnostic_msgs::DiagnosticStatus::ERROR;
				Driver.message = "Driver disconnected. Check connection,plaese!!";
			}else if(alarm_status[ID-1] != 0)
				{
					Driver.level = diagnostic_msgs::DiagnosticStatus::ERROR;
					Driver.message = "Driver is alarm. Reset device, please!!";
				}
			else if (warning_status[ID-1] !=0)
				{
					Driver.level = diagnostic_msgs::DiagnosticStatus::WARN;
					Driver.message = " Warning from driver. Attention!!";
				}
			else
				{
					Driver.level = diagnostic_msgs::DiagnosticStatus::OK;
					Driver.message = "Driver seem to be ok.";
				}

			getSpeed.key = "Feed back speed";
			getSpeed.value = std::to_string((int16_t)feedback_speed[ID-1]);
			Driver.values.push_back(getSpeed);

			warningRecord.key = "Warning record";
			warningRecord.value = std::to_string(warning_status[ID-1]);
			Driver.values.push_back(warningRecord);

			alarmRecord.key = "Alarm record";
			alarmRecord.value = std::to_string(alarm_status[ID-1]);
			Driver.values.push_back(alarmRecord);
			dir_array.status.push_back(Driver);

			timer.stamp.nsec = ros::Time::now().toNSec();
			timer.stamp.sec = ros::Time::now().toSec();
			dir_array.header = timer;
		
			diagnostic_pub.publish(dir_array);
	
			loop_rate.sleep();
			ros::spinOnce();
		}

		Mb_close_device();
		sleep(2);
	}

	return 0;
}

