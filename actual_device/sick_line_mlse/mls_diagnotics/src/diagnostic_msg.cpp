#include <string>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <diagnostic_msgs/KeyValue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <mls_diagnotics/MLS_Measurement.h>
// #include "mls_diagnotics/mls_measurement.h"

#define LCP_NUM_BIT   	    0b00000111
#define MAKER_NUM_BIT 	   	0b11111000
#define LINE_IS_GOOD_BIT  	0b00000001
#define TRACK_LEVEL_BIT    	0b00001110
#define SENSOR_FLIPPED_BIT  0b00010000
#define POLARITY_BIT 	      0b00100000
#define READING_CODE_BIT    0b01000000
#define EVENT_FLAG_BIT      0b10000000

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

void mlsCallback(const mls_diagnotics::MLS_Measurement& msg)
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
	* 7 => Three tracks found or 90 °C intersection2
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

	/* position line */
	for(int i = 0; i < 3; i++) mlse.position[i] = msg.position[i];

} //echo_line_previousCallback

int main(int argc, char **argv)
{
	int ucIndex;
	char topicSubscribe[20];

	if (argc > 1) 
  	{
	    if(sscanf(argv[1],"%d", &ucIndex)==1) 
	    {
			sprintf(topicSubscribe, "mls%d",ucIndex);
			
	    }else 
	    	{
			    ROS_ERROR("Not connect mls%d !!",ucIndex);
			    return 1;
	  		}
	}else 
		{
			strcpy(topicSubscribe, "mls0");
		}

	/*create ros node*/
	ros::init(argc, argv, "mls_diagnotics");
	ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
	/* Subscriber */
	ros::Subscriber mls =  nh->subscribe(topicSubscribe, 20,mlsCallback); 
	/* Publisher */
    ros::Publisher diagnostic_pub = nh->advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 20);
	
    diagnostic_msgs::DiagnosticArray dir_array;
	diagnostic_msgs::DiagnosticStatus device;

	diagnostic_msgs::KeyValue LCP;
	diagnostic_msgs::KeyValue marker;
	diagnostic_msgs::KeyValue lineGood;
	diagnostic_msgs::KeyValue trackLevel;
	diagnostic_msgs::KeyValue sensorFlipped;
	diagnostic_msgs::KeyValue polarity;
	diagnostic_msgs::KeyValue readingCode;
	diagnostic_msgs::KeyValue lcp1;
  diagnostic_msgs::KeyValue lcp2;
  diagnostic_msgs::KeyValue lcp3;

	std_msgs::Header timer;
	ros::Time a_little_after_the_beginning(0, 1000000);
	timer.frame_id = "mlsID";


	std::string Char_name = "/LMSE_";
	device.name = Char_name + std::to_string(ucIndex);
	std::string Char_id = "0x0";
	device.hardware_id = Char_id + std::to_string(ucIndex);


    ros::Rate loop_rate(20); 
    while(ros::ok())
	{
		device.values.clear();
		if(!mlse.error_register)
		{
			device.level = diagnostic_msgs::DiagnosticStatus::OK;
			device.message = "Device seem to be ok!!";
		}
		else 
		{
			device.level = diagnostic_msgs::DiagnosticStatus::ERROR;
			device.message = "No error output!!";
		}

		LCP.key = "number line center points";
		switch(lcp.lcp_nummber){	
			case 2: {
				LCP.value = "One track found!!";
				break;
			}	
			case 3: {
				LCP.value = "Two tracks found: Left diverter!!";
				break;
			}
			case 6: {
				LCP.value = "Two tracks found: Left diverter!!";
				break;
			}
			case 7: {
				LCP.value = "Three tracks found or 90 °C!!";
				break;
			}		
			default:{
		       LCP.value = "No track found!!";
		    }
		}
		device.values.push_back(LCP);

		lineGood.key = "Line status";
		if(status.line_good) lineGood.value = "Sufficiently strong track detected!!";
			else lineGood.value = "No track or track too weak!!";
		device.values.push_back(lineGood);

		trackLevel.key = "Line level";
		trackLevel.value = std::to_string(status.track_level);
		device.values.push_back(trackLevel);

		sensorFlipped.key = "Sensor flipped";
		if(status.sensor_fipped) sensorFlipped.value = "Positive positions on cable outlet side!!";
			else sensorFlipped.value = "Negative positions on cable outlet side!!";
		device.values.push_back(sensorFlipped);
		
		polarity.key = "Polarity";
		if(status.polarity) polarity.value = "North pole!!";
			else polarity.value = "South pole!!";
		device.values.push_back(polarity);

		readingCode.key = "Reading code";
		if(status.reading_code) readingCode.value = " Sensor is reading code!!";
			else readingCode.value = " No code present to read!!";
		device.values.push_back(readingCode);

		lcp1.key = "Lcp1";
		lcp1.value = std::to_string(mlse.position[0]);
		device.values.push_back(lcp1);

		lcp2.key = "Lcp2";
		lcp2.value = std::to_string(mlse.position[1]);
		device.values.push_back(lcp2);

		lcp3.key = "Lcp3";
		lcp3.value = std::to_string(mlse.position[2]);
		device.values.push_back(lcp3);

		timer.stamp.nsec = ros::Time::now().toNSec();
		timer.stamp.sec = ros::Time::now().toSec();
		dir_array.header = timer;

		dir_array.status.push_back(device);
		diagnostic_pub.publish(dir_array);

		loop_rate.sleep(); 
		ros::spinOnce();
	}

	return 0;
}