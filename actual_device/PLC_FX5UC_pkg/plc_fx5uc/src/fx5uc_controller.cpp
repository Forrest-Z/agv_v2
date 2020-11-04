#include "mbtcp/modbus.h"
#include "fx5u_hardware/fx5u_hardware.h"
#include <diagnostic_msgs/KeyValue.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <ros/ros.h>
#include "agv_msgs/agv_action.h"

using namespace std;

#define NO_CL  0
#define RED    1
#define GREEN  2
#define YELLOW 3
#define BULE   4
#define PINK   5
#define OCEAN  6
#define WHITE  7

bool isLiftUp = false;
bool isLiftDown = true;
uint8_t action_ = 0;

void liftActionCallback(const agv_msgs::agv_action& msg)
{
	ROS_INFO("fx5uc_controller.cpp-21-liftActionCallback()");
	action_ = msg.action;
    switch(action_){
      case 9:
        isLiftUp = true;
        isLiftDown = false;
        ROS_INFO("fx5uc_controller.cpp-34- isLiftUp = true");
        break;
      case 10:
        isLiftUp = false;
        isLiftDown = true;
        ROS_INFO("fx5uc_controller.cpp-34- isLiftUp = false");
        break;
      default:
      {}
    }
}

int main(int argc, char **argv)
{
  /* create a modbus object */
  modbus *fx5uc = new modbus("192.168.1.51", 502);
  /* set slave id */
  fx5uc->modbus_set_slave_id(1);
  /* connect with the server */
  fx5uc->modbus_connect();  
  /* create hardware */
  FX5U_series device;
  /* create ros ndoe */
  ros::init(argc, argv, "PLC_control");
  ros::NodeHandle nh;
  ros::Rate loop_rate(20);
  /* Publisher */
  ros::Publisher cmd_PLC;
  cmd_PLC = nh.advertise<diagnostic_msgs::DiagnosticStatus>("PLC_infomation", 20);

  diagnostic_msgs::DiagnosticArray dir_array;
  diagnostic_msgs::DiagnosticStatus PLC;
  diagnostic_msgs::KeyValue LED;
  diagnostic_msgs::KeyValue Dock;
  diagnostic_msgs::KeyValue Xilanh;

  PLC.name = "PLC-Fx5UC";
	PLC.hardware_id = "192.168.1.51:502";

  bool bitM_echo[300];
  bool bitM_pub[100];
    
	ros::Subscriber action = nh.subscribe("lift_action", 20, liftActionCallback);
  ros::Publisher lift_status_pub = nh.advertise<agv_msgs::agv_action>("lift_status", 1000);

    while(ros::ok())
    {  
        fx5uc->modbus_read_coils(Mbit, 300, bitM_echo);
        fx5uc->modbus_read_coils(PORT1, 7,device.y1);
        if(bitM_echo[0] == ON)// Nếu M0 on 
        {
            if(bitM_echo[5] == ON){
                device.D[1] = RED;
                bitM_pub[1] = ON;bitM_pub[2] = ON;bitM_pub[3] = OFF;
            }

            else if(bitM_echo[6] == ON){
                device.D[1] = YELLOW;
                bitM_pub[1] = ON;bitM_pub[2] = ON;bitM_pub[3] = OFF;
            }
            else if(bitM_echo[7]== ON){
                device.D[1] = YELLOW;
                bitM_pub[1] = ON;bitM_pub[2] = OFF;bitM_pub[3] = OFF;
            }
            else if(bitM_echo[8]== ON){
                device.D[1] = GREEN;
                bitM_pub[1] = ON;bitM_pub[2] = OFF;bitM_pub[3] = OFF;
            }
            else if(bitM_echo[9]== ON){
                device.D[1] = OCEAN;
                bitM_pub[1] = ON;bitM_pub[2] = OFF;bitM_pub[3] = OFF;
            }
            bitM_pub[16] = ON;  // nang xilanh
            if((isLiftUp == true) && (isLiftDown == false)){   
              //  ROS_INFO("fx5uc_controller.cpp-49- Start LIFT UP M12 = %d M200 = %d",bitM_echo[12],bitM_echo[200]);      
              bitM_pub[12] = ON;  // LIFT_UP
              bitM_pub[14] = OFF;  // LIFT_DOWN
              if(bitM_echo[201] == ON){
                isLiftUp = false;
                agv_msgs::agv_action lift_action;
                lift_action.action = action_;
                lift_action.status = 3;
                lift_status_pub.publish(lift_action);
              } 
            }
            if((isLiftUp == false)&& (isLiftDown == true)){
              //  ROS_INFO("fx5uc_controller.cpp-49- Start LIFT DOWN M14 = %d M201 = %d",bitM_echo[14],bitM_echo[201]);
              bitM_pub[14] = ON;  // LIFT_DOWN
              bitM_pub[12] = OFF;  // LIFT_DOWN
              if(bitM_echo[200] == ON){
                isLiftDown = false;
                agv_msgs::agv_action lift_action;
                lift_action.action = action_;
                lift_action.status = 3;
                lift_status_pub.publish(lift_action);
              } 
            }
            
        //     ///// Add by DuNV
        //     // ROS_INFO("fx5uc_controller.cpp-89- bitM_echo[20]: %d", bitM_echo[20]);
            //  std::cout << "M20 = "<<bitM_echo[20] << " Y13 = "<< device.y1[3]<< std::endl;
            if(bitM_echo[20] == ON) //sensor charging AGV
            {
                // bitM_pub[18] = ON;   // send to PLC
                // ros::Duration(5).sleep(); 
                ROS_INFO("fx5uc_controller.cpp-89- Shutdown the IPC");
                //system("sudo shutdown now");
            }

            fx5uc->modbus_write_coils(Mbit, 100, bitM_pub); 
            fx5uc->modbus_write_register(0, device.D[1]); 
        } else {
            // ROS_WARN("fx5uc_controller.cpp-80-not listen"); 
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
    //close connection and free the memory
    fx5uc->modbus_close();  
    delete(fx5uc);
    return 0;
}  