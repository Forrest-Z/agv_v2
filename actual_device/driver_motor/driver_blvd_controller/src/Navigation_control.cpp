#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include "driver_blvd_controller/speed_wheel.h"
#include "agv_msgs/agv_action.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <unistd.h>

#define BLVD20KM_SPEED_MIN 80
#define BLVD20KM_SPEED_MAX 4000
#define BLVD20KM_TORQUE_MAX 200

#define Pi 3.1415926535
#define rad_rpm 9.5492965964254
#define L  0.255 // wheelbase (in meters per radian)
#define R  0.075 //wheel radius (in meters per radian)
int16_t W_l, W_r; // speed befor gear 
clock_t start;
const unsigned long timeoutMs = 1; //sec
uint8_t action_ ;

void actionCallback(const agv_msgs::agv_action& msg)
{
	ROS_INFO("Navigation_control.cpp-26-actionCallback()-action: %d", msg.action);
  action_ = msg.action;
}//teleop_keyCallback 

void cmd_velCallback(const geometry_msgs::Twist& msg)
{
  start = clock();
  float k_v = 1;    // percent speed %
  float V_max ;     // speed max when percent speed = 100%  (m/s)
  float K = 30;          // He so chuyen
  float V;  // forward velocity (ie meters per second)
  float W;  // angular velocity (ie radians per second)
  float v_r; // clockwise angular velocity of right wheel (ie radians per second)
  float v_l; // counter-clockwise angular velocity of left wheel (ie radians per second)
  float w_r, w_l; // speed rad/s of one

  V_max = msg.linear.x;  W = msg.angular.z;
  V = V_max*k_v;

  /* Van toc goc 2 banh */
  w_r = ((2 * V) + (W * L)) / (2 * R);   //(rad/s)
  w_l = ((2 * V) - (W * L)) / (2 * R);   //(rad/s)

  /* Van toc 2 banh */
  v_r = w_r*rad_rpm;  // (rpm)  
  v_l = w_l*rad_rpm;  // (rpm) 

  /* van toc truoc hop so */
  W_r = v_r*K; 
  W_l = v_l*K;
  /* Kiem  tra van toc */
  if(abs(W_r) > BLVD20KM_SPEED_MAX) W_r = BLVD20KM_SPEED_MAX;
  if(abs(W_l) > BLVD20KM_SPEED_MAX) W_l = BLVD20KM_SPEED_MAX;

  if(abs(W_r) < BLVD20KM_SPEED_MIN) W_r = 0;
  if(abs(W_l) < BLVD20KM_SPEED_MIN) W_l = 0;
  ROS_INFO("Navigation_control.cpp-83- Wheel left: %d  Wheel right: %d", W_l, W_r);
} //cmd_velCallback

int main(int argc, char **argv)
{
  /**
   Khoi tao Node 
   */
  driver_blvd_controller::speed_wheel robot;
  ros::init(argc, argv, "Echo_navigation");
  ros::NodeHandle nh;
  ros::Rate loop_rate(20);

  /* Publisher */
  ros::Publisher Navigation_control;
  Navigation_control = nh.advertise<driver_blvd_controller::speed_wheel>("cmd_vel_to_wheel", 20);

  /* Subscriber */
  ros::Subscriber cmd_vel;
  cmd_vel = nh.subscribe("cmd_vel", 20,cmd_velCallback);
  ros::Subscriber action = nh.subscribe("agv_action", 20,actionCallback);
  uint64_t time_count;
  while (ros::ok())
  {
  /*
  * This is a message object. You stuff it with data, and then publish it.
  */
  if(action_ != 6 && action_ != 8)  // if not Charging in and Lift in
  {
    robot.wheel_letf = W_l;
    robot.wheel_right = -W_r;
    Navigation_control.publish(robot);
    // ROS_INFO("Navigation_control.cpp-95- Publish to driver - Wheel left: %d  Wheel right: %d", robot.wheel_letf, robot.wheel_right);
  }
    // if((clock() - start)/CLOCKS_PER_SEC >= timeoutMs) 
    //     W_r = W_l= 0;
    loop_rate.sleep();
    ros::spinOnce();
  }
   
  return 0;
}


