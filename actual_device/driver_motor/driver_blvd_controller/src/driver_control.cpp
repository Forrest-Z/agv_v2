#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include "driver_blvd_controller/speed_wheel.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
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

ros::Publisher control_wheel_left_pub, control_wheel_right_pub;
void publishControlWheel (int16_t W_l, int16_t W_r);

void velCallback(const geometry_msgs::Twist& msg)
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
  // ROS_INFO("driver_control.cpp-67- Wheel left: %d  Wheel right: %d", W_l, W_r);

  publishControlWheel(W_l, W_r);
} //cmd_velCallback

void publishControlWheel (int16_t W_l, int16_t W_r){
  // ROS_INFO("driver_control.cpp-73- publishControlWheel: Wheel left: %d  Wheel right: %d", W_l, W_r);
  driver_blvd_controller::speed_wheel robot;
  robot.wheel_letf = W_l;
  robot.wheel_right = -W_r;
  control_wheel_left_pub.publish(robot);
  control_wheel_right_pub.publish(robot);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "driver_control");

  /* Publisher */
  ros::NodeHandle nh;
  control_wheel_left_pub = nh.advertise<driver_blvd_controller::speed_wheel>("left_wheel/control_wheel", 20);
  control_wheel_right_pub = nh.advertise<driver_blvd_controller::speed_wheel>("right_wheel/control_wheel", 20);

  /* Subscriber */
  ros::Subscriber cmd_vel;
  cmd_vel = nh.subscribe("cmd_vel", 20, velCallback);

  ros::spin();
   
  return 0;
}


