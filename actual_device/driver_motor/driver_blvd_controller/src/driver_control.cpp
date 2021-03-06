#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include "driver_blvd_controller/speed_wheel.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define Pi 3.1415926535
#define rad_rpm 9.5492965964254  //convert rpm to rad/minues

int16_t W_l, W_r; // speed befor gear 
clock_t start;

// Parameter
float L, R, K;
float BLVD20KM_SPEED_MIN, BLVD20KM_SPEED_MAX, BLVD20KM_TORQUE_MAX;
std::string driver_left, driver_right;

ros::Publisher control_wheel_left_pub, control_wheel_right_pub;
void publishControlWheel (int16_t W_l, int16_t W_r);
void cmdVelToWheel (geometry_msgs::Twist cmd_vel);
bool loadParam(std::string node_name);

void velCallback(const geometry_msgs::Twist& msg)
{
  ROS_INFO("driver_control.cpp-29- velCallback()");
  cmdVelToWheel(msg);
} //cmd_velCallback

void cmdVelToWheel (geometry_msgs::Twist cmd_vel)
{
  start = clock();
  float k_v = 1;    // percent speed %
  float V_max ;     // speed max when percent speed = 100%  (m/s)
  float V;  // forward velocity (ie meters per second)
  float W;  // angular velocity (ie radians per second)
  float v_r; // clockwise angular velocity of right wheel (ie radians per second)
  float v_l; // counter-clockwise angular velocity of left wheel (ie radians per second)
  float w_r, w_l; // speed rad/s of one

  V_max = cmd_vel.linear.x;  W = cmd_vel.angular.z;
	ROS_INFO("driver_control.cpp-46-linear = %f, angular = %f", cmd_vel.linear.x, cmd_vel.angular.z);
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
}

void publishControlWheel (int16_t W_l, int16_t W_r){
  ROS_INFO("driver_control.cpp-73- publishControlWheel: Wheel left: %d  Wheel right: %d", W_l, W_r);
  driver_blvd_controller::speed_wheel robot;
  robot.wheel_letf = W_l;
  robot.wheel_right = -W_r;
  control_wheel_left_pub.publish(robot);
  control_wheel_right_pub.publish(robot);
}

bool loadParam(std::string node_name){
  ROS_INFO("driver_control.cpp-loadParam() - node_name: %s", node_name.c_str());

  if(!ros::param::get(node_name + "/L", L)){
    return false;
  }
  ROS_INFO("driver_control.cpp- L: %0.3f", L);
  if(!ros::param::get(node_name + "/R", R)){
    return false;
  }
  ROS_INFO("driver_control.cpp- R: %0.3f", R);
  if(!ros::param::get(node_name + "/K", K)){
    return false;
  }
  ROS_INFO("driver_control.cpp- K: %0.0f", K);
    if(!ros::param::get(node_name + "/BLVD20KM_SPEED_MIN", BLVD20KM_SPEED_MIN)){
    return false;
  }
  ROS_INFO("driver_control.cpp- BLVD20KM_SPEED_MIN: %0.0f", BLVD20KM_SPEED_MIN);
  if(!ros::param::get(node_name + "/BLVD20KM_SPEED_MAX", BLVD20KM_SPEED_MAX)){
    return false;
  }
  ROS_INFO("driver_control.cpp- BLVD20KM_SPEED_MAX: %0.0f", BLVD20KM_SPEED_MAX);
  if(!ros::param::get(node_name + "/BLVD20KM_TORQUE_MAX", BLVD20KM_TORQUE_MAX)){
    return false;
  }
  ROS_INFO("driver_control.cpp- BLVD20KM_TORQUE_MAX: %0.0f", BLVD20KM_TORQUE_MAX);
  if(!ros::param::get(node_name + "/driver_left", driver_left)){
    return false;
  }
  ROS_INFO("driver_control.cpp- driver_left: %s", driver_left.c_str());
  if(!ros::param::get(node_name + "/driver_right", driver_right)){
    return false;
  }
  ROS_INFO("driver_control.cpp- driver_right: %s", driver_right.c_str());

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "driver_control");
  ros::NodeHandle nh;

  std::string node_name = ros::this_node::getName();
  ROS_INFO("driver_control.cpp-node_name: %s", node_name.c_str());

  if(loadParam(node_name)){
    ROS_INFO("driver_control.cpp-Load parameter successfull");
  }else{
    ROS_ERROR("driver_control.cpp-Error when load parameter");
  }

  /* Publisher */
  control_wheel_left_pub = nh.advertise<driver_blvd_controller::speed_wheel>(driver_left + "/control_wheel", 20);
  control_wheel_right_pub = nh.advertise<driver_blvd_controller::speed_wheel>(driver_right + "/control_wheel", 20);

  /* Subscriber */
  ros::Subscriber cmd_vel;
  cmd_vel = nh.subscribe("cmd_vel", 20, velCallback);

  ros::spin();
   
  return 0;
}


