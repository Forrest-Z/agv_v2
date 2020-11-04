#!/usr/bin/env python
import rospy
import os
from rospy_message_converter import json_message_converter
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult

from agv_msgs.msg import agv_action

pub_init_pose = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
rospy.loginfo("agv_main.py-Publisher topic /initialpose")
pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rospy.loginfo("agv_main.py-Publisher topic /cmd_vel")
pub_charging = rospy.Publisher('charging_action', agv_action, queue_size=10)
rospy.loginfo("agv_main.py-Publisher topic /charging_action")
pub_lift = rospy.Publisher('lift_action', agv_action, queue_size=10)
rospy.loginfo("agv_main.py-Publisher topic /lift_action")
pub_move_base_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
rospy.loginfo("agv_main.py-Publisher topic /move_base_simple/goal")

pub_action_status = rospy.Publisher('action_status', agv_action, queue_size=10)
rospy.loginfo("agv_main.py-Publisher topic /action_status")

action_ = 0
action_id_ = ""

def actionMode(argument):
    switcher = {
        0:  "ACTION_FLOAT",
        1:  "ACTION_MANUAL",
        2:  "ACTION_INITIAL_POSE",
        3:  "ACTION_QUALITY_POSE",
        4:  "ACTION_ROTATE_GOAL",
        5:  "ACTION_MOVE_GOAL",
        6:  "ACTION_CHARGING_IN",
        7:  "ACTION_CHARGING_OUT",
        8:  "ACTION_LIFT_IN",
        9:  "ACTION_LIFT_UP",
        10: "ACTION_LIFT_DOWN",
        11: "ACTION_LIFT_OUT"
    }
    return switcher.get(argument, "Invalid action")

def initialAgvAction(msg):
    rospy.loginfo("agv_main.py-initialAgvAction()") 
    global action_, action_id_
    action_ = msg.action
    action_id_ = msg.action_id
    action_mode = actionMode(msg.action)

    rospy.loginfo("agv_main.py-action_mode: " + str(action_mode))
    if (action_mode == "ACTION_FLOAT"):
        floatFunction()
    elif (action_mode == "ACTION_MANUAL"):
        message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
        manualFunction(message)
    elif (action_mode == "ACTION_INITIAL_POSE"):
        message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
        initPoseFunction(message)
    elif (action_mode == "ACTION_QUALITY_POSE"):
        message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
        qualityPoseFunction(message)
    elif (action_mode == "ACTION_ROTATE_GOAL"):
        message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
        moveGoalFunction(message)
    elif (action_mode == "ACTION_MOVE_GOAL"):
        message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
        moveGoalFunction(message)
    elif (action_mode == "ACTION_CHARGING_IN"):
        lineInFunction(msg)
    elif (action_mode == "ACTION_CHARGING_OUT"):
        message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
        moveGoalFunction(message)
    elif (action_mode == "ACTION_LIFT_IN"):
        lineInFunction(msg)
    elif (action_mode == "ACTION_LIFT_UP"):
        liftFunction(msg)
    elif (action_mode == "ACTION_LIFT_DOWN"):
        liftFunction(msg)
    elif (action_mode == "ACTION_LIFT_OUT"):
        message = json_message_converter.convert_json_to_ros_message(msg.type, msg.data)
        moveGoalFunction(message)

def floatFunction():
    print ("agv_main.py-Do Nothing")
def manualFunction(msg):
    print ("agv_main.py-manualFunction")
    global pub_cmd_vel
    pub_cmd_vel.publish(msg)
    rospy.loginfo("agv_main.py-pub_cmd_vel")
def qualityPoseFunction():
    print ("agv_main.py-qualityPoseFunction()")
def moveGoalFunction(msg):
    print ("agv_main.py-moveGoalFunction")
    global pub_move_base_goal
    pubActionStatus(msg, 1)
    pub_move_base_goal.publish(msg)
    rospy.loginfo("agv_main.py-pub_move_base_goal")
def initPoseFunction(msg):
    print ("agv_main.py-initPoseFunction")
    global pub_init_pose
    pubActionStatus(msg, 1)
    initPose = PoseWithCovarianceStamped()
    initPose.header = msg.header
    initPose.pose.pose = msg.pose
    pub_init_pose.publish(initPose)
def lineInFunction(msg):
    print ("agv_main.py-lineInFunction")
    global pub_charging
    pubActionStatus(msg, 1)
    pub_charging.publish(msg)
    rospy.loginfo("agv_main.py-publish chargingIn: " + str(msg))
def liftFunction(msg):
    print ("agv_main.py-liftFunction")
    global pub_lift
    pubActionStatus(msg, 1)
    pub_lift.publish(msg)


def moveResult(msg):
    rospy.loginfo("agv_main.py-moveResult()")
    status_ = msg.status.status
    pubActionStatus(msg, status_)

def liftResult(msg):
    rospy.loginfo("agv_main.py-liftResult()")
    status_ = msg.status
    pubActionStatus(msg, status_)

def chargingResult(msg):
    rospy.loginfo("agv_main.py-chargingResult()")
    status_ = msg.status
    pubActionStatus(msg, status_)

def pubActionStatus(msg, status):
    global pub_action_status, action_, action_id_
    action_status_ = agv_action()
    # action_status_.header = msg.header
    action_status_.action = action_
    action_status_.status = status
    action_status_.action_id = action_id_

    # rospy.loginfo("agv_main.py-action_status: " + str(action_status_))
    pub_action_status.publish(action_status_)
    rospy.loginfo("agv_main.py-publish action_status: " + str(action_status_))

if __name__ == '__main__':
    rospy.init_node('agv_main', log_level=rospy.DEBUG)

    os.system('sudo rm -rf /home/robotics/.ros/log')
    rospy.loginfo("agv_main.py-$sudo rm -rf /home/robotics/.ros/log")

    os.system('sudo ip link set can0 type can')
    os.system('sudo ip link set can0 up type can bitrate 125000')
    rospy.loginfo("agv_main.py-Config Can port")

    rospy.Subscriber("agv_action", agv_action, initialAgvAction)
    rospy.loginfo("agv_main.py-Subscriber topic /agv_action")

    rospy.Subscriber("/move_base/result", MoveBaseActionResult, moveResult)
    rospy.loginfo("agv_main.py-Subscriber topic /move_base/result")

    rospy.Subscriber("/lift_status", agv_action, liftResult)
    rospy.loginfo("agv_main.py-Subscriber topic /lift_status")

    rospy.Subscriber("/charging_status", agv_action, chargingResult)
    rospy.loginfo("agv_main.py-Subscriber topic /charging_status")

    rospy.spin()