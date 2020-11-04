#!/usr/bin/env python
import rospy
from rospy_message_converter import json_message_converter
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

# from agv_main.msg import agv_action
from rospy_message_converter.msg import agv_action


pub = rospy.Publisher('agv_action', agv_action, queue_size=10)
rospy.loginfo("agv_test_web.py-Publisher topic /agv_action")

def callbackGoal(data):
    rospy.loginfo("agv_test_web.py-callbackGoal()")
    typeData = "geometry_msgs/PoseStamped"
    data = json_message_converter.convert_ros_message_to_json(data)
    # rospy.loginfo("agv_main.py-data: %s", data)
    action = agv_action()
    action.action = 2
    action.type = typeData
    action.data = data
    
    rospy.loginfo("agv_test_web.py-action.type: %s", action.type)
    rospy.loginfo("agv_test_web.py-action.data: %s", action.data)

    global pub
    pub.publish(action)
    rospy.loginfo("agv_test_web.py-Publisher to topic /agv_action")

def callbackManual(data):
    rospy.loginfo("agv_test_web.py-callbackManual()")
    typeData = "geometry_msgs/Twist"
    data = json_message_converter.convert_ros_message_to_json(data)
    # rospy.loginfo("agv_main.py-data: %s", data)
    action = agv_action()
    action.action = 1
    action.type = typeData
    action.data = data
    
    rospy.loginfo("agv_test_web.py-action.type: %s", action.type)
    rospy.loginfo("agv_test_web.py-action.data: %s", action.data)

    global pub
    pub.publish(action)
    rospy.loginfo("agv_test_web.py-Publisher to topic /agv_action")

if __name__ == '__main__':
    rospy.init_node('agv_test_web', log_level=rospy.DEBUG)

    rospy.Subscriber("goal", PoseStamped, callbackGoal)
    rospy.loginfo("agv_test_web.py-Subscriber topic /goal")

    rospy.Subscriber("manual", Twist, callbackManual)
    rospy.loginfo("agv_test_web.py-Subscriber topic /manual")

    rospy.spin()