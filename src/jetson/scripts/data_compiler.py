#subscribe to "nav_ned" with type nav_msgs/Odometry for USV pose
#subscribe to "current_goal_pose" with type jetson/usv_pose_msg for desired goal pose
#subscribe to "control_efforts_topic" with type jetson/control_efforts for low level PID control efforts

#!/usr/bin/env/python3

import rospy
import message_filters
import math
from nav_msgs.msg import Odometry
from jetson.msg import usv_pose_msg
#from jetson.msg import control_efforts


file = open("pose_data.txt", "w")
file.write("Cosstrack_Error,Angular_Error,x,y,theta,x_desired,y_desired,theta_desired\n")
file.close

def callback(desiredData, locationData):	# , controlData
    desiredX = desiredData.position.x
    desiredY = desiredData.position.y
    desiredPsi = desiredData.psi.data
    currentX = locationData.pose.pose.position.x
    currentY = locationData.pose.pose.position.y
    currentPsi = locationData.pose.pose.orientation.z
    
    dist =  math.sqrt(pow(abs(desiredX-currentX),2)+pow(abs(desiredY-currentY),2))

    ang = desiredPsi-currentPsi

    #controlX = controlData.t_x.data
    #controlY = controlData.t_y.data
    #controlPsi = controlData.m_z.data

    rospy.loginfo(dist)

    file = open("pose_data.txt", "a")
    file.write(str(dist))
    file.write(",")
    file.write(str(ang))
    file.write(",")
    file.write(str(currentX))
    file.write(",")
    file.write(str(currentY))
    file.write(",")
    file.write(str(currentPsi))
    file.write(",")
    file.write(str(desiredX))
    file.write(",")
    file.write(str(desiredY))
    file.write(",")
    file.write(str(desiredPsi))
    #file.write(",")
    #file.write(str(controlX))
    #file.write(",")
    #file.write(str(controlY))
    #file.write(",")
    #file.write(str(controlPsi))
    file.write("\n")
    file.close


def listener():
    rospy.init_node('data_compiler')
    
    desiredSub = message_filters.Subscriber("current_goal_pose", usv_pose_msg)
    locationSub = message_filters.Subscriber("nav_ned", Odometry)
    #controlSub = message_filters.Subscriber("control_efforts_topic", control_efforts)

    ts = message_filters.ApproximateTimeSynchronizer([desiredSub, locationSub], 10, 1, allow_headerless=True)
    ts.registerCallback(callback)

    rospy.spin()

if __name__== '__main__':
    listener()
