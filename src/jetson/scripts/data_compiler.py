#subscribe to "nav_ned" with type nav_msgs/Odometry for USV pose
#subscribe to "current_goal_pose" with type jetson/usv_pose_msg for desired goal pose
#subscribe to /vehicle_pose with type geometry_msgs/Pose2d for position
#subscribe to /traj_desired_state with type std_msgs/Float32MultiArray for expected position

#format for std_msgs/Float32MultiArray
#x_ned position
#y_ned position
#Psi_ned angle;
#VelX_ned velocity
#VelY_ned velocity
#Wz_ned angular velocity
#AccX_ned acceleration
#AccY_ned acceleration
#AlphaZ_ned angular acceleration

#!/usr/bin/env/python3
##!/usr/bin/env python

import rospy
import message_filters
import math
from nav_msgs.msg import Odometry
from jetson.msg import usv_pose_msg
#from geometry_msgs.msg import Pose2D
#from std_msgs.msg import Float32MultiArray, Float64
#from vehicle_control.msg import control_effort


file = open("data.txt", "w")
file.write("Cosstrack_Error,Angular_Error,x,y,theta,x_desired,y_desired,theta_desired\n")
file.close

def callback(desiredData, locationData):
    #desiredX = desiredData.data[0]
    #desiredY = desiredData.data[1]
    desiredX = desiredData.position.x
    desiredY = desiredData.position.y
    desiredPsi = desiredData.psi.data
    #currentX = locationData.x
    #currentY = locationData.y
    currentX = locationData.pose.pose.position.x
    currentY = locationData.pose.pose.position.y
    currentPsi = locationData.pose.pose.orientation.z
    
    dist =  math.sqrt(pow(abs(desiredX-currentX),2)+pow(abs(desiredY-currentY),2))

    ang = desiredPsi-currentPsi
    #ang = desiredData.data[2]-locationData.theta

    # controlX = controlData.tau[0].data
    # controlY = controlData.tau[1].data
    # controlTheta = controlData.tau[2].data

    rospy.loginfo(dist)

    file = open("data.txt", "a")
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
    file.write("\n")
    file.close


def listener():
    #rospy.init_node('error_calc')
    rospy.init_node('data_compiler')
    
    desiredSub = message_filters.Subscriber("current_goal_pose", usv_pose_msg)
    locationSub = message_filters.Subscriber("nav_ned", Odometry)
    #desiredSub = message_filters.Subscriber("/traj_desired_state", Float32MultiArray)
    #locationSub = message_filters.Subscriber("/vehicle_pose", Pose2D)
    # effortSub = message_filters.Subscriber("/control_effort", control_effort)

    ts = message_filters.ApproximateTimeSynchronizer([desiredSub, locationSub], 10, 1, allow_headerless=True)
    ts.registerCallback(callback)

    rospy.spin()

if __name__== '__main__':
    listener()
