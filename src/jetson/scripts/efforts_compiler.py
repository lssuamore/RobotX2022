#subscribe to "nav_ned" with type nav_msgs/Odometry for USV pose
#subscribe to "current_goal_pose" with type jetson/usv_pose_msg for desired goal pose
#subscribe to "control_efforts_topic" with type jetson/control_efforts for low level PID control efforts

#!/usr/bin/env/python3

import rospy
import message_filters
from jetson.msg import control_efforts


file = open("control_data.txt", "w")
file.write("T_X,T_Y,M_Z\n")
file.close

def callback(controlData):
    controlX = controlData.t_x.data
    controlY = controlData.t_y.data
    controlPsi = controlData.m_z.data
    
    rospy.loginfo(controlX)
    rospy.loginfo(controlY)
    rospy.loginfo(controlPsi)
    
    file = open("control_data.txt", "a")
    file.write(str(controlX))
    file.write(",")
    file.write(str(controlY))
    file.write(",")
    file.write(str(controlPsi))
    file.write("\n")
    file.close


def listener():
    rospy.init_node('efforts_compiler')
    
    controlSub = message_filters.Subscriber("control_efforts_topic", control_efforts)

    ts = message_filters.ApproximateTimeSynchronizer([controlSub], 10, 1, allow_headerless=True)
    ts.registerCallback(callback)

    rospy.spin()

if __name__== '__main__':
    listener()
