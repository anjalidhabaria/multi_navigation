#!/usr/bin/env python

# Import required services, messages and libraries
import sys
import rospy
import numpy as np
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from sensor_msgs.msg import *

def set_joint_effort(joint,effort,start,duration):
    rospy.wait_for_service('/gazebo/apply_joint_effort')
    try:
        set_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        success = set_effort(joint,effort,start,duration)
        return success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
def clear_joint_effort(joint):
    rospy.wait_for_service('/gazebo/clear_joint_forces')
    try:
        clear_effort = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)
        success = clear_effort(joint)
        return success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
# Function to get joint position readings from gazebo as sensor messages, returns the joint positions
def gazebo_get():

    rospy.init_node('gazebo_get', anonymous=True)
    data = rospy.wait_for_message("/custom_scara/joint_states", JointState)
    return data.position

# Main function
if __name__ == "__main__":
    
    # Get joint position readings
    while True:
            q = None
            q = gazebo_get()
            print(q)
            duration = rospy.Time(1)
            #duration.secs = -0.1
            start = rospy.Time(0)
            time = rospy.get_time()
            time_old = time
            e_dot = 0
            e_int = 0
            reference = float(input("Reference joint position: "))
            eps = 0.05
            count = 0
            e = reference - q[0]
            e_old = e
            while count<75:
                q = gazebo_get()
                time = rospy.get_time()
                e = reference - q[0]
                e_dot = (e - e_old)/(time_old - time)
                e_int = e_int + e
                print(e_dot)
                d = q[0]
                K_p = 250
                K_d = 10
                K_i = 1
                effort = K_p*e + K_d*e_dot #+ K_i*e_int
                clear_success = clear_joint_effort('joint3')
                effort_success = set_joint_effort('joint3',effort,start,duration)
                time_old = time
                e_old = e
                print("error: {}".format(e))
                if abs(e) < eps:
                        count += 1
                      
