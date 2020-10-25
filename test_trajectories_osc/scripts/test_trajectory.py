#!/usr/bin/python

import rospy

from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist
from mobile_manipulator_msgs.msg import Trajectory

import tf2_ros
import tf2_geometry_msgs

import tf_conversions

import numpy as np
import time

#-----------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------#

if __name__ == '__main__':

    rospy.init_node('path_publisher', anonymous=True)
    rospy.loginfo("Node init")

    pubTrajectory = rospy.Publisher('/mobile_manipulator/desired_traj', Trajectory, queue_size=1)

    init_pos = np.array([0.53649, 0, 0.74675])

    period = 15.0
    frecuency = 2*np.math.pi / period
    offset_time = 0.0

    Msg = Trajectory()
    Msg.pose.translation.x = init_pos[0]
    Msg.pose.translation.y = init_pos[1]
    Msg.pose.translation.z = init_pos[2]
    Msg.pose.rotation.x    = 0.0
    Msg.pose.rotation.y    = 0.0
    Msg.pose.rotation.z    = 0.0
    Msg.pose.rotation.w    = 1.0

    Quat0 =  np.array([0,0,0,1]) #np.array([-0.412, -0.192, -0.412, 0.790]) 
    Quat0 = (1.0/np.linalg.norm(Quat0))*Quat0

    Quat1 = np.array([0.5, 0.5, -0.5, 0.5]) #np.array([0.191, 0.462, 0.191, 0.845]) #np.array([0,0.707,0,0.707])
    Quat1 = (1.0/np.linalg.norm(Quat1))*Quat1

    rate = rospy.Rate(200.0)
    init_time = None

    while(rospy.is_shutdown() is not True):
        
        if(init_time is None):
            init_time = rospy.Time.now()

        current_time = (rospy.Time.now() - init_time).to_sec()
        Msg.pose.translation.x = init_pos[0]  + 1.0*np.math.sin(frecuency*current_time + offset_time)
        Msg.pose.translation.y = init_pos[1]  + 1.0*np.math.sin(frecuency*current_time + offset_time)
        Msg.pose.translation.z = 0.5  + 0.1*np.math.sin(frecuency*current_time + offset_time)

        scale = np.abs(np.math.sin(frecuency*current_time))
        #print(scale)

        Quat_int = tf_conversions.transformations.quaternion_slerp(Quat0, Quat1, scale )

        Msg.pose.rotation.x = Quat_int[0]
        Msg.pose.rotation.y = Quat_int[1]
        Msg.pose.rotation.z = Quat_int[2]
        Msg.pose.rotation.w = Quat_int[3]

        pubTrajectory.publish(Msg)
        rate.sleep()
    
    #Quat1 = np.array([0,0,0,1])
    #Quat2 = tf_conversions.transformations.random_quaternion()
    #Quat_int = tf_conversions.transformations.quaternion_slerp(Quat1, Quat2, 0.5)
    #print( "Quat1: \n" , Quat1)
    #print( "Quat2: \n" , Quat2)
    #print( "Quat Int: \n" , Quat_int)
    #tf_conversions.transformations.quaternion_slerp



    rospy.loginfo("Final Command published")

    rospy.signal_shutdown("End of path")
