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

from std_srvs.srv import Empty

#-----------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------#

if __name__ == '__main__':

    rospy.init_node('path_publisher', anonymous=True)
    rospy.loginfo("Node init")

    rospy.wait_for_service('/gazebo/unpause_physics')
    rospy.wait_for_service('/gazebo/pause_physics')

    pubTrajectory = rospy.Publisher('/mobile_manipulator/desired_traj', Trajectory, queue_size=1)

    init_pos = np.array([0.53649, 0, 0.74675])

    period = 20.0
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

    Quat0 =  np.array([0,0,0,1]) # np.array([-0.412, -0.192, -0.412, 0.790]) 
    Quat0 = (1.0/np.linalg.norm(Quat0))*Quat0

    Quat1 = np.array([0,0,0,1]) # np.array([0.177, -0.306, -0.177, 0.919]) 
    Quat1 = (1.0/np.linalg.norm(Quat1))*Quat1

    rate = rospy.Rate(200.0)
    init_time = None

    try:
        unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        unpause_gazebo()
        rospy.loginfo("Unpause gazebo")
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    rospy.sleep(3)

    while(rospy.is_shutdown() is not True):
        
        if(init_time is None):
            init_time = rospy.Time.now()

        current_time = (rospy.Time.now() - init_time).to_sec()
        #print(current_time)
        if(current_time>= 3.0*period):
            break

        Msg.pose.translation.x = init_pos[0]  # + 1.0*np.math.sin(frecuency*current_time + offset_time)
        Msg.pose.translation.y = init_pos[1]  # + 1.0*np.math.sin(frecuency*current_time + offset_time)
        Msg.pose.translation.z = init_pos[2]  + 0.1*np.math.sin(frecuency*current_time + offset_time)

        scale = np.abs(np.math.sin(frecuency*current_time))
        #print(scale)

        Quat_int = tf_conversions.transformations.quaternion_slerp(Quat0, Quat1, scale )

        Msg.pose.rotation.x = Quat_int[0]
        Msg.pose.rotation.y = Quat_int[1]
        Msg.pose.rotation.z = Quat_int[2]
        Msg.pose.rotation.w = Quat_int[3]

        Msg.joints.mobjoint3 = -10.0 #0.349066*np.math.sin(frecuency*current_time + offset_time)

        pubTrajectory.publish(Msg)
        rate.sleep()
    

    Msg.pose.translation.x = init_pos[0]  
    Msg.pose.translation.y = init_pos[1] 
    Msg.pose.translation.z = init_pos[2]
    Msg.pose.rotation.x = Quat0[0]
    Msg.pose.rotation.y = Quat0[1]
    Msg.pose.rotation.z = Quat0[2]
    Msg.pose.rotation.w = Quat0[3]
    Msg.joints.mobjoint3 = -10.0
    pubTrajectory.publish(Msg)

    rospy.sleep(10)

    try:
        pause_gazebo = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        pause_gazebo()
        rospy.loginfo("Pause gazebo")
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    rospy.loginfo("Final Command published")

    rospy.signal_shutdown("End of path")
