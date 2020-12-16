#!/usr/bin/python

#*******************************************************************************
# Copyright 2020 Jose Manuel Fajardo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#******************************************************************************/

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

    period = 40.0
    frecuency = 2*np.math.pi / period
    offset_time = 0.0

    period2 = 15.0
    frecuency2 = 2*np.math.pi / period2

    Msg = Trajectory()
    Msg.pose.translation.x = init_pos[0]
    Msg.pose.translation.y = init_pos[1]
    Msg.pose.translation.z = init_pos[2]
    Msg.pose.rotation.x    = 0.0
    Msg.pose.rotation.y    = 0.0
    Msg.pose.rotation.z    = 0.0
    Msg.pose.rotation.w    = 1.0

    Msg.joints.mobjoint3 = -10.0
    Msg.joints.joint1 =  -10.0
    Msg.joints.joint2 =  -10.0
    Msg.joints.joint3 =  -10.0
    Msg.joints.joint4 = -10.0
    Msg.joints.joint5 = -10.0
    Msg.joints.joint6 = -10.0

    Quat0 =  np.array([0,0,0,1]) #np.array([-0.412, -0.192, -0.412, 0.790]) 
    Quat0 = (1.0/np.linalg.norm(Quat0))*Quat0

    Quat1 = np.array([0,0,0,1]) #np.array([0.5, 0.5, -0.5, 0.5]) #np.array([0.191, 0.462, 0.191, 0.845]) #np.array([0,0.707,0,0.707])
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
        #print("time: ", current_time)
        if(current_time>= period):
            break

        offset_x = 0.2*np.math.cos(frecuency*current_time + offset_time)
        offset_y = 0.2*np.math.sin(frecuency*current_time + offset_time)

        Msg.pose.translation.x = init_pos[0] + 0.2 - offset_x
        Msg.pose.translation.y = init_pos[1] + offset_y
        Msg.pose.translation.z = init_pos[2]  # + 0.1*np.math.sin(frecuency*current_time + offset_time)

        angle = -np.math.atan2(offset_y, offset_x)

        Quat_int = tf_conversions.transformations.quaternion_about_axis(angle, (0,0,1))

        Msg.pose.rotation.x = 0#Quat_int[0]
        Msg.pose.rotation.y = 0#Quat_int[1]
        Msg.pose.rotation.z = 0#Quat_int[2]
        Msg.pose.rotation.w = 1#Quat_int[3]

        Msg.joints.mobjoint3 = 0.349066*np.math.sin(frecuency2*current_time + offset_time)
        # 0.349066 rad , 15 s period
        #if(angle<0):
        #    Msg.joints.mobjoint3 = angle
            

        pubTrajectory.publish(Msg)
        rate.sleep()
    

    # Final message
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
