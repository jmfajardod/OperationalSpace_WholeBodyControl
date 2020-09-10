#!/usr/bin/python

import rospy

from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist

import tf2_ros
import tf2_geometry_msgs

import numpy as np
import time

#-----------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------#
#-----------------------------------------------------------------------------------------#

if __name__ == '__main__':

    rospy.init_node('path_publisher', anonymous=True)

    rospy.loginfo("Node init")
    VelCmd = Twist()
    VelCmd.linear.x = 0.0
    VelCmd.linear.y = 0.0
    VelCmd.angular.z = 0.0

    pubVelCommand = rospy.Publisher('/Robotino/commands/velocity', Twist, queue_size=1)
    pubVelCommand.publish(VelCmd)
    pubVelCommand.publish(VelCmd)
    rospy.sleep(10.0)

    #--------------------------------------------------------------------------------#
    # Init path

    VelCmd.linear.x = 0.2
    VelCmd.linear.y = 0.0
    VelCmd.angular.z = 0.0
    pubVelCommand.publish(VelCmd)
    rospy.loginfo("Command published")
    rospy.sleep(10.0)

    VelCmd.linear.x = 0.0
    VelCmd.linear.y = 0.5
    VelCmd.angular.z = 0.0
    pubVelCommand.publish(VelCmd)
    rospy.loginfo("Command published")
    rospy.sleep(6.0)

    VelCmd.linear.x = 0.1
    VelCmd.linear.y = 0.1
    VelCmd.angular.z = 0.0
    pubVelCommand.publish(VelCmd)
    rospy.loginfo("Command published")
    rospy.sleep(40.0)

    VelCmd.linear.x  = -0.1
    VelCmd.linear.y  = -0.4
    VelCmd.angular.z = -0.3
    pubVelCommand.publish(VelCmd)
    rospy.loginfo("Command published")
    rospy.sleep(10.0)

    VelCmd.linear.x  =  0.2
    VelCmd.linear.y  = -0.5
    VelCmd.angular.z = -0.8
    pubVelCommand.publish(VelCmd)
    rospy.loginfo("Command published")
    rospy.sleep(10.0)

    VelCmd.linear.x = 0.1
    VelCmd.linear.y = 0.0
    VelCmd.angular.z = 0.0
    pubVelCommand.publish(VelCmd)
    rospy.loginfo("Command published")
    rospy.sleep(40.0)

    VelCmd.linear.x  =  0.25
    VelCmd.linear.y  = -0.25
    VelCmd.angular.z = -1.5
    pubVelCommand.publish(VelCmd)
    rospy.loginfo("Command published")
    rospy.sleep(8.0)

    VelCmd.linear.x = 0.0
    VelCmd.linear.y = -0.5
    VelCmd.angular.z = 0.0
    pubVelCommand.publish(VelCmd)
    rospy.loginfo("Command published")
    rospy.sleep(8.0)

    VelCmd.linear.x = -0.5
    VelCmd.linear.y =  0.0
    VelCmd.angular.z = 1.5
    pubVelCommand.publish(VelCmd)
    rospy.loginfo("Command published")
    rospy.sleep(26.0)

    VelCmd.linear.x = 0.0
    VelCmd.linear.y = 0.25
    VelCmd.angular.z = 0.0
    pubVelCommand.publish(VelCmd)
    rospy.loginfo("Command published")
    rospy.sleep(32.0)



    # Finish path
    #---------------------------------------------------------------------------------#

    VelCmd.linear.x = 0.0
    VelCmd.linear.y = 0.0
    VelCmd.linear.z = 0.0
    VelCmd.angular.x = 0.0
    VelCmd.angular.y = 0.0
    VelCmd.angular.z = 0.0
    pubVelCommand.publish(VelCmd)
    rospy.loginfo("Final Command published")

    rospy.signal_shutdown("End of path")
