#!/usr/bin/env python
# modefied as right-hand follower, requires /tf topic published from rbx1_vision/openni_tracker 
"""
    follower.py - Version 1.0 2012-06-01
    
    Follow a "person" by tracking the nearest object in x-y-z space.
    
    Based on the follower application by Tony Pratkanis at:
    
    http://ros.org/wiki/turtlebot_follower
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import roslib; roslib.load_manifest('rbx1_apps')
import rospy
#from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist,Point
from math import copysign
import point_cloud2
import tf

class Follower():
    def __init__(self):
        rospy.init_node("follower")
        
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        rate = rospy.get_param('~rate', 20)
        r = rospy.Rate(rate)
        self.tf_prefix = rospy.get_param('~tf_prefix', '/skeleton')
        self.fixed_frame = rospy.get_param('~fixed_frame', 'openni_depth_frame')
        self.fixed_frame = self.tf_prefix + '/' + self.fixed_frame
        tf_listener = tf.TransformListener()

        # The goal distance (in meters) to keep between the robot and the person
        self.goal_z = rospy.get_param("~goal_z", 0.6)
        
        # How far away from the goal distance (in meters) before the robot reacts
        self.z_threshold = rospy.get_param("~z_threshold", 0.05)
        
        # How far away from being centered (x displacement) on the person
        # before the robot reacts
        self.y_threshold = rospy.get_param("~y_threshold", 0.05)
        
        # How much do we weight the goal distance (z) when making a movement
        self.z_scale = rospy.get_param("~z_scale", 1.0)

        # How much do we weight x-displacement of the person when making a movement        
        self.y_scale = rospy.get_param("~y_scale", 1.0)
        
        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 2.0)
        
        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.0)
        
        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.3)
        
        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.1)
    
        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)

        rospy.loginfo("Waiting for tf...")
        
        tf_listener.waitForTransform(self.fixed_frame, self.fixed_frame, rospy.Time(), rospy.Duration(60.0))   
        rospy.loginfo("Ready to follow!")


        while not rospy.is_shutdown():

            frame='/skeleton/left_hand_1'  # left_hand_1 corresponds to the right hand actually in real world
                                           # default to have only one body recognized and it's the first one
            try:
                position = Point()
                # Get the transformation from the fixed frame to the skeleton frame
                (trans, rot) = tf_listener.lookupTransform(self.fixed_frame, frame, rospy.Time(0))
                position.x = trans[0]
                position.y = trans[1]
                position.z = trans[2]
                self.set_cmd_vel(position)                                            
                 # Set a marker at the origin of this frame
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.logerr("tf error when looking up " + frame + ' and ' + self.fixed_frame)
            continue
                          
            r.sleep()



        
    def set_cmd_vel(self, pos):
        x = y = z = 0
        x = pos.x
        y = pos.y
        z = pos.z
                
        # Stop the robot by default
        move_cmd = Twist()
        
        #for debugging, print the x,y,z data
        a = '[ x:'+str(x)+' y:'+str(y)+' z:'+str(z)+' ]' 
        rospy.loginfo(a)    
        
        # Check our movement thresholds
        if (abs(z) > self.z_threshold) or (abs(y) > self.y_threshold):     
            # Compute the linear and angular components of the movement
            linear_speed = z * self.z_scale
            angular_speed = -y * self.y_scale
                
            # Make sure we meet our min/max specifications
            linear_speed = copysign(max(self.min_linear_speed, 
                                        min(self.max_linear_speed, abs(linear_speed))), linear_speed)   # min<=|speed|<=max
            angular_speed = copysign(max(self.min_angular_speed, 
                                         min(self.max_angular_speed, abs(angular_speed))), angular_speed)
    
            move_cmd.linear.x = linear_speed
            move_cmd.angular.z = angular_speed
                        
        # Publish the movement command
        self.cmd_vel_pub.publish(move_cmd)

        
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)     
                   
if __name__ == '__main__':
    try:
        Follower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Follower node terminated.")
