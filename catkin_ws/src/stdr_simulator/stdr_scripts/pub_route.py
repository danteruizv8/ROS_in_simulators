#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2020, Dante Ruiz.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import rospy

from geometry_msgs.msg import Twist #for publish
from nav_msgs.msg import Odometry  #for subscriber
#from std_srvs.srv import Empty, EmptyRequest  # for gazebo pause


### services
#pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
#rst_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
#rst_w = rospy.ServiceProxy('/gazebo/reset_world', Empty)
#unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

## cond. ini
vel_x = 0.0
vel_ang = 0.0
## define subscrip type
pos = Odometry()
pos_x = 0.0
pos_y = 0.0
dir_w = 1.0

## publisher
def vel(vel_x,vel_ang):
    twist = Twist()
    twist.linear.x = vel_x; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = vel_ang
    pub.publish(twist)
    
## listener
def escucha(pos):
     #print('x:', pos.pose.pose.position.x)
     global pos_x
     global pos_y
     global dir_w
     pos_x = pos.pose.pose.position.x
     pos_y = pos.pose.pose.position.y
     dir_w = pos.pose.pose.orientation.w

## route trace
def route():
   rate = rospy.Rate(15) # 15hz
   # while not rospy.is_shutdown():
   global pos_x
   global pos_y
   global dir_w
   rate.sleep()
   x_vel = 0.1
   w_vel = 0.1
   printdat = 1
   offset_x = pos_x # posicion inicial
   offset_y = pos_y
   print offset_x,offset_y
   while pos_x < (1.0 + offset_x):
      vel(x_vel,0.0)
      if printdat: print('x: ', pos_x)
      rate.sleep()
     
   while dir_w > 0.707:
      vel(0.0,w_vel)
      if printdat: print('z: ', dir_w)
      rate.sleep()
     
   while pos_y < (1.0 + offset_y):
       vel(x_vel,0.0)
       if printdat: print('y: ', pos_y)
       rate.sleep()
       
   while dir_w > 0:
      vel(0.0,w_vel)
      if printdat: print('z: ', dir_w)
      rate.sleep()
     
   while pos_x > (0.0 + offset_x):
       vel(x_vel,0.0)
       if printdat: print('y: ', pos_y)
       rate.sleep()
      
   while dir_w > -0.707:
      vel(0.0,w_vel)
      if printdat: print('z: ', dir_w)
      rate.sleep()
     
   while pos_y > (0.0 + offset_y):
       vel(x_vel,0.0)
       if printdat:  print('x: ', pos_x)
       rate.sleep()
     
   while dir_w > -0.999:
      vel(0.0,0.1)
      if printdat: print('z: ', dir_w)
      rate.sleep()    

   vel(0.0,0.0)  ## stop

if __name__ == '__main__':

   ## odom --->  [turtlebot3_route] ---> cmd_vel 
         rospy.init_node('simple_robot_route')
         pub = rospy.Publisher('robot0/cmd_vel', Twist, queue_size=10)
         rospy.Subscriber('robot0/odom', Odometry, escucha)
         

         try:
#              pause(EmptyRequest())
#              rst_sim(EmptyRequest())
#              rst_w(EmptyRequest())
#              unpause(EmptyRequest()) #init sim
              route()
#              pause(EmptyRequest())  #end sim
              
         except rospy.ROSInterruptException:
              pass
            
         finally:
              twist = Twist()
              twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
              twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
              pub.publish(twist)
