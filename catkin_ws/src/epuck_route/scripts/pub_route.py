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
import time


from geometry_msgs.msg import Twist #for publish
from nav_msgs.msg import Odometry  #for subscriber

## sec to h:min:sec
def convert_t(seconds): 
    seconds = seconds % (24 * 3600) 
    hour = seconds // 3600
    seconds %= 3600
    minutes = seconds // 60
    seconds %= 60
      
    return "%d:%02d:%02d" % (hour, minutes, seconds) 


## cond. ini
vel_x = 0.0
vel_ang = 0.0
## define subscrip type
pos = Odometry()

## publisher
def vel(vel_x,vel_ang):
    twist = Twist()
    twist.linear.x = vel_x; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = vel_ang
    pub.publish(twist)
    
## listener
def escucha(pos):
     global pos_x
     global pos_y
     global dir_w
     pos_x = pos.pose.pose.position.x
     pos_y = pos.pose.pose.position.y
     dir_w = pos.pose.pose.orientation.w
     
## route trace
def route():
   rate = rospy.Rate(15) # 15hz
   rate.sleep()
   x_vel = 1
   w_vel = 1
   printdat = 0
   while pos_x < 1:
      vel(x_vel,0.0)
      if printdat: print('x: ', pos_x)
      rate.sleep()
     
   while dir_w < 0:
      vel(0.0,w_vel)
      if printdat: print('z: ', dir_w)
      rate.sleep()
     
   while pos_y < 1.0:
      vel(x_vel,0.0)
      if printdat: print('y: ', pos_y)
      rate.sleep()
       
   while dir_w > 0.707:
      vel(0.0,w_vel)
      if printdat: print('z: ', dir_w)
      rate.sleep()
     
   while pos_x > 0.0:
      vel(x_vel,0.0)
      if printdat: print('x: ', pos_x)
      rate.sleep()
      
   while dir_w > 0:
      vel(0.0,w_vel)
      if printdat: print('z: ', dir_w)
      rate.sleep()
     
   while pos_y > 0.0:
       vel(x_vel,0.0)
       if printdat: print('y: ', pos_y)
       rate.sleep()
     
   while dir_w > -0.707:
      vel(0.0,w_vel)
      if printdat: print('z: ', dir_w)
      rate.sleep()    

   vel(0.0,0.0)  ## stop
   
   print "Fin de la simulacion... "

if __name__ == '__main__':

   ## odom --->  [ePuck] ---> cmd_vel 
         rospy.init_node('ePuck_route')
         pub = rospy.Publisher('ePuck0/cmd_vel', Twist, queue_size=10)
         rospy.Subscriber('ePuck0/odom', Odometry, escucha)
         

         try:
               t = time.time() #init sim
               route()
               elapsed = time.time() - t #end sim
               print("tiempo total de sim:",convert_t(elapsed))
          
         except rospy.ROSInterruptException:
              pass
            
         finally:
              twist = Twist()
              twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
              twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
              pub.publish(twist)
