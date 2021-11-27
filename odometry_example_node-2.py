#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 28 00:13:45 2020

@author: andres
"""

import rospy
from nav_msgs.msg import Odometry
import math
import tf
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32


#the Publisher is initialized in this line 
pub_odom = rospy.Publisher('odom', Odometry, queue_size=1)
pub_odom_gps = rospy.Publisher('odometer_gps', Odometry, queue_size=1)

class listenerNode():
    
    #accelerometer
    x_accel = 0.0
    y_accel = 0.0
    z_accel = 0.0

    #velocity 
    speed_bl = 0.0
    speed_br = 0.0
    speed_fl = 0.0
    speed_fr = 0.0

    #gyroscope
    pitch_gyro = 0.0
    roll_gyro = 0.0
    yaw_gyro = 0.0

    timestamp_var = 0.0

    #gps
    latitude_var = 0.0
    longitute_var = 0.0

    x_before_gps = 0.0
    y_before_gps = 0.0

    clock = 0.0

    

    


   
    def __init__(self):
        self.loop_hertz = 10.0 #loop frequency
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.velocity_x = 0.0
        self.vy = 0.0
        self.yaw_rate = 0.0


        self.velocity_x_gps = 0.0
        self.gps_delta_x = 0.0
        self.gps_delta_y = 0.0
        self.theta_gps = 0.0
        self.dt = 0.025
        self.yaw_rate_gps = 0.0

        self.gps_x = 0.0
        self.gps_y = 0.0
        self.x_before_gps = 0.0
        self.y_before_gps = 0.0

        self.gps_x_new = 0.0
        self.gps_y_new = 0.0

        self.gps_x_rviz = 0.0
        self.gps_y_rviz = 0.0

        self.flag = 0.0

        # self.yaw_rate = (self.yaw_gyro - self.theta)/self.dt



        
    def run(self):
        self.rate = rospy.Rate(self.loop_hertz)#this line is used to declare the time loop
        rospy.Subscriber("Accelx", Float32, self.callback)
        rospy.Subscriber("Accely", Float32, self.callback1)
        rospy.Subscriber("Accelz", Float32, self.callback2)

        rospy.Subscriber("Blspeed", Float32, self.callback3)
        rospy.Subscriber("Brspeed", Float32, self.callback4)
        rospy.Subscriber("Flspeed", Float32, self.callback5)
        rospy.Subscriber("Frspeed", Float32, self.callback6)

        rospy.Subscriber("Gyro_pitch", Float32, self.callback7)
        rospy.Subscriber("Gyro_roll", Float32, self.callback8)
        rospy.Subscriber("Gyro_yaw", Float32, self.callback9)

        rospy.Subscriber("Timestamp", Float32, self.callback10)

        rospy.Subscriber("latitude", Float32, self.callback11)
        rospy.Subscriber("longitude", Float32, self.callback12)

        rospy.Subscriber("clock", Float32, self.callback13)




        self.br = tf.TransformBroadcaster()#Initialize the object to be used in the frame transformation
        while not rospy.is_shutdown():
            
            self.linearvelocity()#call the function used to create the Odometry ROS message
            self.linearvelocity_gps()

            self.br.sendTransform((self.x, self.y, 0.0), self.q, rospy.Time.now(),"/base_link" , "/map")#this line is used to transform from local frame to global frame and it is necessary to plot the trajectory in RVIZ

            self.br.sendTransform((self.gps_x_new, self.gps_y_new, 0.0), self.q_gps, rospy.Time.now(),"/base_link" , "/map")#this line is used to transform from local frame to global frame and it is necessary to plot the trajectory in RVIZ
            # print(self.x)
            pub_odom.publish(self.odom)
            pub_odom_gps.publish(self.odometer_gps)
            
############Replace this part with the x and y positions determined using the information from the rosbag##################
            # if self.x < 10.0 and self.y == 0.0:
            #     self.x+=0.5
            #     self.y == 0.0
            #     if self.x == 10.0:
            #         self.theta = -math.pi/2
        
            # elif self.x == 10.0 and self.y == 0.0:
            #     self.y -= 0.5
            #     self.theta = -math.pi/2
            # elif self.x == 10.0 and self.y < 0 and self.y > -10.0:
            #     self.y -= 0.5
            #     self.x = 10.0
            #     if self.y == -10.0:
            #         self.theta -=math.pi/2
            # elif self.x > 0.0 and self.y == -10.0:
            #     self.x-=0.5
            #     self.y = -10.0
            #     if self.x == 0.0:
            #         self.theta -=math.pi/2
            # elif self.x == 0.0 and self.y < 0.0:
            #     self.y += 0.5
            #     self.x = 0.0
            #     if self.y == 0.0:
            #         self.theta -=math.pi/2
                
###########################################################################################################################
            self.rate.sleep()
       
        
    def linearvelocity(self):

        #calculate the speed using average of 4 wheels
        self.velocity_x = (self.speed_bl + self.speed_br + self.speed_fl + self.speed_fr)*0.25
        # print("Self.velocity is: ",self.velocity_x)


        if self.velocity_x is not None:
            
            # self.theta = self.theta + (self.yaw_rate*self.dt)
            self.theta = self.theta + (self.yaw_gyro*self.dt)
            self.x = self.x + self.velocity_x*math.cos(self.theta)*self.dt
            self.y = self.y + self.velocity_x*math.sin(self.theta)*self.dt

            
        self.q = quaternion_from_euler(0.0, 0.0, self.theta)#function used to convert euler angles to quaternions
        self.odom = Odometry()
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation.x = self.q[0]
        self.odom.pose.pose.orientation.y = self.q[1]
        self.odom.pose.pose.orientation.z = self.q[2]
        self.odom.pose.pose.orientation.w = self.q[3]
        self.odom.twist.twist.linear.x = self.velocity_x
        self.odom.twist.twist.angular.z = self.yaw_rate
        self.odom.header.stamp = rospy.Time.now()
        self.odom.header.frame_id = "/map"
        self.odom.child_frame_id = "/base_link"

        




        

        




    def linearvelocity_gps(self):

        self.q_gps = quaternion_from_euler(0.0, 0.0, 0.0)#function used to convert euler angles to quaternions
        self.odometer_gps = Odometry()
        self.odometer_gps.pose.pose.position.x = self.gps_x_new
        self.odometer_gps.pose.pose.position.y = self.gps_y_new
        self.odometer_gps.pose.pose.position.z = 0.0
        self.odometer_gps.pose.pose.orientation.x = self.q_gps[0]
        self.odometer_gps.pose.pose.orientation.y = self.q_gps[1]
        self.odometer_gps.pose.pose.orientation.z = self.q_gps[2]
        self.odometer_gps.pose.pose.orientation.w = self.q_gps[3]
        self.odometer_gps.twist.twist.linear.x = 0.0
        self.odometer_gps.twist.twist.angular.z = 0.0
        self.odometer_gps.header.stamp = rospy.Time.now()
        self.odometer_gps.header.frame_id = "/map"
        self.odometer_gps.child_frame_id = "/base_link"

        self.gps_x,self.gps_y = self.longitude_latitude_to_x_y_z(self.latitude_var, self.longitute_var)
        print("GPS x is", self.gps_x,"GPS y is",self.gps_y)

        
        # self.theta = self.theta + (self.yaw_rate*self.dt)
        # self.theta = self.theta + (self.yaw_gyro*self.dt)

        # since we have a very big value of x in beginning, like "152716.27356009468,"
        # we should set the before value to this value for first iteration
        # then we change the flag
        if self.gps_y != 0 and self.gps_x !=0 and self.flag == 0.0:
            self.flag = 1.0
            self.x_before_gps = self.gps_x
            self.y_before_gps = self.gps_y
            print("self.x_before_gps: ",self.x_before_gps,"self.y_before_gps: ",self.y_before_gps)

        # elif self.gps_x<165700:
        else:
            #in the beginning we have very large value of x and y is zero. These values cause wrong initialization of gps_x_new and 
            #gps_y_new. So we introduce a condition to prevent these zero values. 
            if self.gps_y != 0 and self.gps_x !=0 and self.x_before_gps != 0 and self.y_before_gps != 0:
                self.gps_delta_x = self.gps_x - self.x_before_gps
                self.gps_delta_y = self.gps_y - self.y_before_gps

            self.gps_x_new = self.gps_x_new + self.gps_delta_x
            self.gps_y_new = self.gps_y_new + self.gps_delta_y

            print("self.gps_x_new: ",self.gps_x_new,"self.gps_y_new: ",self.gps_y_new, "theta is: ",self.theta_gps)



            self.x_before_gps = self.gps_x
            self.y_before_gps = self.gps_y



        

        

    # def lonlat2xyz(self,lat, lon):
    #     x=[]
    #     y=[]
    #     for time in range(0,len(lat)):
    #         latitude=lat[time]
    #         longitude=lon[time]
            
    #         # WGS84 ellipsoid constants:
    #         a = 6378137
    #         e = 8.1819190842622e-2
    #         #Urbana Altitude in meters
    #         altitude = 222
    #         #(prime vertical radius of curvature)
    #         N = a / math.sqrt(1 - e*e * math.sin(math.radians(latitude))* math.sin(math.radians(latitude)))
    #         x.append((N+altitude) * math.cos(math.radians(latitude)) * math.cos(math.radians(longitude)))
    #         y.append((N+altitude) * math.cos(math.radians(latitude)) * math.sin(math.radians(longitude)))
    #         #z = ((1-e*e) * N + altitude) * math.sin(math.radians(latitude))
        
    #     minx = min(x)
    #     miny = y[x.index(minx)]
        
    #     for t in range(0, len(x)):
    #         x[t] = x[t]-minx
    #         y[t] = y[t]-miny
        
    #     return x, y

    def longitude_latitude_to_x_y_z(self,latitude,longitude):
                   
        # WGS84 ellipsoid constants:
        a = 6378137
        e = 8.1819190842622e-2
        #Urbana Altitude in meters
        altitude = 222
        #(prime vertical radius of curvature)
        N = a / math.sqrt(1 - e*e * math.sin(math.radians(latitude))* math.sin(math.radians(latitude)))
        x = (N+altitude) * math.cos(math.radians(latitude)) * math.cos(math.radians(longitude))
        y = (N+altitude) * math.cos(math.radians(latitude)) * math.sin(math.radians(longitude))
        #z = ((1-e*e) * N + altitude) * math.sin(math.radians(latitude))
        
        return x, y

    def callback(self,msg):
        listenerNode.x_accel = msg.data
        # rospy.loginfo("x_accel %s", self.x_accel)

    def callback1(self,msg):
        listenerNode.y_accel = msg.data
        # rospy.loginfo("y_accel %s", self.y_accel)

    def callback2(self,msg):
        listenerNode.z_accel = msg.data
        # rospy.loginfo("z_accel %s", self.z_accel)



    def callback3(self,msg):
        listenerNode.speed_bl = msg.data
        # rospy.loginfo("speed_bl %s", self.speed_bl)

    def callback4(self,msg):
        listenerNode.speed_br = msg.data
        # rospy.loginfo("speed_br %s", self.speed_br)

    def callback5(self,msg):
        listenerNode.speed_fl = msg.data
        # rospy.loginfo("speed_fl %s", self.speed_fl)

    def callback6(self,msg):
        listenerNode.speed_fr = msg.data
        # rospy.loginfo("speed_fr %s", self.speed_fr)




    def callback7(self,msg):
        listenerNode.pitch_gyro = msg.data
        # rospy.loginfo("pitch_gyro %s", self.pitch_gyro)

    def callback8(self,msg):
        listenerNode.roll_gyro = msg.data
        # rospy.loginfo("roll_gyro %s", self.roll_gyro)

    def callback9(self,msg):
        listenerNode.yaw_gyro = msg.data
        # rospy.loginfo("yaw_gyro %s", self.yaw_gyro)



    def callback10(self,msg):
        listenerNode.timestamp_var = msg.data
        # rospy.loginfo("timestamp_var %s", self.timestamp_var)


    def callback11(self,msg):
        listenerNode.latitude_var = msg.data
        # rospy.loginfo("latitude_var %s", self.latitude_var)

    def callback12(self,msg):
        listenerNode.longitute_var = msg.data
        # rospy.loginfo("longitute_var %s", self.longitute_var)

    def callback13(self,msg):
        listenerNode.clock = msg.data
        # rospy.loginfo("clock %s", self.clock)

    # def callback14(self,msg):
    #     listenerNode.x_accel = msg.data
    #     rospy.loginfo("number %s", self.x_accel)

 
# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    
    rospy.init_node('odom_example_node', anonymous = True)
    # Go to the main loop.
    ne = listenerNode()
    ne.run()

