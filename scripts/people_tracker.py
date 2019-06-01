#!/usr/bin/env python

import rospy

from leg_tracker.msg import Person,PersonArray
from std_msgs.msg import Int32
import numpy as np
import math
from geometry_msgs.msg import Twist,PointStamped
import tf

class PeopleTracker:
    """
    This class is tracker a people with a constant ID
    """
    def __init__(self):
        """
        Constructor
        """
        self.people_topic = rospy.get_param("people_topic","/people_tracked")
        self.base_frame = rospy.get_param("base_frame","/base_footprint")
        self.laser_frame = rospy.get_param("laser_frame","/laser")
        self.velocity_topic = rospy.get_param("velocity_topic","/cmd_vel")
        self.track_select_topic = rospy.get_param("tracker_selection_frame","/tracker_id")
        self.max_search_radius = rospy.get_param("max_search_radius",0.5)
        self.listener = tf.TransformListener()
        self.isTracked = False # flag: it respresent whether the tracker's ID is identify
        self.trackID = -1 # tracker's ID initialized with -1
        self.isFind = False
        self.pose = PointStamped()
        #ROS Publisher
        self.velocity_pub = rospy.Publisher(self.velocity_topic,Twist,queue_size=10)

        #ROS subscriber
        self.people_sub = rospy.Subscriber(self.people_topic,PersonArray,self.people_trakc_callback)
        self.tracker_id_sub = rospy.Subscriber(self.track_select_topic,Int32,self.tracker_change)
        rospy.spin() #run the node
    
    def tracker_change(self,data):
        """
        change the tracker id
        """
        self.trackID = data.data
    
    def people_trakc_callback(self,person_array_msg):
        """
        When detect personarray message,it will be execute.
        """
        pose = PointStamped()
        pose.header.frame_id = person_array_msg.header.frame_id
        pose.header.stamp = person_array_msg.header.stamp
        now = person_array_msg.header.stamp
        fixed_frame = person_array_msg.header.frame_id
        
        if(not self.isTracked):
            if(len(person_array_msg.people) > 0):
                self.trackID = person_array_msg.people[0].id
                self.isTracked = True
            else:
                print "No people detected!"
            return
        else:
            print self.trackID
            self.isFind = False
            for person in person_array_msg.people:
                if person.id == self.trackID:
                    pose.point.x = person.pose.position.x
                    pose.point.y = person.pose.position.y
                    self.isFind = True
                    self.pose.point.x = pose.point.x
                    self.pose.point.y = pose.point.y
                    break
            if not self.isFind:
                for person in person_array_msg.people:
                    d = ((person.pose.position.x - self.pose.point.x)**2 + (person.position.point.y - self.pose.point.y)**2)** (1./2.)
                    if d < self.max_search_radius:
                        pose.point.x = person.pose.position.x
                        pose.point.y = person.pose.position.y
                        self.isFind = True
                        self.pose.point.x = pose.point.x
                        self.pose.point.y = pose.point.y
                        self.trackID = person.id
                        break
        velocity = Twist()
        if not self.isFind:
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            self.velocity_pub.publish(velocity)
            return
        dist = (pose.point.y ** 2 + pose.point.x ** 2)** 1./2.
        if(dist < 0.5):
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            self.velocity_pub.publish(velocity)
        else:
            dist_away = -0.7
            theta = math.atan2(pose.point.y,pose.point.x)
            dist_x = dist_away * math.cos(theta)
            dist_y = dist_away * math.sin(theta)
            pose.point.x = pose.point.x + dist_x
            pose.point.y = pose.point.y + dist_y
            velocity.linear.x = 0.6#0.1 + 0.2 * math.sqrt(pose.point.x ** 2 + pose.point.y ** 2)
            if theta < 0.32 and theta > -0.32:
                velocity.angular.z = 0
            elif theta >= 0.32:
                velocity.linear.x = 0.0
                velocity.angular.z = 0.4 # * theta #math.atan2(pose.point.x,pose.point.y)
            else:
                velocity.linear.x = 0.0
                velocity.angular.z = -0.4
            # print "vel: " ,velocity.linear.x, "angular: ",velocity.angular.z
            self.velocity_pub.publish(velocity)

if __name__ == '__main__':
    rospy.init_node('people_tracker', anonymous=True)
    people_tracker =PeopleTracker()
