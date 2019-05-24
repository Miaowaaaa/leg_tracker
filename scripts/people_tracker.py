#!/usr/bin/env python

import rospy

from leg_tracker.msg import Person,PersonArray
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
        self.isTracked = False # flag: it respresent whether the tracker's ID is identify
        self.trackID = -1 # tracker's ID initialized with -1

        #ROS Publisher
        self.velocity_pub = rospy.Publisher(self.velocity_topic,Twist,queue_size=10)

        #ROS subscriber
        self.people_sub = rospy.Subscriber(self.people_topic,PersonArray,self.people_trakc_callback)
        
        rospy.spin() #run the node
    
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
            for person in person_array_msg.people:
                if person.id == self.trackID:
                    pose.point.x = person.pose.position.x
                    pose.point.y = person.pose.position.y
        dist = (pose.point.y ** 2 + pose.point.x ** 2)** 1./2.
        print "dist is :", dist
        velocity = Twist()
        if(dist > 0.3 and dist < 0.5):
            velocity.linear.x = 0.0
            velocity.angular.z = 0.0
            self.velocity_pub.publish(velocity)
        else:
            dist_away = -0.5
            theta = math.atan2(pose.point.y,pose.point.x)
            dist_x = dist_away * math.cos(theta)
            dist_y = dist_away * math.sin(theta)
            print "dist:",dist_x, dist_y
            pose.point.x = pose.point.x + dist_x
            pose.point.y = pose.point.y + dist_y
            velocity.linear.x = 0.1 + 0.2 * math.sqrt(pose.point.x ** 2 + pose.point.y ** 2)
            if theta < 0.32 and theta > -0.32:
                velocity.angular.z = 0
            else:
                velocity.angular.z = 4 * theta #math.atan2(pose.point.x,pose.point.y)
            print "vel: " ,velocity.linear.x, velocity.angular.z
            self.velocity_pub.publish(velocity)

if __name__ == '__main__':
    rospy.init_node('people_tracker', anonymous=True)
    people_tracker =PeopleTracker()
