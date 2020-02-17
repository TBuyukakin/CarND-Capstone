#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree


import math


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
#decelMax = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        """
        Initialize variables before callback is called
        """
        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.traffic = -1
        self.decelMax = 0.5
        
        
        """
        Arrenge publishing frequency less than 50 since Autoware
        works on 50
        """
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.pose and self.waypoint_tree:
                final_lane = self.closestWp2Lane()
                self.final_waypoints_pub.publish(final_lane)
                
            rate.sleep()
    
    def closestWp2Lane(self):
        """
        Obtain x,y pose points from custom message type Lane
        Use query to return position and index 
        query: 
        1st arg ... an array of points to query.
        2nd arg ... the number of nearest neighbors to return.
        returns:
           d: the distances to the nearest neighbors.
           i: the locations of the neighbors in self.data.
        """
        
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        indexClosest = self.waypoint_tree.query([x,y], 1)[1]

        """
        Obtain the index of closest waypoint and the previos point
        Ensure the point is in front of the current point by checing with
        dot product calculation. If dot product > 0 it is behind and 
        take the closest which is next one.
        
        """
        ptClosest = self.waypoints_2d[indexClosest]
        ptPrev = self.waypoints_2d[indexClosest - 1]

        if np.dot(np.array(ptClosest) - np.array(ptPrev), np.array([x,y]) - np.array(ptClosest)) > 0:
            indexClosest = (indexClosest + 1) % len(self.waypoints_2d)
        
        """
        Create lane object
        Check if traffic light waypoint is further than created waypoints,
        no need for slowing down. Use created waypoints and dont care about
        traffic lights. Otherwise car needs to slow down.
        
        """
        lane = Lane()
        lane.header = self.base_waypoints.header
        
        createdWaypoints = self.base_waypoints.waypoints[indexClosest:(indexClosest + LOOKAHEAD_WPS)]
        
        if self.traffic == -1 or (self.traffic >= (indexClosest + LOOKAHEAD_WPS)):
            lane.waypoints = createdWaypoints
        else:
            lane.waypoints = self.decelWaypoints(createdWaypoints, indexClosest)

        return lane  


    def decelWaypoints(self, waypoints, indexClosest):
        """
        Create a new list of waypoints
        To arrenge safe stoppic distance take the 3 previous index to stop
        on time behind the line.
        Calculate the stoppind distance by using "distance" function
        Obtain the velocity. Decreasing level is related with the distance.
        If distance is smaller, velocity should be smaller too to be prepared
        to stop on time. Multiply but constant to have a linear deceleration
        If velocity is small enough make the car stop.
        Append it to waypoint object linear twist
        """
        temp = []
        indexStop = max(self.traffic - indexClosest - 3, 0)
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            distStop = self.distance(waypoints, i, indexStop)
            vel = math.sqrt(2 * self.decelMax * distStop)
            if vel < 1.:
                vel = 0.

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        """
        Log info to better check on cmd window
        """
        d = self.distance(waypoints, 0, indexStop)
        rospy.loginfo('target vel={}, distance to the stopline={}'.format(temp[0].twist.twist.linear.x, d))
        return temp

    def pose_cb(self, msg):
        """
        # Callback function of subscriber to Car's pose
        """
        self.pose = msg

    def waypoints_cb(self, waypoints):
        """
        # Callback function of subscriber to waypoints
        Store waypoints in in object
        Latched subscriber -> callback is called once 
        In order to find closest waypoint to the car use KDTree from scipy
        Search through waypoints
        Obtain waypoints x and y and use on KDTree
        """
        self.base_waypoints = waypoints
     
        ref_v = waypoints.waypoints[0].twist.twist.linear.x
        self.decelMax = ref_v * ref_v / 20. # Begin applying the brake about 20m before
        rospy.logwarn('MAX_DECEL={}'.format(self.decelMax))
        
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        """
        # Callback function of traffic light waypoint
        """
        self.traffic = msg.data

    
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    """
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity
    """
    
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')