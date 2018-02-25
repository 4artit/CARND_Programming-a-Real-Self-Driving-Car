#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below        
        self.base_waypoints = None
        self.base_wp_size = 0
        self.has_base_wp = False
        self.is_initialized = False
        self.current_pos_index = 0

        rospy.spin()


    def pose_cb(self, msg):
        # Check has base waypoints and find min distance waypoint
        if self.has_base_wp is True:
	    index = self.current_pos_index
	    diff_x = msg.pose.position.x - self.base_waypoints[index].pose.pose.position.x
            diff_y = msg.pose.position.y - self.base_waypoints[index].pose.pose.position.y
            min_dist = diff_x ** 2 + diff_y ** 2
            waypoint_size = 0
        # If is not initialized, check all waypoints/ initialized just check in before final_waypoints
            if self.is_initialized is False:
                waypoint_size = self.base_wp_size
                self.is_initialized = True
            else:
                waypoint_size = LOOKAHEAD_WPS
            for i in range(1, waypoint_size):
                diff_x = msg.pose.position.x - self.base_waypoints[i].pose.pose.position.x
                diff_y = msg.pose.position.y - self.base_waypoints[i].pose.pose.position.y
                dist = diff_x ** 2 + diff_y ** 2
                if dist < min_dist:
                    min_dist = dist
                    index = i

        # Check min distance is forward waypoint from car using inner product space
            if index + 1 == self.base_wp_size:
                next_index = 0
            else:
                next_index = index + 1
            a_vector_x = self.base_waypoints[next_index].pose.pose.position.x - self.base_waypoints[index].pose.pose.position.x
            a_vector_y = self.base_waypoints[next_index].pose.pose.position.y - self.base_waypoints[index].pose.pose.position.y
            b_vector_x = msg.pose.position.x - self.base_waypoints[index].pose.pose.position.x
            b_vector_y = msg.pose.position.y - self.base_waypoints[index].pose.pose.position.y
            cos_theta = (a_vector_x * b_vector_x + a_vector_y * b_vector_y) / (math.sqrt(a_vector_x ** 2 + a_vector_y ** 2) * math.sqrt(b_vector_x ** 2 + b_vector_y ** 2))
            if cos_theta >= 0:
                index = next_index
            self.current_pos_index = index
        # publish final waypoints
            final_waypoints = Lane()
            final_waypoints.waypoints = []
        
            for i in range(LOOKAHEAD_WPS):
                final_waypoints.waypoints.append(self.base_waypoints[(index + i)%self.base_wp_size])
            self.final_waypoints_pub.publish(final_waypoints)


    def waypoints_cb(self, waypoints):
	# Save base waypoints and waypoints list size
        self.base_waypoints = waypoints.waypoints
        self.base_wp_size = len(waypoints.waypoints)
        if self.has_base_wp is False:
            self.has_base_wp = True

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

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
