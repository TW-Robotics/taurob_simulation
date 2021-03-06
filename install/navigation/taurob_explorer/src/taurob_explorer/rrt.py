#!/usr/bin/env python
import random
import math
import threading
from time import sleep

import rospy
import tf2_ros
import tf2_py as tf2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray



class Node():
    def __init__(self, x, y, th):
        self.x = x
        self.y = y
        self.th = th
        self.node_cnt = 0

    def __str__(self):
        rospy.logdebug("Node Number: {} \nx : {}\ny : {}\nth : {}\n".format(self.node_cnt, self.x, self.y, self.th))

    def __getitem__(self, nr):
        if nr == "x":
            return self.x
        elif nr == "y":
            return self.y
        elif nr == "th":
            return self.th
        elif nr == "node_cnt":
            return self.node_cnt
        else:
            raise Exception('Class Node has no {}. variable'.format(nr))


class RRT():
    def __init__(self, map_frame, target_frame, start, rand_angle, max_dist, timeout, show_marker, show_nodes, max_reject, range_visited_node):
        self.map_frame = map_frame
        self.target_frame = target_frame
        self.start = Node(start[0], start[1], start[2])
        self.goal = PoseStamped()
        self.rand_angle = rand_angle
        self.max_dist = max_dist
        self.max_reject = max_reject
        self.timeout = timeout
        self.visited_nodes = []
        self.range_visited_node = range_visited_node
        self.show_marker = show_marker
        self.show_nodes = show_nodes
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_goal_marker = rospy.Publisher('/visualization_marker', Marker, queue_size=1)
        self.pub_marker_nodes = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(self.timeout))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)

        self.iter_cnt = 0
        self.node_cnt = 0
        self.reject_cnt = 0


    # ------------------#
    # Helper functions #
    # ------------------#
    def update(self):
        """
        Updates the config params of rrt | will be replaced with rqt_reconfigure
        """
        try:
            self.rand_angle = rospy.get_param("/rand_angle")
            self.timeout = rospy.get_param("/timeout")
            self.max_dist = rospy.get_param("/max_dist")
        except KeyError:
            rospy.logdebug("KeyError when updating -> No Rosparams set")


    def get_transfrom(self):
        """
        Calculates the transformation from self.map_frame to self.target_frame
        :return Returns state, a Node Object, if transformation is available or None if no tf available
        """
        #rospy.sleep(0.5)
        try:
            rospy.logdebug("Transforming pose from {} to {}".format(self.target_frame, self.map_frame))
            self.trans = self.tf_buffer.lookup_transform(self.map_frame, self.target_frame, rospy.Time(),
                                                         rospy.Duration(
                                                             10))  # get transformation from target_frame to map_frame
            theta = euler_from_quaternion(
                [self.trans.transform.rotation.x, self.trans.transform.rotation.y, self.trans.transform.rotation.z,
                 self.trans.transform.rotation.w])[2] * 180 / math.pi
            state = Node(self.trans.transform.translation.x, self.trans.transform.translation.y, theta)
            return state

        except tf2.LookupException as ex:
            rospy.logwarn_throttle(3, ex)
            rospy.logwarn_throttle(3, "Could not get transformation from {} to {}".format(self.target_frame,
                                                                                          self.map_frame))
            return None

    def compare_nodes(self, node):
        """
        Compares the current sampled goal with the already visitied nodes
        :param node: Is the current sampled goal
        :return: True if new node is not near visited nodes or if last self.reject_cnt nodes are rejected for being too near, else False
        """
        for visited_node in self.visited_nodes:
            if (abs(visited_node["x"] - node.pose.position.x) < (self.range_visited_node - self.range_visited_node * 0.7) and
                    abs(visited_node["y"] - node.pose.position.y) < (self.range_visited_node - self.range_visited_node * 0.7)) and (self.reject_cnt < self.max_reject):
                rospy.logwarn("Goal rejected since it is too close to a visited node")
                self.reject_cnt += 1
                return False
            elif self.reject_cnt == self.max_reject:
                self.max_dist=self.max_dist*4
            else:
                try:
                    self.max_dist = rospy.get_param("/max_dist")
                except KeyError:
                    rospy.logdebug("KeyError when updating -> No Rosparams set")
                self.reject_cnt = 0
        return True

    def pub_marker_goal(self):
        """
        Publishes a red cube at current goal
        """
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "marker_goal"
        marker.action = marker.ADD
        marker.type = marker.CUBE
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 0.4
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.pose = self.goal.pose
        self.pub_goal_marker.publish(marker)

    def pub_marker_array(self):
        """
        Publishes a MarkerArray of all visited nodes
        """
        markerArray = MarkerArray()
        cnt = 0
        for node in self.visited_nodes:
            marker = Marker()
            marker.id=cnt
            marker.header.frame_id = self.map_frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "marker_nodes"
            marker.action = marker.ADD
            marker.type = marker.CUBE
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.a = 0.2
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.pose.position.x = node["x"]
            marker.pose.position.y = node["y"]
            marker.pose.position.z = 0.0

            marker_circ = Marker()
            marker_circ.id = cnt*1000
            marker_circ.header = marker.header
            marker_circ.ns = "marker_nodes"
            marker_circ.action = marker_circ.ADD
            marker_circ.type = marker_circ.CYLINDER
            marker_circ.scale.x = self.range_visited_node - self.range_visited_node * 0.7
            marker_circ.scale.y = self.range_visited_node - self.range_visited_node * 0.7
            marker_circ.scale.z = 0.5
            marker_circ.color.a = 0.2
            marker_circ.color.r = 0.0
            marker_circ.color.g = 0.0
            marker_circ.color.b = 0.6
            marker_circ.pose.position.x = node["x"]
            marker_circ.pose.position.y = node["y"]
            marker_circ.pose.position.z = 0.0

            if (cnt > len(self.visited_nodes)):
                markerArray.markers.pop(0)
            markerArray.markers.append(marker)
            markerArray.markers.append(marker_circ)
            cnt += 1
        self.pub_marker_nodes.publish(markerArray)


    # --------------------#
    # Explorer functions #
    # --------------------#
    def append_nodes(self):
        """
        Get current pose and put it in visited list
        """
        self.visited_nodes.append(self.get_transfrom())
        self.node_cnt += 1

    def sample(self):
        """
        Samples goal from space around robot and transforms this goal in tf of map
        """
        rand_dist = random.uniform(self.max_dist / 10, self.max_dist)
        curr_angle = self.start["th"]
        rand_angle = random.uniform(self.rand_angle / 2, 90 + self.rand_angle / 2)

        goal_angle = rand_angle / 10 + curr_angle
        rospy.logdebug("Angles are \n Rand_angle: {} \n Curr_angle: {} \n Goal_angle: {}".format(rand_angle, curr_angle,
                                                                                                 goal_angle))
        x_ = math.sin(rand_angle * math.pi / 180) * rand_dist
        y_ = math.cos(rand_angle * math.pi / 180) * rand_dist

        point_temp = PoseStamped()
        point_temp.header.frame_id = self.target_frame
        point_temp.pose.position.x = x_
        point_temp.pose.position.y = y_
        point_temp.pose.position.z = 0.0
        temp = quaternion_from_euler(0.0, 0.0, math.radians(goal_angle))
        point_temp.pose.orientation.x = temp[0]
        point_temp.pose.orientation.y = temp[1]
        point_temp.pose.orientation.z = temp[2]
        point_temp.pose.orientation.w = temp[3]
        try:
            goal = do_transform_pose(point_temp, self.trans)
            rospy.logdebug("Got Goal")  #: \n{}\n{}".format(self.goal.pose.position, self.goal.pose.orientation))
            return goal
        except tf2.LookupException as ex:
            rospy.logwarn_throttle(3, ex)
            rospy.logwarn_throttle(3, "Could not get transformation from {} to {}".format(self.target_frame,
                                                                                          self.map_frame))
            return None

    def move(self):
        """
        Publishes the current goal
        """
        self.goal.header.frame_id = self.map_frame
        self.pub_goal.publish(self.goal)


    def plan(self):
        """
        Executes all steps necessary for sampling and planning
        """
        # ---------------------------------------------#
        # Get current pose and put it in visited list #
        # --------------------------------------------#
        self.append_nodes()
        self.update()
        self.start = self.get_transfrom()
        if not self.visited_nodes:  # Init visited list
            self.visited_nodes.append(self.start)
        # Generate new goal #
        goal = self.sample()
        if self.compare_nodes(goal):  # check if goal is near node in visited list
            self.goal = goal
            if self.show_marker:
                self.pub_marker_goal()
            if self.show_nodes:
                self.pub_marker_array()
            self.move()
        else:
            self.plan()
        self.iter_cnt += 1
