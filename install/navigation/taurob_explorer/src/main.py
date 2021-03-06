#!/usr/bin/env python
import yaml

from taurob_explorer.rrt import RRT

import rospy
import rospkg
from actionlib_msgs.msg import GoalStatusArray
from mbf_msgs.msg import ExePathActionFeedback

status_dict={
    0 : "No Goal",
    1 : "Active",
    2 : "Preempted",
    3 : "Succeeded",
    4 : "Aborted",
    5 : "Rejected",
    6 : "Preempting",
    7 : "Recalling",
    8 : "Recalled",
    9 : "Lost"
}

class Explorer():
    def __init__(self):
        self.status=None
        self.goal_reached_cnt=0
        self.dist_to_goal=None
        self.angle_to_goal=None

        self.sub_mbf_feedback = rospy.Subscriber("/move_base_flex/exe_path/status/", GoalStatusArray, self.callback_mbf_status)
        self.sub_mbf_path_feedback = rospy.Subscriber("/move_base_flex/exe_path/feedback", ExePathActionFeedback, self.callback_mbf_path)


        self.change_angle_to_goal_param()


    def callback_mbf_path(self, msg):
        self.angle_to_goal = msg.feedback.angle_to_goal
        self.dist_to_goal = msg.feedback.angle_to_goal

    def callback_mbf_status(self, msg):
        if len(msg.status_list):
            if msg.status_list[len(msg.status_list)-1].status == 3 and self.goal_reached_cnt > 25:# Move_base_flex sends 27 goal reached messages
                self.status = msg.status_list[len(msg.status_list)-1].status
                self.goal_reached_cnt = 0
            elif msg.status_list[len(msg.status_list)-1].status == 3:
                self.goal_reached_cnt += 1
            else:
                self.status=msg.status_list[len(msg.status_list)-1].status
        else:
            self.status = 0


    def change_angle_to_goal_param(self):
        rospy.set_param("/move_base_flex/dwa/yaw_goal_tolerance", 360)
        rospy.set_param("/move_base_flex/eband/yaw_goal_tolerance", 360)
        rospy.set_param("/move_base_flex/teb/yaw_goal_tolerance", 360)



if __name__ == "__main__":
    rospack = rospkg.RosPack()
    path = rospack.get_path('taurob_explorer')
    with open (path+"/config/config.yaml", 'r') as file:
        try:
            config = yaml.safe_load(file)
            map_frame=config["map_frame"]
            robot_frame=config["robot_frame"]
            timeout=config["timeout"]
            rand_angle=config["rand_angle"]
            max_dist=config["max_dist"]
            max_iter=config["max_iter"]
            max_reject=config["max_reject"]
            show_marker=bool(config["show_marker"])
            show_nodes=bool(config["show_nodes"])
            range_visited_node=config["range_visited_node"]
        except yaml.YAMLError as exc:
            print(exc)
            map_frame = rospy.get_param("/map_frame")
            robot_frame = rospy.get_param("/robot_frame")
            timeout = rospy.get_param("/timeout")
            rand_angle = rospy.get_param("/rand_angle")
            max_dist = rospy.get_param("/max_dist")
            max_iter = rospy.get_param("/max_iter")
            max_reject = rospy.get_param("/max_reject")
            show_marker = rospy.get_param("/show_marker")
            show_nodes = rospy.get_param("/show_nodes")
            range_visited_node=rospy.get_param("range_visited_node")



    rospy.init_node("taurob_explorer", log_level=rospy.INFO)
    rospy.loginfo("Init node {} finnished".format(rospy.get_name()))

    explorer = Explorer()
    rrt = RRT(map_frame, robot_frame, [0.0, 0.0, 0.0], rand_angle, max_dist, timeout, show_marker, show_nodes, max_reject, range_visited_node)


    if not rospy.is_shutdown():
        rrt.plan()
        rospy.sleep(2)
    start = rospy.Time.now()
    while not rospy.is_shutdown():

        if (rospy.Time.now() - start <= rospy.Duration(timeout)) and (explorer.status != 3 or explorer.status != 4 or explorer.status != 1):
            rospy.logdebug_throttle(3, "Status= {}".format(explorer.status))
            if (explorer.status == 3 and explorer.dist_to_goal <= 0.1):
                rospy.loginfo("Reached goal")
                rospy.sleep(1)
                if (rrt.iter_cnt <= max_iter):
                    start = rospy.Time.now()
                    rrt.plan()
            else:
                if (explorer.status == 1):
                    rospy.logdebug_throttle(3, "Executing current goal (x:{}, y:{})".format(rrt.goal.pose.position.x, rrt.goal.pose.position.y))
                elif(2 <= explorer.status <=9 ):
                    rospy.logwarn_throttle(0.5, "Explorer node currently has status: {}".format(status_dict[explorer.status]))
                    if (explorer.status == 4):
                        start = rospy.Time.now()
                        rrt.plan()


        else:
            rospy.logwarn("Execution of current goal took too long --> Replanning")
            rrt.plan()
            start = rospy.Time.now()

