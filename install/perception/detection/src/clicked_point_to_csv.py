#!/usr/bin/env python
import rospy
import rosnode
import rospkg
from geometry_msgs.msg import PointStamped

import csv



rospack = rospkg.RosPack()
path=rospack.get_path('detection')


def callback_point(msg):
    point=msg.point
    with open(path+"/clicked_humans.csv", 'a') as file:
        writer=csv.writer(file)
        row=[point.x, point.y]
        writer.writerow(row)
    file.close()
    rospy.loginfo("Saving point: \n x= %s\n y= %s", str(point.x), str(point.y) )


def main():
    rospy.init_node("clicked_point_csv", log_level=rospy.INFO)
    rospy.Subscriber('/clicked_point', PointStamped, callback_point)

    rospy.spin()



if __name__=="__main__":
    main()

