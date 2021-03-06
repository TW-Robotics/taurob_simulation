#!/usr/bin/env python

import rospy

from maploader.srv import MarkMap, MarkMapResponse
import os
from PIL import Image, ImageDraw, ImageFont
from math import sin, cos, atan, degrees, radians
import cv2
import numpy
from PSpincalc import PSpincalc
from std_msgs.msg import Int16

angle_from_quaternion = 0.0
correction_angle = 0.0

def callback_angle(data):
    correctangle(float(data.data))

def correctangle(input_angle):
    global correction_angle
    global angle_from_quaternion
    correction_angle = angle_from_quaternion *(-1) + input_angle

def callback(request):
    global angle_from_quaternion
    global correction_angle
    deleteimage = bool(rospy.get_param('delete', 'False'))
    image_to_edit = request.input_file_location
    w = request.w
    x = request.x
    y = request.y
    z = request.z

    angle_from_quaternion = degrees(quaternion2angle(w, x, y, z))
    corrected_angle = angle_from_quaternion + correction_angle
    print("service_server: corrected angle = " + repr(corrected_angle))

    try:
        edited_image = drawmap(image_to_edit, radians(corrected_angle))
    except:
        return MarkMapResponse("ERROR2")
    return MarkMapResponse(edited_image)


def drawmap(filename, angle_radians):
    image = Image.open(filename)
    splitpath = os.path.split(filename)
    # add 'ed_' to front of filename to express that it is the edited version
    temppath = os.path.join(splitpath[0] + '/ed_' + splitpath[1])
    # change folder from 'loaded' to 'edited'
    newpath = temppath.replace('/loaded', '/edited')

    width = image.size[0]
    height = image.size[1]
    center_x = width/2
    center_y = height/2

    draw = ImageDraw.Draw(image)

    # add watermark
    draw.rectangle(((0,height - 25), (155,height)), fill='gray')

    font = ImageFont.truetype('FreeSans.ttf', size=20)
    (x,y) = (5,height - 24)
    message = "Powered by Esri"
    color = 'rgb(0,0,0)'
    draw.text((x,y), message, fill=color, font=font)

    # add dot to show position of robot
    draw.ellipse((center_x-25,center_y-25,center_x+25,center_y+25), fill = 'blue', outline ='blue')

    # draw line to show orientation of robot
    hor = cos(angle_radians) * 100
    ver = sin(angle_radians) * -100
    draw.line((center_x, center_y, center_x+hor , center_y+ver), fill='red', width = 10)

    image.save(newpath)
    return newpath

def quaternion2angle(w, x, y, z):
    quaternion = numpy.array([w, x, y, z])
    ea = PSpincalc.Q2EA(quaternion, EulerOrder="zyx", ignoreAllChk=True)[0]
    print("PSpincalc: angle = " + repr(degrees(ea[0])))
    return ea[0]

if __name__ == '__main__':
    rospy.init_node('maploader_serviceserver')
    service = rospy.Service('mark_map', MarkMap, callback)

    # get orientation of the robot at startup (in degrees)
    # 0/360 = East, 90 = North, 180 = West, 270 = South
    angle = int(rospy.get_param('angle_calibration', '0'))
    if angle != 0:
        correctangle(angle)

    # get orientation of the robot while operating
    rospy.Subscriber('angle_correction', Int16, callback_angle)
    rospy.spin()
