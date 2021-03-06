#!/usr/bin/env python
import rospy

import os
import cv2
import time
#from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from PIL import Image, ImageDraw, ImageFont
from maploader.srv import MarkMap
from sensor_msgs.msg import Image, NavSatFix
import actionlib
from maploader.msg import LoadMapAction, LoadMapGoal, LoadMapResult

# global variable to let mapmarker-service know which image not to edit because it is being downloaded
maploader_working = True
# global variable to store received GPS data to compute mean value
gps_data = []

def callback_maploader(data):
    global image_to_mark
    global maploader_working
    global gps_data

    # if GPS data is not valid abort loading the map
    if data.status == -1:
        print("client: GPS data invalid, aborting")
        return

    # wait until gps_data has 10 entries
    if len(gps_data) < 10:
        gps_data.append(data)
        return

    longitude = 0
    latitude = 0
    # compute mean value of received GPS values
    for i in range(len(gps_data)):
        longitude += gps_data[i].longitude
        latitude += gps_data[i].latitude

    longitude /= len(gps_data)
    latitude /= len(gps_data)

    res_x = int(rospy.get_param('res_x', '1920'))
    res_y = int(rospy.get_param('res_y', '1920'))
    delta = float(rospy.get_param('delta', '0.0025'))
    main_path = os.path.join(os.path.expanduser('~'), rospy.get_param('filepath', 'MapImages'))
    maptype = rospy.get_param('maptype', 'aerial')
    deleteimage = bool(rospy.get_param('delete', 'True'))

    if os.path.isdir(main_path) == False:
        print("client: filepath does not exist, creating filepath...")
        os.mkdir(main_path)
        print("client: ", os.path.isdir(main_path))

    client = actionlib.SimpleActionClient('loadmap', LoadMapAction)
    client.wait_for_server()

    print("client: received /fix")
    print("client: longitude: %f" %data.longitude)
    print("client: latitude: %f" %data.latitude)
    goal = LoadMapGoal()
    goal.longitude = data.longitude
    goal.latitude = data.latitude
    goal.delta = delta
    goal.resolution_x = res_x
    goal.resolution_y = res_y
    goal.folder = main_path
    goal.maptype = maptype
    maploader_working = True
    client.send_goal(goal)
    client.wait_for_result()
    image_to_mark = client.get_result()
    print("client: image saved")
    maploader_working = False

    if deleteimage == True:
        cleanup(main_path + "/loaded")

    gps_data = []

def callback_mapmarker(data):
    global maploader_working

    # 4 quaternion variables
    w = data.pose.pose.orientation.w
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z

    loaded_path = os.path.join(os.path.expanduser('~'), rospy.get_param('filepath', 'MapImages')) + "/loaded"

    latest_loaded = findlatest(loaded_path)
    print("client: latest_loaded = " + repr(latest_loaded))
    if latest_loaded == "ERROR1":
        print("client: no maps downloaded yet")
        return
    elif latest_loaded == "ERROR2":
        print("client: first map is being loaded, can not edit map yet")
        return

    deleteimage = bool(rospy.get_param('delete', 'True'))
    rospy.wait_for_service('mark_map')
    map_marker = rospy.ServiceProxy('mark_map', MarkMap)

    try:
        response = map_marker(latest_loaded, w, x, y, z)
    except:
        print("client: ERROR: file not ready")
        return

    ed_image_path = response.output_file_location

    if ed_image_path == 'ERROR1':
        print("client: ERROR: no maps downloaded yet")
        return
    elif ed_image_path == 'ERROR2':
        print("client: ERROR: file not ready")
        return
    elif ed_image_path == 'ERROR3':
        print("client: ERROR: server-function ended without sending path")
        return
    elif ed_image_path == 'ERROR4':
        print("client: ERROR: failed to open image, image deleted")
        return

    print("client: ed_image_path = " + repr(ed_image_path))
    # publishing image in ROS
    image_pub = cv2.imread(ed_image_path)
    publishmap(image_pub, image_publisher)

    if deleteimage == True:
        cleanup(main_path + '/edited')

def findlatest(loaded_path):
    global maploader_working

    # check if images have been downloaded
    if len(os.listdir(loaded_path)) <= 0:
        return "ERROR1"

    # find file changed last
    timestamps = []
    for root, dirs, files in os.walk(loaded_path):
        for basename in files:
            filename = os.path.join(loaded_path, basename)
            status = os.stat(filename)
            timestamps.append(status.st_mtime)

    timestamps.sort()

    if maploader_working == True and len(os.listdir(loaded_path)) <= 1:
        # maploader is working on the only available image, abort 
        return "ERROR2"

    if maploader_working == True:
        # edit 2nd latest file because latest file is being downloaded
        timestamp = timestamps[-2]
    else:
        # edit latest file because maploader is not working
        timestamp = timestamps[-1]

    # edit most recently loaded image 
    for root, dirs, files in os.walk(loaded_path):
        for basename in files:
            filename = os.path.join(loaded_path, basename)
            status = os.stat(filename)
            # find most recently changed file
            if status.st_mtime == timestamp:
                return filename

def publishmap(image, image_publisher):
    stamp = rospy.Time.from_sec(time.time())

    img_msg = Image()
    img_msg.height = image.shape[0]
    img_msg.width = image.shape[1]
    img_msg.step = image.strides[0]
    img_msg.encoding = 'bgr8'
    img_msg.header.frame_id = 'image_rect'
    img_msg.header.stamp = stamp
    img_msg.data = image.flatten().tolist()

    image_publisher.publish(img_msg)
    print("client: image published")

def cleanup(path):
    # if /edited should be cleaned up, delete all files in folder
    if path.endswith("edited"):
        print("client: deleting files in 'edited' folder")
        filelist = [f for f in os.listdir(path)]
        for f in filelist:
            os.remove(os.path.join(path, f))

    # if /loaded should be cleaned up, delete all files in folder except 2 most recent ones
    elif path.endswith("loaded"):
        print("client: deleting files in loaded folder")
        if len(os.listdir(path)) <= 3:
            return
        timestamps = []
        for root, dirs, files in os.walk(path):
            for basename in files:
                filename = os.path.join(path, basename)
                status = os.stat(filename)
                timestamps.append(status.st_mtime)
        timestamps.sort()

        for root, dirs, files in os.walk(path):
            for basename in files:
                filename = os.path.join(path, basename)
                status = os.stat(filename)
                # find most recently changed file
                if status.st_mtime != timestamps[-1] and status.st_mtime != timestamps[-2]:
                    os.remove(filename)

if __name__ == '__main__':
    # create directories
    main_path = os.path.join(os.path.expanduser('~'), rospy.get_param('filepath', 'MapImages'))
    use_ekf = bool(rospy.get_param('use_ekf', 'False'))
    odom_input = rospy.get_param('odom_input', 'odom')
    offline = bool(rospy.get_param("offline", "False"))
    gps_input = rospy.get_param("gps_input", "fix")

    try:
        # Create target Directory
        os.mkdir(main_path)
    except:
        pass
    try:
        # Create target Directory
        os.mkdir(main_path + '/loaded')
    except:
        pass
    try:
        # Create target Directory
        os.mkdir(main_path + '/edited')
    except:
        pass

    rospy.init_node('maploader_client')

    if offline == False:
        rospy.Subscriber(gps_input, NavSatFix, callback_maploader, queue_size=1)
    if use_ekf == True:
        rospy.Subscriber(odom_input, PoseWithCovarianceStamped, callback_mapmarker, queue_size=1)
    else:
        rospy.Subscriber(odom_input, Odometry, callback_mapmarker, queue_size=1)

    image_publisher = rospy.Publisher('gps_map', Image, queue_size=1)
    rospy.spin()
