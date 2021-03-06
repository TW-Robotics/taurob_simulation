#! /usr/bin/env python
import rospy

import requests
import urllib
import re
import time
import os
import actionlib
from maploader.msg import LoadMapAction, LoadMapGoal, LoadMapResult
from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from PIL import Image, ImageDraw, ImageFont

def callback(goal):
    result = LoadMapResult()
    lon = goal.longitude
    lat = goal.latitude
    resx = goal.resolution_x
    resy = goal.resolution_y
    gps_delta = goal.delta
    path = goal.folder + '/loaded'
    maptype = goal.maptype

    #calculate 4 corners of rectangular image to download
    lon_min = lon - gps_delta
    lon_max = lon + gps_delta
    lat_min = lat - gps_delta
    lat_max = lat + gps_delta

    print("action_server: longitude = %f" %lon)
    print("action_server: latitude = %f" %lat)
    print("action_server: gps_delta = %f" %gps_delta)
    print("action_server: lon_min = %f" %lon_min)
    print("action_server: lat_min = %f" %lat_min)
    print("action_server: lon_max = %f" %lon_max)
    print("action_server: lat_max = %f" %lat_max)
    print("action_server: maptype = %s" %maptype)

    url = "http://utility.arcgisonline.com/arcgis/rest/services/Utilities/PrintingTools/GPServer/Export%20Web%20Map%20Task/execute"

    # URL to satellite image ArcGIS mapserver
    if maptype == "aerial":
        mapserver_url = "http://services.arcgisonline.com/arcgis/rest/services/World_Imagery/MapServer/WMTS/tile/1.0.0/World_Imagery"

    # URL to streetmap ArcGIS mapserver
    elif maptype == 'streetmap':
        mapserver_url = "http://services.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer"
    else:
        print("action_server: invalid mapserver URL")
        return

    # POST request payload with 4 corners of requested image, resolution and URL to mapserver
    payload = '------WebKitFormBoundary7MA4YWxkTrZu0gW\r\nContent-Disposition: form-data; name=\"f\"\r\n\r\njson\r\n------WebKitFormBoundary7MA4YWxkTrZu0gW\r\nContent-Disposition: form-data; name=\"Format\"\r\n\r\nPNG32\r\n------WebKitFormBoundary7MA4YWxkTrZu0gW\r\nContent-Disposition: form-data; name=\"Layout_Template\"\r\n\r\nMAP_ONLY\r\n------WebKitFormBoundary7MA4YWxkTrZu0gW\r\nContent-Disposition: form-data; name=\"Web_Map_as_JSON\"\r\n\r\n{ \"mapOptions\": { \"extent\": { \"xmin\":' + repr(lon_min) + ', \"ymin\":' + repr(lat_min) + ', \"xmax\":' + repr(lon_max) + ', \"ymax\":' + repr(lat_max) + ' , \"spatialReference\": { \"wkid\": 4326 } } }, \"operationalLayers\": [], \"baseMap\" : { \"title\" : \"Topographic Basemap\", \"baseMapLayers\" :  [ { \"url\" : \"' + mapserver_url + '\" } ] }, \"exportOptions\": { \"outputSize\" :  [' + repr(resx) + ','+ repr(resy) + '] } }\r\n------WebKitFormBoundary7MA4YWxkTrZu0gW--'

    headers = {
            'content-type': "multipart/form-data; boundary=----WebKitFormBoundary7MA4YWxkTrZu0gW",
            'User-Agent': "PostmanRuntime/7.11.0",
            'Accept': "*/*",
            'Cache-Control': "no-cache",
            'Postman-Token': "9d2247a3-c5e5-4113-8d01-92d05c349f67,30e7f90d-0bd9-4b95-88d3-c5e66b46e713",
            'Host': "utility.arcgisonline.com",
            'accept-encoding': "gzip, deflate",
            'content-length': "930",
            'Connection': "keep-alive",
            'cache-control': "no-cache"
            }

    # send POST request to ArcGIS server to receive URL to image
    response = requests.request("POST", url, data=payload, headers=headers)
    retrieve_url = response.text
    print("action_server: ", retrieve_url)

    # extract URL to image
    m = re.search('url\":\"(.+?)\"}}]', retrieve_url)
    if m:
        found = m.group(1)
        print("action_server: printing URL : ")
        print("action_server: ", found)
        timestr = time.strftime("%Y%m%d-%H%M%S")

        # try to download image using the found URL until success
        while True:
            print("action_server: downloading map as .png")

            #download satellite image
            urllib.urlretrieve(found, path + "/" + timestr + ".png")

            # look at downloaded image to see if download was successfull
            imagepath = path + "/" + timestr + ".png"
            try:
                image = open(imagepath, "r")
            except:
                print("action_server: ERROR: file not found")
                server.set_aborted(result, "ERROR: file not found")

            # often the URL leads to "Error 400: Invalid URL, formatted as HTML"
            if image.read(5) == "<html":
                print("action_server: Error: downloaded file is HTML")
                continue
            else:
                print("action_server: Image successfully downloaded at: " + imagepath)
                result.file_location = imagepath
                server.set_succeeded(result)
                break

if __name__ == '__main__':
    rospy.init_node('maploader_actionserver')
    server = actionlib.SimpleActionServer('loadmap', LoadMapAction, callback, False)
    server.start()

    rospy.spin()
