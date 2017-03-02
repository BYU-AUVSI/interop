#!/usr/bin/python
import os
import sys
import getopt
import rospy
import httplib
import threading
import urllib
import json
from time import sleep
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix

SERVERADDR = '127.0.0.1'
SERVERPORT = 80
GLOBALCONN = None
GLOBALCOOKIE = None
CONNECTED = False
RETRY_MAX = 3

new_lat = False
new_long = False
new_alt = False
new_hdg = False

connectionLock = threading.Lock()
cookieLock = threading.Lock()
connectedLock = threading.Lock()


class Telemetry(object):
    def __init__(self, latitude, longitude, altitude, heading):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.heading = heading

    def printTelemetry(self):
        print('Latitude: ', self.latitude)
        print('Longitude: ', self.longitude)
        print('Altitude: ', self.altitude)
        print('Heading: ', self.heading)

    # use __getattr__('heading') or other attributes for the data
    # use __set__('heading', 90) or other attributes to set data

# global telemetry object
telemetry = Telemetry(0, 0, 0, 0)


class Obstacle(object):
    # if is_sphere is false, it's a cylinder
    is_sphere = True

    def __init__(self, latitude, longitude, alt_height, radius, is_moving):
        self.latitude = latitude
        self.longitude = longitude
        self.alt_height = alt_height
        self.radius = radius
        self.is_moving = is_moving

    def printObstacle(self):
        if self.is_sphere:
            print('Sphere...')
            print('latitude: ', self.latitude)
            print('longitude: ', self.longitude)
            print('altitude: ', self.alt_height)
            print('radius: ', self.radius)
        else:
            print('Cylinder...')
            print('latitude: ', self.latitude)
            print('longitude: ', self.longitude)
            print('height: ', self.alt_height)
            print('radius: ', self.radius)


class Target(object):
    id = -1
    user = -1

    def __init__(self, type, latitude, longitude, orientation, shape, background_color, alphanumeric,
                 alphanumeric_color, description):
        self.type = type
        self.latitude = latitude
        self.longitude = longitude
        self.orientation = orientation
        self.shape = shape
        self.background_color = background_color
        self.alphanumeric = alphanumeric
        self.alphanumeric_color = alphanumeric_color
        self.description = description

        # Orientation Types: N, NE, E, SE, S, SW, W, NW
        # Shape Types: circle, semicircle, quarter_circle, triangle, square, rectangle, trapezoid, pentagon, hexagon,
        #  heptagon, octagon, star, cross
        # Color Types: white, black, gray, red, blue, green, yellow, purple, brown, orange


def gps_callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "GPS Latitude: %s, Longitude: %s, Altitude: %s", data.latitude, data.longitude, data.altitude)
    telem = dict()
    telem['lat'] = data.latitude
    telem['long'] = data.longitude
    telem['alt'] = data.altitude
    update_telemetry(telem)


def hdg_callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "AUVSI Heading: %s", data.data)
    telem = dict()
    telem['hdg'] = data.data
    update_telemetry(telem)


def targetcallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # Setup target model and pass to post_target() and post_target_image() [possibly spin up a thread to do this????]


def listener():
    print('Listening')
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)  # gps information + altitude
    rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, hdg_callback)  # heading
    # rospy.Subscriber("chatter", String, targetcallback)  # This should be the listener for target images from image
    #  processing
    rospy.spin()


def update_telemetry(data):
    global telemetry
    global new_lat
    global new_long
    global new_alt
    global new_hdg

    if 'lat' in data:
        telemetry.latitude = data['lat']
        new_lat = True
    if 'long' in data:
        telemetry.longitude = data['long']
        new_long = True
    if 'alt' in data:
        telemetry.altitude = data['alt']
        new_alt = True
    if 'hdg' in data:
        telemetry.heading = data['hdg']
        new_hdg = True

    if new_lat and new_long and new_alt and new_hdg:
        sendTelemThread = threading.Thread(target=send_telemetry)
        sendTelemThread.start()


def talker():
    print('Talking')
    publisher = rospy.Publisher('obstacles', String, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        string = get_obstacles()
        rospy.loginfo(string)
        publisher.publish(string)
        rate.sleep()


def take_connection():
    global GLOBALCONN
    connectionLock.acquire()
    return GLOBALCONN


def return_connection():
    connectionLock.release()


def get_cookie():
    global GLOBALCOOKIE

    cookieLock.acquire()
    cookie = GLOBALCOOKIE
    cookieLock.release()

    return cookie


def set_cookie(cookie):
    global GLOBALCOOKIE

    cookieLock.acquire()
    GLOBALCOOKIE = cookie
    cookieLock.release()


def is_connected():
    global CONNECTED

    connectedLock.acquire()
    connected = CONNECTED
    connectedLock.release()

    return connected


def set_is_connected(connected):
    global CONNECTED

    connectedLock.acquire()
    CONNECTED = connected
    connectedLock.release()


def connect():
    # conn = None
    global GLOBALCONN

    params = urllib.urlencode({'username': 'testuser', 'password': 'testpass'})
    retry_count = 0
    while not is_connected() and retry_count < RETRY_MAX:
        retry_count+=1
        # print('Opening Connection')

        # must be outside of try block else will cause deadlock in except block
        take_connection()

        try:
            GLOBALCONN = httplib.HTTPConnection(SERVERADDR, SERVERPORT)
            # print('Connection Opened')

            # print('Logging in')
            headers = {"Content-Type": "application/x-www-form-urlencoded", "Accept": "text/plain"}
            GLOBALCONN.request('POST', '/api/login', params, headers)
            response = GLOBALCONN.getresponse()

            return_connection()

            # print(response.status, response.reason)

            if response.status == 200:
                set_cookie(response.getheader('Set-Cookie'))
                # print('Cookie:', get_cookie())

                print('Successfully Logged In')
                set_is_connected(True)
            else:
                raise Exception('Error Logging In')
        except Exception as e:
            print('Connection Error: ' + str(e))

            # closing connection
            GLOBALCONN.close()
            return_connection()

            set_is_connected(False)
    if retry_count >= RETRY_MAX:
        print("could not connect to server. (address=" + SERVERADDR + ", port=" + str(SERVERPORT) + ", " + ", ".join(str(params).split("&")) + "). exiting...")
        exit()


def send_request(method, url, params, headers):
    response = None

    while True:
        if not is_connected():
            print('Connecting')
            connect()

        take_connection()
        GLOBALCONN.request(method, url, params, headers)

        response = GLOBALCONN.getresponse()
        return_connection()

        if response.status == 200 or response.status == 201:
            break
        elif response.status == 400:
            print('400 - Bad Request')
            print('url:' + url)
            print(params)
            print(headers)
            break
        elif response.status == 403:
            set_is_connected(False)  # Retry but create a new connection and login again first
            print ('403 - Forbidden: Was the cookie sent?')
            print('url:' + url)
            print(headers)
        elif response.status == 404:
            print('404 - Not Found: {url:' + url + '}')
            break
        elif response.status == 405:
            print('405 - Invalid Request: {url:' + url + ', method:' + method + '}')
            break
        elif response.status == 413:
            print('413 - Image is too large, it needs to be < 1MB')
            break
        elif response.status == 500:
            print('500 - SERVER ERROR')
            break

    return response


def get_obstacles():
    # obstacles = []

    response = send_request('GET', '/api/obstacles', None, headers={'Cookie': get_cookie()})
    return response.read().decode()
    # jSon = json.loads(conn.getresponse().read().decode())

    # for movObst in jSon['moving_obstacles']:
    #     newObst = Obstacle(movObst['latitude'], movObst['longitude'], movObst['altitude_msl'],
    #                         movObst['sphere_radius'], True)
    #     newObst.is_sphere = True
    #     obstacles.append(newObst)
    #
    # for statObst in jSon['stationary_obstacles']:
    #     newObst = Obstacle(statObst['latitude'], statObst['longitude'], statObst['cylinder_height'],
    #                        statObst['cylinder_radius'], False)
    #     newObst.is_sphere = False
    #     obstacles.append(newObst)
    #
    # # for obst in obstacles:
    # #     obst.printObstacle()
    #
    # return obstacles


def send_telemetry():
    global telemetry
    global new_lat
    global new_long
    global new_alt
    global new_hdg

    telemetry.printTelemetry()
    post_telemetry()
    new_lat = False
    new_long = False
    new_alt = False
    new_hdg = False



def post_telemetry():
    global telemetry
    params = urllib.urlencode(
                {'latitude': telemetry.latitude, 'longitude': telemetry.longitude, 'altitude_msl': telemetry.altitude,
                    'uas_heading': telemetry.heading})
    headers = {"Content-Type": "application/x-www-form-urlencoded", "Accept": "text/plain", 'Cookie': get_cookie()}
    response = send_request('POST', '/api/telemetry', params, headers)


def post_target(target):

    params = {'type': target.type, 'latitude': target.latitude, 'longitude': target.longitude,
              'orientation': target.orientation, 'shape': target.shape, 'background_color': target.background_color,
              'alphanumeric': target.alphanumeric, 'alphanumeric_color': target.alphanumeric_color,
              'description': target.description}

    json_params = json.dumps(params)

    headers = {"Content-Type": "application/json", "Accept": "text/plain", 'Cookie': get_cookie()}
    response = send_request('POST', '/api/targets', json_params, headers)

    if response.status == 201:
        print("Target was submitted successfully!")
        jSon = json.loads(response.read().decode())
        return jSon['id']
    else:
        print("Something went wrong with posting a target!")
        return -1


def post_target_image(target_id, image_name):
    with open(image_name, "rb") as image_file:
        encoded_image = image_file.read()

    headers = {"Content-Type": "image/jpeg", 'Cookie': get_cookie()}
    response = send_request('POST', '/api/targets/' + str(target_id) + '/image', encoded_image, headers)

    if response.reason == 200:
        print("*****Target image was submitted successfully!******")
    else:
        print("*****Something went wrong with posting an image!*****")
        print(response.read().decode())


# def test_run():
#     test_image()
#
#     while True:
#         try:
#             obstacles = get_obstacles()
#             telemetry = Telemetry(0, 0, 0, 0)
#             for obst in obstacles:
#                 if obst.is_moving:
#                     telemetry.altitude = obst.alt_height + 150
#                     telemetry.longitude = obst.longitude
#                     telemetry.latitude = obst.latitude
#                     telemetry.heading = 90
#                     break
#
#             post_telemetry(telemetry)
#             sleep(.1)
#         except Exception as e:
#             print(e)


# def test_image():
#     image_name = os.path.relpath("images/test.jpg")
#
#     target = Target("standard", 76.11111, 57.12345, "N", "circle", "red", "A", "white", None)
#     target_id = post_target(target)
#     if target_id != -1:
#         post_target_image(target_id, image_name)
#     else:
#         print("Couldn't post the image, post_target failed.")


if __name__ == '__main__':
    rospy.init_node('ground_station', anonymous=True)
    connect()

    listenerThread = threading.Thread(target=listener)
    listenerThread.start()
    talker()
