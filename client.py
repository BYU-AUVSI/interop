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

SERVERADDR = '192.168.0.104'
SERVERPORT = 80
GLOBALCONN = None
GLOBALCOOKIE = None
CONNECTED = False

connectionLock = threading.Lock()
cookieLock = threading.Lock()
connectedLock = threading.Lock()


class Telemetry(object):
    def __init__(self, latitude, longitude, altitude, heading):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.heading = heading

    # use __getattr__('heading') or other attributes for the data
    # use __set__('heading', 90) or other attributes to set data


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


def telemcallback(data):
    rospy.loginfo(rospy.get_caller_id() + "T heard %s", data.data)
    # Setup telemetry model and pass to post_telemetry() [possibly spin up a thread to do this????]


def targetcallback(data):
    rospy.loginfo(rospy.get_caller_id() + "T heard %s", data.data)
    # Setup target model and pass to post_target() and post_target_image() [possibly spin up a thread to do this????]


# def listener():
#     print('Listening')
#     rospy.init_node('listener', anonymous=True)
#     rospy.Subscriber("chatter", String, telemcallback)  # This should be the listener for (at least part of)
#     # telemetry from autopilot
#     # rospy.Subscriber("chatter", String, targetcallback)  # This should be the listener for target images from image
#     #  processing
#     rospy.spin()
#
#
def talker():
    print('Talking')
    rospy.init_node('talker', anonymous=True)
    publisher = rospy.Publisher('obstacles', String, 10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        string = get_obstacles()
        rospy.loginfo(string)
        publisher.publish(string)
        rate.sleep()


def main():
    print('Listening')
    rospy.init_node('ground_station', anonymous=True)
    rospy.Subscriber("chatter", String, telemcallback)  # This should be the listener for (at least part of)
    # telemetry from autopilot
    # rospy.Subscriber("chatter", String, targetcallback)  # This should be the listener for target images from image
    #  processing
    rospy.spin()

    # print('Talking')
    # publisher = rospy.Publisher('obstacles', String)
    # rate = rospy.Rate(10)
    #
    # while not rospy.is_shutdown():
    #     string = get_obstacles()
    #     rospy.loginfo(string)
    #     publisher.publish(string)
    #     rate.sleep()


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

    while not is_connected():
        try:
            # print('Opening Connection')

            take_connection()

            GLOBALCONN = httplib.HTTPConnection(SERVERADDR, SERVERPORT)
            # print('Connection Opened')

            # print('Logging in')
            params = urllib.urlencode({'username': 'testuser', 'password': 'testpass'})
            # print(str(params))
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

                # while True:
                #     sleep(.2)
                #     GLOBALCONN.request('GET', '/api/server_info', None, headers={'Cookie': GLOBALCOOKIE})
                #     response = GLOBALCONN.getresponse()
                #
                #     # print('Server Response: ' + response.read().decode())
                #
                #     if response.status != 200:
                #         raise Exception('Connection with server failed')
            else:
                raise Exception('Error Logging In')
        except Exception as e:
            print('Error:', e)
            print('Closing Connection')

            take_connection()
            GLOBALCONN.close()
            return_connection()

            set_is_connected(False)


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


def post_telemetry(telemetry):
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


def test_image():
    image_name = os.path.relpath("images/test.jpg")

    target = Target("standard", 76.11111, 57.12345, "N", "circle", "red", "A", "white", None)
    target_id = post_target(target)
    if target_id != -1:
        post_target_image(target_id, image_name)
    else:
        print("Couldn't post the image, post_target failed.")


if __name__ == '__main__':
    # main()
    # connectionThread = threading.Thread(target=connect(SERVERADDR, SERVERPORT))
    # listener()
    mainThread = threading.Thread(target=main)
    mainThread.daemon = True
    mainThread.start()
    talker()
    # listenerThread = threading.Thread(target=listener)
    # listenerThread.start()
