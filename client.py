#!/usr/bin/python
import os
import sys
import getopt
import rospy
import urllib
import threading
import urlparse
import json
from time import sleep
from std_msgs.msg import String


PORT_NUM = 28874  # AUVSI


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


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "T heard %s", data.data)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()


def main():
    serveraddr = '127.0.0.1'
    serverport = 80

    try:
        opts, args = getopt.getopt(sys.argv[1:], 'hs:p:', ['server=', 'port='])
    except getopt.GetoptError as err:
        print(err)
        usage()
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            usage()
            sys.exit()
        elif opt in ('-s', '--server'):
            serveraddr = arg
        elif opt in ('-p', '--port'):
            serverport = arg
        else:
            usage()
            sys.exit(2)

    print('Server Address: ', serveraddr)
    print('Server Port: ', serverport)

    connect(serveraddr, serverport)


def usage():
    print('client.py -s <server address> -p <server port>')


def connect(serveraddr, serverport):
    conn = None

    while True:
        try:
            print('Opening Connection')
            conn = urllib.HTTPConnection(serveraddr, serverport)
            print('Connection Opened')

            print('Logging in')
            params = urlparse.urlencode({'username': 'testuser', 'password': 'testpass'})
            print(str(params))
            headers = {"Content-Type": "application/x-www-form-urlencoded", "Accept": "text/plain"}
            conn.request('POST', '/api/login', params, headers)
            response = conn.getresponse()
            print(response.status, response.reason)

            if response.status == 200:
                cookie = response.getheader('Set-Cookie')
                print('Cookie:', cookie)
                print('Successfully Logged In')
                test_run(conn, cookie)
            else:
                print('Error Logging In')
        except Exception as e:
            print('Error:', e)
            print('Closing Connection')
            conn.close()


def get_obstacles(conn, cookie):

    obstacles = []

    conn.request('GET', '/api/obstacles', None, headers={'Cookie': cookie})
    jSon = json.loads(conn.getresponse().read().decode())

    for movObst in jSon['moving_obstacles']:
        newObst = Obstacle(movObst['latitude'], movObst['longitude'], movObst['altitude_msl'],
                            movObst['sphere_radius'], True)
        newObst.is_sphere = True
        obstacles.append(newObst)

    for statObst in jSon['stationary_obstacles']:
        newObst = Obstacle(statObst['latitude'], statObst['longitude'], statObst['cylinder_height'],
                           statObst['cylinder_radius'], False)
        newObst.is_sphere = False
        obstacles.append(newObst)

    # for obst in obstacles:
    #     obst.printObstacle()

    return obstacles


def post_telemetry(conn, cookie, telemetry):
    params = urlparse.urlencode(
                {'latitude': telemetry.latitude, 'longitude': telemetry.longitude, 'altitude_msl': telemetry.altitude,
                    'uas_heading': telemetry.heading})
    headers = {"Content-Type": "application/x-www-form-urlencoded", "Accept": "text/plain", 'Cookie': cookie}
    conn.request('POST', '/api/telemetry', params, headers)
    response = conn.getresponse()


def post_target(conn, cookie, target):
    params = {'type': target.type, 'latitude': target.latitude, 'longitude': target.longitude,
              'orientation': target.orientation, 'shape': target.shape, 'background_color': target.background_color,
              'alphanumeric': target.alphanumeric, 'alphanumeric_color': target.alphanumeric_color,
              'description': target.description}

    json_params = json.dumps(params)

    headers = {"Content-Type": "application/json", "Accept": "text/plain", 'Cookie': cookie}
    conn.request('POST', '/api/targets', json_params, headers)

    response = conn.getresponse()

    if response.reason == 'CREATED':
        print("Target was submitted successfully!")
        jSon = json.loads(response.read().decode())
        return jSon['id']
    else:
        print("Something went wrong with posting a target!")
        return -1


def post_target_image(conn, cookie, target_id, image_name):
    with open(image_name, "rb") as image_file:
        encoded_image = image_file.read()

    headers = {"Content-Type": "image/jpeg", 'Cookie': cookie}
    conn.request('POST', '/api/targets/' + str(target_id) + '/image', encoded_image, headers)

    response = conn.getresponse()
    if response.reason == 'OK':
        print("*****Target image was submitted successfully!******")
    else:
        print("*****Something went wrong with posting an image!*****")
        print(response.read().decode())


def test_run(conn, cookie):

    test_image(conn, cookie)

    while True:
        try:
            obstacles = get_obstacles(conn, cookie)
            telemetry = Telemetry(0, 0, 0, 0)
            for obst in obstacles:
                if obst.is_moving:
                    telemetry.altitude = obst.alt_height + 150
                    telemetry.longitude = obst.longitude
                    telemetry.latitude = obst.latitude
                    telemetry.heading = 90
                    break

            post_telemetry(conn, cookie, telemetry)
            sleep(.1)
        except Exception as e:
            print(e)


def test_image(conn, cookie):
    image_name = os.path.relpath("images/test.jpg")

    target = Target("standard", 76.11111, 57.12345, "N", "circle", "red", "A", "white", None)
    target_id = post_target(conn, cookie, target)
    if target_id != -1:
        post_target_image(conn, cookie, target_id, image_name)
    else:
        print("Couldn't post the image, post_target failed.")


if __name__ == '__main__':
    mainThread = threading.Thread(target=main)
    mainThread.start()
    listenerThread = threading.Thread(target=listener)
    listenerThread.start()
