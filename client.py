#!/usr/bin/python

import sys
import getopt
import http.client
import urllib.parse
import json
import threading
import traceback
from time import sleep

isConnected = False
connectedLock = threading.Lock()
obstacleLock = threading.Lock()
telemetryLock = threading.Lock()
connectionLock = threading.Lock()
conn = None
cookie = ''

class Telemetry(object):
    def __init__(self, latitude, longitude, altitude, heading):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.heading = heading

        # use __getattr__('heading') or other attributes for the data
        # use __set__('heading', 90) or other attributes to set data


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
        if(self.is_sphere):
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

obstacles = []


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


class clientThread(threading.Thread):
    def __init__(self, threadID, function):
        threading.Thread.__init__(self)
        self.id = threadID
        self.function = function

    def run(self):
        try:
            self.function()
        except http.client.HTTPException as e:
            print('Exception: ', str(e))
            traceback.print_exc()
            setIsConnected(False)


def send_request(method, endpoint, params, headers):
    connectionLock.acquire(1)
    conn.request(method, endpoint, params, headers)
    response = conn.getresponse()
    connectionLock.release()

    return response


def send_post_request(params, endpoint, include_cookie = True):
    global cookie

    if include_cookie:
        headers = {"Content-type": "application/x-www-form-urlencoded", "Accept": "text/plain", 'Cookie': cookie}
    else:
        headers = {"Content-type": "application/x-www-form-urlencoded", "Accept": "text/plain"}

    return send_request('POST', endpoint, params, headers)


def send_get_request(endpoint, include_cookie = True):
    global cookie

    if include_cookie:
        headers = {'Accept': 'text/plain', 'Cookie': cookie}
    else:
        headers = {'Accept': 'text/plain'}

    return send_request('GET', endpoint, None, headers)


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
    global conn, cookie

    while True:
        try:
            print('Opening Connection')
            conn = http.client.HTTPConnection(serveraddr, serverport)
            setIsConnected(True)
            print('Connection Opened')

            print('Logging in')
            params = urllib.parse.urlencode({'username': 'testadmin', 'password': 'testpass'})
            print(str(params))

            response = send_post_request(params, '/api/login', False)
            print(response.status, response.reason)

            if response.status == 200:
                cookie = response.getheader('Set-Cookie')
                print('Cookie:', cookie)
                print('Successfully Logged In')

                # Starting threads to talk to server
                obstThread = clientThread(1, get_obstacles)
                obstThread.start()

                testThread = clientThread(2, test_run)
                testThread.start()

                print('Continuing after starting thread')

                i = 0
                while getIsConnected():
                    i += 1
                    if i > 1000:
                        i = 0
                else:
                    raise Exception('I\'m free!')

                # test_run(conn, cookie)
            else:
                print('Error Logging In')
        except Exception as e:
            # print('Error:', str(e))
            print('Closing Connection')
            # traceback.print_exc()
            conn.close()
            setIsConnected(False)


def get_obstacles():
    global obstacles

    while getIsConnected():
        newObstacles = []

        # conn.request('GET', '/api/obstacles', None, headers={'Cookie': cookie})
        response = send_get_request('/api/obstacles')

        jSon = json.loads(response.read().decode())

        for movObst in jSon['moving_obstacles']:
            newObst = Obstacle(movObst['latitude'], movObst['longitude'], movObst['altitude_msl'],
                                movObst['sphere_radius'], True)
            newObst.is_sphere = True
            newObstacles.append(newObst)

        for statObst in jSon['stationary_obstacles']:
            newObst = Obstacle(statObst['latitude'], statObst['longitude'], statObst['cylinder_height'],
                               statObst['cylinder_radius'], False)
            newObst.is_sphere = False
            newObstacles.append(newObst)

        # for obst in newObstacles:
        #     obst.printObstacle()

        obstacleLock.acquire(1)
        obstacles = newObstacles
        obstacleLock.release()

    # print(json.dumps(jSon, indent=4, sort_keys=True))
    # print(jSon['moving_obstacles'][0]['longitude'])
    # exit()


def post_telemetry():
    global telemetry

    while getIsConnected():
        telemetryLock.acquire(1)
        params = urllib.parse.urlencode({'latitude': telemetry.latitude, 'longitude': telemetry.longitude, 'altitude_msl': float(telemetry.altitude), 'uas_heading': telemetry.heading})
        print(telemetry.latitude)
        telemetryLock.release()

        response = send_post_request(params, '/api/telemetry')
        # print(response.read().decode())


def post_target(target):
    params = urllib.parse.urlencode({'type': target.type, 'latitude': target.latitude, 'longitude': target.longitude,
                                     'orinetation': target.orientation, 'shape': target.shape, 'background_color':
                                         target.background_color, 'alphanumeric': target.alphanumeric,
                                     'alphanumeric_color': target.alphanumeric_color, 'description': target.description})
    headers = {"Content-type": "application/x-www-form-urlencoded", "Accept": "text/plain", 'Cookie': cookie}
    conn.request('POST', '/api/targets', params, headers)

    response = conn.getresponse()
    if(response.reason == 'CREATED'):
        print("Target was submitted successfully!")
        # jSon = json.loads(response.read().decode())
    else:
        print("Something went wrong with posting a target!")


def test_run():
    global telemetry
    global obstacles
    global telemetryLock

    while getIsConnected():
        for obst in obstacles:
            if obst.is_moving:
                telemetryLock.acquire(1)
                telemetry.altitude = obst.alt_height + 150
                telemetry.longitude = obst.longitude
                telemetry.latitude = obst.latitude
                telemetry.heading = 90
                telemetryLock.release()
                break

        post_telemetry()
        sleep(.1)


def getIsConnected():
    global connectedLock
    global isConnected

    connectedLock.acquire(1)
    tempCon = isConnected
    connectedLock.release()

    return tempCon

def setIsConnected(value):
    global connectedLock
    global isConnected

    connectedLock.acquire(1)
    isConnected = value
    connectedLock.release()

if __name__ == '__main__':
    main()
