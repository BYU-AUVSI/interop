#!/usr/bin/python

import sys
import getopt
import http.client
import urllib.parse
import json
from time import sleep


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
    print('Opening Connection')
    conn = http.client.HTTPConnection(serveraddr, serverport)
    print('Connection Opened')

    print('Logging in')
    params = urllib.parse.urlencode({'username': 'testadmin', 'password': 'testpass'})
    print(str(params))
    headers = {"Content-type": "application/x-www-form-urlencoded", "Accept": "text/plain"}
    conn.request('POST', '/api/login', params, headers)
    response = conn.getresponse()
    print(response.status, response.reason)
    if response.status == 200:
        cookie = response.getheader('Set-Cookie')
        print('Cookie:', cookie)
        print('Successfully Logged In')
        get_obstacles(conn, cookie)
    else:
        print('Error Logging In')

    print('Closing Connection')
    conn.close()


def get_obstacles(conn, cookie):
    print('Starting loop')
    while True:
        conn.request('GET', '/api/obstacles', None, headers = {'Cookie': cookie})
        print(json.dumps(json.loads(conn.getresponse().read().decode()), indent = 4, sort_keys = True))
        # sleep(.1)

if __name__ == '__main__':
    main()
