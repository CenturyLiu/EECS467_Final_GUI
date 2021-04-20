#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar  28 15:02:02 2021

@author: ShaoxiongYao
"""


import time
import socket
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--stop-server", help="send stop command to server", action="store_true")
args = parser.parse_args()

while True:
    # create an ipv4 (AF_INET) socket object using the tcp protocol (SOCK_STREAM)
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # connect the client
    client.connect(('127.1.0.0', 9999))

    # send some data (in this case a HTTP GET request)
    if args.stop_server:
        client.send("stop".encode('utf8'))
        break
    else:
        client.send("joinName:1.0,1.0,1.0\n".encode('utf8'))

    # receive the response data (4096 is recommended buffer size)
    response = client.recv(4096)

    print(response)

    time.sleep(1.0)