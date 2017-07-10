#!/usr/bin/env python
# consumer
# Stand-in for the basicbot.  Simply attempts to maximize the four genome values.
 
import json
import zmq

import std_msgs.msg

# Setup the contexts for communicating with the outside server. 
context = zmq.Context()
receiver = context.socket(zmq.PULL)
receiver.connect('tcp://127.0.0.1:5000')
sender = context.socket(zmq.PUSH)
sender.connect('tcp://127.0.0.1:5010')
 
while True:
    # Get data off the pipe from the external source
    data = json.loads(receiver.recv())
    print(data)

    msg = json.dumps({'id':data['id'],'fitness':sum(data['genome'])})
    sender.send(msg)
