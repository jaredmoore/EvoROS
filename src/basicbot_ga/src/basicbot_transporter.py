#!/usr/bin/env python
# consumer
 
import json
import rospy
import zmq

import std_msgs.msg

basicbot_scan_result = ''

def callback(data):
    """ Handle the return of adding information. """
    global basicbot_scan_result

    basicbot_scan_result = data.data

# Setup the contexts for communicating with the outside server. 
context = zmq.Context()
receiver = context.socket(zmq.PULL)
receiver.connect('tcp://127.0.0.1:5000')
sender = context.socket(zmq.PUSH)
sender.connect('tcp://127.0.0.1:5010')
 
# Setup the ROS topics for communicating with connected nodes.
rospy.init_node('transporter',anonymous=True)
pub = rospy.Publisher('simulation_start', std_msgs.msg.Empty, queue_size=1)
sub = rospy.Subscriber('simulation_result', std_msgs.msg.Float64, callback)

while True:
    # Get data off the pipe from the external source
    data = json.loads(receiver.recv())
    print(data)

    # Load the data into a parameter in ROS
    rospy.set_param('basicbot_genome', data['genome'])

    # Send a ready message on the topic to the basicbot node
    pub.publish(std_msgs.msg.Empty())

    # Wait for a result to return from the basicbot node
    # TODO: Remove this spinning while loop with a different construct.
    # Dependent upon the internal structure of ROS nodes!
    while basicbot_scan_result == '':
        pass

    # Transmit the result back to the external source

    msg = json.dumps({'id':data['id'],'fitness':basicbot_scan_result, 'ns':rospy.get_namespace(), 'name':rospy.get_name()})
    sender.send(msg)
    print (rospy.get_namespace(), basicbot_scan_result)
    basicbot_scan_result = ''
