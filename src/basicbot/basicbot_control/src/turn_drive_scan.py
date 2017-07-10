#!/usr/bin/env python
"""
    Implements a basic controller that turns and drives towards an object.  

    Interacts with the world by reading in a rostopic "laser_scanner" that gives 
    sensor vision to the robot.

    Requires: 
        smach: sudo apt-get install ros-indigo-executive-smach
"""

import random
import rospy

from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan

from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetWorldProperties
from gazebo_msgs.msg import LinkStates

from world_step import WorldStep

###########################

class GetLinkStates(object):
    """ Connect to the /gazebo/link_states topic and get information about each link. 
        Message has three lists: name, pose, twist
    """

    def __init__(self):
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.lsCallback)

        self.msg = ""
        self.formatted_msg = {}

    def lsCallback(self,msg):
        """ Callback for the link_states topic. """
        self.msg = msg
        for n,p,t in zip(msg.name,msg.pose,msg.twist):
            self.formatted_msg[n] = {'pose':p,'twist':t}

    def getStates(self):
        """ Return the last received message. """
        if self.msg:
            return type(self.msg.pose)
        else:
            return ""
    
    def getLinkPose(self,link):
        """ Return the pose of a specific link from the last message. """
        if link in self.formatted_msg:
            return self.formatted_msg[link]['pose']
        else:
            return ""

###########################

class GetLaserScanner(object):
    """ Connect to /laser_scanner rostopic and get information about the sensor.
    Message format is: 
    """

    def __init__(self):
        rospy.Subscriber('/laser_scanner', LaserScan, self.lsCallback)

        self.msg = ""
        self.formatted_msg = ""

    def lsCallback(self,msg):
        """ Callback for the laser_scanner topic. """
        self.msg = msg
        self.formatted_msg = {'time':str(msg.header.stamp.secs)+"."+str(msg.header.stamp.nsecs), 'sum_ranges':sum(msg.ranges), 'ranges':msg.ranges}

    def getScanState(self):
        """ Get the current scan information. """
        return self.formatted_msg
        
    def getLeftCenterRightScanState(self):
        """ Divide the vision into three sections and report on their average sum. """

        partitioned_vision = {'right':10.0,'left':10.0,'center':10.0}

        # If no message yet return blank
        if self.formatted_msg:
            partitions = [len(self.formatted_msg['ranges'])/3 for i in range(3)]

            # Add additional ones to middle if don't match sum.
            if sum(partitions) < len(self.formatted_msg['ranges']):
                partitions[1] += len(self.formatted_msg['ranges']) - sum(partitions)

            # Calculate the index offsets.
            partitions[1] = partitions[0] + partitions[1]
            partitions[2] = partitions[1] + partitions[2]

            # Get right, center, and left averages.
            partitioned_vision['right'] = sum(self.formatted_msg['ranges'][0:partitions[0]])/len(self.formatted_msg['ranges'][0:partitions[0]])
            partitioned_vision['center'] = sum(self.formatted_msg['ranges'][partitions[0]:partitions[1]])/len(self.formatted_msg['ranges'][partitions[0]:partitions[1]])
            partitioned_vision['left'] = sum(self.formatted_msg['ranges'][partitions[1]:partitions[2]])/len(self.formatted_msg['ranges'][partitions[1]:partitions[2]])

        return partitioned_vision

###########################

final_time = 0.0

# Setup the driving messages.
twist = {}
twist['forward'] = Twist()
twist['forward'].linear.x = 0.5

twist['stop'] = Twist()
twist['stop'].linear.x = 0.0

twist['left'] = Twist()
twist['left'].linear.x = 0.0
twist['left'].angular.z = -0.5

twist['right'] = Twist()
twist['right'].linear.x = 0.0
twist['right'].angular.z = 0.5

def MoveRobot(movement):
    """ Movements: 'left', 'forward', 'right', 'stop' """
    cmd_vel_pub.publish(twist[movement])

def checkAtFinalTime():
    """ Check to see if we have exceeded the execution time. """
    if final_time <= getWorldProp().sim_time:
        return True
    return False
###########################

import smach
import smach_ros

# Define the states for the robot

class SpinLeft(smach.State):
    def __init__(self, forward_threshold):
        smach.State.__init__(self, outcomes=['spin_left','drive_forward','failed'],
            input_keys=['detect_center'])
        self.forward_threshold = forward_threshold

    def execute(self, userdata):
        MoveRobot('left')
        ws.stepPhysics(steps=1)
        if checkAtFinalTime():
            return 'failed'
        scan_data = scan.getLeftCenterRightScanState()
        if scan_data['center'] < self.forward_threshold:
            return 'drive_forward'
        else: 
            return 'spin_left'

class SpinRight(smach.State):
    def __init__(self, forward_threshold):
        smach.State.__init__(self, outcomes=['spin_right','drive_forward','failed'],
            input_keys=['detect_center'])
        self.forward_threshold = forward_threshold

    def execute(self, userdata):
        MoveRobot('right')
        ws.stepPhysics(steps=1)
        if checkAtFinalTime():
            return 'failed'
        scan_data = scan.getLeftCenterRightScanState()
        if scan_data['center'] < self.forward_threshold:
            return 'drive_forward'
        else: 
            return 'spin_right'

class DriveForward(smach.State):
    def __init__(self, threshold, spin_threshold):
        smach.State.__init__(self, outcomes=['spin_right','drive_forward','spin_left','within_threshold','failed'],
            input_keys=['detect_center','detect_right','detect_left'])
        self.threshold = threshold
        self.spin_threshold = spin_threshold

    def execute(self, userdata):
        MoveRobot('forward')
        ws.stepPhysics(steps=1)
        if checkAtFinalTime():
            return 'failed'
        scan_data = scan.getLeftCenterRightScanState()
        if scan_data['center'] < self.threshold:
            return 'within_threshold'
        elif scan_data['center'] < 10.0: 
            return 'drive_forward'
        elif scan_data['left'] < self.spin_threshold and scan_data['center'] >= 10.0 and scan_data['right'] >= 10.0: 
            return 'spin_left'
        else:
            return 'spin_right'

class Stop(smach.State):
    def __init__(self, stop_thresh):
        smach.State.__init__(self, outcomes=['succeeded','spin_right','failed'],
            input_keys=['detect_center'])
        self.stop_thresh = stop_thresh

    def execute(self, userdata):
        MoveRobot('stop')
        ws.stepPhysics(steps=1)
        if checkAtFinalTime():
            return 'failed'
        scan_data = scan.getLeftCenterRightScanState()
        if scan_data['center'] < self.stop_thresh:
            return 'succeeded'
        else:
            return 'spin_right'

###########################

# Setup the reset world and reset simulation services
rospy.wait_for_service('/gazebo/get_world_properties')
rospy.wait_for_service('/gazebo/reset_world')
rospy.wait_for_service('/gazebo/reset_simulation')
rospy.wait_for_service('/gazebo/pause_physics')
rospy.wait_for_service('/gazebo/unpause_physics')

getWorldProp = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
resetWorld = rospy.ServiceProxy('/gazebo/reset_world', Empty)
resetSimulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

ws = WorldStep()
# Setup the messages we will use to drive the robot.
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander', log_level=rospy.WARN)

# Setup the topic subscribers.
ls = GetLinkStates()
scan = GetLaserScanner()

for i in range(5):

    genome = {
        'center_spin_thresh': random.random()*10.0,
        'center_drive_thresh': 9.0 + random.random() * 1.0,
        'center_stop_thresh': random.random() * 10.0,
        'stopping_thresh': random.random() * 10.0
    }

    sm = smach.StateMachine(outcomes=['succeeded','failed'])

    # Set the first timestep
    ws.stepPhysics(steps=1)
    current_time = getWorldProp().sim_time 
    final_time = getWorldProp().sim_time + 20.0
    print(genome)

    with sm:
            smach.StateMachine.add('SPIN_RIGHT', SpinRight(genome['center_drive_thresh']), transitions={ 'spin_right':'SPIN_RIGHT',
                'drive_forward':'DRIVE_FORWARD','failed':'failed'
                })
            smach.StateMachine.add('SPIN_LEFT', SpinLeft(genome['center_drive_thresh']), transitions={ 'spin_left':'SPIN_LEFT',
                'drive_forward':'DRIVE_FORWARD','failed':'failed'
                })
            smach.StateMachine.add('DRIVE_FORWARD', DriveForward(genome['center_stop_thresh'],genome['center_spin_thresh']), transitions={ 'spin_right':'SPIN_RIGHT',
                'drive_forward':'DRIVE_FORWARD',
                'spin_left':'SPIN_LEFT',
                'within_threshold':'STOP',
                'failed':'failed'
                })
            smach.StateMachine.add('STOP', Stop(genome['stopping_thresh']), transitions={ 'succeeded':'succeeded', 'spin_right':'SPIN_RIGHT', 'failed':'failed'})

            sis = smach_ros.IntrospectionServer('test_sm',sm,'/SM_ROOT')
            sis.start()

            outcome = sm.execute()

    current_time = getWorldProp().sim_time 
    print("Current Time: "+str(current_time))

    if outcome == 'succeeded':
        print(scan.getLeftCenterRightScanState())
    else:
        print("Robot failed to find the cylinder in time.")

    resetWorld()
    resetSimulation()