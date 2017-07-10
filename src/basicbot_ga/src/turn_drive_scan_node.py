#!/usr/bin/env python
"""
    Implements a basic controller that turns and drives towards an object.    

    Interacts with the world by reading in a rostopic "laser_scanner" that gives 
    sensor vision to the robot.

    Requires: 
        smach: sudo apt-get install ros-indigo-executive-smach
"""

import time

import random
import rospy
import std_msgs.msg

from std_srvs.srv import Empty

from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetWorldProperties

from rosgraph_msgs.msg import Clock

from basicbot_utils import GetLaserScanner, GetLinkStates

###########################

# For tracking robot progression.
bot_position = []
bot_id = 0

# Keep track of time.
current_second = 0.0
start_time = 0.0
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
    global final_time
    if final_time <= current_second:
        return True
    return False

def update_world(mv_command):
    """ Update the world by processing the move command, stepping the world
        and checking for final time failure.

        Args:
            mv_command: argument to move.

        Returns:
            scan_data: if None, failed state
    """
    global bot_position, scan

    MoveRobot(mv_command)
    # ws.stepPhysics(steps=1)
    if checkAtFinalTime():
        return None
    scan_data = scan.getLeftCenterRightScanState()

    return scan_data

def log_bot_position(id):
    """ Log the bot position over time for later log tracking. """
    pass
    #global bot_position

    #with open("/user/moore112/bot_logging.dat","a") as f:
    #    for b in bot_position:
    #        f.write(str(id)+","+str(b[0])+","+str(b[1])+","+str(b[2])+"\n")

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
        scan_data = update_world('left')
        if not scan_data:
            return 'failed'
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
        scan_data = update_world('right')
        if not scan_data:
            return 'failed'
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
        scan_data = update_world('forward')
        if not scan_data:
            return 'failed'
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
        global current_second, start_time
        scan_data = update_world('stop')
        if not scan_data:
            return 'failed'
        if scan_data['center'] < self.stop_thresh and current_second-start_time > 5.0:
            return 'succeeded'
        else:
            return 'spin_right'

###########################

def clock_callback(data):
    """ Subscriber to /clock

    This callback is responsible for getting the current simulation
    time in seconds.
    """
    global current_second
    current_second = data.clock.secs

def simCallback(data):
    """ Callback to conduct a simulation. """
    global start_time, final_time, pub, bot_position, bot_id, scan

    genome_data = rospy.get_param('basicbot_genome')

    genome = {
        'center_spin_thresh': genome_data[0],
        'center_drive_thresh': genome_data[1],
        'center_stop_thresh': genome_data[2],
        'stopping_thresh': genome_data[3]
    }

    sm = smach.StateMachine(outcomes=['succeeded','failed'])

    current_time = current_second
    start_time = current_time
    final_time = current_time + 100.0

    # Start running physics now that everything is setup.
    unpause_physics()

    # Sleep for three seconds allowing the sensors to come online.
    time.sleep(3)

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

    current_time = current_second 
    print("Current Time: "+str(current_time-start_time))

    # Log the basicbot position information.
    log_bot_position(bot_id)
    bot_id += 1
    bot_position = []

    # Publish the fitness on the topic.
    time_objective = current_time-start_time
    dist_objective = scan.getLeftCenterRightScanState()['center']
    pub.publish(time_objective*dist_objective)

    resetWorld()

    # Pause updates until we receive a new genome.
    pause_physics()

###########################

# Initial setup of the node, required services, etc.

ns = rospy.get_namespace()
print(ns)
# Setup the reset world and reset simulation services
rospy.wait_for_service(ns+'/gazebo/get_world_properties')
rospy.wait_for_service(ns+'/gazebo/reset_world')
rospy.wait_for_service(ns+'/gazebo/reset_simulation')
rospy.wait_for_service(ns+'/gazebo/pause_physics')
rospy.wait_for_service(ns+'/gazebo/unpause_physics')

resetWorld = rospy.ServiceProxy(ns+'/gazebo/reset_world', Empty)
resetSimulation = rospy.ServiceProxy(ns+'/gazebo/reset_simulation', Empty)

pause_physics = rospy.ServiceProxy(ns+'/gazebo/pause_physics',Empty)
unpause_physics = rospy.ServiceProxy(ns+'/gazebo/unpause_physics',Empty)

# Setup the messages publisher we will use to drive the robot.
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

# Initialize the node.
rospy.init_node('turn_drive_scan_node', log_level=rospy.WARN, anonymous=True)

# Setup the topic subscribers for getting the state of the robot and sensors.
ls = GetLinkStates()
scan = GetLaserScanner()

# Setup the callbacks for starting and reporting results.
sub = rospy.Subscriber('simulation_start', std_msgs.msg.Empty, simCallback)
pub = rospy.Publisher('simulation_result', std_msgs.msg.Float64, queue_size=1)
clk = rospy.Subscriber(ns+'clock', Clock, clock_callback)

# Spin the node and wait for the callback.
rospy.spin()