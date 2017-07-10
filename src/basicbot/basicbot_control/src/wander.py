#!/usr/bin/env python
import random
import rospy

from std_srvs.srv import Empty

from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetWorldProperties
from gazebo_msgs.msg import LinkStates

from world_step.srv import step_world 

class WorldStep(object):
    """ Connect to the step_world service and handle stepping the physics simulation. """

    def __init__(self):
        # Wait for the StepWorld node to start.
        rospy.wait_for_service('step_world')
        self.step = rospy.ServiceProxy('step_world',step_world)
    
    def stepPhysics(self,steps=1):
        """ Step the simulation. """
        self.step()


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
rospy.init_node('wander')

ls = GetLinkStates()

for i in range(10):
    wander_twist = Twist()
    wander_twist.linear.x = 0.5

    turn_twist = Twist()
    turn_twist.linear.x = 0.0
    turn_twist.angular.z = 0.5

    driving_forward = False
    state_change_time = getWorldProp().sim_time 

    ws.stepPhysics(steps=1)
    current_time = getWorldProp().sim_time 
    final_time = getWorldProp().sim_time + 20.0
    print("Final Time: "+str(final_time)+", Current Time: "+str(current_time))
    ws.stepPhysics(steps=1)
    
    while not rospy.is_shutdown():
        ws.stepPhysics(steps=1)
        if driving_forward:
            cmd_vel_pub.publish(turn_twist)
        else:
            cmd_vel_pub.publish(wander_twist)
        if state_change_time < getWorldProp().sim_time:
            driving_forward = not driving_forward
            turn_twist.angular.z = turn_twist.angular.z# * random.choice([-1,1])
            state_change_time = getWorldProp().sim_time + 2.5 
            print(str(getWorldProp().sim_time)+","+str(ls.getLinkPose('basicbot::base_link').position.x)+","+str(ls.getLinkPose('basicbot::base_link').position.y))
        if final_time <= getWorldProp().sim_time:
            break
    
    print(str(getWorldProp().sim_time)+","+str(ls.getLinkPose('basicbot::base_link').position.x)+","+str(ls.getLinkPose('basicbot::base_link').position.y))

    resetWorld()
    resetSimulation()
