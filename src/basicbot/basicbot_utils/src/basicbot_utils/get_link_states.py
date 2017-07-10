"""
    Get the link states of the basicbot and return in a formatted message.
"""

import rospy

from gazebo_msgs.msg import LinkStates

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