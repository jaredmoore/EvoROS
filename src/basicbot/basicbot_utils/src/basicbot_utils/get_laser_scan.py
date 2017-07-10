"""
    Get laser scanner information for the basicbot.
"""

import numpy as np
import rospy

from sensor_msgs.msg import LaserScan

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

        # Correct for infinite values by replacing them with max range.
        self.formatted_msg = {'time':str(msg.header.stamp.secs)+"."+str(msg.header.stamp.nsecs), 'sum_ranges':sum([float("{0:.6f}".format(i)) if i != np.inf else msg.range_max for i in msg.ranges]), 'ranges':[float("{0:.6f}".format(i)) if i != np.inf else msg.range_max for i in msg.ranges]}

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