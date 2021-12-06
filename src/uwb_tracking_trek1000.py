#!/usr/bin/env python3
""" 
    For more info on the documentation of TREK1000/EVK1000, go to the following links:
    1. https://www.decawave.com/wp-content/uploads/2018/09/trek1000_user_manual.pdf
    2. https://www.decawave.com/product-documentation/    
"""

import rospy, time, serial, os
from dwm1001_apiCommands import DWM1001_API_COMMANDS
from geometry_msgs.msg import PoseStamped
from rospy.core import loginfo
from uwb_tracking_ros.msg import Tag2AnchorRanges
import numpy as np


class trek1000_localizer:

    def __init__(self) :
        """
        Initialize the node and open serial port
        """        
        # Init node
        rospy.init_node('TREK1000_Anchor_ID0', anonymous=False)

        # allow serial port to be detected by user
        # NOTE: USB is assumed to be connected to ttyACM0. If not, need to modified it.
        os.popen("sudo chmod 777 /dev/ttyACM0", "w")  
        
        # Set a ROS rate
        self.rate = rospy.Rate(1)
        
        # Empty dictionary to store topics being published
        self.topics = {}
        self.mat_Anc =[]
        
        # Serial port settings
        self.dwm_port = rospy.get_param('~port')        
        self.verbose = rospy.get_param('~verbose', True)
        self.serialPortTREK1000 = serial.Serial(
            port = self.dwm_port,
            baudrate = 115200,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_ONE, # STOPBITS_TWO
            bytesize = serial.SEVENBITS
        )
    

    def main(self) :
        """
        Initialize port and dwm1001 api
        :param:
        :returns: none
        """

        # close the serial port in case the previous run didn't closed it properly
        self.serialPortTREK1000.close()
        # sleep for one sec
        time.sleep(1)
        # open serial port
        self.serialPortTREK1000.open()

        # check if the serial port is opened
        if(self.serialPortTREK1000.isOpen()):
            rospy.loginfo("Port opened: "+ str(self.serialPortTREK1000.name) )
            # send some commands to the board via serial for streaming the data
            self.initializeTREK1000()
            # give some time to wake up the EVK1000 module
            time.sleep(2)           
            rospy.loginfo("Reading Data from TREK1000 Serial Port")
        else:
            rospy.loginfo("Can't open port: "+ str(self.serialPortTREK1000.name))

        try:

            while not rospy.is_shutdown():
                # just read everything from serial port
                serialReadLine = self.serialPortTREK1000.read_until()  
                # print(str(serialReadLine)) # just for debug 

                meas_tag_dist = []
                meas_anc_dist = []

                try:
                    serDataList = [x.strip() for x in serialReadLine.strip().split()] # split(b'\t')
                    # print(str(serialDataArray)) # just for debug
                    # print(len(serDataList))

                    # The corrected ranges between tag and anchors (i.e., calibrated ranges upon EVK1000 settings)
                    # For raw ranges measurement without calibration, modify "mc" to "mr"
                    if b"mc" in serDataList[0]:

                        # The ranges from USB are in millimeter with 32-bit HEX value 
                        # Therefore, it is needed to tranform them into floating point no. in meter                        
                        dist_t2a0 = float((int(serDataList[2], base=16))/1000)  # Tag to A0 
                        dist_t2a1 = float((int(serDataList[3], base=16))/1000)  # Tag to A1 
                        dist_t2a2 = float((int(serDataList[4], base=16))/1000)  # Tag to A2 
                        dist_t2a3 = float((int(serDataList[5], base=16))/1000)  # Tag to A3                        
           
                        id_mask = serDataList[9]
                        tag_id_mask = [id.strip() for id in id_mask.strip().split(b':')]
                        tag_id = tag_id_mask[0]
                        
                        # tag 2 anc measrued distances with tag id 
                        meas_tag_dist = [tag_id, dist_t2a0, dist_t2a1, dist_t2a2, dist_t2a3]
                        # print(meas_tag_dist)  # just for debug 

                        self.publishTagDistances(meas_tag_dist)
                    
                    # The anchor to anchor measurement data for auto-positioning of the Anchor nodes
                    if b'ma' in serDataList[0]:
                        dist_a02a1 = serDataList[3]  # A0 to A1 range (in mm, 32-bit HEX)
                        dist_a02a2 = serDataList[4]  # A0 to A2 range 
                        dist_a12a2 = serDataList[5]  # A1 to A2 range

                        meas_anc_dist = [dist_a02a1, dist_a02a2, dist_a12a2]
                        # print(str(meas_anc_dist)) 
                    
                    # This returns list of [Tag ID, tag.x, tag.y, tag.z, A0.x, A0.y, A1.x, A1.y, A2.x, A2.y]
                    # tagPose = self.calculateTagPosition(meas_anc_dist, meas_tag_dist)             

                    # self.publishTagPositions(tagPose)

                except IndexError:
                    rospy.loginfo("Found index error in the Serial Data List !DO SOMETHING!")

        except KeyboardInterrupt:
            rospy.loginfo("Quitting and closing the serial port, allow 1 second for UWB recovery")
            time.sleep(1)            

        # finally:
        #     rospy.loginfo("Quitting, and sending reset command to dev board")
        #     # self.serialPortTREK1000.reset_input_buffer()

    
    def publishTagDistances(self, estimatedTagDistances):

        tag_id = str(estimatedTagDistances[0], 'UTF8')       
        t_dist = Tag2AnchorRanges(tag_id,
                    float(estimatedTagDistances[1]),
                    float(estimatedTagDistances[2]),
                    float(estimatedTagDistances[3]),
                    float(estimatedTagDistances[4]))
        # rospy.loginfo(t_dist) # just for debug 
        
        if tag_id not in self.topics :
            self.topics[tag_id] = rospy.Publisher("/trek1000/id_" + tag_id + "/ranges", Tag2AnchorRanges, queue_size=10)
           
        self.topics[tag_id].publish(t_dist)
        # print(t_dist)

        if self.verbose :
            rospy.loginfo("Range " + str(tag_id) + ": "
                + " A0: "
                + str(t_dist.dist_t2a0)
                + " A1: "
                + str(t_dist.dist_t2a1)
                + " A2: "
                + str(t_dist.dist_t2a2)
                + " A3 "
                + str(t_dist.dist_t2a3)
                )
 

    def publishTagPositions(self, estPoseData):

        tag_id = str(estPoseData[0], 'UTF8') 
        ps = PoseStamped()
        ps.pose.position.x = float(estPoseData[1])
        ps.pose.position.y = float(estPoseData[2])
        ps.pose.position.z = float(estPoseData[3])
        ps.pose.orientation.x = 0.0
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 1.0
        ps.header.stamp = rospy.Time.now()   
        ps.header.frame_id = tag_id

        if tag_id not in self.topics :
            self.topics[tag_id] = rospy.Publisher("/trek1000/id_" + tag_id + "/pose", PoseStamped, queue_size=10)
            #rospy.loginfo("Pose Tag {}. x: {}m, y: {}m, z: {}m".format(
            #    str(tag_id),
            #    ps.pose.position.x,
            #    ps.pose.position.y,
            #    ps.pose.position.z
            #))
            
        self.topics[tag_id].publish(ps)

        if self.verbose :
            rospy.loginfo("Tag " + str(tag_macID) + ": "
                + " x: "
                + str(ps.pose.position.x)
                + " y: "
                + str(ps.pose.position.y)
                + " z: "
                + str(ps.pose.position.z)
            )
            

    def initializeTREK1000(self):
        """
        Initialize TREK1000 by sending sending bytes
        :returns: none
        """
        # send this command to quiry the data via Serial port
        self.serialPortTREK1000.write(b'deca$q')
        # self.serialPortTREK1000.write(b'deca?')  # show available commands

        # sleep for half a second
        time.sleep(0.5)
        # send the second time to make it sure (just for in case) 
        self.serialPortTREK1000.write(b'deca$q') 


def start():
    trek1000 = trek1000_localizer()
    trek1000.main()


if __name__ == '__main__':
    try:
        start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
