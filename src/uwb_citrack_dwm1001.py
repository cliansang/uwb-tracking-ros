#!/usr/bin/env python3
""" 
    For more info on the documentation go to https://www.decawave.com/sites/default/files/dwm1001-api-guide.pdf
    This src is adapted partly from this repo: https://github.com/TIERS/ros-dwm1001-uwb-localization
"""

import rospy, time, serial, os
from dwm1001_apiCommands            import DWM1001_API_COMMANDS

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class dwm1001_localizer:

    def __init__(self) :
        """
        Initialize the node, open serial port
        """
        
        # Init node
        rospy.init_node('DWM1001_Passive', anonymous=False)
        
        # Set a ROS rate
        self.rate = rospy.Rate(1)
        
        # Empty dictionary to store topics being published
        self.topics = {}
        
        # Serial port settings
        self.dwm_port = rospy.get_param('~port')
        self.verbose = rospy.get_param('~verbose', True)
        self.serialPortDWM1001 = serial.Serial(
            port = self.dwm_port,
            baudrate = 115200,
            parity = serial.PARITY_ODD,
            stopbits = serial.STOPBITS_TWO,
            bytesize = serial.SEVENBITS
        )
    

    def main(self) :
        """
        Initialize port and dwm1001 api
        :param:
        :returns: none
        """

        # close the serial port in case the previous run didn't closed it properly
        self.serialPortDWM1001.close()
        # sleep for one sec
        time.sleep(1)
        # open serial port
        self.serialPortDWM1001.open()

        # check if the serial port is opened
        if(self.serialPortDWM1001.isOpen()):
            rospy.loginfo("Port opened: "+ str(self.serialPortDWM1001.name) )
            # start sending commands to the board so we can initialize the board
            self.initializeDWM1001API()
            # give some time to DWM1001 to wake up
            time.sleep(2)
            # send command lec, so we can get positions is CSV format
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.LEC)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            rospy.loginfo("Reading DWM1001 coordinates")
        else:
            rospy.loginfo("Can't open port: "+ str(self.serialPortDWM1001.name))

        try:

            while not rospy.is_shutdown():
                # just read everything from serial port
                serialReadLine = self.serialPortDWM1001.read_until()

                try:
                    self.publishTagPositions(serialReadLine)

                except IndexError:
                    rospy.loginfo("Found index error in the network array!DO SOMETHING!")



        except KeyboardInterrupt:
            rospy.loginfo("Quitting DWM1001 Shell Mode and closing port, allow 1 second for UWB recovery")
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)

        finally:
            rospy.loginfo("Quitting, and sending reset command to dev board")
            # self.serialPortDWM1001.reset_input_buffer()
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
            self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
            self.rate.sleep()
            serialReadLine = self.serialPortDWM1001.read_until()
            if b"reset" in serialReadLine:
                rospy.loginfo("succesfully closed ")
                self.serialPortDWM1001.close()



    def publishTagPositions(self, serialData):
        """
        Publish anchors and tag in topics using Tag and Anchor Object
        :param networkDataArray:  Array from serial port containing all informations, tag xyz and anchor xyz
        :returns: none
        """
 
        arrayData = [x.strip() for x in serialData.strip().split(b',')]

        # If getting a tag position
        if b"POS" in arrayData[0] :
            #rospy.loginfo(arrayData)  # just for debug

            tag_id = str(arrayData[1], 'UTF8')  # IDs in 0 - 15
            tag_macID = str(arrayData[2], 'UTF8')

            p = PoseStamped()
            p.pose.position.x = float(arrayData[3])
            p.pose.position.y = float(arrayData[4])
            p.pose.position.z = float(arrayData[5])
            p.pose.orientation.x = 0.0
            p.pose.orientation.y = 0.0
            p.pose.orientation.z = 0.0
            p.pose.orientation.w = 1.0
            p.header.stamp = rospy.Time.now()   
            p.header.frame_id = tag_macID # TODO: Currently, MAC ID of the Tag is set as a frame ID 

            if tag_id not in self.topics :
                self.topics[tag_id] = rospy.Publisher("/dwm1001/tagId_" + tag_macID + "/pose", PoseStamped, queue_size=100)
                # self.topics[tag_id] = rospy.Publisher("/dwm1001/tagId_"+ str(int(tag_id)) + "/position", PoseStamped, queue_size=100)
                #rospy.loginfo("New tag {}. x: {}m, y: {}m, z: {}m".format(
                #    str(tag_id),
                #    p.pose.position.x,
                #    p.pose.position.y,
                #    p.pose.position.z
                #))
            
            self.topics[tag_id].publish(p)

            if self.verbose :
                rospy.loginfo("Tag " + str(tag_macID) + ": "
                    + " x: "
                    + str(p.pose.position.x)
                    + " y: "
                    + str(p.pose.position.y)
                    + " z: "
                    + str(p.pose.position.z)
                )
            
            # publish tag as Visual Odometry
            # The following code snippet is copied from Michael implementation
            odo = Odometry()
            odo.child_frame_id ="uwb_map" # "base_link"
            odo.header.frame_id = "odom"
            odo.pose.pose.position.x = p.pose.position.x # tag.x
            odo.pose.pose.position.y = p.pose.position.y # tag.y
            odo.pose.covariance = [99, 0, 0, 0, 0, 0,   0, 99, 0, 0, 0, 0,  0, 0, 99, 0, 0, 0,  0, 0,
                                   0, 99999, 0, 0, 0, 0, 0, 0, 99999, 0, 0, 0, 0, 0, 0, 99999]  # large covariance on rot z
            pub_odom = rospy.Publisher('/vo', Odometry, queue_size=1)
            pub_odom.publish(odo)




    def initializeDWM1001API(self):
        """
        Initialize dwm1001 api, by sending sending bytes
        :param:
        :returns: none
        """
        # reset incase previuos run didn't close properly
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.RESET)
        # send ENTER two times in order to access api
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half a second
        time.sleep(0.5)
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)
        # sleep for half second
        time.sleep(0.5)
        # send a third one - just in case
        self.serialPortDWM1001.write(DWM1001_API_COMMANDS.SINGLE_ENTER)



def start():
    dwm1001 = dwm1001_localizer()
    dwm1001.main()

if __name__ == '__main__':
    try:
        start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        