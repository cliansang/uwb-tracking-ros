#/##!/usr/bin/env python3
#/#""" 
#/#    For more info on the documentation of TREK1000/EVK1000, go to the following links:
#/#    1. https://www.decawave.com/wp-content/uploads/2018/09/trek1000_user_manual.pdf
#/#    2. https://www.decawave.com/product-documentation/    
"""

#/#import rospy, time, serial, os
#/#from geometry_msgs.msg import PoseStamped
#/#from rospy.core import loginfo
#/#from uwb_tracking_ros.msg import Tag2AnchorRanges
#/#import numpy as np


#/#class trek1000_localizer:

#/#    def __init__(self) :
#/#        """
#/#        Initialize the node and open serial port
#/#        """        
#/#        # Init node
#/#        rospy.init_node('TREK1000_Anchor_ID0', anonymous=False)

#/#        # allow serial port to be detected by user
#/#        # NOTE: USB is assumed to be connected to ttyACM0. If not, need to modified it.
#/#        os.popen("sudo chmod 777 /dev/ttyACM0", "w")  
#/#        
#/#        # Set a ROS rate
#/#        self.rate = rospy.Rate(1)
#/#        
#/#        # Empty dictionary to store topics being published
#/#        self.distTopics = {}
#/#        self.poseTopics = {}
#/#        self.mat_Anc =[]
#/#        
#/#        # Serial port settings
#/#        self.dwm_port = rospy.get_param('~port')        
#/#        self.verbose = rospy.get_param('~verbose', True)
#/#        self.serialPortTREK1000 = serial.Serial(
#/#            port = self.dwm_port,
#/#            baudrate = 115200,
#/#            parity = serial.PARITY_ODD,
#/#            stopbits = serial.STOPBITS_ONE, # STOPBITS_TWO
#/#            bytesize = serial.SEVENBITS
#/#        )
#/#    

#/#    def main(self) :
#/#        """
#/#        Initialize port and dwm1001 api
#/#        :param:
#/#        :returns: none
#/#        """

#/#        # close the serial port in case the previous run didn't closed it properly
#/#        self.serialPortTREK1000.close()
#/#        # sleep for one sec
#/#        time.sleep(1)
#/#        # open serial port
#/#        self.serialPortTREK1000.open()

#/#        # check if the serial port is opened
#/#        if(self.serialPortTREK1000.isOpen()):
#/#            rospy.loginfo("Port opened: "+ str(self.serialPortTREK1000.name) )
#/#            # send some commands to the board via serial for streaming the data
#/#            self.initializeTREK1000()
#/#            # give some time to wake up the EVK1000 module
#/#            time.sleep(2)           
#/#            rospy.loginfo("Reading Data from TREK1000 Serial Port")
#/#        else:
#/#            rospy.loginfo("Can't open port: "+ str(self.serialPortTREK1000.name))

#/#        try:

#/#            while not rospy.is_shutdown():
#/#                # just read everything from serial port
#/#                serialReadLine = self.serialPortTREK1000.read_until()  
#/#                # print(str(serialReadLine)) # just for debug 

#/#                meas_tag_dist = []
#/#                meas_anc_dist = []

#/#                try:
#/#                    serDataList = [x.strip() for x in serialReadLine.strip().split()] # split(b'\t')
#/#                    # print(str(serialDataArray)) # just for debug
#/#                    # print(len(serDataList))

#/#                    # The corrected ranges between tag and anchors (i.e., calibrated ranges upon EVK1000 settings)
#/#                    if b"mc" in serDataList[0]:

#/#                        # The ranges from USB are in millimeter with 32-bit HEX value 
#/#                        # Therefore, it is needed to tranform them into double or float in meter                        
#/#                        dist_t2a0 = float((int(serDataList[2], base=16))/1000)  # Tag to A0 
#/#                        dist_t2a1 = float((int(serDataList[3], base=16))/1000)  # Tag to A1 
#/#                        dist_t2a2 = float((int(serDataList[4], base=16))/1000)  # Tag to A2 
#/#                        dist_t2a3 = float((int(serDataList[5], base=16))/1000)  # Tag to A3                        
#/#           
#/#                        id_mask = serDataList[9]
#/#                        tag_id_mask = [id.strip() for id in id_mask.strip().split(b':')]
#/#                        # tag_id = tag_id_mask[0]
#/#                        tag_id = str(tag_id_mask[0])
#/#                        # print(tag_id[3])  # the id number
#/#                        
#/#                        # tag 2 anc measrued distances including the tag id 
#/#                        meas_tag_dist = [tag_id[3], dist_t2a0, dist_t2a1, dist_t2a2, dist_t2a3]
#/#                        self.publishTagDistances(meas_tag_dist)

#/#                        # Estimate the position of the unknown tag based on the measured ranges 
#/#                        est_tag_pose = self.estimateTagPosition(meas_tag_dist)
#/#                        tag_position = [tag_id[3], est_tag_pose[0], est_tag_pose[1], est_tag_pose[2]]                                              
#/#                        self.publishTagPositions(tag_position)
#/#                    
#/#                    # The anchor to anchor measurement data for auto-positioning of the Anchor nodes
#/#                    if b'ma' in serDataList[0]:
#/#                        dist_a02a1 = serDataList[3]  # A0 to A1 range (in mm, 32-bit HEX)
#/#                        dist_a02a2 = serDataList[4]  # A0 to A2 range 
#/#                        dist_a12a2 = serDataList[5]  # A1 to A2 range

#/#                        meas_a2a_dist = [dist_a02a1, dist_a02a2, dist_a12a2] # list of anchor to anchor ranges
#/#                 
#/#                    # Uncorrected raw measured ranges in TREK1000 (The measurement may include errors due to uncalibrated signal attenuation)
#/#                    if b"mr" in serDataList[0]:                                             
#/#                        dist_t2a0_raw = float((int(serDataList[2], base=16))/1000)  # Tag to A0 
#/#                        dist_t2a1_raw = float((int(serDataList[3], base=16))/1000)  # Tag to A1 
#/#                        dist_t2a2_raw = float((int(serDataList[4], base=16))/1000)  # Tag to A2 
#/#                        dist_t2a3_raw = float((int(serDataList[5], base=16))/1000)  # Tag to A3                        
#/#           
#/#                        id_mask_raw = serDataList[9]
#/#                        tag_id_mask_raw = [id.strip() for id in id_mask_raw.strip().split(b':')]
#/#                        tag_id_raw = str(tag_id_mask_raw[0])                        
#/#                        
#/#                        # meas_tag_dist_raw = [tag_id_raw[3], dist_t2a0_raw, dist_t2a1_raw, dist_t2a2_raw, dist_t2a3_raw]
#/#                        # self.publishTagDistances(meas_tag_dist_raw)
#/#                         
#/#                        # est_tag_pose_raw = self.estimateTagPosition(meas_tag_dist)
#/#                        # tag_position_raw = [tag_id_raw[3], est_tag_pose_raw[0], est_tag_pose_raw[1], est_tag_pose_raw[3]]
#/#                        # self.publishTagPositions(tag_position_raw)

#/#                except IndexError:
#/#                    rospy.loginfo("Found index error! Do something!")

#/#        except KeyboardInterrupt:
#/#            rospy.loginfo("Quitting and closing the serial port, allow 1 second for UWB recovery")
#/#            time.sleep(1)            

#/#        # finally:
#/#        #     rospy.loginfo("Quitting, and sending reset command to dev board")
#/#        #     # self.serialPortTREK1000.reset_input_buffer()

#/#    
#/#    def publishTagDistances(self, estimatedTagDistances):

#/#        tag_id = estimatedTagDistances[0]
#/#        # tag_id = str(estimatedTagDistances[0], 'UTF8')       
#/#        t_dist = Tag2AnchorRanges(tag_id,
#/#                    float(estimatedTagDistances[1]),
#/#                    float(estimatedTagDistances[2]),
#/#                    float(estimatedTagDistances[3]),
#/#                    float(estimatedTagDistances[4]))
#/#        # rospy.loginfo(t_dist) # just for debug 
#/#        
#/#        if tag_id not in self.distTopics :
#/#            self.distTopics[tag_id] = rospy.Publisher("/trek1000/id_t" + tag_id + "/ranges", Tag2AnchorRanges, queue_size=10)
#/#           
#/#        self.distTopics[tag_id].publish(t_dist)
#/#        # print(t_dist)

#/#        # if self.verbose :
#/#        #     rospy.loginfo("Range " + str(tag_id) + ": "
#/#        #         + " A0: "
#/#        #         + str(t_dist.dist_t2a0)
#/#        #         + " A1: "
#/#        #         + str(t_dist.dist_t2a1)
#/#        #         + " A2: "
#/#        #         + str(t_dist.dist_t2a2)
#/#        #         + " A3 "
#/#        #         + str(t_dist.dist_t2a3)
#/#        #         )
 

#/#    def publishTagPositions(self, estPoseData):

#/#        tag_id = estPoseData[0]
#/#        # tag_id = str(estPoseData[0], 'UTF8') 
#/#        ps = PoseStamped()
#/#        ps.pose.position.x = float(estPoseData[1])
#/#        ps.pose.position.y = float(estPoseData[2])
#/#        ps.pose.position.z = float(estPoseData[3])
#/#        ps.pose.orientation.x = 0.0
#/#        ps.pose.orientation.y = 0.0
#/#        ps.pose.orientation.z = 0.0
#/#        ps.pose.orientation.w = 1.0
#/#        ps.header.stamp = rospy.Time.now()   
#/#        ps.header.frame_id = "id_t"+ tag_id  # id number
#/#        # print(ps)

#/#        if tag_id not in self.poseTopics :
#/#            self.poseTopics[tag_id] = rospy.Publisher("/trek1000/id_t" + str(tag_id) + "/pose", PoseStamped, queue_size=10)
#/#            
#/#            #rospy.loginfo("Pose Tag {}. x: {}m, y: {}m, z: {}m".format(
#/#            #    str(tag_id),
#/#            #    ps.pose.position.x,
#/#            #    ps.pose.position.y,
#/#            #    ps.pose.position.z
#/#            #))
#/#            
#/#        self.poseTopics[tag_id].publish(ps)

#/#        # if self.verbose :
#/#        #     rospy.loginfo("Tag " + str(tag_id) + ": "
#/#        #         + " x: "
#/#        #         + str(ps.pose.position.x)
#/#        #         + " y: "
#/#        #         + str(ps.pose.position.y)
#/#        #         + " z: "
#/#        #         + str(ps.pose.position.z)
#/#        #     )
#/#    

#/#    def estimateTagPosition(self, t2a_dist):
#/#        """
#/#        Estimate the unknown tag postion based on the known anchors set-up and measured ranges 
#/#        :returns: calculated tag's position (x, y, z)
#/#        : Note: we applied True-range mulitilateration method.  The details can be found in our paper 
#/#                https://ieeexplore.ieee.org/abstract/document/8911811
#/#        """

#/#        tag_pose = np.zeros((3,1))  # place holder 

#/#        # Suppose the known anchor positions are as follows (as a default)      
#/#        anc_0 = [0, 0, 2.6]
#/#        anc_1 = [5.5, 0, 2.6]
#/#        anc_2 = [5.5, 5.6, 2.6]
#/#        anc_3 = [0, 5.7, 2.6]

#/#        matA = np.array([[anc_1[0] - anc_0[0],  anc_1[1] - anc_0[1],  anc_1[2] - anc_0[2]],
#/#              [anc_2[0] - anc_0[0],  anc_2[1] - anc_0[1],  anc_2[2] - anc_0[2]],
#/#              [anc_3[0] - anc_0[0],  anc_3[1] - anc_0[1],  anc_3[2] - anc_0[2]] ])
#/#        # print(matA)

#/#        # Limitation of True-Range Multiliteration method/algorithm (see the details on our paper mentioned above):
#/#        # All the coordinates of the anchors cannot lie at a collinear line in 2D (i.e. all 3 anchors on a line) and/or 
#/#        # a coplanar plane in 3D (i.e. all 4 or more anchors are on the same plane, e.g. they all have the same high in Z-axis)
#/#        # The reason is that, this would cause the matrix "A.transpose(A)" to become a singular (or) non-invertible matrix
#/#        # print("\tThe matrix Rank must be 2 for 2D and 3 for 3D!.\n \
#/#        # Otherwise, rearrange your anchors' coordinates.\n \
#/#        # Rank of the current anchors' setup is: {}\n".format(np.linalg.matrix_rank(matA)))

#/#        anc_col_rank = np.linalg.matrix_rank(matA)
#/#        print("Anchors column rank: ", anc_col_rank)

#/#        if (anc_col_rank == 3 and t2a_dist[3] != 0):
#/#            dim = 3     # 3D 
#/#        else:
#/#            dim = 2     # 2D 

#/#        sqr_a0 = np.square(anc_0)
#/#        sqr_a1 = np.square(anc_1)
#/#        sqr_a2 = np.square(anc_2)
#/#        sqr_a3 = np.square(anc_3)

#/#        b_0 = np.array([np.square(t2a_dist[1]) - np.square(t2a_dist[2])  
#/#                + (sqr_a1[0] + sqr_a1[1] + sqr_a1[2]) 
#/#                - (sqr_a0[0] + sqr_a0[1] + sqr_a0[2])])

#/#        b_1 = np.array([np.square(t2a_dist[1]) - np.square(t2a_dist[3])  
#/#                + (sqr_a2[0] + sqr_a2[1] + sqr_a2[2]) 
#/#                - (sqr_a0[0] + sqr_a0[1] + sqr_a0[2])])

#/#        b_2 = np.array([np.square(t2a_dist[1]) - np.square(t2a_dist[4])  
#/#                + (sqr_a3[0] + sqr_a3[1] + sqr_a3[2]) 
#/#                - (sqr_a0[0] + sqr_a0[1] + sqr_a0[2])])

#/#        vec_b = (np.array([b_0, b_1, b_2]))/2.0 

#/#        # For 2D  and 3D position data of Tag in TREK1000 set-up
#/#        if dim == 2:
#/#            A = matA[0:2, :]
#/#            b = vec_b[0:2]
#/#        else:      
#/#            A = matA[:] 
#/#            b = vec_b[:]
#/#       
#/#        # The unknown position of the Tag can now be calculated with the least-square method
#/#        AtrA = A.transpose().dot(A)
#/#        AtrA_inv = np.linalg.pinv(AtrA)
#/#        Atrb = A.transpose().dot(b)

#/#        tag_loc_data = AtrA_inv.dot(Atrb)
#/#        # tag_pose = (np.linalg.pinv(np.transpose(A).dot(A))).dot(np.transpose(A).dot(b))

#/#        if dim == 2:
#/#            tag_pose[0:2] = tag_loc_data[0:2]
#/#            tag_pose[2] = 0    # undefined z-axis in 2D 
#/#        else:
#/#            tag_pose = tag_loc_data[:] # tag location in 3D 

#/#        print("Pose is calculated for {}D: {}\n".format(dim, tag_pose))
#/#        return tag_pose


#/#    def initializeTREK1000(self):
#/#        """
#/#        Initialize TREK1000 by sending sending bytes
#/#        :returns: none
#/#        """
#/#        # send this command to quiry the data via Serial port
#/#        self.serialPortTREK1000.write(b'deca$q')
#/#        # self.serialPortTREK1000.write(b'deca?')  # show available commands

#/#        # sleep for half a second
#/#        time.sleep(0.5)
#/#        # send the second time to make it sure (just for in case) 
#/#        self.serialPortTREK1000.write(b'deca$q') 


#/#def start():
#/#    trek1000 = trek1000_localizer()
#/#    trek1000.main()


#/#if __name__ == '__main__':
#/#    try:
#/#        start()
#/#        rospy.spin()
#/#    except rospy.ROSInterruptException:
#/#        pass
#/#        
