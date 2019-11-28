#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, Trigger, TriggerResponse
import os


GPD_PATH = "/home/tpatten/Code/gpd/"
PROGRAM_NAME = os.path.join(GPD_PATH, "build/./detect_grasps")
PROGRAM_ARGUMENTS = os.path.join(GPD_PATH, "cfg/eigen_params.cfg")
PCD_FILE = os.path.join(GPD_PATH, "tutorials/krylon.pcd")

class GPDService:
    def __init__(self):
        # Advertise service
        self.service = rospy.Service('/gpd_detect_grasps', Trigger, self.service_callback)
        rospy.loginfo('Service ready...')
        
    def service_callback(self, req):
        rospy.loginfo('GPD service called')
        ret = TriggerResponse()
        ret.success = False
        ret.message = "Location"
        
        cmd = PROGRAM_NAME + " " + PROGRAM_ARGUMENTS + " " + PCD_FILE
        print(cmd)
        os.system(cmd)
        
        return ret

if __name__ == '__main__':
    rospy.init_node('GPD_service')
    rospy.loginfo('Starting GPD service')
    gpd_service = GPDService()
    rospy.spin()
