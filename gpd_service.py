#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, Trigger, TriggerResponse
import os


GPD_PATH = "/home/tpatten/Code/gpd/"
DATA_PATH = os.path.join(GPD_PATH, "hsrb_data")
CONFIG_PATH = os.path.join(GPD_PATH, "cfg")
CONFIG_FILE = "hsrb_params.cfg"
PCD_FILE = "temp.pcd"
GRASPS_FILE = "grasps.txt"
PROGRAM_NAME = os.path.join(GPD_PATH, "build/./detect_grasps_hsrb")
PROGRAM_ARGUMENTS = os.path.join(CONFIG_PATH, CONFIG_FILE) + " " + os.path.join(DATA_PATH, PCD_FILE) + " " + os.path.join(DATA_PATH, GRASPS_FILE)

class GPDService:
    def __init__(self):
        # Advertise service
        self.service = rospy.Service('/gpd_detect_grasps', Trigger, self.service_callback)
        rospy.loginfo('Service ready...')
        
    def service_callback(self, req):
        rospy.loginfo('GPD service called')
        ret = TriggerResponse()
        ret.success = False
        
        cmd = PROGRAM_NAME + " " + PROGRAM_ARGUMENTS
        print('CMD ' + cmd)
        cmd_res = os.system(cmd)
        print('CMD returned ' + str(cmd_res))

        if cmd_res == 0:
            rospy.loginfo('GPD service succeeded')
            ret.success = True
        else:
            rospy.logwarn('GPD service failed')

        return ret

if __name__ == '__main__':
    rospy.init_node('GPD_service')
    rospy.loginfo('Starting GPD service')
    gpd_service = GPDService()
    rospy.spin()
