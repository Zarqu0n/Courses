# UAV Takeoff Publisher with Mavros and usign service
#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest

current_state = State()

class drone:
    def __init__(self):
        self.state_sub = rospy.Subscriber("mavros/state", State, self.state_cb)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)   
        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True
        self.current_state = State()
        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True

    def state_cb(self, msg):
        self.current_state = msg

    def takeoff(self):
        if self.current_state.armed == False:
            if self.arming_client.call(self.arm_cmd).success == True:
                rospy.loginfo("ARMED")
            else:
                rospy.loginfo("ARMING FAILED")
        else:
            rospy.loginfo("ALREADY ARMED")
        try:
            takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
            response = takeoff_cl(altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Takeoff failed: %s" %e)

if __name__ == "__main__":
    rospy.init_node("offb_node_py")
    drone = drone()
    drone.takeoff()