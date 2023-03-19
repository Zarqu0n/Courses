#!/usr/bin/env python
import rospy
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.msg import State


class ArmPublisher:
    def __init__(self):
        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)   
        self.state_sub = rospy.Subscriber('mavros/state', State, self.state_cb)
        self.current_state = State()
        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True

    def state_cb(self, msg):
        self.current_state = msg

    def arm(self):
        if self.current_state.armed == False:
            if self.arming_client.call(self.arm_cmd).success == True:
                rospy.loginfo("ARMED")
            else:
                rospy.loginfo("ARMING FAILED")
        else:
            rospy.loginfo("ALREADY ARMED")

if __name__ == '__main__':
    rospy.init_node('arm_pub_node')
    arm_pub = ArmPublisher()
    rospy.sleep(1)
    arm_pub.arm()
    rospy.spin()