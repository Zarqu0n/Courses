### create basic ROS subscriber

import rospy
from std_msgs.msg import String
from mavros_msgs.msg import State

class Sub:
    def __init__(self):
        self.sub = rospy.Subscriber('mavros/state', State, self.callback)

    def callback(self, msg):
        rospy.loginfo(msg)


if __name__ == '__main__':
    rospy.init_node('sub_node') 
    sub = Sub()
    rospy.spin()
    