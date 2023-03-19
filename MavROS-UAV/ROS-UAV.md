tags: #course 
date: 19.03.2023 
[ROS with Gazebo](https://docs.px4.io/main/en/simulation/ros_interface.html)

---
#### Node, topic , publisher, subscriber,Service-client
![[Nodes-TopicandService.gif]]


# Open MavROS
#### Open SIM
```sh
noetic
make px4_sitl_default gazebo_plane_cam
```

#### Launch MavROS
```sh
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

#### For multiple mavlink port
```sh
cd <PX4-Autopilot_clone>
DONT_RUN=1 make px4_sitl_default gazebo-classic
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch px4 posix_sitl.launch
```


## Crate new Workspace
```sh
mkdir -p <project_name>/src
```

### Create new package
```sh
catkin_create_pkg <pkg_name> <dependencies>
```

#### Creating Node
>[!Note]
>Don't forget write shebang
> `#!usr/bin/env pyhton3`

Uncomment and write <node_names.py> in CMakeList.py at 161.line
```cmake
catkin_install_python(PROGRAMS

scripts/sub.py

DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}

)
```

### Subscriber



```python
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
    
```
### Publisher
```python
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
```
