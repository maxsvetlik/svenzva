#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

def callback(data):
    global finger_load, cur_pos, is_moving
    finger_load = data.load
    cur_pos = data.current_pos
    is_moving = data.is_moving

def talker():
    pub = rospy.Publisher('/finger_controller/command', Float64, queue_size=1)
    rospy.init_node('gripper_demo', anonymous=True)
    rospy.Subscriber("/finger_controller/state", JointState, callback)
    global finger_load, cur_pos, is_moving
    finger_load = 0.0
    cur_pos = 0.0
    is_moving = True
    rate = rospy.Rate(20)
    torque_cutoff = 0.3 #[0,1] and must be lower than the max torque value set for the servos
    #vel = 0.05
    starting_pos = 3.14
    target_pos = 4.17 #4.5
    hit_cutoff = False
    cmd = Float64()
    cmd.data = starting_pos

    ##NOTE:
        """
            TODO: With torque limit in place, can just send to a point
                """


    while not rospy.is_shutdown() and not hit_cutoff:
        rospy.loginfo("Load: %f", finger_load)
        cmd.data = cmd.data + 0.1
        if abs(finger_load) >= torque_cutoff:# or not is_moving:
            hit_cutoff = True
            #cmd.data = cur_pos + 0.1 #- (pos - cur_pos)/2
        pub.publish(cmd)
        rate.sleep()
    #rospy.sleep(0.5)
    #cmd.data = 0.0
    #pub.publish(cmd)

    rospy.loginfo("Exiting node. Gripper should be closed.")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
