#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float64

class JointPub(object):
    def __init__(self):

        self.publishers_array = []
        self._FL_hip_pub = rospy.Publisher('/a1_gazebo/FL_hip_controller/command', Float64, queue_size=1)
        self._FL_thigh_pub = rospy.Publisher('/a1_gazebo/FL_thigh_controller/command', Float64, queue_size=1)
        self._FL_calf_pub = rospy.Publisher('/a1_gazebo/FL_calf_controller/command', Float64, queue_size=1)
        self._FR_hip_pub = rospy.Publisher('/a1_gazebo/FR_hip_controller/command', Float64, queue_size=1)
        self._FR_thigh_pub = rospy.Publisher('/a1_gazebo/FR_thigh_controller/command', Float64, queue_size=1)
        self._FR_calf_pub = rospy.Publisher('/a1_gazebo/FR_calf_controller/command', Float64, queue_size=1)
        self._RL_hip_pub = rospy.Publisher('/a1_gazebo/RL_hip_controller/command', Float64, queue_size=1)
        self._RL_thigh_pub = rospy.Publisher('/a1_gazebo/RL_thigh_controller/command', Float64, queue_size=1)
        self._RL_calf_pub = rospy.Publisher('/a1_gazebo/RL_calf_controller/command', Float64, queue_size=1)
        self._RR_hip_pub = rospy.Publisher('/a1_gazebo/RR_hip_controller/command', Float64, queue_size=1)
        self._RR_thigh_pub = rospy.Publisher('/a1_gazebo/RR_thigh_controller/command', Float64, queue_size=1)
        self._RR_calf_pub = rospy.Publisher('/a1_gazebo/RR_calf_controller/command', Float64, queue_size=1)
        
        self.publishers_array.append(self._FL_hip_pub)
        self.publishers_array.append(self._FL_thigh_pub)
        self.publishers_array.append(self._FL_calf_pub)
        self.publishers_array.append(self._FR_hip_pub)
        self.publishers_array.append(self._FR_thigh_pub)
        self.publishers_array.append(self._FR_calf_pub)
        self.publishers_array.append(self._RL_hip_pub)
        self.publishers_array.append(self._RL_thigh_pub)
        self.publishers_array.append(self._RL_calf_pub)
        self.publishers_array.append(self._RR_hip_pub)
        self.publishers_array.append(self._RR_thigh_pub)
        self.publishers_array.append(self._RR_calf_pub)
       
        self.init_pos = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3]

    def set_init_pose(self):
        self.check_publishers_connection()
        self.move_joints(self.init_pos)

    def check_publishers_connection(self):
        rate = rospy.Rate(10)  # 10hz
        while (self._FL_hip_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _FL_hip_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_FL_hip_pub Publisher Connected")

        while (self._FL_thigh_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _FL_thigh_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_FL_thigh_pub Publisher Connected")

        while (self._FL_calf_pub.get_num_connections() == 0):
            rospy.logdebug("No susbribers to _FL_calf_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_FL_calf_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")

    def joint_mono_des_callback(self, msg):
        rospy.logdebug(str(msg.joint_state.position))

        self.move_joints(msg.joint_state.position)

    def move_joints(self, joints_array):

        i = 0
        for publisher_object in self.publishers_array:
          joint_value = Float64()
          joint_value.data = joints_array[i]
          rospy.loginfo(str(joint_value))
          publisher_object.publish(joint_value)
          i += 1


    def start_loop(self, rate_value = 2.0):
        rospy.loginfo("Start Loop")
        pos1 = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 
                0.0, 0.67, -1.3, 0.0, 0.67, -1.3]
        pos2 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        position = "pos1"
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
          if position == "pos1":
            self.move_joints(pos1)
            position = "pos2"
          else:
            self.move_joints(pos2)
            position = "pos1"
          rate.sleep()


if __name__=="__main__":
    rospy.init_node('joint_publisher_node')
    joint_publisher = JointPub()
    rate_value = 4
    joint_publisher.start_loop(rate_value)
