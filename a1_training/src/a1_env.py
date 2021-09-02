#!/usr/bin/env python

import gym
import rospy 
import numpy as np
import time
from gym import spaces,utils
from geometry_msgs.msg import Pose
from gym.utils import seeding
from gym.envs.registration import register
from gazebo_connection import GazeboConnection
from joint_publisher import JointPub
from controllers_connection import ControllersConnection
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
import tf
import math

reg = register(
    id='a1-v0',
    entry_point='a1_env:LeggedEnv',
    max_episode_steps=1000,
    )

DEFAULT_SIZE = 500

"""
/FL_foot_ContactSensor_state
/a1_gazebo/FL_calf_controller/command
/a1_gazebo/RL_hip_controller/state
/odom
/trunk_imu
/a1_gazebo/joint_states
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
"""

class LeggedEnv(gym.Env):
    
      def __init__(self):

          self._list_of_observations = ["base_roll",
                 "base_pitch",
                 "base_yaw",
                 "joint_states_FL_hip", 
                 "joint_states_FL_thigh",
                 "joint_states_FL_calf",
                 "joint_states_FR_hip",
                 "joint_states_FR_thigh",
                 "joint_states_FR_calf",
                 "joint_states_RL_hip",
                 "joint_states_RL_thigh",
                 "joint_states_RL_calf ",
                 "joint_states_RR_hip",
                 "joint_states_RR_thigh", 
                 "joint_states_RR_calf"] 

          rospy.Subscriber("/trunk_imu", Imu, self.imu_callback)
          rospy.Subscriber("/odom", Odometry, self.odom_callback)
          rospy.Subscriber("/FL_foot_ContactSensor_state", ContactsState, self.FL_contact_callback)
          rospy.Subscriber("/FR_foot_ContactSensor_state", ContactsState, self.FR_contact_callback)
          rospy.Subscriber("/RL_foot_ContactSensor_state", ContactsState, self.RL_contact_callback)
          rospy.Subscriber("/RR_foot_ContactSensor_state", ContactsState, self.RR_contact_callback)
          rospy.Subscriber("/a1_gazebo/joint_states", JointState, self.joints_state_callback)   

          self.base_position = Point()
          self.base_orientation = Quaternion()
          self.base_linear_acceleration = Vector3()
          self.joints_state = JointState()
          self.n_actions = 12
          self.running_step = 0.001
          self.num_envs = 1

          self.gazebo = GazeboConnection()
          self.joint_publisher = JointPub()
          self.controllers_object = ControllersConnection(namespace="a1_gazebo")

          self.set_action_space()
          self.reward_range = (-np.inf, np.inf)
          self.set_observation_space()
          
          self.seed()

      def set_action_space(self):
          self.action_space = spaces.Box(-3., 3., shape=(self.n_actions,), dtype='float64')
          return self.action_space

      def set_observation_space(self):
          obs = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0,0,0,0]
          obs = np.array(obs)
          low = np.full(obs.shape, -float('inf'), dtype=np.float32)
          high = np.full(obs.shape, float('inf'), dtype=np.float32)
          self.observation_space = spaces.Box(low, high, dtype=np.float32)
          return self.observation_space

      def _render(self,
               mode='human',
               width=DEFAULT_SIZE,
               height=DEFAULT_SIZE,
               camera_id=None,
               camera_name=None):
        if mode == 'rgb_array' or mode == 'depth_array':
            if camera_id is not None and camera_name is not None:
                raise ValueError("Both `camera_id` and `camera_name` cannot be"
                                 " specified at the same time.")

            no_camera_specified = camera_name is None and camera_id is None
            if no_camera_specified:
                camera_name = 'track'

            if camera_id is None and camera_name in self.model._camera_name2id:
                camera_id = self.model.camera_name2id(camera_name)

            self._get_viewer(mode).render(width, height, camera_id=camera_id)

        if mode == 'rgb_array':
            # window size used for old mujoco-py:
            data = self._get_viewer(mode).read_pixels(width, height, depth=False)
            # original image is upside-down, so flip it
            return data[::-1, :, :]
        elif mode == 'depth_array':
            self._get_viewer(mode).render(width, height)
            # window size used for old mujoco-py:
            # Extract depth part of the read_pixels() tuple
            data = self._get_viewer(mode).read_pixels(width, height, depth=True)[1]
            # original image is upside-down, so flip it
            return data[::-1, :]
        elif mode == 'human':
            self._get_viewer(mode).render()

      def close(self):

        rospy.logdebug("Closing RobotGazeboEnvironment")
        rospy.signal_shutdown("Closing RobotGazeboEnvironment")
        
      def seed(self, seed=None):
          self.np_random, seed = seeding.np_random(seed)
          return [seed]
  
      """def check_all_systems_ready(self):
        data_pose = None
        while data_pose is None and not rospy.is_shutdown():
            try:
                data_pose = rospy.wait_for_message("/odom", Odometry, timeout=0.1)
                self.base_pose = data_pose.pose.pose.position
                rospy.logdebug("Current odom READY")
            except:
                rospy.logdebug("Current odom pose not ready yet, retrying for getting robot base_position")


        while contacts_data is None and not rospy.is_shutdown():
            try:
                contacts_data = rospy.wait_for_message("/FL_foot_ContactSensor_state", ContactsState, timeout=0.1)
                for state in contacts_data.states:
                    self.contact_force = state.total_wrench.force
            except:
                rospy.logdebug("Current contacts_data not ready yet, retrying")
      
        joint_states_msg = None
        while joint_states_msg is None and not rospy.is_shutdown():
            try:
                joint_states_msg = rospy.wait_for_message("/a1_gazebo/joint_states", JointState, timeout=0.1)
                self.joints_state = joint_states_msg
            except Exception as e:
                rospy.logdebug("Current joint_states not ready yet, retrying==>"+str(e))

        rospy.logdebug("ALL SYSTEMS READY")"""

      def reset(self):


        self.gazebo.resetSim()

        self.gazebo.change_gravity(0.0, 0.0, 0.0)

        try:
            self.controllers_object.reset_joint_controllers()
        except:
            print("reset_joint_controllers failed")

        try:
            self.joint_publisher.set_init_pose()
        except:
            print("set init pose failed")

        self.joint_publisher.check_publishers_connection()

        """try:
            self.check_all_systems_ready()
        except:
            print("check_systems_ready failed")"""
        self.get_joint_states()
        time.sleep(0.05)
        print(self.joints_state.position)
        try:
            observation = self.get_observations()
        except:
            print("get_observations failed during reset")
       
        self.gazebo.change_gravity(0.0, 0.0, -9.81)
        self.gazebo.pauseSim()

        try:
            self.base_height_original=self.base_position.z
        except:
            print("base_height_original set failed!")

        return observation

      def step(self, action):
        
        self.last_base_pose = self.base_position.x
        print(self.last_base_pose)

        next_action_position = self.get_action_to_position(action)

        self.gazebo.unpauseSim()
        self.joint_publisher.move_joints(next_action_position)
        time.sleep(0.5)
        observation = self.get_observations()
        reward,done = self.process_data()

        return observation, reward, done, {}

      def get_observations(self):
        contact_list = self.get_foot_contact()
        print("getting observation")
        base_orientation = self.get_base_rpy()
        base_roll = base_orientation.x
        base_pitch = base_orientation.y
        base_yaw = base_orientation.z
        
        print("get base orientation ok")
        joint_states = self.get_joint_states()
        print("get joint_states ok")
        joint_states_FL_hip = joint_states.position[0]
        print("joint position[0] ok")
        joint_states_FL_thigh = joint_states.position[1]
        joint_states_FL_calf = joint_states.position[2]
        joint_states_FR_hip = joint_states.position[3]
        joint_states_FR_thigh = joint_states.position[4]
        joint_states_FR_calf = joint_states.position[5]
        joint_states_RL_hip = joint_states.position[6]
        joint_states_RL_thigh = joint_states.position[7]
        joint_states_RL_calf = joint_states.position[8]
        joint_states_RR_hip = joint_states.position[9]
        joint_states_RR_thigh = joint_states.position[10]
        joint_states_RR_calf = joint_states.position[11]
        print("joint_states defined")
           
        observation = []
        for obs_name in self._list_of_observations:
            if obs_name == "base_roll":
                observation.append(base_roll)
                print("append base_roll success")
            elif obs_name == "base_pitch":
                observation.append(base_pitch)
            elif obs_name == "base_yaw":
                observation.append(base_yaw)
            elif obs_name == "joint_states_FL_hip":
                observation.append(joint_states_FL_hip)
                print("get joint_states_FL_hip success")
            elif obs_name == "joint_states_FL_thigh":
                observation.append(joint_states_FL_thigh)
            elif obs_name == "joint_states_FL_calf":
                observation.append(joint_states_FL_calf)
            elif obs_name == "joint_states_FR_hip":
                observation.append(joint_states_FR_hip)
            elif obs_name == "joint_states_FR_thigh":
                observation.append(joint_states_FR_thigh)
            elif obs_name == "joint_states_FR_calf":
                observation.append(joint_states_FR_calf)
            elif obs_name == "joint_states_RL_hip":
                observation.append(joint_states_RL_hip)
            elif obs_name == "joint_states_RL_thigh":
                observation.append(joint_states_RL_thigh)
            elif obs_name == "joint_states_RL_calf":
                observation.append(joint_states_RL_calf)
            elif obs_name == "joint_states_RR_hip":
                observation.append(joint_states_RR_hip)
            elif obs_name == "joint_states_RR_thigh":
                observation.append(joint_states_RR_thigh)
            else:
                observation.append(joint_states_RR_calf)
               
        for contact in contact_list:
            observation.append(contact)
        observation=np.array(observation)
        print(observation)
        return observation
         
      def FL_contact_callback(self,msg):    
          if not msg.states:
              self.FL_contact = 0
          else:
              self.FL_contact = 1

      def FR_contact_callback(self,msg):    
          if not msg.states:
              self.FR_contact = 0
          else:
              self.FR_contact = 1

      def RL_contact_callback(self,msg):    
          if not msg.states:
              self.RL_contact = 0
          else:
              self.RL_contact = 1

      def RR_contact_callback(self,msg):   
 
          if not msg.states:
              self.RR_contact = 0
          else:
              self.RR_contact = 1

      def imu_callback(self,msg):
          self.base_orientation = msg.orientation
          self.base_angular_velocity = msg.angular_velocity
          self.base_linear_acceleration = msg.linear_acceleration


      def joints_state_callback(self,msg):
           self.joints_state = msg

      def get_joint_states(self):
          return self.joints_state
 
      def odom_callback(self,msg):
          self.base_position = msg.pose.pose.position

      def get_foot_contact(self):
          contact_list = [self.FL_contact,self.FR_contact,self.RL_contact,self.RR_contact]
          return contact_list

      def get_base_rpy(self):
          euler_rpy = Vector3()
          euler = tf.transformations.euler_from_quaternion(
            [self.base_orientation.x, self.base_orientation.y, self.base_orientation.z, self.base_orientation.w])
          euler_rpy.x = euler[0]
          euler_rpy.y = euler[1]
          euler_rpy.z = euler[2]
          return euler_rpy

      def get_action_to_position(self,action):
           joint_states = self.get_joint_states()
           joint_states_position=joint_states.position
           action_position = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
           action = action[0]
           print(action)
           action_position[0] = joint_states_position[0] + action[0]
           action_position[1] = joint_states_position[1] + action[1]
           action_position[2] = joint_states_position[2] + action[2]
           action_position[3] = joint_states_position[3] + action[3]
           action_position[4] = joint_states_position[4] + action[4]
           action_position[5] = joint_states_position[5] + action[5]
           action_position[6] = joint_states_position[6] + action[6]
           action_position[7] = joint_states_position[7] + action[7]
           action_position[8] = joint_states_position[8] + action[8]
           action_position[9] = joint_states_position[9] + action[9]
           action_position[10] = joint_states_position[10] + action[10]
           action_position[11] = joint_states_position[11] + action[11]

           return action_position

      def process_data(self):

           base_height_now=self.base_position.z
           height_ok = 0.4*self.base_height_original<= base_height_now < 1.5*self.base_height_original
           done = not height_ok
           
           velocity = (self.base_position.x-self.last_base_pose)*1000
           if not done:
               reward = velocity
           else:
               reward = -50
        
           return reward, done


