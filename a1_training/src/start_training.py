#!/usr/bin/env python
import gym
import rospy
import rospkg
import a1_env
from gym import wrappers
import baselines.ppo2.ppo2 as ppo2
from baselines.common import tf_util as U

def train(num_timesteps):

    rospy.init_node('a1-v0', anonymous=True, log_level=rospy.INFO)
    U.make_session(num_cpu=1).__enter__()
    env = gym.make('a1-v0')
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('a1_training')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)

    ppo2.learn(network='mlp',env=env,total_timesteps=num_timesteps)
    env.close()

def main():
    train(num_timesteps=1e6)

if __name__ == '__main__':
    main()
