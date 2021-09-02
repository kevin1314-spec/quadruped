#!/usr/bin/env python
import gym
import rospy
import rospkg
import monoped_env
from gym import wrappers
def reset_a1():

    rospy.init_node('Monoped-v0', anonymous=True, log_level=rospy.INFO)
    env = gym.make('monoped-v0')
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('my_hopper_training')
    outdir = pkg_path + '/training_results'
    env = wrappers.Monitor(env, outdir, force=True)
    env.reset()
    action = [1,1,1]
    obs,reward,done,info = env.step(action)
    print(obs,"reward:",reward,"done",done,"info",info)

def main():
    reset_a1()

if __name__ == '__main__':
    main()
