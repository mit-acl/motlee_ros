#!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt
import rospy
import geometry_msgs.msg as geometry_msgs

def plot_map(msg):
    global counter
    if counter % 3 == 0:
        for pose in msg.poses:
            plt.plot(pose.position.y, pose.position.x, 'ko')
            plt.axis("equal")
            plt.grid()
            plt.draw()
            plt.pause(0.00000000001)
    counter += 1

if __name__ == '__main__':
    counter = 0
    rospy.init_node("plotter")
    rospy.Subscriber("map/poses_only", geometry_msgs.PoseArray, plot_map)
    # note: subscribing to objects to get their ground truth positions is a bad idea. 
    # there are so many objects in the space and saturates the network. Do not turn on objects while flying.
    plt.ion()
    plt.show()
    rospy.spin()