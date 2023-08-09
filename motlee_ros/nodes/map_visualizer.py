#!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt
import rospy
import geometry_msgs.msg as geometry_msgs

import matplotlib as mpl
mpl.rcParams['figure.dpi'] = 300


# positions = [[1.3053724759933878, 0.08808389701085713], [1.734143345621286, 1.5116296620102785], [-2.9705120875244995, -0.0179882408397972], [3.470811381980292, 4.078357397070458], [2.1686157373540556, -1.2237962882641875], [-3.94573764769314, -1.5780270900376752], [-2.471661663291828, 4.221480212170319], [4.441559075302128, -1.6921031831398285], [4.2556216246548075, 2.3007493959881944], [-1.278859861307652, 0.8624486976627428], [-2.0182618454246044, 2.9027208920539636], [-0.2716494731840169, 2.8821353822922937], [-0.652901581745163, -4.7894527365305875], [4.100639728118636, -4.316321115877415], [0.3327825250049198, 2.0297260260808363], [-4.188930348738766, 3.6177118150874223], [-1.7370431117782956, -2.7399543325838143]]
positions = [[1.30538948174781, 0.08792017608525951], [1.7556711854938247, 1.5301845738388788], [-2.970445795397385, -0.017968445918466327], [3.470787181274709, 4.078329613986586], [2.168708267646973, -1.2237931460359912], [-3.9456521452453295, -1.5780622937245332], [-2.4715796031824846, 4.221399753581286], [4.441561003442656, -1.692115998046444], [4.255669637763099, 2.300721891392908], [-1.2788058555668842, 0.8623606354570972], [-2.0180484955869553, 2.902511955121203], [-0.27154462548986463, 2.8820569403751874], [-1.8699706004964698, -2.008375434619125], [2.7665396650365186, 0.1135119037351044], [-0.7926963921536332, -0.6462376920580662], [0.3326033986672144, 2.029590565125389], [-4.188906698042496, 3.617683236245243], [-1.73699333120812, -2.7400201356279594], [4.0859296012985356, 0.4219624323470336]]

def plot_map(msg):
    global counter
    # global ax
    
    # ax.set_aspect('equal')
    # ax.set_xlim([-8, 8])
    # ax.set_ylim([-8, 8])
    # fig, ax = plt.subplots()
    if counter % 3 == 0:
        plt.cla()

        for pose in msg.poses:
            plt.plot(pose.position.x, pose.position.y, 'ko')
        
        # vicon gt
        for pos in positions:
            plt.plot(pos[0], pos[1], 'rx')

        plt.gca().set_aspect("equal")
        plt.grid(True)
        
        plt.xlim([-6, 6])
        plt.ylim([-6, 6])

        plt.draw()
        plt.pause(0.05)
    counter += 1

def print_msg(msg):
    positions.appedn(msg.pose.position)

if __name__ == '__main__':
    counter = 0
    # fig, ax = plt.subplots()
    rospy.init_node("plotter")
    rospy.Subscriber("map/poses_only", geometry_msgs.PoseArray, plot_map)
    # note: subscribing to objects to get their ground truth positions is a bad idea. 
    # there are so many objects in the space and saturates the network. Do not turn on objects while flying.
    # for i in range(1,24):
    #     if i in [10, 14, 18, 19]:
    #         continue
    #     print(f'OBJ{i}')
    #     data = rospy.wait_for_message(f"/OBJ{i}/world", geometry_msgs.PoseStamped, timeout=10)
    #     positions.append([data.pose.position.x, data.pose.position.y])

    # print("positions: ", positions)
    plt.ion()
    plt.show()
    rospy.spin()