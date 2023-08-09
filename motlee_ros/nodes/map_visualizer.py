#!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt
import rospy
import geometry_msgs.msg as geometry_msgs
import matplotlib as mpl

mpl.rcParams['figure.dpi'] = 300


# params
COLLECT_OBJECT_GT = False

# object ground truch
positions = [[1.30538948174781, 0.08792017608525951], [1.7556711854938247, 1.5301845738388788], [-2.970445795397385, -0.017968445918466327], [3.470787181274709, 4.078329613986586], [2.168708267646973, -1.2237931460359912], [-3.9456521452453295, -1.5780622937245332], [-2.4715796031824846, 4.221399753581286], [4.441561003442656, -1.692115998046444], [4.255669637763099, 2.300721891392908], [-1.2788058555668842, 0.8623606354570972], [-2.0180484955869553, 2.902511955121203], [-0.27154462548986463, 2.8820569403751874], [-1.8699706004964698, -2.008375434619125], [2.7665396650365186, 0.1135119037351044], [-0.7926963921536332, -0.6462376920580662], [0.3326033986672144, 2.029590565125389], [-4.188906698042496, 3.617683236245243], [-1.73699333120812, -2.7400201356279594], [4.0859296012985356, 0.4219624323470336]]

def plot_map(msg):
    global counter
    if counter % 3 == 0:
        plt.cla()

        # map
        for pose in msg.poses:
            plt.plot(pose.position.x, pose.position.y, 'ko')
        
        # vicon gt
        for pos in positions:
            plt.plot(pos[0], pos[1], 'rx')

        # plot
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

    # note: subscribing to objects to get their ground truth positions is a bad idea. 
    # there are so many objects in the space and saturates the network. Do not turn on objects while flying.
    # so we just get the ground truth positions once offline and hard code it
    if COLLECT_OBJECT_GT:
        for i in range(1,24):
            if i in [10, 14, 18, 19]: # exclude these objects
                continue
            print(f'OBJ{i}')
            data = rospy.wait_for_message(f"/OBJ{i}/world", geometry_msgs.PoseStamped, timeout=10)
            positions.append([data.pose.position.x, data.pose.position.y])
        print("positions: ", positions)
        exit()
    else:
        rospy.Subscriber("map/poses_only", geometry_msgs.PoseArray, plot_map)

    plt.ion()
    plt.show()
    rospy.spin()