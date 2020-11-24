
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

mapts = np.load('slam_data/mapts.npy')
print('pts:', mapts.shape)
poses = np.load('slam_data/pose.npy')
print('pose:', poses.shape)
octomap = np.load('slam_data/octomap.npy')
print('octo:', octomap.shape)

def plot_data(mapts, poses, octomap):
    print('mean:{}, stdev:{}, median:{}, min:{}, max:{}'.format(
      mapts.mean(0), mapts.std(0), np.median(mapts, 0), mapts.min(0), mapts.max(0)
    ))

    plt.scatter(octomap[:,0], octomap[:,1])
    plt.scatter(poses[:,0], poses[:,1], color='r')
    plt.plot(poses[:,0], poses[:,1], color='r')
    plt.savefig('octocloud.png')

    plt.scatter(mapts[:,0], mapts[:,1])
    plt.scatter(poses[:,0], poses[:,1], color='r')
    plt.plot(poses[:,0], poses[:,1], color='r')
    plt.scatter(poses[:,3], poses[:,4], color='g')
    plt.plot(poses[:,3], poses[:,4], color='g')
    plt.savefig('mapcloud.png')

    plt.clf()
    plt.scatter(mapts[:,0], mapts[:,2])
    plt.scatter(poses[:,0], poses[:,2], color='r')
    plt.plot(poses[:,0], poses[:,2], color='r')
    plt.scatter(poses[:,3], poses[:,5], color='g')
    plt.plot(poses[:,3], poses[:,5], color='g')
    plt.savefig('mapcloudxz.png')

    plt.clf()
    plt.scatter(mapts[:,1], mapts[:,2])
    plt.scatter(poses[:,1], poses[:,2], color='r')
    plt.plot(poses[:,1], poses[:,2], color='r')
    plt.scatter(poses[:,4], poses[:,5], color='g')
    plt.plot(poses[:,4], poses[:,5], color='g')
    plt.savefig('mapcloudyz.png')

    plt.clf()
    zpose = poses.mean(0)[2]
    zmin = zpose - 0.05
    zmax = zpose + 0.25
    cutmap = mapts[mapts[:,2]>=zmin]
    # import pdb; pdb.set_trace()
    plt.scatter(cutmap[:,0], cutmap[:,1])
    plt.scatter(poses[:,0], poses[:,1], color='r')
    plt.plot(poses[:,0], poses[:,1], color='r')
    plt.savefig('mapcloudxy.png')

# plt.clf()
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#
# ax.scatter(mapts[:,0], mapts[:,1], mapts[:,2])
#
# ax.set_xlabel('X Label')
# ax.set_ylabel('Y Label')
# ax.set_zlabel('Z Label')
#
# plt.show()
