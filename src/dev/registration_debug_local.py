from src.utils.map_registrator import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import glob, os
from src.utils.icp import *

reg = MapRegistrator()
if 0:
    pc=[]
    odom=[]
    bird=[]
    os.chdir("../data/rotation")
    for file in glob.glob("pc_*.npy"):
        pc.append(np.load(file))
        odom.append(np.load('odom_'+ file[3:]))
        #print(odom[-1])
    os.chdir("../..")

    fig1=plt.figure()
    for i in range(len(pc)):
        birdi=reg.pc_to_bird(pc[i])
        birdi=reg.current2standard(odom[i], birdi)
        print('processed')
        plt.scatter(birdi[:,0],birdi[:,1],s=1)
    plt.savefig('results/odometry_reg_integral.png')
    plt.show()
elif 0:
    n1=8174
    n2=12247
    n2=10087
    n2=9088
    n2=8669
    pc1=np.load('../data/sample/pc_'+str(n1)+'.npy')
    pc2=np.load('../data/sample/pc_'+str(n2)+'.npy')
    odom1=np.load('../data/sample/odom_'+str(n1)+'.npy')
    odom2=np.load('../data/sample/odom_'+str(n2)+'.npy')
    bird1=reg.pc_to_bird(pc1)
    bird2=reg.pc_to_bird(pc2)
    bird1=reg.current2standard(odom1, bird1)
    bird2=reg.current2standard(odom2, bird2)
    print(odom1)
    print(odom2)

    fig1 = plt.figure()
    plt.scatter(bird1[:, 0], bird1[:, 1], c='r', s=1)
    plt.scatter(bird2[:, 0], bird2[:, 1], c='b', s=1)
    #plt.savefig('../results/odometry_reg_3.png')
    # fig1.show()
    #
    # bird1c=icp(bird1[::10,:].T,bird2[::10,:].T)
    # plt.figure()
    # plt.scatter(bird1c[:, 0], bird1c[:, 1], c='r', s=1)
    # plt.scatter(bird2[:, 0], bird2[:, 1], c='b', s=1)
    plt.show()

else:
    n1=8174
    n2=5741
    pc1=np.load('../data/sample/pc_'+str(n1)+'.npy')
    pc2=np.load('../data/sample/pc_'+str(n2)+'.npy')
    odom1=np.load('../data/sample/odom_'+str(n1)+'.npy')
    odom2=np.load('../data/sample/odom_'+str(n2)+'.npy')
    bird1=reg.pc_to_bird(pc1)
    bird2=reg.pc_to_bird(pc2)
    bird1=reg.current2standard(odom1, bird1)
    bird2=reg.current2standard(odom2, bird2)
    print(odom1)
    print(odom2)
    bird1=bird1.T
    bird2=bird2.T

    fig1 = plt.figure()
    plt.axis('equal')
    plt.scatter(bird1[0], bird1[1], c='g', s=1)
    plt.show()

    # fig1 = plt.figure()
    # plt.axis('equal')
    # plt.scatter(bird1[0], bird1[ 1], c='g', s=1)
    # angle2R = lambda a: np.array([[cos(a), -sin(a)], [sin(a), cos(a)]])
    # bird1rot=angle2R(pi / 3).dot(bird1)
    # plt.scatter(bird1rot[0], bird1rot[1], c='b', s=1)
    # R,t=register(bird1rot, bird1)
    # bird1reg=R.dot(bird1rot)+np.tile(t, [1,bird1rot.shape[1]])
    # plt.scatter(bird1reg[0], bird1reg[1], c='r', s=1)
    # plt.show()