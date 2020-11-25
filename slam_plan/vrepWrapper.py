# -*- coding: utf-8 -*-
"""
Created on Wed Aug 28 14:34:14 2019

@author: tabor
"""

import sys
import time
import numpy as np
import os
import matplotlib.pyplot as plt

try:
    from vrepfiles import vrep
except:
    print(
        "Import of vrep failed. Make sure the 'vrep.py' file is in this directory."
    )
    sys.exit(1)


class VrepWrapper:
    def __init__(self,shm = True,sa=None):
        vrep.simxFinish(-1)
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

        if self.clientID != -1:
            print("Successfully connected to remote API server.")
            #vrep.simxSynchronous(self.clientID, True)
            vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)

        else:
            print("Failed connecting to remote API server")

        self.robot_handle = vrep.simxGetObjectHandle(
          self.clientID, 'youBot', vrep.simx_opmode_blocking
        )[1]
        print('robot:', self.robot_handle)
        self.goal_handles = [vrep.simxGetObjectHandle(
          self.clientID, 'ConcretBlock#{}'.format(i), vrep.simx_opmode_blocking
        )[1] for i in range(3) ]
        print('goals:', self.goal_handles)

        walls = [
          '80cmHighWall50cm', '80cmHighWall100cm', '80cmHighWall200cm',
          '80cmHighWall200cm1', '80cmHighWall200cm10', '80cmHighWall200cm16',
          '80cmHighWall200cm17', '80cmHighWall200cm20', '80cmHighWall100cm8',
          '80cmHighWall200cm19', '80cmHighWall200cm18', '80cmHighWall200cm15',
          '80cmHighWall200cm11', '80cmHighWall100cm6', '80cmHighWall100cm5',
          '80cmHighWall200cm14', '80cmHighWall50cm9', '80cmHighWall50cm8',
          '80cmHighWall50cm7', '80cmHighWall50cm6', '80cmHighWall50cm5',
          '80cmHighWall200cm12', '80cmHighWall100cm4', '80cmHighWall200cm9',
          '80cmHighWall200cm8', '80cmHighWall200cm7', '80cmHighWall200cm6',
          '80cmHighWall50cm3', '80cmHighWall50cm2', '80cmHighWall50cm1',
          '80cmHighWall200cm5', '80cmHighWall200cm0', '80cmHighWall100cm0',
          '80cmHighWall500cm0'
        ]
        walls2 = [
          '80cmHighWall200cm2', '80cmHighWall100cm1', '80cmHighWall50cm4',
          '80cmHighWall200cm3', '80cmHighWall200cm4'
        ]
        walls3 = ['80cmHighWall200cm13']

        # handle = vrep.simxGetObjectHandle(
        #   self.clientID, '80cmHighWall50cm',
        #   vrep.simx_opmode_blocking
        # )[1]
        # center = vrep.simxGetObjectPosition(
        #   self.clientID, handle, -1,
        #   vrep.simx_opmode_blocking
        # )[1]
        # mesh = vrep.simxGetObjectOrientation(
        #   self.clientID, handle, -1,
        #   vrep.simx_opmode_blocking
        # )[1]
        # import pdb; pdb.set_trace()

        wall_handles = [vrep.simxGetObjectHandle(
          self.clientID, wallname,
          vrep.simx_opmode_blocking
        )[1] for wallname in walls]
        wall_handles2 = [vrep.simxGetObjectHandle(
          self.clientID, wallname,
          vrep.simx_opmode_blocking
        )[1] for wallname in walls2]
        wall_handles3 = [vrep.simxGetObjectHandle(
          self.clientID, wallname,
          vrep.simx_opmode_blocking
        )[1] for wallname in walls2]

        final_walls = [vrep.simxGetObjectPosition(
          self.clientID, wallh, -1,
          vrep.simx_opmode_blocking
        )[1] for wallh in wall_handles3]

        steps = 5
        stepsize = 1 / (steps+1)

        wall_pos = np.array([vrep.simxGetObjectPosition(
          self.clientID, wallh, -1,
          vrep.simx_opmode_blocking
        )[1] for wallh in wall_handles])
        points = []
        for wi in range(len(wall_pos)-1):
            wpos0 = wall_pos[wi]
            wpos1 = wall_pos[wi+1]
            vec = wpos1 - wpos0
            norm = np.sqrt(np.power(vec, 2).sum())
            vec = (vec / norm) * stepsize
            npos = wpos0+vec
            points.append(npos)
            for step in range(steps):
                npos += vec
                points.append(npos)
        final_walls = final_walls + wall_pos.tolist() + points
        print('final_walls', len(final_walls))

        wall_pos = np.array([vrep.simxGetObjectPosition(
          self.clientID, wallh, -1,
          vrep.simx_opmode_blocking
        )[1] for wallh in wall_handles2])
        points = []
        for wi in range(len(wall_pos)-1):
            wpos0 = wall_pos[wi]
            wpos1 = wall_pos[wi+1]
            vec = wpos1 - wpos0
            norm = np.sqrt(np.power(vec, 2).sum())
            vec = (vec / norm) * stepsize
            npos = wpos0+vec
            points.append(npos)
            for step in range(steps):
                npos += vec
                points.append(npos)
        final_walls = final_walls + wall_pos.tolist() + points
        print('final_walls', len(final_walls))

        final_walls = np.array(final_walls)
        mins = final_walls.min(0)
        maxs = final_walls.max(0)
        # print('mins:', mins, 'maxs:', maxs)
        expans = 0.5
        self.lims = np.array([[mins[0]-expans, maxs[0]+expans], [mins[1]-expans, maxs[1]+expans]])
        # self.lims = np.array([[-7.26916552,  4.59999681], [-4.49999833,  7.65804672]])
        self.wall_poses = final_walls

        self.sa=sa
        self.shm = shm

        robot_pos = vrep.simxGetObjectPosition(
          self.clientID, self.robot_handle, -1,
          vrep.simx_opmode_blocking
        )[1]
        goal_poss = [vrep.simxGetObjectPosition(
          self.clientID, goalh, -1,
          vrep.simx_opmode_blocking
        )[1] for goalh in self.goal_handles]
        # print('robot pos:', robot_pos)
        # print('goal pos:', goal_poss)

        self.start = np.array( robot_pos )[:2]
        self.goal = np.array( goal_poss[0] )[:2]

        self.threshold = 0.25

        plt.scatter(self.wall_poses[:,0], self.wall_poses[:,1])
        plt.scatter([self.start[0]], [self.start[1]], label='start', color='r')
        plt.scatter([self.goal[0]], [self.goal[1]], label='goal', color='g')
        plt.savefig('testmap.png')


    def checkCollission(self,states):
        dists = np.power((self.wall_poses - states), 2).sum(1)
        if (dists<=self.threshold).sum() > 0:
            return True
        return False
        # num_states = len(states)
        # single_dim_states = np.reshape(states,-1)
        #
        # # res = vrep.getScriptAssociatedWithObject(self.clientID, 'youBot')
        # # import pdb; pdb.set_trace()
        # # simxGetCollisionHandle
        # emptyBuff = bytearray()
        # res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(
        #   self.clientID, 'Dummy', vrep.sim_scripttype_childscript,
        #   #'CollisionDetection',
        #   'CollisionDetection',
        #   [num_states], single_dim_states, ['strings'], emptyBuff,
        #   vrep.simx_opmode_blocking
        # )
        # print('res:', res)
        # if res!=vrep.simx_return_ok:
        #     return
        # # import pdb; pdb.set_trace()
        # return
        # if( res&2 == 2):
        #     print('deeper')
        #     new = np.array_split(states,2)
        #     collides1,fk1 = self.checkCollission(new[0])
        #     collides2,fk2 = self.checkCollission(new[1])
        #     retFloats = np.concatenate([fk1,fk2],0)
        #     retInts = collides1.extend(collides2)
        #
        # fk = np.reshape(np.array(retFloats),(-1,3))
        # return (retInts,fk)

    def runTrajectory(self,angles):
        for angle in angles:
            for i in range(7):
                vrep.simxSetJointPosition( self.clientID,
                  self.joint_handles[i], angle[i], vrep.simx_opmode_blocking
                )
            vrep.simxSynchronousTrigger( self.clientID )
            time.sleep(0.01)

    #assumes (num,6) array (x1,y1,z1,x2,y2,z2)
    def addLine(self,lines,isPlan = 0):
        print(lines.shape)
        num_lines = len(lines)
        single_dim_lines = np.reshape(lines,-1)
        [res, retInts, retFloats, retStrings, retBuffer] = vrep.simxCallScriptFunction(
          self.clientID, "LBR4p", vrep.sim_scripttype_childscript, 'addLines',
          [num_lines,isPlan], single_dim_lines, 'Hello world!', 'blah',
          vrep.simx_opmode_blocking
        )

    def addPoint(self,points,isEnd = 0):
        num_points  = len(points)
        single_dim_points = np.reshape(points,-1)
        [res, retInts, retFloats, retStrings, retBuffer] = vrep.simxCallScriptFunction(
          self.clientID, "LBR4p", vrep.sim_scripttype_childscript, 'addPoints',
          [num_points,isEnd], single_dim_points, 'Hello world!', 'blah',
          vrep.simx_opmode_blocking
        )

    def vrepStop(self):
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
        vrep.simxGetPingTime(self.clientID)
        vrep.simxFinish(self.clientID)

    def vrepReset(self):
        self.vrepStop()
        self.__init__(self.shm,self.sa)
        self.vrepStop()
        self.__init__(self.shm,self.sa)

    def test_collisions(self,state):
        expanded = np.reshape(np.array(state),(-1,2))
        formatted = np.zeros((expanded.shape[0], 3))
        formatted[:,:2] = expanded[:]
        formatted[:,2] += 0.4
        return self.checkCollission(formatted)
        # formatted = np.reshape(np.array(state),(-1,7))
        # collides,fk = self.checkCollission(formatted)
        # return np.sum(collides)>0

    def draw_plan(self, path, states):

        Qs, edges = states
        print('states:', len(Qs))
        # Draw tree for each of the robot links
        for i, e in enumerate(edges):
            e0 = e[0]
            e1 = e[1]
            plt.plot([e0[0], e1[0]], [e0[1], e1[1]], 'b', zorder=-1)
            plt.plot([e0[0], e1[0]], [e0[1], e1[1]], 'b.', zorder=-1)

        if path is not None:
            path = np.array(path)
            plt.plot(path[:,0], path[:,1], color='purple')

        plt.scatter(self.wall_poses[:,0], self.wall_poses[:,1])
        plt.scatter([self.start[0]], [self.start[1]], label='start', color='r')
        plt.scatter([self.goal[0]], [self.goal[1]], label='goal', color='g')
        plt.savefig('test.png')


def main():

    vrapper = VrepWrapper(False, None)
    print('successes:')
    print('collided:', vrapper.checkCollission([0.0, 0.0, 0.4]))
    print('collided:', vrapper.checkCollission([-1.95, 4.35, 0.4]))
    print('collided:', vrapper.checkCollission([-2.9, 1.0, 0.4]))
    print('collided:', vrapper.checkCollission([-2.32,  0.8,   0.4]))

    print('fails:')
    print('collided:', vrapper.checkCollission([-2.32,  -0.8,   0.4]))
    print('collided:', vrapper.checkCollission([-3.3,  -0.1,   0.4]))
    print('collided:', vrapper.checkCollission([0.87499923,  -3.34999871,   0.4]))


if __name__ == '__main__':
    main()
