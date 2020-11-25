# -*- coding: utf-8 -*-
"""
Created on Wed Aug 28 14:34:14 2019

@author: tabor
"""

import sys
import time
import numpy as np
import os

try:
    from vrepfiles import vrep
except:
    print(
        "Import of vrep failed. Make sure the 'vrep.py' file is in this directory."
    )
    sys.exit(1)


class vrepWrapper:
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

        walls = []
        walls += [vrep.simxGetObjectHandle(
          self.clientID, '80cmHighWall100cm{}'.format(i),
          vrep.simx_opmode_blocking
        )[1] for i in range(9)]
        walls += [vrep.simxGetObjectHandle(
          self.clientID, '80cmHighWall200cm{}'.format(i),
          vrep.simx_opmode_blocking
        )[1] for i in range(21)]
        walls += [vrep.simxGetObjectHandle(
          self.clientID, '80cmHighWall50cm{}'.format(i),
          vrep.simx_opmode_blocking
        )[1] for i in range(12)]

        wall_pos = np.array([vrep.simxGetObjectPosition(
          self.clientID, wallh, -1,
          vrep.simx_opmode_blocking
        )[1] for wallh in walls])
        mins = wall_pos.min(0)
        maxs = wall_pos.max(0)
        # print('mins:', mins, 'maxs:', maxs)
        self.lims = np.array([[mins[0]-1, maxs[0]+1], [mins[1]-1, maxs[1]+1]])
        # import pdb; pdb.set_trace()
        self.lims = np.array([[-7.26916552,  4.59999681], [-4.49999833,  7.65804672]])
        self.wall_poses = wall_pos

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

        self.threshold = 0.1


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

        formatted = np.reshape(np.array(state),(-1,7))
        collides,fk = self.checkCollission(formatted)
        return np.sum(collides)>0

    def draw_plan(self, plan, planner, dynamic_tree=False, dynamic_plan=True, show=True):
        if(self.shm):
           self.vrepReset()
        #dynamic_tree, dynamic_plan and show are all dummy values and do not function
        planLines = np.zeros((len(plan),6))

        collisions,fk = self.checkCollission(np.array(plan))
        if(np.sum(collisions) > 0):
            print('plan failed checker')
        for i in range(len(plan)):
            if(i >0):
                planLines[i-1,3:] = fk[i,:]
            planLines[i,:3] = fk[i,:]
        self.addLine(planLines[:-1],1)

        if planner is not None:
            print('drawing tree')
            Qs, edges = planner.T.get_states_and_edges()
            totalNodes = np.reshape(np.array(edges),(-1,7))
            collisions,fk = self.checkCollission(totalNodes)

            if(np.sum(collisions) > 0):
                print('some nodes are in collision in tree')
            self.addLine(np.reshape(fk,(-1,6)))
            self.addPoint(fk)
#        return fk
        col,fk = self.checkCollission(np.reshape(planner.goal,(-1,7)))
        self.addPoint(np.reshape(fk,(-1,3)),1)
        self.runTrajectory(plan)

def main():

    vrapper = vrepWrapper(False, None)
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
