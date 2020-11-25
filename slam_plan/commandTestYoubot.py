# This example illustrates how to execute complex commands from
# a remote API client. You can also use a similar construct for
# commands that are not directly supported by the remote API.
#
# Load the demo scene 'DummyExample.ttt' in V-REP, then
# start the simulation and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    from vrepfiles import vrep
except:
    print('--------------------------------------------------------------')
    print('"vrep.py" could not be imported. This means very probably that')
    print('either "vrep.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "vrep.py"')
    print('--------------------------------------------------------------')
    print('')

import sys
import ctypes
print('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print('Connected to remote API server')

    # 1. First send a command to display a specific message in a dialog box:
    emptyBuff = bytearray()
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction( clientID, 'Dummy', vrep.sim_scripttype_childscript, 'displayText_function', [], [], ['Hello world!'], emptyBuff, vrep.simx_opmode_blocking )
    if res==vrep.simx_return_ok:
        print('Return string: ',retStrings[0]) # display the reply from V-REP (in this case, just a string)
    else:
        print('Remote function call failed')

    # 2. Now create a dummy object at coordinate 0.1,0.2,0.3 with name 'MyDummyName':
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID, 'Dummy',vrep.sim_scripttype_childscript,'createDummy_function',[],[0.1,0.2,0.3],['MyDummyName'],emptyBuff,vrep.simx_opmode_blocking)
    if res==vrep.simx_return_ok:
        print('Dummy handle: ',retInts[0]) # display the reply from V-REP (in this case, the handle of the created dummy)
    else:
        print('Remote function call failed')

    # 3. Now send a code string to execute some random functions:
    code="local octreeHandle=simCreateOctree(0.5,0,1)\n" \
    "simInsertVoxelsIntoOctree(octreeHandle,0,{0.1,0.1,0.1},{255,0,255})\n" \
    "return 'done'"
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(clientID, 'Dummy', vrep.sim_scripttype_childscript, 'executeCode_function', [], [], [code], emptyBuff, vrep.simx_opmode_blocking)
    if res==vrep.simx_return_ok:
        print('Code execution returned: ',retStrings[0])
    else:
        print('Remote function call failed')

    # 4. Now execute special function:
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction( clientID, 'Dummy', vrep.sim_scripttype_childscript, 'collision_run', [], [], ['Hello world!'], emptyBuff, vrep.simx_opmode_blocking )
    if res==vrep.simx_return_ok:
        print('Return string: ', retStrings[0]) # display the reply from V-REP (in this case, just a string)
    else:
        print('Remote function call failed')

    # 5. test to succeed:
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(
      clientID, 'Dummy', vrep.sim_scripttype_childscript, 'CollisionDetection',
      [-1, 0], [-1.85, 5.5, 0.4], ['test_dummy'], emptyBuff, vrep.simx_opmode_blocking
    )
    if res==vrep.simx_return_ok:
        print('Return string: ', retStrings[0]) # display the reply from V-REP (in this case, just a string)
        print('return floats:', retFloats)
        print('return ints:', retInts)
        poss = [vrep.simxGetObjectPosition(
          clientID, retInts[i], -1,
          vrep.simx_opmode_blocking
        )[1] for i in range(2,7)]
        print('positions:', poss)
    else:
        print('Remote function call failed')

    # 6. Test for fail:
    res,retInts,retFloats,retStrings,retBuffer=vrep.simxCallScriptFunction(
      clientID, 'Dummy', vrep.sim_scripttype_childscript, 'CollisionDetection',
      [-1, 0], [0.8,  0.85,   0.4], ['test_dummy'], emptyBuff, vrep.simx_opmode_blocking
    )
    if res==vrep.simx_return_ok:
        print('Return string: ', retStrings[0]) # display the reply from V-REP (in this case, just a string)
        print('return floats:', retFloats)
        print('return ints:', retInts)
        poss = [vrep.simxGetObjectPosition(
          clientID, retInts[i], -1,
          vrep.simx_opmode_blocking
        )[1] for i in range(2,7)]
        print('positions:', poss)
    else:
        print('Remote function call failed')

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
