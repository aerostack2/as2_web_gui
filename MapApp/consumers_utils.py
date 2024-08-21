# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__authors__ = 'Rafael Pérez Seguí'

class UAV():
    def __init__(self, id, state, pose, odom=[], desiredPath=[], sensors={}):
        self.id = id
        self.state = state
        self.pose = pose
        self.odom = odom
        self.desiredPath = desiredPath
        self.sensors = sensors
        
    async def getInfo(self):
        return {
            'id': self.id,
            'state': self.state,
            'pose': self.pose,
            'odom': self.odom,
            'desiredPath': self.desiredPath,
            'sensors': self.sensors
        }
        
    async def setUav(self, id, state, pose, odom, desiredPath, sensors):
        self.id = id
        self.state = state
        self.pose = pose
        self.odom = odom
        self.desiredPath = desiredPath
        self.sensors = sensors

    async def setUavState(self, state):
        self.state = state
        
    async def setUavPose(self, pose):
        self.pose = pose

    async def setUavOdom(self, odom):
        self.odom = odom

    async def setUavDesiredPath(self, desiredPath):
        self.desiredPath = desiredPath

    async def setUavSensors(self, sensors):
        self.sensors = sensors
        

class Mission():
    def __init__(self, id, state, uavList, layers):
        self.setMission(id, state, uavList, layers)
        
    async def getInfo(self):
        return {
            'id': self.id,
            'state': self.state,
            'uavList': self.uavList,
            'layers': self.layers
        }
        
    async def setMission(self, id, state, uavList, layers):
        self.id = id
        self.state = state
        self.uavList = uavList
        self.layers = layers
        
    async def setMissionState(self, state):
        self.state = state
        
    async def setMissionUavList(self, uavList):
        self.uavList = uavList
        
    async def setMissionLayers(self, layers):
        self.layers = layers
        

class SmartList():
    def __init__(self):
        self.objectList = []
        self.objectDict = {}
        
    async def getList(self):
        return self.objectList;

    async def getDict(self):
        return self.objectDict;
    
    async def getDictById(self, id):
        return self.objectDict[id]

    async def getDictInfo(self):
        objectDict = []
        for id in self.objectList:
            objectDict.append(await self.objectDict[id].getInfo())
        return objectDict;

    async def getDictInfoById(self, id):
        return {
            id: await self.objectDict[id].getInfo(),
        }

    async def removeObject(self, id):
        self.objectList.remove(id);
        del self.objectDict[id];

    async def addObject(self, id, object):
        self.objectList.append(id);
        self.objectDict[id] = object;