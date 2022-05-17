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