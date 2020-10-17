"""
marvinData maintains the global data used in the InMoov control
"""
import os
import time
import simplejson as json
import threading

from multiprocessing.managers import SyncManager
from multiprocessing import Queue, Manager
#from shared_memory_dict import SharedMemoryDict
import config
import marvinglobal.marvinglobal as mg

lastPositionSaveTime = None

class ShareManager(SyncManager): pass


class MarvinData():
    "shared ressources in its own process"
    def __init__(self):
        super().__init__()

        # shared dictionaries
        shareManager = Manager()

        self.processDict = shareManager.dict()
        self.processDict.update({'skeletonControl': {'lastUpdate': 0}})

        self.arduinoDict = shareManager.dict()
        self.arduinoDict.update({0:{'arduinoIndex': 0, 'arduinoName': 'left, lower arduino', 'comPort': 'COM8', 'connected': False}})
        self.arduinoDict.update({1:{'arduinoIndex': 1, 'arduinoName': 'right, upper arduino', 'comPort': 'COM4', 'connected': False}})

        config.log(f"open servo types file {config.SERVO_TYPE_DEFINITIONS_FILE}")
        self.servoTypeDict = shareManager.dict()
        self.loadServoTypes()

        config.log(f"open servo definitions file {config.SERVO_STATIC_DEFINITIONS_FILE}")
        self.servoStaticDict = shareManager.dict()
        self.loadServoStaticDefinitions()

        config.log(f"create servoDerivedDict")
        self.servoDerivedDict = shareManager.dict()

        for servoName, servoStatic in self.servoStaticDict.items():
            servoType = self.servoTypeDict[servoStatic.servoType]
            servoDerived = mg.ServoDerived(servoStatic, servoType)
            self.servoDerivedDict.update({servoName: servoDerived})

        self.servoCurrentDict = shareManager.dict()
        self.loadServoPositions()


    def loadServoTypes(self):
        # servo types
        config.log(f"open servo types file {config.SERVO_TYPE_DEFINITIONS_FILE}")
        try:
            with open(config.SERVO_TYPE_DEFINITIONS_FILE, 'r') as infile:
                servoTypeDefinitions = json.load(infile)
            with open(config.SERVO_TYPE_DEFINITIONS_FILE + ".bak", 'w') as outfile:
                json.dump(servoTypeDefinitions, outfile, indent=2)

        except Exception as e:
            config.log(f"missing {config.SERVO_TYPE_DEFINITIONS_FILE} file, try using the backup file")
            os._exit(3)

        for servoTypeName, servoTypeData in servoTypeDefinitions.items():
            servoType = mg.ServoType(servoTypeData)   # inst of servoTypeData class
            self.servoTypeDict.update({servoTypeName: servoType})

        config.log(f"servoTypeDict loaded")


    def loadServoStaticDefinitions(self):

        try:
            with open(config.SERVO_STATIC_DEFINITIONS_FILE, 'r') as infile:
                servoStaticDefinitions = json.load(infile)
            # if successfully read create a backup just in case
            with open(config.SERVO_STATIC_DEFINITIONS_FILE + ".bak", 'w') as outfile:
                json.dump(servoStaticDefinitions, outfile, indent=2)

        except Exception as e:
            config.log(f"missing {config.SERVO_STATIC_DEFINITIONS_FILE} file, try using the backup file")
            os._exit(2)

        for servoName in servoStaticDefinitions:
            servoStatic = mg.ServoStatic(servoStaticDefinitions[servoName])
            servoType = self.servoTypeDict[servoStatic.servoType]

            # data cleansing
            # BE AWARE: inversion is handled in the arduino only, maxPos > minPos is a MUST!
            # check for valid min/max position values in servo type definition
            if servoType.typeMaxPos < servoType.typeMinPos:
                config.log(f"wrong servo type values, typeMaxPos < typeMinPos, servo disabled")
                servoStatic.enabled = False

            # check for valid min/max degree values in servo definition
            if servoStatic.maxDeg < servoStatic.minDeg:
                config.log(f"wrong servo min/max values, maxDeg < minDeg for {servoName}, servo disabled")
                servoStatic.enabled = False

            # check for servo values in range of servo type definition
            if servoStatic.minPos < servoType.typeMinPos:
                config.log(f"servo min pos lower than servo type min pos for {servoName}, servo min pos adjusted")
                servoStatic.minPos = servoType.typeMinPos

            if servoStatic.maxPos > servoType.typeMaxPos:
                config.log(f"servo max pos higher than servo type max pos for {servoName}, servo max pos adjusted")
                servoStatic.maxPos = servoType.typeMaxPos

            # add object to the servoStaticDict
            self.servoStaticDict.update({servoName: servoStatic})


    def loadServoPositions(self):

        #global persistedServoPositions, servoCurrentDict

        config.log("load last known servo positions")
        if os.path.isfile(config.PERSISTED_SERVO_POSITIONS_FILE):
            with open(config.PERSISTED_SERVO_POSITIONS_FILE, 'r') as infile:
                config.persistedServoPositions = json.load(infile)
                if len(config.persistedServoPositions) != len(self.servoStaticDict):
                    config.log(f"mismatch of servoDict and persistedServoPositions")
                    createPersistedDefaultServoPositions()
        else:
            createPersistedDefaultServoPositions()

        # check for valid persisted position
        for servoName in self.servoStaticDict:

            servoStatic: mg.ServoStatic = self.servoStaticDict[servoName]

            # try to assign the persisted last known servo position
            try:
                p = config.persistedServoPositions[servoName]
            except KeyError:
                # in case we do not have a last known servo position use 90 as default
                p = 90

            # set current position to min or max if outside range
            if p < servoStatic.minPos:
                p = servoStatic.minPos
            if p > servoStatic.maxPos:
                p = servoStatic.maxPos

            # create cServoCurrent object
            servoCurrent = mg.ServoCurrent(servoName)

            servoCurrent.position = p

            # set degrees from pos
            #servoCurrent.degrees = mg.evalDegFromPos(servoName, p)

            # add object to servoCurrentDict with key servoName
            self.servoCurrentDict.update({servoName: servoCurrent})

        config.log("servoPositions loaded")


    def getProcessDict(self):
        return self.processDict

    def getArduinoDict(self):
        return self.arduinoDict

    def getServoTypeDict(self):
        return self.servoTypeDict

    def getServoStaticDict(self):
        return self.servoStaticDict

    #def updateServoCurrentDict(self, servoName, servoCurrent):
    #    self.servoCurrentDict[servoName] = servoCurrent

    def getServoDerivedDict(self):
        return self.servoDerivedDict

    def getServoCurrentDict(self):
        return self.servoCurrentDict


    def run(self):
        # register the shared dicts
        ShareManager.register('getProcessDict', self.getProcessDict)
        ShareManager.register('getArduinoDict', self.getArduinoDict)
        ShareManager.register('getServoTypeDict', self.getServoTypeDict)
        ShareManager.register('getServoStaticDict', self.getServoStaticDict)
        ShareManager.register('getServoDerivedDict', self.getServoDerivedDict)
        ShareManager.register('getServoCurrentDict', self.getServoCurrentDict)

        # register the shared queues
        self.dictUpdateQueue = Queue()
        ShareManager.register('getDictUpdateQueue', callable=lambda: self.dictUpdateQueue)

        self.guiUpdateQueue = Queue()
        ShareManager.register('getGuiUpdateQueue', callable=lambda: self.guiUpdateQueue)

        self.servoRequestQueue = Queue()
        ShareManager.register('getServoRequestQueue', callable=lambda: self.servoRequestQueue)

        self.speakRequestQueue = Queue()
        ShareManager.register('getSpeakRequestQueue', callable=lambda:self.speakRequestQueue)
        self.speakRespondQueue = Queue()
        ShareManager.register('getSpeakRespondQueue', callable=lambda:self.speakRespondQueue)

        self.imageProcessingQueue = Queue()
        ShareManager.register('getImageProcessingQueue', callable=lambda: self.imageProcessingQueue)


        # start thread for updating the dicts
        config.log(f"start updateQueue thread")
        updateRequestsThread = threading.Thread(name="updateRequests", target=self.updateRequests, args={})
        updateRequestsThread.start()

        try:
            m = ShareManager(address=('', 50000), authkey=b'marvin')
            config.log(f"marvinData: shareManager instanciated")
            s = m.get_server()
            config.log(f"marvinData: registered, wait for shared data update requests")
            config.log(f"-----------------")
            s.serve_forever()

        except Exception as e:
            config.log(f"marvin shares problem, {e}")
            m.shutdown()


    # thread to handle updates
    def updateRequests(self):
        while True:

            try:
                # as with python 3.7 a shared (proxied) nested dict can not directly be updated
                # - copy the existing subDict into a local dict
                # - update the local dict
                # - replace whole subDict with updated dict
                stmt = self.dictUpdateQueue.get()

                if 'processDict' in stmt[0]:
                    self.processDict[stmt[1]] = stmt[2]

                elif 'arduinoDict' in stmt[0]:
                    curr = self.arduinoDict[stmt[1]]
                    curr.update(stmt[2])
                    self.arduinoDict[stmt[1]] = curr

                elif 'servoCurrentDict' in stmt[0]:
                    # as of python 3.8 we can not update a shared object
                    # instead create a dict from the current objects values
                    # update this dict with the new values passed in as dict
                    # update the object with the new values of this dict
                    # replace the object in the shared dict with the updated object
                    servoName = stmt[1]
                    existingServoCurrentAsDict = dict(self.servoCurrentDict[servoName].__dict__)
                    existingServoCurrentAsDict.update(stmt[2])
                    #def updateData(self, newDegrees, newPosition, newAssigned, newMoving, newAttached, newAutoDetach,newVerbose):
                    newServoCurrent = mg.ServoCurrent(servoName)
                    newServoCurrent.updateData(
                        existingServoCurrentAsDict['degrees'],
                        existingServoCurrentAsDict['position'],
                        existingServoCurrentAsDict['assigned'],
                        existingServoCurrentAsDict['moving'],
                        existingServoCurrentAsDict['attached'],
                        existingServoCurrentAsDict['autoDetach'],
                        existingServoCurrentAsDict['verbose'],
                        existingServoCurrentAsDict['swiping'],
                        existingServoCurrentAsDict['timeOfLastMoveRequest'])

                    self.servoCurrentDict[servoName] = newServoCurrent
                    # when move has ended persist the position
                    if not newServoCurrent.moving:
                        saveServoPosition(servoName, newServoCurrent.position)

            except Exception as e:
                config.log(f"error in eval: {stmt}, {e}")



def persistServoPositions():

    config.lastPositionSaveTime = time.time()
    with open(config.PERSISTED_SERVO_POSITIONS_FILE, 'w') as outfile:
        json.dump(config.persistedServoPositions, outfile, indent=2)


def saveServoPosition(servoName, position, maxDelay=10):
    '''
    save current servo position to json file if last safe time differs
    more than maxDelay seconds
    :param servoName:
    :param position:
    :param maxDelay:
    :return:
    '''

    config.persistedServoPositions[servoName] = position
    if time.time() - config.lastPositionSaveTime > maxDelay:
        persistServoPositions()


def createPersistedDefaultServoPositions():
    '''
    initialize default servo positions in case of missing or differing json file
    :return:
    '''

    config.persistedServoPositions = {}
    for servoName in config.md.servoStaticDict:
        config.persistedServoPositions.update({servoName: 90})
    persistServoPositions()



if __name__ == "__main__":

    config.mg = mg.MarvinGlobal()

    config.startLogging()

    config.log("start thread marvinShares")
    config.marvinData = MarvinData()

    # eval degrees from pos after creating an instance of MarvinData
    for servoName, servoCurrent in config.marvinData.servoCurrentDict.items():
        servoDerived = config.marvinData.servoDerivedDict[servoName]
        servoStatic = config.marvinData.servoStaticDict[servoName]
        p = servoCurrent.position
        servoCurrent.degrees = mg.evalDegFromPos(servoStatic, servoDerived, p)

    config.log("marvinData initialized")
    config.marvinData.run()

    config.log("marvinData stopped")




