"""
marvinData maintains the global data used in the InMoov control
"""
import os
import time
import simplejson as json
import threading

from multiprocessing.managers import SyncManager
#from multiprocessing import Queue, Manager
import multiprocessing
import subprocess
import psutil

#from shared_memory_dict import SharedMemoryDict
import config
import marvinglobal.marvinglobal as mg

lastPositionSaveTime = None

class ShareManager(SyncManager): pass


def updateCartGui(type):

    if 'marvinGui' in config.md.processDict.keys():
        # config.log(f"update skeletonGui {servoName=}")
        config.md.cartGuiUpdateQueue.put({'type': type})


def updateSkeletonGui(type):

    if 'marvinGui' in config.md.processDict.keys():
        # config.log(f"update skeletonGui {servoName=}")
        config.md.skeletonGuiUpdateQueue.put({'type': type})


class MarvinData():
    "shared ressources in its own process"
    def __init__(self):
        super().__init__()

        # shared dictionaries
        shareManager = multiprocessing.Manager()

        self.processDict = shareManager.dict()
        self.arduinoDict = shareManager.dict()

        #config.log(f"open servo types file {config.SERVO_TYPE_DEFINITIONS_FILE}")
        self.servoTypeDict = shareManager.dict()

        #config.log(f"open servo definitions file {config.SERVO_STATIC_DEFINITIONS_FILE}")
        self.servoStaticDict = shareManager.dict()

        config.log(f"create servoDerivedDict")
        self.servoDerivedDict = shareManager.dict()

        self.servoCurrentDict = shareManager.dict()

        self.cartDict = shareManager.dict()
        self.cartDict.update({mg.SharedDataUpdate.CART_STATE: config.cartStateLocal})
        self.cartDict.update({mg.SharedDataUpdate.SENSOR_TEST_DATA: config.sensorTestDataLocal})
        self.cartDict.update({mg.SharedDataUpdate.FLOOR_OFFSET: config.floorOffsetLocal})
        self.cartDict.update({mg.SharedDataUpdate.OBSTACLE_DISTANCE: config.obstacleDistanceLocal})

    def saveServoStaticDict(self):
        '''
        servoStaticDict is a dict of cServoStatic objects by servoName
        to store it in json revert the objects back to dictionaries
        :return:
        '''
        servoStaticDefinitions = {}
        for servoName, servoObject in self.servoStaticDict.items():
            servoStaticDefinitions.update({servoName: servoObject.__dict__})

        with open(mg.SERVO_STATIC_DEFINITIONS_FILE, 'w') as outfile:
            json.dump(servoStaticDefinitions, outfile, indent=2)


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

    def getCartDict(self):
        return self.cartDict

    def run(self):
        # register the shared dicts
        ShareManager.register('getProcessDict', self.getProcessDict)
        ShareManager.register('getArduinoDict', self.getArduinoDict)
        ShareManager.register('getServoTypeDict', self.getServoTypeDict)
        ShareManager.register('getServoStaticDict', self.getServoStaticDict)
        ShareManager.register('getServoDerivedDict', self.getServoDerivedDict)
        ShareManager.register('getServoCurrentDict', self.getServoCurrentDict)

        ShareManager.register('getCartDict', self.getCartDict)

        # register the shared queues
        self.sharedDataUpdateQueue = multiprocessing.Queue()
        ShareManager.register('getSharedDataUpdateQueue', callable=lambda: self.sharedDataUpdateQueue)

        self.ikUpdateQueue = multiprocessing.Queue()
        ShareManager.register('getIkUpdateQueue', callable=lambda: self.ikUpdateQueue)

        self.skeletonGuiUpdateQueue = multiprocessing.Queue()
        ShareManager.register('getSkeletonGuiUpdateQueue', callable=lambda: self.skeletonGuiUpdateQueue)

        self.cartGuiUpdateQueue = multiprocessing.Queue()
        ShareManager.register('getCartGuiUpdateQueue', callable=lambda: self.cartGuiUpdateQueue)

        self.servoRequestQueue = multiprocessing.Queue()
        ShareManager.register('getServoRequestQueue', callable=lambda: self.servoRequestQueue)

        self.cartRequestQueue = multiprocessing.Queue()
        ShareManager.register('getCartRequestQueue', callable=lambda: self.cartRequestQueue)

        self.speakRequestQueue = multiprocessing.Queue()
        ShareManager.register('getSpeakRequestQueue', callable=lambda:self.speakRequestQueue)
        self.speakRespondQueue = multiprocessing.Queue()
        ShareManager.register('getSpeakRespondQueue', callable=lambda:self.speakRespondQueue)

        self.imageProcessingQueue = multiprocessing.Queue()
        ShareManager.register('getImageProcessingQueue', callable=lambda: self.imageProcessingQueue)


        # start thread for updating the dicts
        config.log(f"start shareDataUpdateQueue thread")
        updateRequestsThread = threading.Thread(name="sharedDataUpdateRequests", target=self.sharedDataUpdateRequests, args={})
        updateRequestsThread.start()

        try:
            self.m = ShareManager(address=('', mg.SHARED_DATA_PORT), authkey=b'marvin')
            config.log(f"marvinData: shareManager instanciated, {mg.SHARED_DATA_PORT=}")
            self.s = self.m.get_server()
            config.log(f"marvinData: registered, wait for shared data update requests")
            config.log(f"-----------------")
            self.s.serve_forever()

        except Exception as e:
            config.log(f"marvin shares problem, {e}")
            os._exit(0)


    # thread to handle updates
    def sharedDataUpdateRequests(self):
        # as of python 3.8 we can not update a shared object
        # instead create a dict from the current objects values
        # update this dict with the new values passed in as dict
        # update the object with the new values of this dict
        # replace the object in the shared dict with the updated object
        while True:

            stmt = "self.sharedDataUpdateQueue.get()"
            try:
                # as with python 3.7 a shared (proxied) nested dict can not directly be updated
                # - copy the existing subDict into a local dict
                # - update the local dict
                # - replace whole subDict with updated dict
                stmt = self.sharedDataUpdateQueue.get()
                #print(f"{stmt=}")
                updType = stmt[0]

                if updType == mg.SharedDataUpdate.PROCESS:
                    # ("processDict", processName, {'lastUpdate': time.time()})
                    # ("processDict", processName, {'remove': True})

                    processName = stmt[1]
                    updStmt = stmt[2]

                    if 'remove' in updStmt:
                        try:
                            del self.processDict[processName]
                        except KeyError:
                            config.log(f"removal of process requested that is not in the processDict:{processName}")
                    else:
                        if processName in self.processDict.keys():
                            curr = self.processDict[processName]
                            curr.update(updStmt)
                        else:
                            curr = updStmt
                        self.processDict[processName] = curr

                elif updType == mg.SharedDataUpdate.ARDUINO:
                    arduino = stmt[1]
                    updStmt = stmt[2]
                    if arduino in self.arduinoDict.keys():
                        curr = self.arduinoDict[arduino]
                        curr.update(updStmt)
                    else:
                        curr = updStmt
                    self.arduinoDict[arduino] = curr

                    if 'marvinGui' in config.md.processDict.keys():
                        info = {'type': mg.SharedDataUpdate.ARDUINO, 'arduino': curr, 'connected': True}
                        #config.log(f"update skeletonGui {info}")
                        config.md.skeletonGuiUpdateQueue.put(info)


                elif updType == mg.SharedDataUpdate.SERVO_TYPE:

                    servoTypeName = stmt[1]

                    newServoType:mg.ServoType = mg.ServoType(stmt[2])
                    #self.servoTypeDict[servoType] = newServoType
                    self.servoTypeDict.update({servoTypeName: newServoType})


                elif updType == mg.SharedDataUpdate.SERVO_STATIC:
                    
                    servoName = stmt[1]

                    newServoStatic: mg.ServoStatic = mg.ServoStatic(stmt[2])
                    self.servoStaticDict[servoName] = newServoStatic

                    # update derived servo values too
                    servoType = self.servoTypeDict[newServoStatic.servoType]
                    newServoDerived = mg.ServoDerived(newServoStatic, servoType)
                    self.servoDerivedDict[servoName] = newServoDerived


                    # update skeleton gui if running
                    if 'marvinGui' in config.md.processDict.keys():
                        #config.log(f"update skeletonGui {servoName=}")
                        config.md.skeletonGuiUpdateQueue.put({'type': mg.SharedDataUpdate.SERVO_CURRENT, 'servoName': servoName})


                elif updType == mg.SharedDataUpdate.SERVO_CURRENT:
                    # this can be a partial update request and the dict may not contain the servo yet
                    servoName = stmt[1]
                    if servoName not in self.servoCurrentDict.keys():
                        servoCurrent = mg.ServoCurrent(servoName)
                        self.servoCurrentDict.update({servoName: servoCurrent})

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

                    # update skeleton gui if running
                    if 'marvinGui' in config.md.processDict.keys():
                        #config.log(f"update skeletonGui {servoName=}")
                        config.md.skeletonGuiUpdateQueue.put({'type': mg.SharedDataUpdate.SERVO_CURRENT, 'servoName': servoName})

                # single instance shared dicts
                elif updType in [mg.SharedDataUpdate.CART_STATE,
                                 mg.SharedDataUpdate.CART_LOCATION,
                                 mg.SharedDataUpdate.CART_MOVEMENT,
                                 mg.SharedDataUpdate.PLATFORM_IMU,
                                 mg.SharedDataUpdate.OBSTACLE_DISTANCE]:
                    #if updType in self.cartDict.keys():
                    #    config.log(f"before: {updType=}, {self.cartDict[updType]} ")
                    self.cartDict[updType] = stmt[1]
                    updateCartGui(updType)
                    #config.log(f"after:  {updType=}, {self.cartDict[updType]} ")

                elif updType == mg.SharedDataUpdate.HEAD_IMU:
                    self.cartDict[updType] = stmt[1]
                    updateSkeletonGui(updType)

                elif updType == mg.SharedDataUpdate.FLOOR_OFFSET:
                    sensorId = stmt[1]

                    # update the local copy of the dict
                    config.floorOffsetLocal.distance[sensorId] = stmt[2]
                    config.floorOffsetLocal.timeStamp[sensorId] = time.time()

                    # then reassign the local dict to the shared DictProxy
                    self.cartDict.update({mg.SharedDataUpdate.FLOOR_OFFSET: config.floorOffsetLocal})
                    updateCartGui(updType)

                elif updType == mg.SharedDataUpdate.SENSOR_TEST_DATA:

                    # update the local copy of the dict
                    config.sensorTestDataLocal = stmt[1]

                    # then reassign the local dict to the shared DictProxy
                    self.cartDict.update({mg.SharedDataUpdate.SENSOR_TEST_DATA: config.sensorTestDataLocal})
                    updateCartGui(updType)


            except Exception as e:
                config.log(f"error in dict update: {stmt}, {e}")







def cleanup():

    for process in psutil.process_iter():
        try:
            # find processes started with debugger too
            if 'python' in process.cmdline()[0]:
                for processName in config.allMarvinTasks:
                    if processName in ''.join(process.cmdline()):
                        print(f'stop {processName}')
                        process.kill()

        except Exception as e:  # some processes do not allow inquiries
            pass


def startServer(server):
    """
    starts the server <server> by calling the batch file f"c:/projekte/inmoov/start_{server}.bat"
    :param server:
    :return:
    """
    print(f"{time.time()} - restart server {server}")
    serverBatch = f"c:/projekte/inmoov/start_{server}.bat"
    print(f"{time.time()} - subprocess.call({serverBatch})")
    try:
        subprocess.call(serverBatch)
        return True
    except Exception as e:
        print(f"start server {server} failed, {e}")
        return False


if __name__ == "__main__":

    # when start/restart of marvinData happens
    cleanup()

    config.share = mg.MarvinShares()

    config.startLogging()

    config.md = MarvinData()
    config.cc = mg.CartCommands()

    # eval degrees from pos after creating an instance of MarvinData
    for servoName, servoCurrent in config.md.servoCurrentDict.items():
        servoDerived = config.md.servoDerivedDict[servoName]
        servoStatic = config.md.servoStaticDict[servoName]
        p = servoCurrent.position
        servoCurrent.degrees = mg.evalDegFromPos(servoStatic, servoDerived, p)

    config.log("marvinData initialized")
    config.md.run()

    # startServer works only for running tasks in run mode (no debugging)
    # for process in config.allMarvinTasks:
    #    startServer(process)
    # during development you might want to start tasks through the pycharm ide

    config.log("marvinData stopped")



