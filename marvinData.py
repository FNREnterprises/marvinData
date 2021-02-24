"""
marvinData maintains the global data used in the InMoov control
"""
import os
import sys
import time
import simplejson as json
import threading
import numpy as np

from multiprocessing.managers import SyncManager

import multiprocessing
import subprocess
import psutil

import config

from marvinglobal import marvinglobal as mg
from marvinglobal import skeletonClasses
from marvinglobal import marvinShares

lastPositionSaveTime = None

class ShareManager(SyncManager): pass


def updateCartGui(msgType):

    if 'marvinGui' in config.md.processDict.keys():
        # config.log(f"update skeletonGui {servoName=}")
        config.md.cartGuiUpdateQueue.put({'msgType': msgType})


def updateSkeletonGui(msgType):

    if 'marvinGui' in config.md.processDict.keys():
        # config.log(f"update skeletonGui {servoName=}")
        config.md.skeletonGuiUpdateQueue.put({'msgType': msgType})


class MarvinData:
    """ shared ressources in its own process"""
    def __init__(self):
        super().__init__()

        self.verbose = False
        # shared dictionaries
        shareManager = multiprocessing.Manager()
        self.m = None
        self.s = None

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
        self.cartDict.update({mg.SharedDataItem.CART_CONFIGURATION: config.cartConfigurationLocal})
        self.cartDict.update({mg.SharedDataItem.CART_STATE: config.cartStateLocal})
        self.cartDict.update({mg.SharedDataItem.SENSOR_TEST_DATA: config.sensorTestDataLocal})
        self.cartDict.update({mg.SharedDataItem.FLOOR_OFFSET: config.floorOffsetLocal})
        self.cartDict.update({mg.SharedDataItem.OBSTACLE_DISTANCE: config.obstacleDistanceLocal})
        self.cartDict.update({mg.SharedDataItem.IR_SENSOR_REFERENCE_DISTANCE: config.irSensorReferenceDistanceLocal})
        self.cartDict.update({mg.SharedDataItem.HEAD_IMU: config.headImuLocal})
        self.cartDict.update({mg.SharedDataItem.PLATFORM_IMU: config.platformImuLocal})

        self.sharedDataUpdateQueue = multiprocessing.Queue()
        self.ikUpdateQueue = multiprocessing.Queue()
        self.skeletonGuiUpdateQueue = multiprocessing.Queue()
        self.cartGuiUpdateQueue = multiprocessing.Queue()
        self.mainGuiUpdateQueue = multiprocessing.Queue()
        self.skeletonRequestQueue = multiprocessing.Queue()
        self.cartRequestQueue = multiprocessing.Queue()
        self.speakRequestQueue = multiprocessing.Queue()
        self.playGestureQueue = multiprocessing.Queue()
        self.navManagerRequestQueue = multiprocessing.Queue()
        self.imageProcessingQueue = multiprocessing.Queue()


    def saveServoStaticDict(self):
        """
        servoStaticDict is a dict of cServoStatic objects by servoName
        to store it in json revert the objects back to dictionaries
        :return:
        """
        servoStaticDefinitions = {}
        for thisServoName, servoObject in self.servoStaticDict.items():
            servoStaticDefinitions.update({thisServoName: servoObject.__dict__})

        with open(mg.SERVO_STATIC_DEFINITIONS_FILE, 'w') as outfile:
            json.dump(servoStaticDefinitions, outfile, indent=2)

        if self.verbose: config.log(f"servoStaticDefinitions updated into file")


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
        ShareManager.register('getSharedDataUpdateQueue', callable=lambda: self.sharedDataUpdateQueue)
        ShareManager.register('getIkUpdateQueue', callable=lambda: self.ikUpdateQueue)
        ShareManager.register('getSkeletonGuiUpdateQueue', callable=lambda: self.skeletonGuiUpdateQueue)
        ShareManager.register('getCartGuiUpdateQueue', callable=lambda: self.cartGuiUpdateQueue)
        ShareManager.register('getMainGuiUpdateQueue', callable=lambda: self.mainGuiUpdateQueue)
        ShareManager.register('getSkeletonRequestQueue', callable=lambda: self.skeletonRequestQueue)
        ShareManager.register('getCartRequestQueue', callable=lambda: self.cartRequestQueue)
        ShareManager.register('getSpeakRequestQueue', callable=lambda:self.speakRequestQueue)
        ShareManager.register('getPlayGestureQueue', callable=lambda:self.playGestureQueue)
        ShareManager.register('getNavManagerRequestQueue', callable=lambda: self.navManagerRequestQueue)
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


    def updateProcessDict(self, sender, info):

        # processes are expected to send an update every second
        processName = info['processName']
        stateChanged = False

        # processes should check for a request to stop by another process
        # a process can be stopped by adding a 'remove' key into the messaage
        if 'remove' in info:
            # prepare the message for the gui update
            msg2 = {'cmd': mg.SharedDataItem.PROCESS, 'sender': config.processName,
                    'info': {'processName': processName, 'running': False}}
            try:
                del self.processDict[processName]
                stateChanged = True
                if config.logProcessListChanges:
                    config.log(f"request from {sender} to remove process {processName} from process list")
                # if 'marvinGui' in self.processDict.keys():
                #    self.cartGuiUpdateQueue.put()
            except KeyError:
                config.log(f"removal of process requested by {sender} that is not in the process list: {processName}")
        else:
            # life signal from process
            if processName in self.processDict.keys():
                # process already running, update the process dict
                curr = self.processDict[processName]
                curr.update(info)
            else:
                # new process, create the dict
                curr = info
                stateChanged = True

            # update the shared dict by reassigning the local state
            self.processDict[processName] = curr

            # prepare the message for the gui update
            msg2 = {'cmd': mg.SharedDataItem.PROCESS, 'sender': config.processName,
                    'info': {'processName': processName, 'running': True}}

        # update main gui with new process state when gui is running
        if stateChanged and 'marvinGui' in config.md.processDict.keys():
            if config.logProcessListChanges: config.log(f"update gui with new process state {msg2['running']}")
            config.md.mainGuiUpdateQueue.put(msg2)


    def updateArduinoDict(self, sender, info):

        arduinoIndex = info['arduinoIndex']

        if arduinoIndex in self.arduinoDict.keys():
            curr = self.arduinoDict[arduinoIndex]
            curr.update(info['data'])
        else:
            curr = info['data']
        self.arduinoDict[arduinoIndex] = curr

        if 'marvinGui' in config.md.processDict.keys():
            msg2 = {'cmd': mg.SharedDataItem.ARDUINO, 'sender': config.processName,
                    'info': {'arduinoIndex': arduinoIndex, 'connected': True}}
            # config.log(f"update skeletonGui {info}")
            config.md.skeletonGuiUpdateQueue.put(msg2)


    def updateServoTypeDict(self, sender, info):

        servoTypeName = info['type']

        newServoType: skeletonClasses.ServoType = skeletonClasses.ServoType(info['data'])
        # self.servoTypeDict[servoType] = newServoType
        self.servoTypeDict.update({servoTypeName: newServoType})


    def updateServoStaticDict(self, sender, info):

        servoStaticName = info['servoName']

        newServoStatic: skeletonClasses.ServoStatic = skeletonClasses.ServoStatic(info['data'])
        self.servoStaticDict[servoStaticName] = newServoStatic

        # update derived servo values too
        servoType = self.servoTypeDict[newServoStatic.servoType]
        newServoDerived = skeletonClasses.ServoDerived(newServoStatic, servoType)
        self.servoDerivedDict[servoStaticName] = newServoDerived

        # save modified static dict to file
        config.md.saveServoStaticDict()

        # request new assign in skeletonControl when static data was changed by another process
        if sender != 'skeletonControl':
            msg2 = {'cmd': 'reassign', 'sender': config.processName,
                    'info': {'servoName': servoStaticName}}
            config.md.skeletonRequestQueue.put(msg2)

        # update skeleton gui if running
        if 'marvinGui' in config.md.processDict.keys():
            # config.log(f"update skeletonGui {servoName=}")
            msg = {'cmd': mg.SharedDataItem.SERVO_CURRENT, 'sender': config.processName,
                   'info': {'servoName': servoStaticName}}
            config.md.skeletonGuiUpdateQueue.put(msg)


    def updateServoCurrentDict(self, sender:str, info:dict):

        servoName = info['servoName']

        newServoCurrent = skeletonClasses.ServoCurrent()
        newServoCurrent.updateData(info['data'])

        if config.logInRequestList and servoName in self.servoCurrentDict:
            if not self.servoCurrentDict[servoName].inRequestList and newServoCurrent.inRequestList:
                config.log(f"{servoName=} now inRequestList")

            if self.servoCurrentDict[servoName].inRequestList and not newServoCurrent.inRequestList:
                config.log(f"{servoName=} not inRequestList any more")

        self.servoCurrentDict[servoName] = newServoCurrent


        # update skeleton gui if running
        if 'marvinGui' in config.md.processDict.keys():
            # config.log(f"update skeletonGui {servoName=}")
            msg2 = {'cmd': mg.SharedDataItem.SERVO_CURRENT, 'sender': config.processName,
                    'info': {'servoName': servoName}}
            config.md.skeletonGuiUpdateQueue.put(msg2)

    ################################################
    # thread to handle updates
    ################################################
    def sharedDataUpdateRequests(self):
        # as of python 3.8 we can not update a shared object
        # instead create a dict from the current objects values
        # update this dict with the new values passed in as dict
        # update the object with the new values of this dict
        # replace the object in the shared dict with the updated object
        while True:

            msg = "self.sharedDataUpdateQueue.get()"
            try:
                # as with python 3.7 a shared (proxied) nested dict can not directly be updated
                # - copy the existing subDict into a local dict
                # - update the local dict
                # - replace whole subDict with updated dict
                msg = self.sharedDataUpdateQueue.get()
                #print(f"{stmt=}")
                #updType = updStmt[0]
                cmd = msg['cmd']
                sender = msg['sender']
                info = msg['info']

                if cmd == mg.SharedDataItem.PROCESS:
                    self.updateProcessDict(sender, info)

                elif cmd == mg.SharedDataItem.ARDUINO:
                    self.updateArduinoDict(sender, info)

                elif cmd == mg.SharedDataItem.SERVO_TYPE:
                    self.updateServoTypeDict(sender, info)

                elif cmd == mg.SharedDataItem.SERVO_STATIC:
                    self.updateServoStaticDict(sender, info)

                elif cmd == mg.SharedDataItem.SERVO_CURRENT:
                    self.updateServoCurrentDict(sender, info)


                # single instance shared cart dicts
                elif cmd in [mg.SharedDataItem.CART_CONFIGURATION,
                                 mg.SharedDataItem.CART_STATE,
                                 mg.SharedDataItem.CART_LOCATION,
                                 mg.SharedDataItem.CART_MOVEMENT,
                                 mg.SharedDataItem.PLATFORM_IMU,
                                 mg.SharedDataItem.OBSTACLE_DISTANCE]:
                    #if updType in self.cartDict.keys():
                    #    config.log(f"before: {updType=}, {self.cartDict[updType]} ")
                    self.cartDict[cmd] = info
                    updateCartGui(cmd)
                    #config.log(f"after:  {updType=}, {self.cartDict[updType]} ")

                elif cmd == mg.SharedDataItem.HEAD_IMU:
                    self.cartDict[cmd] = info
                    updateSkeletonGui(cmd)


                elif cmd == mg.SharedDataItem.IR_SENSOR_REFERENCE_DISTANCE:

                    if config.logFlags.irSensor:
                        config.log(f"IR_SENSOR_REFERENCE_DISTANCE {msg}")

                    sensorId = info['sensorId']

                    # update the local copy of the dict
                    config.irSensorReferenceDistanceLocal[sensorId].distances = np.asarray(info['data'], dtype=np.int16)

                    # then reassign the local dict to the shared DictProxy
                    self.cartDict.update({mg.SharedDataItem.IR_SENSOR_REFERENCE_DISTANCE: config.irSensorReferenceDistanceLocal})


                elif cmd == mg.SharedDataItem.FLOOR_OFFSET:
                    # (mg.SharedDataItem.FLOOR_OFFSET, sensorId, step, obstacleHeight, abyssDepth)
                    sensorId = info['sensorId']
                    step = info['step']

                    # update the local copy of the dict
                    config.floorOffsetLocal[sensorId].obstacleHeight[step] = info['height']
                    config.floorOffsetLocal[sensorId].abyssDepth[step] = info['depth']
                    config.floorOffsetLocal[sensorId].lastUpdate = time.time()

                    # then reassign the local dict to the shared DictProxy
                    self.cartDict.update({mg.SharedDataItem.FLOOR_OFFSET: config.floorOffsetLocal})
                    #updateCartGui(updType) updateGui request from cartControl when all sensors have been queried


                else:
                    config.log(f"unknown updType {msg}")

            except Exception as e:
                xc_type, exc_obj, exc_tb = sys.exc_info()
                config.log(f"error in dict update: {msg}, line: {exc_tb.tb_lineno}, {e=}")



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
    starts the server <server> by calling the batch file f"d:/projekte/inmoov/start_{server}.bat"
    :param server:
    :return:
    """
    print(f"{time.time()} - restart server {server}")
    serverBatch = f"d:/projekte/inmoov/start_{server}.bat"
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

    config.share = marvinShares.MarvinShares()

    config.startLogging()

    config.md = MarvinData()
    #config.cc = cartCommandMethods.CartCommandMethods()

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



