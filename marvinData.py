#!/home/marvin/miniconda3/envs/py39/bin/python3.9

"""
marvinData maintains the global data used in the InMoov control
"""
import os
import sys
import time
import simplejson as json
import pickle
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


def updateMapGui(msgType):

    if 'marvinGui' in config.md.processDict.keys():
        # config.log(f"update skeletonGui {servoName=}")
        config.md.mapGuiUpdateQueue.put({'msgType': msgType})


class MarvinData:
    """ shared ressources in its own process"""
    def __init__(self):
        super().__init__()

        self.verbose = False
        # shared dictionaries
        shareManager = multiprocessing.Manager()
        self.m = None
        self.s = None

        config.log(f"initialize shared dictionaries")

        self.processDict = shareManager.dict()
        self.arduinoDict = shareManager.dict()

        #config.log(f"open servo types file {config.SERVO_TYPE_DEFINITIONS_FILE}")
        self.servoDict = shareManager.dict()
        # add the different servo data sections
        self.servoDict.update({mg.SharedDataItems.SERVO_TYPE: config.servoTypeLocal})
        self.servoDict.update({mg.SharedDataItems.SERVO_STATIC: config.servoStaticLocal})
        self.servoDict.update({mg.SharedDataItems.SERVO_FEEDBACK: config.servoFeedbackLocal})
        self.servoDict.update({mg.SharedDataItems.SERVO_DERIVED: config.servoDerivedLocal})
        self.servoDict.update({mg.SharedDataItems.SERVO_CURRENT: config.servoCurrentLocal})

        # cart related data dict
        self.cartDict = shareManager.dict()
        # add the different cart data sections
        self.cartDict.update({mg.SharedDataItems.CART_CONFIGURATION: config.cartConfigurationLocal})
        self.cartDict.update({mg.SharedDataItems.CART_STATE: config.cartStateLocal})
        self.cartDict.update({mg.SharedDataItems.CART_LOCATION: config.cartLocationLocal})
        self.cartDict.update({mg.SharedDataItems.CART_MOVEMENT: config.cartMovementLocal})
        self.cartDict.update({mg.SharedDataItems.CART_TARGET: config.cartTargetLocal})
        self.cartDict.update({mg.SharedDataItems.SENSOR_TEST_DATA: config.sensorTestDataLocal})
        self.cartDict.update({mg.SharedDataItems.FLOOR_OFFSET: config.floorOffsetLocal})
        self.cartDict.update({mg.SharedDataItems.ULTRASONIC_SENSORS: config.usSensorDistanceLocal})
        self.cartDict.update({mg.SharedDataItems.IR_SENSOR_REFERENCE_DISTANCE: config.irSensorReferenceDistanceLocal})
        self.cartDict.update({mg.SharedDataItems.HEAD_IMU: config.headImuLocal})
        self.cartDict.update({mg.SharedDataItems.PLATFORM_IMU: config.platformImuLocal})

        self.environmentDict = shareManager.dict()
        self.environmentDict.update({mg.SharedDataItems.ENVIRONMENT_ROOM: config.roomDataLocal})
        self.environmentDict.update({mg.SharedDataItems.ENVIRONMENT_MARKER_LIST: config.markerListLocal})
        self.environmentDict.update({mg.SharedDataItems.ENVIRONMENT_SCAN_LOCATION_LIST: config.scanLocationListLocal})

        config.log(f"setup Queues")
        self.sharedDataUpdateQueue = multiprocessing.Queue()
        self.ikUpdateQueue = multiprocessing.Queue()
        self.skeletonGuiUpdateQueue = multiprocessing.Queue()
        self.cartGuiUpdateQueue = multiprocessing.Queue()
        self.mapGuiUpdateQueue = multiprocessing.Queue()
        self.mainGuiUpdateQueue = multiprocessing.Queue()
        self.skeletonRequestQueue = multiprocessing.Queue()
        self.cartRequestQueue = multiprocessing.Queue()
        self.speakRequestQueue = multiprocessing.Queue()
        self.playGestureQueue = multiprocessing.Queue()
        self.navManagerRequestQueue = multiprocessing.Queue()
        self.imageProcessingRequestQueue = multiprocessing.Queue()


    def saveServoStaticDict(self):
        """
        servoStaticDict is a dict of ServoStatic objects by servoName
        :return:
        """
        servoStaticDefinitions = {}
        servoStaticDict = self.servoDict.get(mg.SharedDataItems.SERVO_STATIC)
        for thisServoName, servoObject in servoStaticDict.items():
            servoStaticDefinitions.update({thisServoName: servoObject.__dict__})

        with open(mg.SERVO_STATIC_DEFINITIONS_FILE, 'w') as outfile:
            json.dump(servoStaticDefinitions, outfile, indent=2)

        if self.verbose: config.log(f"servoStaticDefinitions updated into file")


    def getProcessDict(self):
        return self.processDict

    def getArduinoDict(self):
        return self.arduinoDict

    def getServoDict(self):
        return self.servoDict

    def getCartDict(self):
        return self.cartDict

    def getEnvironmentDict(self):
        return self.environmentDict

    def run(self):
        # register the shared dicts
        ShareManager.register('getProcessDict', self.getProcessDict)
        ShareManager.register('getArduinoDict', self.getArduinoDict)
        ShareManager.register('getServoDict', self.getServoDict)
        ShareManager.register('getCartDict', self.getCartDict)
        ShareManager.register('getEnvironmentDict', self.getEnvironmentDict)

        # register the shared queues
        ShareManager.register('getSharedDataUpdateQueue', callable=lambda: self.sharedDataUpdateQueue)
        ShareManager.register('getIkUpdateQueue', callable=lambda: self.ikUpdateQueue)
        ShareManager.register('getSkeletonGuiUpdateQueue', callable=lambda: self.skeletonGuiUpdateQueue)
        ShareManager.register('getCartGuiUpdateQueue', callable=lambda: self.cartGuiUpdateQueue)
        ShareManager.register('getMapGuiUpdateQueue', callable=lambda: self.mapGuiUpdateQueue)
        ShareManager.register('getMainGuiUpdateQueue', callable=lambda: self.mainGuiUpdateQueue)
        ShareManager.register('getSkeletonRequestQueue', callable=lambda: self.skeletonRequestQueue)
        ShareManager.register('getCartRequestQueue', callable=lambda: self.cartRequestQueue)
        ShareManager.register('getSpeakRequestQueue', callable=lambda:self.speakRequestQueue)
        ShareManager.register('getPlayGestureQueue', callable=lambda:self.playGestureQueue)
        ShareManager.register('getNavManagerRequestQueue', callable=lambda: self.navManagerRequestQueue)
        ShareManager.register('getImageProcessingRequestQueue', callable=lambda: self.imageProcessingRequestQueue)

        # start thread for updating the dicts
        config.log(f"start shareDataUpdateQueue thread")
        updateRequestsThread = threading.Thread(name="sharedDataUpdateRequests", target=self.sharedDataUpdateRequests, args={})
        updateRequestsThread.start()

        # request all processes to be started
        request =  {'msgType': mg.SharedDataItems.START_ALL_PROCESSES, 'sender': 'marvinData', 'info': ''}
        #self.sharedDataUpdateQueue.put(request)
        # 22. april 2021 found no way to make this work (marvinglobal, startProcess)

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
            # check for stopping marvinData process
            if info['processName'] == 'marvinData':
                config.log(f"received request from {sender} to stop marvinData process, going down")
                os.execv()  # restart this process

            # prepare the message for the gui update
            msg2 = {'msgType': mg.SharedDataItems.PROCESS, 'sender': config.processName,
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
            msg2 = {'msgType': mg.SharedDataItems.PROCESS, 'sender': config.processName,
                    'info': {'processName': processName, 'running': True}}

        # update main gui with new process state when gui is running
        if stateChanged and 'marvinGui' in config.md.processDict.keys():
            if config.logProcessListChanges: config.log(f"update gui with new process state {msg2['info']}")
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
            msg2 = {'msgType': mg.SharedDataItems.ARDUINO, 'sender': config.processName,
                    'info': {'arduinoIndex': arduinoIndex, 'connected': True}}
            # config.log(f"update skeletonGui {info}")
            config.md.skeletonGuiUpdateQueue.put(msg2)


    def updateServoTypeDict(self, sender, info):
        servoTypeName = info['type']
        newServoType: skeletonClasses.ServoType = skeletonClasses.ServoType()
        newServoType.updateValues(info['data'])
        config.servoTypeLocal.update({servoTypeName: newServoType})
        self.servoDict.update({mg.SharedDataItems.SERVO_TYPE: config.servoTypeLocal})

    def updateServoStaticDict(self, sender, info):
        servoStaticName = info['servoName']
        newServoStatic: skeletonClasses.ServoStatic = skeletonClasses.ServoStatic()
        newServoStatic.updateValues(info['data'])
        config.servoStaticLocal.update({servoStaticName: newServoStatic})
        self.servoDict.update({mg.SharedDataItems.SERVO_STATIC: config.servoStaticLocal})

        # update derived servo values too
        servoTypeDict = self.servoDict.get(mg.SharedDataItems.SERVO_TYPE)
        servoType = servoTypeDict[newServoStatic.servoType]
        newServoDerived = skeletonClasses.ServoDerived()
        newServoDerived.updateValues(newServoStatic, servoType)
        config.servoDerivedLocal.update({servoStaticName: newServoDerived})
        self.servoDict.update({mg.SharedDataItems.SERVO_DERIVED: config.servoDerivedLocal})

        # save modified static dict to file
        config.md.saveServoStaticDict()

        # request new assign in skeletonControl when static data was changed by another process
        if sender != 'skeletonControl':
            msg2 = {'msgType': 'reassign', 'sender': config.processName,
                    'info': {'servoName': servoStaticName}}
            config.md.skeletonRequestQueue.put(msg2)

        # update skeleton gui if running
        if 'marvinGui' in config.md.processDict.keys():
            # config.log(f"update skeletonGui {servoName=}")
            msg = {'msgType': mg.SharedDataItems.SERVO_CURRENT, 'sender': config.processName,
                   'info': {'servoName': servoStaticName}}
            config.md.skeletonGuiUpdateQueue.put(msg)

    def updateServoFeedbackDict(self, sender:str, info:dict):
        servoName = info['servoName']
        newServoFeedback = skeletonClasses.ServoFeedback()
        newServoFeedback.updateValues(info['data'])  # allow partial update
        config.servoFeedbackLocal.update({servoName: newServoFeedback})
        self.servoDict.update({mg.SharedDataItems.SERVO_FEEDBACK: config.servoFeedbackLocal})

    def updateServoCurrentDict(self, sender:str, info:dict):

        servoName = info['servoName']

        #newServoCurrent.updateValues(info['data'])
        # into can be all fields or position only
        newServoCurrent = skeletonClasses.ServoCurrent()
        newServoCurrent.updateValues(info['data'])

        if config.logInRequestList and servoName in self.servoCurrentDict:
            if not self.servoCurrentDict[servoName].inRequestList and newServoCurrent.inRequestList:
                config.log(f"{servoName=} now inRequestList")

            if self.servoCurrentDict[servoName].inRequestList and not newServoCurrent.inRequestList:
                config.log(f"{servoName=} not inRequestList any more")

        #self.servoCurrentDict[servoName] = newServoCurrent
        config.servoCurrentLocal.update({servoName: newServoCurrent})
        self.servoDict.update({mg.SharedDataItems.SERVO_CURRENT: config.servoCurrentLocal})


        # update skeleton gui if running
        if 'marvinGui' in config.md.processDict.keys():
            # config.log(f"update skeletonGui {servoName=}")
            msg2 = {'msgType': mg.SharedDataItems.SERVO_CURRENT, 'sender': config.processName,
                    'info': {'servoName': servoName}}
            config.md.skeletonGuiUpdateQueue.put(msg2)


    ################################################
    # thread to handle shared data updates
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
                cmd = msg['msgType']
                sender = msg['sender']
                info = msg['info']

                if cmd == mg.SharedDataItems.START_ALL_PROCESSES:
                    config.marvinShares.startAllProcesses()

                elif cmd == mg.SharedDataItems.PROCESS:
                    self.updateProcessDict(sender, info)

                elif cmd == mg.SharedDataItems.ARDUINO:
                    self.updateArduinoDict(sender, info)

                elif cmd == mg.SharedDataItems.SERVO_TYPE:
                    self.updateServoTypeDict(sender, info)

                elif cmd == mg.SharedDataItems.SERVO_STATIC:
                    self.updateServoStaticDict(sender, info)

                elif cmd == mg.SharedDataItems.SERVO_FEEDBACK:
                    self.updateServoFeedbackDict(sender, info)


                elif cmd == mg.SharedDataItems.SERVO_CURRENT:
                    self.updateServoCurrentDict(sender, info)

                elif cmd == mg.SharedDataItems.CART_MOVEMENT:
                    self.cartDict[cmd] = info  #pickle.loads(info)
                    updateCartGui(cmd)

                # single instance shared cart dicts
                elif cmd in [mg.SharedDataItems.CART_CONFIGURATION,
                                 mg.SharedDataItems.CART_STATE,
                                 mg.SharedDataItems.CART_LOCATION,
                                 mg.SharedDataItems.CART_TARGET,
                                 mg.SharedDataItems.PLATFORM_IMU,
                                 mg.SharedDataItems.ULTRASONIC_SENSORS]:
                    #if updType in self.cartDict.keys():
                    #    config.log(f"before: {updType=}, {self.cartDict[updType]} ")
                    self.cartDict[cmd] = info
                    updateCartGui(cmd)
                    #config.log(f"after:  {updType=}, {self.cartDict[updType]} ")

                elif cmd == mg.SharedDataItems.HEAD_IMU:
                    self.cartDict[cmd] = info
                    config.log(f"updateSkeletonGui triggered by HEAD_IMU")
                    updateSkeletonGui(cmd)

                elif cmd == mg.SharedDataItems.SENSOR_TEST_DATA:
                    # sensorId + distances of a single scan
                    if config.logFlags.irSensor:
                        config.log(f"SENSOR_TEST_DATA {msg}")

                    irSensorId = info['irSensorId']
                    newTest = info['newTest']
                    if newTest:
                        config.sensorTestDataLocal.numMeasures = np.zeros((mg.NUM_SCAN_STEPS), dtype=np.int8)
                        config.sensorTestDataLocal.sumMeasures = np.zeros((mg.NUM_SCAN_STEPS), dtype=np.int16)

                    # update the local copy of the dict including building the sums
                    distArray = np.asarray(info['distances'], dtype=np.int16)
                    config.sensorTestDataLocal.distance = distArray
                    config.sensorTestDataLocal.numMeasures[distArray > 0] += 1
                    config.sensorTestDataLocal.sumMeasures += distArray

                    # then reassign the local dict to the shared DictProxy
                    self.cartDict.update({mg.SharedDataItems.SENSOR_TEST_DATA: config.sensorTestDataLocal})

                    # and inform cart gui about the new values
                    updateCartGui(cmd)

                elif cmd == mg.SharedDataItems.IR_SENSOR_REFERENCE_DISTANCE:

                    if config.logFlags.irSensor:
                        config.log(f"IR_SENSOR_REFERENCE_DISTANCE {msg}")

                    irSensorId = info['irSensorId']

                    # update the local copy of the dict
                    config.irSensorReferenceDistanceLocal[irSensorId].distances = np.asarray(info['distances'], dtype=np.int16)

                    # then reassign the local dict to the shared DictProxy
                    self.cartDict.update({mg.SharedDataItems.IR_SENSOR_REFERENCE_DISTANCE: config.irSensorReferenceDistanceLocal})


                elif cmd == mg.SharedDataItems.FLOOR_OFFSET:
                    # (mg.SharedDataItems.FLOOR_OFFSET, irSensorId, step, obstacleHeight, abyssDepth)
                    irSensorId = info['irSensorId']
                    step = info['step']

                    # update the local copy of the dict
                    config.floorOffsetLocal[irSensorId].obstacleHeight[step] = info['height']
                    config.floorOffsetLocal[irSensorId].abyssDepth[step] = info['depth']
                    config.floorOffsetLocal[irSensorId].lastUpdate = time.time()

                    # then reassign the local dict to the shared DictProxy
                    self.cartDict.update({mg.SharedDataItems.FLOOR_OFFSET: config.floorOffsetLocal})
                    #updateCartGui(updType) updateGui request from cartControl when all sensors have been queried

                elif cmd == mg.SharedDataItems.ENVIRONMENT_ROOM:
                    self.environmentDict[cmd] = info
                    updateMapGui(cmd)

                elif cmd == mg.SharedDataItems.ENVIRONMENT_SCAN_LOCATION_LIST:
                    self.environmentDict[cmd] = info
                    updateMapGui(cmd)

                elif cmd == mg.SharedDataItems.ENVIRONMENT_MARKER_LIST:
                    self.environmentDict[cmd] = info
                    updateMapGui(cmd)


                else:
                    config.log(f"unknown updType {msg}")

            except Exception as e:
                xc_type, exc_obj, exc_tb = sys.exc_info()
                config.log(f"error in dict update: {msg}, frame: {exc_tb.tb_frame} line: {exc_tb.tb_lineno}, {e=}")



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
    time.sleep(2)   # on a restart wait for processes to terminate themselfs
    cleanup()       # make sure all marvin processes are down, otherwise kill them

    #config.share = marvinShares.MarvinShares()

    config.startLogging()

    config.md = MarvinData()

    # do this when data has been received from skeleton control
    """
    # eval degrees from pos after creating an instance of MarvinData
    for servoName, servoCurrent in config.md.servoCurrentDict.items():
        servoDerived = config.md.servoDerivedDict[servoName]
        servoStatic = config.md.servoStaticDict[servoName]
        p = servoCurrent.position
        servoCurrent.degrees = mg.evalDegFromPos(servoStatic, servoDerived, p)
    """
    config.log("marvinData initialized")

    config.md.run()

    # startServer works only for running tasks in run mode (no debugging)
    # for process in config.allMarvinTasks:
    #    startServer(process)
    # during development you might want to start tasks through the pycharm ide

    config.log("marvinData stopped")



