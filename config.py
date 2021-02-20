
import time, datetime
import logging

from marvinglobal import marvinglobal as mg
from marvinglobal import cartClasses


allMarvinTasks = ["skeletonControl", "skeletonGui", "audio"]

processName = 'marvinData'
persistedServoPositions = {}
lastPositionSaveTime = time.time()
md = None       # MarvinData object
share = None

logServoCurrentMessages = False
logProcessListChanges = True
logInRequestList = True

# create instances of the shared data objects
cartConfigurationLocal = cartClasses.Configuration()
cartStateLocal = cartClasses.State()
cartLocationLocal = cartClasses.Location()
cartMovementLocal = cartClasses.Movement
sensorTestDataLocal = cartClasses.SensorTestData()
floorOffsetLocal = [cartClasses.FloorOffset() for i in range(mg.NUM_IR_DISTANCE_SENSORS)]

obstacleDistanceLocal = cartClasses.ObstacleDistance()
irSensorReferenceDistanceLocal = [cartClasses.IrSensorReferenceDistance() for i in range(mg.NUM_IR_DISTANCE_SENSORS)]

headImuLocal = cartClasses.ImuData()
platformImuLocal = cartClasses.ImuData()

class LogFlags:
    def __init__(self):
        self.irSensor = True

    def clear(self):
        self.irSensor = False

logFlags = LogFlags()


def startLogging():
    logging.basicConfig(
        filename=f"log/marvinData.log",
        level=logging.INFO,
        format='%(message)s',
        filemode="w")


def log(msg):

    logtime = str(datetime.datetime.now())[11:23]
    logging.info(f"{logtime} - {msg}")
    print(f"{logtime} - {msg}")


