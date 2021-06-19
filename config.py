
import time, datetime
import logging

from marvinglobal import marvinglobal as mg
from marvinglobal import skeletonClasses
from marvinglobal import cartClasses
from marvinglobal import environmentClasses
from marvinglobal import marvinShares

allMarvinTasks = ["skeletonControl", "skeletonGui", "audio"]

processName = 'marvinData'
persistedServoPositions = {}
lastPositionSaveTime = time.time()
md = None       # MarvinData object
marvinShares = marvinShares.MarvinShares()

logServoCurrentMessages = False
logProcessListChanges = True
logInRequestList = False

# servo ralated data
servoTypeLocal = {}  #skeletonClasses.ServoType()
servoStaticLocal = {}   #skeletonClasses.ServoStatic()
servoFeedbackLocal = {}
servoDerivedLocal = {}
servoCurrentLocal = {}

# cart related data
cartConfigurationLocal = cartClasses.Configuration()
cartStateLocal = cartClasses.State()
cartLocationLocal = mg.Location()
cartTargetLocal = mg.Location()
cartMovementLocal = cartClasses.Movement
sensorTestDataLocal = cartClasses.SensorTestData()
floorOffsetLocal = [cartClasses.FloorOffset() for i in range(mg.NUM_IR_DISTANCE_SENSORS)]

usSensorDistanceLocal = cartClasses.UsSensorDistance()
irSensorReferenceDistanceLocal = [cartClasses.IrSensorReferenceDistance() for i in range(mg.NUM_IR_DISTANCE_SENSORS)]

headImuLocal = cartClasses.ImuData()
platformImuLocal = cartClasses.ImuData()


# environment related data
roomDataLocal = environmentClasses.RoomData()
scanLocationListLocal = environmentClasses.ScanLocationList()
markerListLocal = environmentClasses.MarkerList()


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


