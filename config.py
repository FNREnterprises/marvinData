
import time, datetime
import logging

from marvinglobal import marvinglobal as mg
from marvinglobal import skeletonClasses
from marvinglobal import cartClasses
from marvinglobal import distanceSensorClasses
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
servoTypeNew = {}  #skeletonClasses.ServoType()
servoStaticNew = {}   #skeletonClasses.ServoStatic()
servoFeedbackNew = {}
servoDerivedNew = {}
servoCurrentNew = {}

# cart related data
cartConfigurationNew = cartClasses.Configuration()
cartStateNew = cartClasses.State()
cartLocationNew = mg.Location()
cartTargetNew = cartClasses.Target()
cartMovementNew = cartClasses.Movement()
sensorTestDataNew = cartClasses.SensorTestData()
floorOffsetNew = [distanceSensorClasses.FloorOffset() for i in range(mg.NUM_IR_DISTANCE_SENSORS)]
swipingIrSensorsNew = {} # [distanceSensorClasses.SwipingIrSensor() for i in range(mg.NUM_SWIPING_IR_DISTANCE_SENSORS)]
staticIrSensorsNew = {} #[distanceSensorClasses.StaticIrSensor() for i in range(mg.NUM_STATIC_IR_DISTANCE_SENSORS)]
usSensorsNew = {} #[distanceSensorClasses.UsSensor() for i in range(mg.NUM_US_DISTANCE_SENSORS)]
#irSensorReferenceDistanceNew = [cartClasses.IrSensorReferenceDistance() for i in range(mg.NUM_IR_DISTANCE_SENSORS)]

headImuNew = cartClasses.HeadImu()
platformImuNew = cartClasses.PlatformImu()


# environment related data
roomDataNew = environmentClasses.RoomData()
scanLocationListNew = environmentClasses.ScanLocationList()
markerListNew = environmentClasses.MarkerList()


class LogFlags:
    def __init__(self):
        self.irSensor = True
        self.usSensor = True

    def clear(self):
        self.irSensor = False
        self.usSensor = False

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


