
import time, datetime
import logging
import marvinglobal.marvinglobal as mg
import marvinglobal.cartClasses
import marvinglobal.servoClasses as servoCls

allMarvinTasks = ["skeletonControl", "skeletonGui", "audio"]

processName = 'marvinData'
persistedServoPositions = {}
lastPositionSaveTime = time.time()
md = None       # MarvinData object
share = None

cartStateLocal = marvinglobal.cartClasses.State()
cartLocationLocal = marvinglobal.cartClasses.Location()
cartMovementLocal = marvinglobal.cartClasses.Movement
sensorTestDataLocal = marvinglobal.cartClasses.SensorTestData()
floorOffsetLocal = [marvinglobal.cartClasses.FloorOffset() for i in range(mg.NUM_IR_DISTANCE_SENSORS)]

obstacleDistanceLocal = marvinglobal.cartClasses.ObstacleDistance()
irSensorReferenceDistanceLocal = [marvinglobal.cartClasses.IrSensorReferenceDistance() for i in range(mg.NUM_IR_DISTANCE_SENSORS)]

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


