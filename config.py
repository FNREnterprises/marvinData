
import time, datetime
import logging
import marvinglobal.marvinglobal as mg


allMarvinTasks = ["skeletonControl", "skeletonGui", "audio"]

processName = 'marvinData'
persistedServoPositions = {}
lastPositionSaveTime = time.time()
md = None       # MarvinData object
share = None

cartStateLocal = mg.State()
cartLocationLocal = mg.Location()
cartMovementLocal = mg.Movement
sensorTestDataLocal = mg.SensorTestData()
floorOffsetLocal = mg.FloorOffset()
obstacleDistanceLocal = mg.ObstacleDistance()

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


