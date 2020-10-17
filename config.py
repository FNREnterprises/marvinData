
import time, datetime
import logging


SERVO_STATIC_DEFINITIONS_FILE = 'c:/projekte/inmoov/robotControl/servoStaticDefinitions.json'
SERVO_TYPE_DEFINITIONS_FILE = 'c:/projekte/inmoov/marvinData/servoTypeDefinitions.json'
PERSISTED_SERVO_POSITIONS_FILE = 'c:/projekte/inmoov/robotControl/persistedServoPositions.json'

persistedServoPositions = {}
lastPositionSaveTime = time.time()
marvinData = None
mg = None

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


