import cv2 as cv
import threading
import queue
import time

import line_following
import bullseye_detect
import lego_detect
import i2c_bus
import log
import green_detect

log.log("INFO", f"[{__name__}]: Master program started")

# start line following again and arm green detection
# when green contour is big enough, stop line following initiate drop off sequence
# disarm green detection and turn towards line
# restart line following

redLineFollowEnable = True;
bullseyeDetected = {"detected": False}
legoApproachEnable = {"enabled": True}
greenDetected = {"detected": False}
greenFound = {"found": False}

lineQueue = queue.Queue(maxsize=2)
bullseyeQueue = queue.Queue(maxsize=2)
legoQueue = queue.Queue(maxsize=2)
greenBoxQueue = queue.Queue(maxsize=2)

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv.CAP_PROP_FPS, 30)
cap.set(cv.CAP_PROP_BRIGHTNESS, 100)
cap.set(cv.CAP_PROP_EXPOSURE, 0)
cap.set(cv.CAP_PROP_EXPOSURE, -4)

# WAIT FOR START SIGNAL
log.log("INFO", f"[{__name__}]: Waiting for start command x")
while input() != 'x':
    pass
i2c_bus.write_to_servo("open")
log.log("INFO", f"[{__name__}]: Servo arm opened")
log.log("INFO", f"[{__name__}]: Starting line following")

# LINE FOLLOWING AND BLUE DETECTION
t1 = threading.Thread(target=line_following.follow, args=(lineQueue, lambda: redLineFollowEnable))
t2 = threading.Thread(target=bullseye_detect.detect, args=(bullseyeQueue, bullseyeDetected))
t1.start()
t2.start()

while redLineFollowEnable:
    ret, rawFrame = cap.read()
    if not ret:
        log.log("CRITICAL", f"[{__name__}]: Failed to grab frame")
        quit(1)
    if not lineQueue.full():
        lineQueue.put(rawFrame)
    if not bullseyeQueue.full():
        bullseyeQueue.put(rawFrame)
    if(bullseyeDetected["detected"]):
        log.log("INFO", f"[{__name__}]: Bullseye detected. Stopping line following")
        redLineFollowEnable = False
log.log("INFO", f"[{__name__}]: Stopping motors")
#i2c_bus.write_to_motor(0x00, 0,0)

t1.join()
t2.join()
i2c_bus.write_to_motor(0x00, 0, 0)
#quit(0)
#i2c_bus.write_to_servo("closed")
#time.sleep(1)

# CENTER ON LEGO GUY
t3 = threading.Thread(target=lego_detect.approach, args=(legoQueue, legoApproachEnable))
t3.start()
while legoApproachEnable["enabled"] == True:
    ret, rawFrame = cap.read()
    if not ret:
        log.log("CRITICAL", f"[{__name__}]: Failed to grab frame")
        quit(1)
    if not legoQueue.full():
        legoQueue.put(rawFrame)
t3.join()
i2c_bus.write_to_motor(0x00, 0, 0)
log.log("INFO", f"[{__name__}]: Lego guy reached")
time.sleep(0.03)

# PICK UP LEGO GUY UP AND TURN AROUND
i2c_bus.write_to_motor(0x00, 35, 35)
time.sleep(0.4)
i2c_bus.write_to_motor(0x00, 0, 0)
time.sleep(0.03)
i2c_bus.write_to_servo("closed")
log.log("INFO", f"[{__name__}]: Servo arm closed")
time.sleep(0.8)
log.log("INFO", f"[{__name__}]: Turning around for return")
i2c_bus.write_to_motor(0x02, 200, 200)
time.sleep(1)
i2c_bus.write_to_motor(0x02, 0, 0)
time.sleep(0.1)

# CONTINUE LINE FOLLOWING THREAD AND START GREEN DETECTION
redLineFollowEnable = True
t4 = threading.Thread(target=line_following.follow, args=(lineQueue, lambda: redLineFollowEnable))
t5 = threading.Thread(target=green_detect.green_detect, args=(greenBoxQueue, greenDetected, greenFound))

t4.start()
t5.start()
                                                              

while redLineFollowEnable:
    ret, rawFrame = cap.read()
    if not ret:
        log.log("CRITICAL", f"[{__name__}]: Failed to grab frame")
        quit(1)
    if not lineQueue.full():
        lineQueue.put(rawFrame)
    if not greenBoxQueue.full():
        greenBoxQueue.put(rawFrame)
    if greenDetected["detected"]:
        log.log("INFO", f"[{__name__}]: Green box detected. Stopping line following")
        redLineFollowEnable = False

t4.join()

while not greenFound["found"]:
    ret, rawFrame = cap.read()
    if not ret:
        log.log("CRITICAL", f"[{__name__}]: Failed to grab frame")
        quit(1)
    if not greenBoxQueue.full():
        greenBoxQueue.put(rawFrame)


t5.join()

redLineFollowEnable = True
t6 = threading.Thread(target=line_following.follow, args=(lineQueue, lambda: redLineFollowEnable))

t6.start()

while redLineFollowEnable:
    ret, rawFrame = cap.read()
    if not ret:
        log.log("CRITICAL", f"[{__name__}]: Failed to grab frame")
        quit(1)
    if not lineQueue.full():
        lineQueue.put(rawFrame)

t6.join()
        
cap.release()
cv.destroyAllWindows()
