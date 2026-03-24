import cv2 as cv
import numpy as np
import i2c_bus
import log
import time

MIN_CONTOUR_AREA = 20
BLUR_KERNEL = (3, 3)
KP = 0.9
KD = 0
HYSTERESIS = 12

BASELINE_MOTOR_SPEED = 255
MIN_MOTOR_SPEED = 20

#LOWER_RED1 = np.array([0,   140, 55])
#UPPER_RED1 = np.array([20,   255, 255])
#LOWER_RED2 = np.array([145, 140, 55])
#UPPER_RED2 = np.array([180, 255, 255])

LOWER_RED1 = np.array([  0, 120,  70])
UPPER_RED1 = np.array([ 10, 255, 255])
LOWER_RED2 = np.array([160, 120,  70])
UPPER_RED2 = np.array([180, 255, 255])


def follow(frameQueue, enableFlag):
    prevError = 0.0
    while enableFlag():
        #if(frameQueue.empty()):
         #   log.log("WARNING", f"[{__name__}]: Line (red) frame queue is empty, skipping iteration")
          #  time.sleep(0.1)
           # continue
        rawFrame = frameQueue.get()
        frame = cv.resize(rawFrame, (600,480)) #(width, height)
        display = frame.copy()

        frameHeight, frameWidth, _ = frame.shape #easier
        bottomThird = frame[frameHeight * 2 // 3 : frameHeight, :, :] #height, width

        blurred = cv.GaussianBlur(bottomThird, BLUR_KERNEL, 0)
        hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
        mask1 = cv.inRange(hsv, LOWER_RED1, UPPER_RED1)
        mask2 = cv.inRange(hsv, LOWER_RED2, UPPER_RED2)
        red_mask = cv.bitwise_or(mask1, mask2)

        contours, _ = cv.findContours(red_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours:
            log.log("WARNING", f"[{__name__}]: No contours found on current frame, skipping iteration")
            continue
        areas = [cv.contourArea(c) for c in contours]
        maxContour = contours[areas.index(max(areas))]
        if cv.contourArea(maxContour) < MIN_CONTOUR_AREA:
            log.log("WARNING", f"[{__name__}]: Contour area too small, skipping iteration")
            continue
        M = cv.moments(maxContour)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        rawError = cx - (frameWidth // 2)
        relativeError = rawError if (abs(rawError) > HYSTERESIS) else 0
        
        #MAX_ERROR = frameWidth // 2
        #dynamic_baseline = -(BASELINE_MOTOR_SPEED / MAX_ERROR) * abs(relativeError) + BASELINE_MOTOR_SPEED # normalize linearly
        #dynamic_baseline = max(MIN_MOTOR_SPEED, dynamic_baseline) # clamp to min speed so robot doesn't slow down too much
        controlSignal = int((KP * relativeError) + KD*(relativeError - prevError))
        prevError = rawError
        #cy_full = cy + (frameHeight * 2 // 3) 
        #cv.circle(display, (cx,cy_full), 5, (0, 0, 225), -1)
        #cv.imshow('frame', display)
        #if cv.waitKey(1) & 0xFF == ord('q'):
        #        break
        leftMotorSpeed = max(0, min(255, BASELINE_MOTOR_SPEED + controlSignal)) 
        rightMotorSpeed = max(0, min(255, BASELINE_MOTOR_SPEED + (-controlSignal)))
        i2c_bus.write_to_motor(0x00, leftMotorSpeed, rightMotorSpeed)
        time.sleep(0.03)

    log.log("INFO", f"[{__name__}]: Red line following terminated")
