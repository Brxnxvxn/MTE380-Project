import cv2 as cv
import numpy as np
import time
import log

MIN_CONTOUR_AREA = 20
BLUR_KERNEL = (3, 3)

LOWER_BLUE = np.array([100, 150, 50])
UPPER_BLUE = np.array([130, 255, 255])

def detect(frameQueue, detectedFlag):
    while detectedFlag["detected"] == False:
       # time.sleep(0.1)
       # if(frameQueue.empty()):
        #    log.log("WARNING", f"[{__name__}]: Bullseye (blue) frame queue is empty, skipping iteration")
         #   continue
        rawFrame = frameQueue.get()
        frame = cv.resize(rawFrame, (600,480)) #(width, height)

        frameHeight, frameWidth, _ = frame.shape #easier
        bottomThird = frame[frameHeight * 2 // 3 : frameHeight, :, :] #height, width

        blurred = cv.GaussianBlur(bottomThird, BLUR_KERNEL, 0)
        hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, LOWER_BLUE, UPPER_BLUE)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours:
            log.log("DEBUG", f"[{__name__}]: Bullseye not found yet")
            time.sleep(0.1) # SHOULD I GET RID OF THIS
            continue
        areas = [cv.contourArea(c) for c in contours]
        if max(areas) < MIN_CONTOUR_AREA:
            log.log("INFO", f"[{__name__}]: Bullseye contour found but area is too small")
            continue
        
        detectedFlag["detected"] = True

    log.log("INFO", f"[{__name__}]: Bullseye detected. Function complete")
