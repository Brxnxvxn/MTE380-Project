import cv2 as cv
import numpy as np
import i2c_bus
import log
import time

MIN_CONTOUR_AREA = 20
BLUR_KERNEL = (5, 5)
KP = 0.1
HYSTERESIS_H = 50
HYSTERESIS_V = 20

BASELINE_MOTOR_SPEED = 20;
MIN_MOTOR_SPEED = 17;

LOWER_YELLOW = np.array([15, 150, 80])
UPPER_YELLOW = np.array([35, 255, 255])

#LOWER_YELLOW = np.array([15, 80, 80])
#UPPER_YELLOW = np.array([35, 255, 255])

def approach(frameQueue, enableFlag):
    while enableFlag["enabled"] == True:
       # if(frameQueue.empty()):
         #   log.log("WARNING", f"[{__name__}]: Lego (yellow) frame queue is empty, skipping iteration")
          #  continue
        rawFrame = frameQueue.get()
        frame = cv.resize(rawFrame, (600,480)) #(width, height)
        frameHeight, frameWidth, _ = frame.shape #easier
        display = frame.copy()
        blurred = cv.GaussianBlur(frame, BLUR_KERNEL, 0)      
        hsvBlurred = cv.cvtColor(blurred, cv.COLOR_BGR2HSV) 
        mask = cv.inRange(hsvBlurred, LOWER_YELLOW, UPPER_YELLOW)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours:
            log.log("CRITICAL", f"[{__name__}]: No contours found on current frame. Robot cannot find lego guy")
            continue
        areas = [cv.contourArea(c) for c in contours]
        maxContour = contours[areas.index(max(areas))]
        if cv.contourArea(maxContour) < MIN_CONTOUR_AREA:
            log.log("WARNING", f"[{__name__}]: Contour area too small, skipping iteration")
            continue
        M = cv.moments(maxContour)
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        horizontalError = cx - (frameWidth // 2)
        verticalError = cy - (5 * frameHeight // 6)
        relativeHorError = horizontalError if (abs(horizontalError) > HYSTERESIS_H) else 0
        relativeVerError = verticalError if (abs(verticalError) > HYSTERESIS_V) else 0
        print(f"H {relativeHorError} V {relativeVerError}") 
        if(relativeHorError == 0 and relativeVerError == 0):
            enableFlag["enabled"] = False;
           # continue

        #cv.circle(display, (cx, cy), 5, (0, 0, 255), -1)
        #cv.imshow("camera", display)
        #if cv.waitKey(1) & 0xFF == ord('q'):
        #    break

        driveSignal = BASELINE_MOTOR_SPEED if (relativeVerError > 0) else 0
       # if abs(driveSignal) < MIN_MOTOR_SPEED:
        #    driveSignal = MIN_MOTOR_SPEED if driveSignal > 0 else -MIN_MOTOR_SPEED 
        turnSignal = relativeHorError * KP

        leftMotorSpeed = max(20, min(255, int(driveSignal + turnSignal)))
        rightMotorSpeed = max(20, min(255, int(driveSignal + (-turnSignal))))

        i2c_bus.write_to_motor(0x00, leftMotorSpeed, rightMotorSpeed)
        time.sleep(0.03)

    log.log("INFO", f"[{__name__}]: Lego approach sequence complete")
