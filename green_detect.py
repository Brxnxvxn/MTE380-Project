import cv2 as cv
import numpy as np
import i2c_bus
import log
import time 

MIN_CONTOUR_AREA = 100
BLUR_KERNEL = (3, 3)
KP = 1
HYSTERESIS = 12

BASELINE_MOTOR_SPEED = 150;
MIN_MOTOR_SPEED = 20;

#lower = np.array([75,  20,  150])
#upper = np.array([105, 130, 255])
lower = np.array([40,  50,  50])
upper = np.array([80, 255, 255])

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv.CAP_PROP_FPS, 30)
cap.set(cv.CAP_PROP_BRIGHTNESS, 100)
cap.set(cv.CAP_PROP_EXPOSURE, -1)

def green_detect(frameQueue, green_detected, green_found):
    while not green_found["found"]:
        
            rawFrame = frameQueue.get()
            frame = cv.resize(rawFrame, (600,480)) #(width, height)
            display = frame.copy()
            frameHeight, frameWidth, _ = frame.shape #easier
            
            blurred = cv.GaussianBlur(frame, BLUR_KERNEL, 0)
            hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
            mask = cv.inRange(hsv, lower, upper)
            
            ## Apply mask to top half of the screen
            if not green_detected["detected"]:
                mask[:frameWidth // 6, :] = 0
            
            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            
            if not contours:
                log.log("WARNING", f"[{__name__}]: Green box not found on current frame, skipping iteration")

                #if the green box is found, perform stop and turn sequence
                if green_detected["detected"]:
                    i2c_bus.write_to_motor(0x00, 0, 0)
                    time.sleep(0.1)
                    i2c_bus.write_to_motor(0x01, 50, 50)
                    time.sleep(0.5)
                    i2c_bus.write_to_motor(0x01, 0, 0)
                    time.sleep(0.3)
                    i2c_bus.write_to_servo("open")
                    time.sleep(0.3)
                    
                    # return to red line following
                    i2c_bus.write_to_motor(0x01, 50, 50)
                    time.sleep(0.9)
                    i2c_bus.write_to_motor(0x01, 0, 0)
                    time.sleep(0.3)
                    i2c_bus.write_to_motor(0x03, 70, 70)
                    time.sleep(0.5)
                    
                    green_found["found"] = True
                    
                continue
            #else:
                #i2c_bus.write_to_motor(0x00, 100, 100)
            
            areas = [cv.contourArea(c) for c in contours]
            maxContour = contours[areas.index(max(areas))]
            if cv.contourArea(maxContour) < MIN_CONTOUR_AREA:
                log.log("WARNING", f"[{__name__}]: Contour area too small, skipping iteration")
                #i2c_bus.write_to_motor(0x00,100, 100)
                continue
            else:
                i2c_bus.write_to_motor(0x00, 100, 255)
                green_detected["detected"] = True
            #else:
            #   print("Found green box")
            #    i2c_bus.write_to_motor(0x00, 0, 0)
            #    continue


            M = cv.moments(maxContour)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            rawError = cy - (frameWidth // 2)
            relativeError = rawError if (abs(rawError) > HYSTERESIS) else 0
            controlSignal = int(KP * relativeError)
            
            #cy_full = cy + (frameHeight * 2 // 3)
            #cv.circle(display, (cx, cy), 5, (0, 0, 255), -1)
            #cv.line(display, (0, frameHeight * 2 // 3), (frameWidth, frameHeight * 2 // 3), (0, 0, 0), 2)
            #cv.imshow("camera", display)
            #if cv.waitKey(1) & 0xFF == ord('q'):
            #    break

            leftMotorSpeed = BASELINE_MOTOR_SPEED + controlSignal
            rightMotorSpeed = BASELINE_MOTOR_SPEED + (-controlSignal)
            print(f"L {leftMotorSpeed} R {rightMotorSpeed}")
            #i2c_bus.write_to_motor(0x00, leftMotorSpeed, rightMotorSpeed)
            time.sleep(0.05)
