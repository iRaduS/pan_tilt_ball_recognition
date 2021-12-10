import cv2 as cv
import numpy as np
import pigpio
import threading
from time import sleep

speed_pulse = 5

delta_x = 7
delta_y = 5

pan = 17
tilt = 18

pi = pigpio.pi()
pi.set_servo_pulsewidth(tilt, 1000)
pi.set_servo_pulsewidth(pan, 1500)

capture = cv.VideoCapture(0)

center = (None, None)
max_height, max_width = None, None
screen_center = (None, None)

######################################################################

def tilt_camera_up():
    while True:
        if center != (None, None) and screen_center != (None, None):
            if center[1] > screen_center[1] + delta_y:
                if pi.get_servo_pulsewidth(tilt) + speed_pulse >= 2500:
                    pi.set_servo_pulsewidth(tilt, 2500)
                else:
                    pi.set_servo_pulsewidth(tilt, pi.get_servo_pulsewidth(tilt) + speed_pulse)
    sleep(1)

def tilt_camera_down():
    while True:
        if center != (None, None) and screen_center != (None, None):
            if center[1] < screen_center[1] - delta_y:
                if pi.get_servo_pulsewidth(tilt) - speed_pulse <= 500:
                    pi.set_servo_pulsewidth(tilt, 500)
                else:
                    pi.set_servo_pulsewidth(tilt, pi.get_servo_pulsewidth(tilt) - speed_pulse)
    sleep(1)

def tilt_camera_left():
    while True:
        if center != (None, None) and screen_center != (None, None):
            if center[0] < screen_center[0] - delta_x:
                if pi.get_servo_pulsewidth(pan) + speed_pulse <= 2500:
                    pi.set_servo_pulsewidth(pan, 2500)
                else:
                    pi.set_servo_pulsewidth(pan, pi.get_servo_pulsewidth(pan) + speed_pulse)
    sleep(1)

def tilt_camera_right():
    while True:
        if center != (None, None) and screen_center != (None, None):
            if center[0] > screen_center[0] + delta_x:
                if pi.get_servo_pulsewidth(pan) - speed_pulse <= 500:
                    pi.set_servo_pulsewidth(pan, 500)
                else:
                    pi.set_servo_pulsewidth(pan, pi.get_servo_pulsewidth(pan) - speed_pulse)
    sleep(1)

threads = {}
threads["tilt_camera_down"] = threading.Thread(target=tilt_camera_down)
threads["tilt_camera_up"] = threading.Thread(target=tilt_camera_up)
threads["tilt_camera_left"] = threading.Thread(target=tilt_camera_left)
threads["tilt_camera_right"] = threading.Thread(target=tilt_camera_right)

for thread in threads.values():
    thread.start()


while True:
    ret, frame = capture.read()
    if frame is None:
        break
    max_height, max_width = frame.shape[:2]
    screen_center = (max_width // 2, max_height // 2)

    frame_blur = cv.GaussianBlur(frame, (11, 11), 0)
    hsv = cv.cvtColor(frame_blur, cv.COLOR_BGR2HSV)
    
    orange_lower_limit = np.array([10, 100, 20])
    orange_upper_limit = np.array([29, 255, 255])
    
    kernel = np.ones((5, 5), "uint8")
    
    orange_mask = cv.inRange(hsv, orange_lower_limit, orange_upper_limit)
    orange_mask = cv.erode(orange_mask, None, iterations=2)
    orange_mask = cv.dilate(orange_mask, None, iterations=2)
    
    orange_res = cv.bitwise_and(frame, frame, mask=orange_mask)
    
    contours, _ = cv.findContours(orange_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        contourOfInteres = max(contours, key=cv.contourArea)
        ((x, y), radius) = cv.minEnclosingCircle(contourOfInteres)
        
        M = cv.moments(contourOfInteres)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        
        if radius > 10:
            cv.circle(orange_res, (int(x), int(y)), int(radius), (0, 255, 255), 5)
            
    cv.imshow('Inspector_Gadget', orange_res)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

for thread in threads.values():
    thread.join()

pi.set_servo_pulsewidth(pan, 0)
pi.set_servo_pulsewidth(tilt, 0)
pi.stop()
cv.destroyAllWindows()
