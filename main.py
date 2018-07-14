#Brace yourself, some illegal stuff. This code is probably not compliant with PEP 8.
#The code was intended to be written with the functional programming philosphy, but I got sloppy because I was in a rush
#This really be fixed, but here it is:
from time import time, sleep
import cv2
import numpy as np
import brickpi3
from statistics import median
from imutils.video import WebcamVideoStream
from imutils.video import FPS
BP = brickpi3.BrickPi3()
cam = WebcamVideoStream(src=0).start()
#Set threshold for converting image to binary

fps = FPS().start()
def drive(left_motor, right_motor, max_speed=50, min_speed=100):
    '''Takes the speed for the motors and processes it for the specific robot'''
    if right_motor < min_speed:
        right_motor = -min_speed
    elif left_motor < -min_speed:
        left_motor = -min_speed

    if left_motor > max_speed:
        left_motor = max_speed
    if right_motor > max_speed:
        right_motor = max_speed
    print('C:', -right_motor)
    print('B:', -left_motor)
    BP.set_motor_dps(BP.PORT_B, -left_motor)
    BP.set_motor_dps(BP.PORT_C, -right_motor)

def preprocess(data):
    '''Takes in a array and converts it to binary and selects a region of interest.'''
    roi = data[238:240, 0:640]
    grayscale = cv2.cvtColor(roi,cv2.COLOR_BGR2GRAY)
    binary = grayscale > 65
    inverted_binary = np.invert(binary)
    return np.array(list(map(lambda x, y: x or y, inverted_binary[0], inverted_binary[1])))

def line_extraction(data):
    '''Returns an array containing the indexes of pixels with a value of True'''

    edges = []
    for i in enumerate(data):
        if i[1] == True:
            edges.append(i[0])
    return edges

def PID_Loop(line_position, error_prior=0, Kp=0.0, Ki=0.00, Kd=0, desired_value=0, integral=1,iteration_time=0):
    '''Takes in an input and puts an output out based on the paremeters. Oh wait that was the definition of a function!
    Well if you ask a dumb question you get a a dumb answer. Okay, that was a little harsh.
    But you should really read up here: https://en.wikipedia.org/wiki/PID_controller if you odn't understand. '''
    error = desired_value - line_position
    integral = integral + (error*iteration_time)
    derivative = (error - error_prior) / iteration_time
    return float("{0:.10f}".format(Kp*error + Ki*integral + Kd*derivative))

def main():
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))
    error_prior = 1
    speed = 100
    center_of_line = 0
    while True:
        start_time = time()
        frame = cam.read()
        k = cv2.waitKey(1)
        
        if k%256 == 27:
            break
        else:
            pass
        binary = preprocess(frame)
        cv2.imshow("binary", binary.astype(float))
        edges = line_extraction(binary)
        try:
            #The thing below this comment is what tracks the line. Yes, it really is that simple. 
            center_of_line = median(edges)
            print('Center of line:', center_of_line)
        except:
            print('WARN could not find black(TRUE) pixels')
        error = 320 - center_of_line
        #That was probably not PEP 8 compliant, I think. Yah, whatever I'm short on time.
        output = abs(PID_Loop(
            center_of_line, error_prior, Kp=1, Kd=0, Ki=0,desired_value=320,
            iteration_time=time()-start_time))
        error_prior = error 
        print('output:', output)
        #Drive the robot from PID output
        if center_of_line == 320:
            drive(speed, speed)
        elif center_of_line < 320:
            drive(speed-output, speed+output, max_speed=speed)
        elif center_of_line > 320:
            drive(speed+output, speed-output, max_speed=speed)
        fps.update()
try:
    main()
except KeyboardInterrupt:
    pass
fps.stop()
print('FPS:', fps.fps())
BP.reset_all()
cam.stop()
cv2.destroyAllWindows()
#So that wasn't so bad was it? 
#So, maybe this isn't the best code in the world. But hey what is the fork button for?
#I actually, should fix my own mistakes.
