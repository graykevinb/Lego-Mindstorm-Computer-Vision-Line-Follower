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
    '''Transforms image data into an inverted binary roi'''
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
        sleep(0.001)
        k =cv2.waitKey(1)
        
        if k%256 == 27:
            break
        else:
            pass
        binary = preprocess(frame)
        cv2.imshow("binary", binary.astype(float))
        #cv2.imshow("frame", frame)
        edges = line_extraction(binary)
        try:
            center_of_line = median(edges)
            print('Center of line:', center_of_line)
        except:
            print('WARN could not find black(TRUE) pixels')
        error = 320 - center_of_line
        output = abs(PID_Loop(
            center_of_line, error_prior, Kp=1, Kd=0, Ki=0,desired_value=320,
            iteration_time=time()-start_time))
        error_prior = error 
        print('output:', output)
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
