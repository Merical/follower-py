#!/usr/bin/ python
# -*- coding:utf-8 -*-
from ctypes import *
import time
import threading
import os
import sys
import tty, termios
import serial
import re
import readchar


class DriverValue(Structure):
    _fields_ = [
        ("x_cmd_value", c_double),
        ("y_cmd_value", c_double),
        ("theta_cmd_value", c_double),
        ("head_servo1_cmd_value", c_double),
        ("head_servo2_cmd_value", c_double),
        ("wheel_cmd0_value", c_double),
        ("wheel_cmd1_value", c_double),
        ("wheel_cmd2_value", c_double),
    ]

def sensor_process(sensor_message):
    global pitch_val
    global roll_val
    global yaw_val
    pattern = re.compile(r'pitch(.*?),roll(.*?),yaw(.*?)$')
    sensor_data = re.findall(pattern, sensor_message.decode('utf-8'))
    pitch_val = float(sensor_data[0][0])
    roll_val = float(sensor_data[0][1])
    yaw_val = float(sensor_data[0][2])
# print("Pitch: %2f, Roll: %2f. Yaw: %2f" %(pitch_val, roll_val, yaw_val))


def run_handsfree(driverValue,loop_time):
    global task_flag
    run_time = 0
    while (run_time < loop_time)&(task_flag == 1):
        time1 = time.time()
        sensor_process(handsfreeDriver(driverValue))
        time2 = time.time()
        run_time += (time2-time1)
#print ('LCH: the run_time is ',run_time)
        if ((loop_time - run_time) < 0.05): break
    return run_time


def braking(): #1
    forwardValue = DriverValue(0,0,0,0,0,0,0,0)
    run_time = run_handsfree(forwardValue,0.5)
    return 0

def forward(speed,loop_time): #1
    forwardValue = DriverValue(speed,0,0,0,0,0,0,0)
    run_time = run_handsfree(forwardValue, loop_time)

def backward(speed,loop_time): #2
    backwardValue = DriverValue(-speed,0,0,0,0,0,0,0)
    run_time = run_handsfree(backwardValue, loop_time)

def turnLeft(speed,local_left): #3
    if (local_left > 45): loop_time = (local_left * phi_value / 180)/local_speed - delta_time
    else : loop_time = (local_left * phi_value / 180)/speed
    leftValue = DriverValue(0,0,speed,0,0,0,0,0)
    run_time = run_handsfree(leftValue, loop_time)

def turnRight(speed,local_right): #4
    if (local_right > 45): loop_time = (local_right * phi_value / 180)/speed - delta_time
    else : loop_time = (local_right * phi_value / 180)/speed
    rightValue = DriverValue(0,0,-speed,0,0,0,0,0)
    run_time = run_handsfree(rightValue, loop_time)

def circle(l_speed,w_speed,count):
    drive_value = DriverValue(l_speed,0,w_speed,0,0,0,0,0)
    loop_time = (360*count*phi_value/180)/w_speed
    run_time = run_handsfree(drive_value, loop_time)

def eight(l_speed,w_speed):
    drive_value = DriverValue(l_speed,0,-w_speed,0,0,0,0,0)
    loop_time = (90*phi_value/180)/w_speed
    run_time = run_handsfree(drive_value,loop_time)
    drive_value = DriverValue(l_speed,0,w_speed,0,0,0,0,0)
    loop_time = (360*phi_value/180)/w_speed
    run_time = run_handsfree(drive_value,loop_time)
    drive_value = DriverValue(l_speed,0,-w_speed,0,0,0,0,0)
    loop_time = (270*phi_value/180)/w_speed
    run_time = run_handsfree(drive_value,loop_time)
    return 'eight'


def thread1_job():
    global task_flag
#    fd = sys.stdin.fileno()
#    old_settings = termios.tcgetattr(fd)
#    while True:
#        try:
#            tty.setraw(fd)
#            ch = sys.stdin.read(1)
#            if ch == 'w':
#                print 'w'
#                print forward(0.2,1)
#            elif ch == 's':
#                print 's'
#                print backward(0.2,1)
#            elif ch == 'a':
#                print 'a'
#                print turnLeft(0.3,10)
#            elif ch == 'd':
#                print 'd'
#                print turnRight(0.3,10)
#            elif ch == '8':
#                print '8'
#                print eight(0.2,0.3)
#            elif ch == 'o':
#                print 'o'
#                print circle(0.2,0.3,1)
#            elif ch == 'q':
#                break
#            else:
#                continue
#        finally:
#            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    while task_flag:
        ch = readchar.readkey()
        if ch == 'w':
            print ('forward')
            forward(0.2,0.1)
        elif ch == 's':
            print ('backward')
            backward(0.2,0.1)
        elif ch == 'a':
            print ('left')
            turnLeft(0.5,10)
        elif ch == 'd':
            print ('right')
            turnRight(0.5,10)
        elif ch == '8':
            print ('eight')
            eight(0.2,0.3)
        elif ch == 'o':
            print ('circle')
            circle(0.2,0.3,1)
        elif ch == 'q':
            break
        else:
            braking()
#time.sleep(0.1)
#braking()

def thread2_job():
    global task_flag
    while True:
        dist = float(hc_sensor())
        print ('HC Sensor: Distance: %0.2f cm' %dist)
        if not (dist>10):
            task_flag = 0
            braking()
            break
        time.sleep(0.05)


if __name__ == '__main__':
    #################################
    ## Default Machine Parameter 
    ## Check with Evironment
    default_forward_speed = 0.4
    max_forward_speed = 1
    max_forward_time = 15
    default_backward_speed = 0.2
    max_backward_speed = 1
    max_backward_time = 15
    default_turning_speed = 0.5
    phi_value = 3.14159265357
    delta_time = 0
    delta_time = 0.05 + 0.05 * default_turning_speed
    task_flag = 1
    pitch_val = 0
    roll_val = 0
    yaw_val = 0
    ###################################

    os.system('sudo chmod 777 /dev/ttyUSB0')

    so = cdll.LoadLibrary
    lib = so("./cpp_extension/libpycallclass.so")   
    lib_hc = so("./cpp_extension/hc_sensor.so")
    handsfreeDriver = lib.handsfreeDriver
    hc_sensor = lib_hc.hc_sensor
    handsfreeDriver.restype = c_char_p
    hc_sensor.restype = c_char_p

#try:
#        ser = serial.Serial('/dev/ttyUSB0',921600)
#    except :
#        print ('open serial failed.')
#        exit(1)
#    print ("LCH: SER IS ", ser)

    thread1 = threading.Thread(target=thread1_job, name = 'MOVE')
    thread2 = threading.Thread(target=thread2_job, name = 'CHECKDIST')
    thread1.start()
#thread2.start()
    print('all done')
