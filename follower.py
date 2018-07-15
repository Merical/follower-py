#!/usr/bin/ python
# -*- coding:utf-8 -*-
from ctypes import *
import numpy as np
import cv2
import math
import time
import multiprocessing
import os
import sys
import serial
import re
import CMT
import yaml


f = open(os.path.join(os.path.split(os.path.realpath(__file__))[0],'config.yaml'),'r',encoding='utf-8')
config = yaml.load(f.read())

MIN_MATCH_COUNT = config['Basic']['MIN_MATCH_COUNT']
FRAME_WIDTH = config['Basic']['FRAME_WIDTH']
FRAME_HEIGHT = config['Basic']['FRAME_HEIGHT']
DEBUG_FLAG = config['Basic']['DEBUG_FLAG']

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


class Vision_process(multiprocessing.Process):
    def __init__(self, input_queue):
        multiprocessing.Process.__init__(self)
        self.CMT = CMT.CMT()
        self.templet = cv2.resize(cv2.imread(config['Video']['original']), (FRAME_WIDTH, FRAME_HEIGHT))
        self.cap = cv2.VideoCapture(config['Video']['cap'])
        self.message_queue = input_queue
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FPS, 10)

    def run(self):
        status, tl, br, img_init = self.feature_matcher()
        if status:
            tl = (tl[0], tl[1]);br = (br[0], br[1])
            self.tracker(tl, br, img_init)

        else:
            print('LCH: The camera is down!')

    def feature_matcher(self):
        templet_gray = cv2.cvtColor(self.templet, cv2.COLOR_BGR2GRAY)
        feature = cv2.BRISK_create()
        kp0, des0 = feature.detectAndCompute(templet_gray, None)
        flann = cv2.BFMatcher(cv2.NORM_HAMMING)
        cv2.namedWindow('Match', cv2.WINDOW_NORMAL)
        cv2.moveWindow('Match', 50, 50)
        ret, frame = self.cap.read()

        good_count = 0
        while ret:
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            kp1, des1 = feature.detectAndCompute(frame_gray, None)
            good_matches = []
            try:
                matches = flann.knnMatch(des0, des1, k=2)
                for m, n in matches:
                    if m.distance < 0.7 * n.distance:
                        # print("LCH: the current m is: ", m)
                        good_matches.append(m)

            except:
                # print("Not enough matches are found. ")
                cv2.imshow('Match', frame)
                ret, frame = self.cap.read()
                cv2.waitKey(5)
                continue

            if len(good_matches) > MIN_MATCH_COUNT:
                src_pts = np.float32([ kp0[m.queryIdx].pt for m in good_matches ]).reshape(-1, 1, 2)
                dst_pts = np.float32([ kp1[m.trainIdx].pt for m in good_matches ]).reshape(-1, 1, 2)
                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
                matchesMask = mask.ravel().tolist()
                h, w = templet_gray.shape
                pts = np.float32([ [0, 0], [0, h-1], [w-1, h-1], [w-1, 0] ]).reshape(-1, 1, 2)
                try:
                    dst = cv2.perspectiveTransform(pts, M)
                except:
                    cv2.imshow('Match', frame)
                    ret, frame = self.cap.read()
                    cv2.waitKey(5)
                    continue

                # print('LCH: The dst is: ', dst)
                frame = cv2.polylines(frame, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)
                locations = np.array(np.int32(dst))
                length_delta = abs(self.get_length(locations[0][0], locations[2][0]) -
                                   self.get_length(locations[1][0], locations[3][0]))
                print("LCH: The length_delta is :", length_delta)
                good_count += 1 if length_delta < 25 else 0

            else:
                # print("Not enough matches are found - {}/{}".format(len(good_matches), MIN_MATCH_COUNT))
                good_count = 0

            cv2.imshow('Match', frame)

            k = cv2.waitKey(5)&0xff
            if k == ord("q"):
                cv2.destroyAllWindows()
                return ret, 0, 0, 0

            # if k == ord("t"):
            if not good_count < 5:
                cv2.destroyAllWindows()
                # self.cap.release()
                try:
                    init_x = []
                    init_y = []
                    for i in range(4):
                        init_x.append(locations[i][0][0])
                        init_y.append(locations[i][0][1])
                    tl = (min(init_x), min(init_y))
                    br = (max(init_x), max(init_y))

                except:
                    print("Please Check if you really want to track it?")
                    continue

                return ret, tl, br, frame

            ret, frame = self.cap.read()
            print("LCH: The good count is :", good_count)

    def tracker(self, tl, br, img0):
        global default_scale
        global Work_flag
        self.CMT.estimate_scale = True
        self.CMT.estimate_rotation = False
        self.CMT.initialise(cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY), tl, br)

        fps = 0
        cv2.namedWindow('Tracker', cv2.WINDOW_NORMAL)
        cv2.moveWindow('Tracker', 100, 50)
        while True:
            status, im_detect = self.cap.read()

            if status:
                tic = time.time()
                # Read image
                im_detect = cv2.resize(im_detect, (FRAME_WIDTH, FRAME_HEIGHT), interpolation=cv2.INTER_CUBIC)
                im_gray = cv2.cvtColor(im_detect, cv2.COLOR_BGR2GRAY)
                im_draw = np.copy(im_detect)

                self.CMT.process_frame(im_gray)

                if self.CMT.has_result:

                    cv2.line(im_draw, self.CMT.tl, self.CMT.tr, (255, 0, 0), 2)
                    cv2.line(im_draw, self.CMT.tr, self.CMT.br, (255, 0, 0), 2)
                    cv2.line(im_draw, self.CMT.br, self.CMT.bl, (255, 0, 0), 2)
                    cv2.line(im_draw, self.CMT.bl, self.CMT.tl, (255, 0, 0), 2)

                # util.draw_keypoints(self.CMT.tracked_keypoints, im_draw, (255, 255, 255))
                # util.draw_keypoints(self.CMT.votes[:, :2], im_draw)  # blue
                # util.draw_keypoints(self.CMT.outliers[:, :2], im_draw, (0, 0, 255))
                cv2.putText(im_draw, str(fps), (20, 30), 1, 1, (255, 0, 0), 1)
                # cv2.putText(im_draw, cmt_message, (20, 50), 1, 1, (255, 0, 255), 2)
                cv2.imshow('Tracker', im_draw)
                # cmt_message = 'center: ({0:.2f},{1:.2f}) scale: {2:.2f}, active: {3:03d}'.format(self.CMT.center[0], self.CMT.center[1], self.CMT.scale_estimate, self.CMT.active_keypoints.shape[0])
                center_delta = (FRAME_WIDTH/2 - float(self.CMT.center[0]))/100
                scale_delta = default_scale - self.CMT.scale_estimate
                output_message = 'center_delta:{0:.4f};scale_delta:{1:.4f}'.format(center_delta, scale_delta)
                self.message_queue.put(output_message)

                toc = time.time()
                fps = 1/(toc - tic)

                k = cv2.waitKey(5)&0xff
                if k == ord("q"):
                    cv2.destroyAllWindows()
                    Work_flag = False
                    break
            else:
                print('LCH: the camera is down')
                break

    def get_length(self, pt1, pt2):
        return math.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)


class Analyze_process(multiprocessing.Process):
    def __init__(self, input_queue, output_queue, k_val):
        multiprocessing.Process.__init__(self)
        self.vis_queue = input_queue
        self.cmd_queue = output_queue
        self.pid_val = k_val

    def run(self):
        global Work_flag
        while Work_flag:
            analyze_flag = False
            while not self.vis_queue.empty():
                vis_message = self.vis_queue.get()
                if DEBUG_FLAG: print('LCH: The vis_message is', vis_message)
                analyze_flag = True
            if analyze_flag:
                cmd_message = self.analyze(vis_message)
                if DEBUG_FLAG: print('LCH: The cmd_message from Analyze_process is', cmd_message)
                self.cmd_queue.put(cmd_message)
        sys.exit()

    def analyze(self, message):
        pattern = re.compile(r':([-\d\.]+)')
        w_delta, l_delta = re.findall(pattern, message)
        w_delta = float(w_delta); l_delta = float(l_delta)
        if DEBUG_FLAG: print('LCH: The w_delta from Analyze_process is {0:.4f}, the l_delta from Analyze_process is '
                             '{1:.4f}'.format(w_delta, l_delta))
        output_message = self.get_speed(w_delta, l_delta)
        return output_message

    def get_speed(self, c_delta, s_delta):
        global max_l_speed
        global max_w_speed
        global c_delta_last
        global s_delta_last
        global w_speed_last
        global l_speed_last
        global scale_threshold
        global center_threshold

        kp_l, kd_l, kp_w, kd_w = self.pid_val
        if (c_delta * c_delta_last) < 0:
            w_speed = 0; l_speed = 0
            if DEBUG_FLAG: print("LCH: Protection mode from analyze_process, waiting next queue...")

        else:
            if DEBUG_FLAG: print("LCH: Action mode from analyze_process, analyzing...")
            if not abs(c_delta) < center_threshold:
                w_speed = kp_w * c_delta + kd_w * (c_delta - c_delta_last)
            else:
                w_speed = 0

            if not abs(s_delta) < scale_threshold:
                l_speed = kp_l * s_delta + kd_l * (s_delta - s_delta_last)
            else:
                l_speed = 0


            w_speed = self.map_speed(w_speed)
            l_speed = self.map_speed(l_speed)

            w_speed = max_w_speed if w_speed > max_w_speed else w_speed
            l_speed = max_l_speed if l_speed > max_l_speed else l_speed
            w_speed = -max_w_speed if w_speed < -max_w_speed else w_speed
            l_speed = -max_l_speed if l_speed < -max_l_speed else l_speed

        output_message = 'w_speed:{0:.4f};l_speed:{1:.4f};'.format(w_speed, l_speed)

        s_delta_last = s_delta
        c_delta_last = c_delta
        w_speed_last = w_speed
        l_speed_last = l_speed
        return output_message

    def map_speed(self, speed):
        if speed > 0:
            if speed > 0.1:
                speed = math.floor(speed*10)/10
            else: speed = 0.05
        else:
            if speed < 0.1:
                speed = math.ceil(speed*10)/10
            else:
                speed = -0.05
        return speed


class Handsfree_process(multiprocessing.Process):
    def __init__(self, input_queue):
        multiprocessing.Process.__init__(self)
        self.cmd_queue = input_queue

    def run(self):
        global Work_flag
        while Work_flag:
            command_flag = False
            while not self.cmd_queue.empty():
                cmd_message = self.cmd_queue.get()
                if DEBUG_FLAG: print('LCH: The cmd_message from Handsfree_process is', cmd_message)
                command_flag = True
            if command_flag:
                handsfree_value = self.analyze(cmd_message)
                pose_data = self.run_handsfree(handsfree_value)
                print('LCH: Pose_data is', pose_data)
            else:
                pose_data = self.run_handsfree(self.get_value(0, 0))
                print('LCH: Pose_data is', pose_data)

        sys.exit()

    def analyze(self, message):
        pattern = re.compile(r':([-\d\.]+)')
        w_speed_cmd, l_speed_cmd = re.findall(pattern, message)
        w_speed_cmd = float(w_speed_cmd); l_speed_cmd = float(l_speed_cmd)
        drive_value = self.get_value(l_speed_cmd, w_speed_cmd)
        return drive_value

    def get_value(self, l_speed, w_speed):
        return DriverValue(l_speed, 0, w_speed, 0, 0, 0, 0, 0)

    def run_handsfree(self, driverValue):
        for _ in range(3):
            data = handsfreeDriver(driverValue)
        return data

if __name__ == '__main__':
    #################################
    ## Default Machine Parameter
    ## Check with Evironment
    max_l_speed = config['Handsfree']['max_l_speed']
    max_w_speed = config['Handsfree']['max_w_speed']
    w_speed_last = 0
    l_speed_last = 0
    phi_value = config['Handsfree']['phi_value']
    Work_flag = config['Handsfree']['Work_flag']
    pitch_val = 0
    roll_val = 0
    yaw_val = 0

    default_scale = config['Handsfree']['default_scale']
    c_delta_last = 0
    s_delta_last = 0
    center_threshold = config['Handsfree']['center_threshold']
    scale_threshold = config['Handsfree']['scale_threshold']
    k_val = config['Handsfree']['k_val']
    ###################################

    os.system('sudo chmod 777 /dev/ttyUSB0')

    so = cdll.LoadLibrary
    lib = so("./cpp_extension/libpycallclass.so")
    lib_hc = so("./cpp_extension/hc_sensor.so")
    handsfreeDriver = lib.handsfreeDriver
    hc_sensor = lib_hc.hc_sensor
    handsfreeDriver.restype = c_char_p
    hc_sensor.restype = c_char_p

    vis2ana_queue = multiprocessing.Queue()
    ana2cmd_queue = multiprocessing.Queue()

    Vision_task = Vision_process(vis2ana_queue)
    Analyze_task = Analyze_process(vis2ana_queue, ana2cmd_queue, k_val)
    Handsfree_task = Handsfree_process(ana2cmd_queue)
    Vision_task.start()
    Analyze_task.start()
    Handsfree_task.start()

    # print("\n EXIT MAIN PROCESS!")