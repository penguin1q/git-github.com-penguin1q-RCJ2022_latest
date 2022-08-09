# -*- coding: utf-8 -*-
import cv2
import configparser
import os
import sys
import errno
import numpy as np
from pathlib import Path
from Camera_class import Find_rect
import time
from multiprocessing import Process, Value, Array
import ctypes

xmin, xmax = 280, 1000
ymin, ymax = 0, 720

GST_STR = 'nvarguscamerasrc \
		! video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12, framerate=(fraction)120/1 \
		! nvvidconv ! video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx \
		!videobalance  \
        ! videoconvert \
		! appsink max-buffers=1 drop=True'
		

class Camera_Module: 
    def __init__(self):
        self.config_ini = configparser.ConfigParser()
        self.config_ini_path = str(Path(__file__).resolve().parent.parent) + '/config.ini'
        if not os.path.exists(self.config_ini_path):
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), self.config_ini_path)
        self.config_ini.read(self.config_ini_path, encoding='utf-8')
        self.orange = Find_rect(eval(self.config_ini.get('Camera', 'Orange_min')), eval(self.config_ini.get('Camera', 'Orange_max')))
        self.blue = Find_rect(eval(self.config_ini.get('Camera', 'Blue_min')), eval(self.config_ini.get('Camera', 'Blue_max')))
        self.yellow = Find_rect(eval(self.config_ini.get('Camera', 'Yellow_min')), eval(self.config_ini.get('Camera', 'Yellow_max')))
        self.capture = cv2.VideoCapture(GST_STR, cv2.CAP_GSTREAMER)
        self.blue_rects = []
        self.yellow_rects = []
        self.orange_rects = []
        self.blue_rect = []
        self.yellow_rect = []
        self.orange_rect = []
        self.loopcount = 0
        self.color = {'orange':0, 'blue':1, 'yellow':2}
        self.pretime = 0
    

    def read_pos_data(self, orange_freq, blue_freq, yellow_freq, freq, raw=False, dposx = 0, dposy = 0):
        self.pretime = time.perf_counter()
        _, frame = self.capture.read()
        dt1 = time.perf_counter() - self.pretime
        pretime1 = time.perf_counter()
        
        hsv = cv2.cvtColor(frame[:,xmin:xmax], cv2.COLOR_BGR2HSV_FULL)
        dt2 = time.perf_counter() - pretime1
        pretime2 = time.perf_counter()
        
        if self.loopcount in orange_freq:
            self.orange_rects = self.orange.find_rect_find_target_color(hsv)
            if self.orange_rects:
                self.orange_rect = max(self.orange_rects, key=(lambda x: x[2] * x[3]))
            #print(1, end=" ")
        elif self.loopcount in blue_freq:
            self.blue_rects = self.blue.find_rect_find_target_color(hsv)
            if self.blue_rects:
                self.blue_rect = max(self.blue_rects, key=(lambda x: x[2] * x[3]))
            #print(2, end=" ")
        elif self.loopcount in yellow_freq:
            self.yellow_rects = self.yellow.find_rect_find_target_color(hsv)
            if self.yellow_rects:
                self.yellow_rect = max(self.yellow_rects, key=(lambda x: x[2] * x[3]))
            #print(3, end=" ")
        elif not orange_freq:
            self.orange_rects = self.orange.find_rect_find_target_color(hsv)
            if self.orange_rects:
                self.orange_rect = max(self.orange_rects, key=(lambda x: x[2] * x[3]))
            #print(1, end=" ")
        elif not blue_freq:
            self.blue_rects = self.blue.find_rect_find_target_color(hsv)
            if self.blue_rects:
                self.blue_rect = max(self.blue_rects, key=(lambda x: x[2] * x[3]))
            #print(2, end=" ")
        elif not yellow_freq:
            self.yellow_rects = self.yellow.find_rect_find_target_color(hsv)
            if self.yellow_rects:
                self.yellow_rect = max(self.yellow_rects, key=(lambda x: x[2] * x[3]))
            #print(3, end=" ")
            
        dt3 = time.perf_counter() - pretime2
        pretime3 = time.perf_counter()
        
        if len(self.orange_rects) >0:
            cv2.rectangle(frame[:,xmin:xmax], tuple(self.orange_rects[0][0:2]), tuple(self.orange_rects[0][0:2] + self.orange_rects[0][2:4]), (0, 0, 255), thickness=2)
        if len(self.blue_rects) >0:
            cv2.rectangle(frame[:,xmin:xmax], tuple(self.blue_rects[0][0:2]), tuple(self.blue_rects[0][0:2] + self.blue_rects[0][2:4]), (255, 0, 0), thickness=2)
        if len(self.yellow_rects) >0:
            cv2.rectangle(frame[:,xmin:xmax], tuple(self.yellow_rects[0][0:2]), tuple(self.yellow_rects[0][0:2] + self.yellow_rects[0][2:4]), (0, 246, 255), thickness=2)
        
        cv2.imshow('result', frame[:,xmin:xmax])
        self.loopcount +=1
        if self.loopcount >= freq:
            self.loopcount = 0

        dt4 = time.perf_counter() - pretime3
        pretime4 = time.perf_counter()
                
        if len(self.orange_rect) > 0:
            self.orange_rect = [int(self.orange_rect[0] + self.orange_rect[2] / 2 - 360 + dposx),
                                int(self.orange_rect[1] + self.orange_rect[3] / 2 - 360 + dposy),
                                int(self.orange_rect[2]), int(self.orange_rect[3])]
        else:
            self.orange_rect = []

        if len(self.blue_rect) > 0:
            self.blue_rect = [int(self.blue_rect[0] + self.blue_rect[2] / 2 - 360 + dposx),
                              int(self.blue_rect[1] + self.blue_rect[3] / 2 - 360 + dposy),
                              int(self.blue_rect[2]), int(self.blue_rect[3])]
        else:
            self.blue_rect = []
        
        if len(self.yellow_rect) > 0:
            self.yellow_rect = [int(self.yellow_rect[0] + self.yellow_rect[2] / 2 - 360 + dposx),
                                int(self.yellow_rect[1] + self.yellow_rect[3] / 2 - 360 + dposy),
                                int(self.yellow_rect[2]), int(self.yellow_rect[3])]
        else:
            self.yellow_rect = []
            
        dt5 = time.perf_counter() - pretime4
        pretime5 = time.perf_counter()
        dt = time.perf_counter() - self.pretime
        self.pretime = time.perf_counter()
        
        #print(f" {dt:<6.4f} {1/dt:<5.2f} {dt1/dt:<5.1%} {dt2/dt:<5.1%} {dt3/dt:<5.1%} {dt4/dt:<5.1%} {dt5/dt:<5.1%}")

        return self.orange_rect, self.blue_rect, self.yellow_rect
        
    def __del__(self):
        self.capture.release()
        cv2.destroyAllWindows()


def main():
    camera = Camera_Module()
    pretime = 0
    try:
        while cv2.waitKey(1) < 0:
            dt = time.perf_counter() - pretime
            pretime = time.perf_counter()
            print(f"{dt:.3} {1/dt:.3}")
            orange_rect, blue_rect, yellow_rect = camera.read_pos_data([], [1], [3], 5)
            
            #print(orange_rect)
            
        
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
	main() 
