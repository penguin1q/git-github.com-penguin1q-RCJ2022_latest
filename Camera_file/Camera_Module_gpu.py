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
		!videobalance   \
        ! videoconvert \
		! appsink max-buffers=1 drop=True'
		

class Camera_Module: 
    def __init__(self):
        self.config_ini = configparser.ConfigParser()
        self.config_ini_path = str(Path(__file__).resolve().parent.parent) + '/config.ini'
        self.capture = cv2.VideoCapture(GST_STR, cv2.CAP_GSTREAMER)
        if not os.path.exists(self.config_ini_path):
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), self.config_ini_path)
        self.config_ini.read(self.config_ini_path, encoding='utf-8')
        self.gpu_src = cv2.cuda_GpuMat()
        self.gpu_dst = cv2.cuda_GpuMat()
        orange_high = np.array(eval(self.config_ini.get('Camera', 'Orange_max')))
        blue_high= np.array(eval(self.config_ini.get('Camera', 'Blue_max')))
        yellow_high = np.array(eval(self.config_ini.get('Camera', 'Yellow_max')))
        orange_low = np.array(eval(self.config_ini.get('Camera', 'Orange_min')))
        blue_low = np.array(eval(self.config_ini.get('Camera', 'Blue_min')))
        yellow_low = np.array(eval(self.config_ini.get('Camera', 'Yellow_min')))
        # self.orange_low_h = cv2.cuda_GpuMat(orange_low[0])
        # self.orange_low_s = cv2.cuda_GpuMat(orange_low[1])
        # self.orange_low_v = cv2.cuda_GpuMat(orange_low[2])
        # self.yellow_low_h = cv2.cuda_GpuMat(yellow_low[0])
        # self.yellow_low_s = cv2.cuda_GpuMat(yellow_low[1])
        # self.yellow_low_v = cv2.cuda_GpuMat(yellow_low[2])
        # self.blue_low_h = cv2.cuda_GpuMat(blue_low[0])
        # self.blue_low_s = cv2.cuda_GpuMat(blue_low[1])
        # self.blue_low_v = cv2.cuda_GpuMat(blue_low[2])
        # self.orange_high_h = cv2.cuda_GpuMat(orange_high[0])
        # self.orange_high_s = cv2.cuda_GpuMat(orange_high[1])
        # self.orange_high_v = cv2.cuda_GpuMat(orange_high[2])
        # self.yellow_high_h = cv2.cuda_GpuMat(yellow_high[0])
        # self.yellow_high_s = cv2.cuda_GpuMat(yellow_high[1])
        # self.yellow_high_v = cv2.cuda_GpuMat(yellow_high[2])
        # self.blue_high_h = cv2.cuda_GpuMat(blue_high[0])
        # self.blue_high_s = cv2.cuda_GpuMat(blue_high[1])
        # self.blue_high_v = cv2.cuda_GpuMat(blue_high[2])
        self.mat_parts_h = cv2.cuda_GpuMat()
        self.mat_parts_s = cv2.cuda_GpuMat()
        self.mat_parts_v = cv2.cuda_GpuMat()
        self.mat_parts_h_low = cv2.cuda_GpuMat()
        self.mat_parts_h_high = cv2.cuda_GpuMat()
        self.mat_parts_s_low = cv2.cuda_GpuMat()
        self.mat_parts_s_high = cv2.cuda_GpuMat()
        self.mat_parts_v_low = cv2.cuda_GpuMat()
        self.mat_parts_v_high = cv2.cuda_GpuMat()
        self.tmp1 = cv2.cuda_GpuMat()
        self.result = cv2.cuda_GpuMat()
        
        self.orange = Find_rect(eval(self.config_ini.get('Camera', 'Orange_min')), eval(self.config_ini.get('Camera', 'Orange_max')))
        self.blue = Find_rect(eval(self.config_ini.get('Camera', 'Blue_min')), eval(self.config_ini.get('Camera', 'Blue_max')))
        self.yellow = Find_rect(eval(self.config_ini.get('Camera', 'Yellow_min')), eval(self.config_ini.get('Camera', 'Yellow_max')))
        # orange_high = np.array(eval(self.config_ini.get('Camera', 'Orange_max')))
        # self.orange_high_h.upload(orange_high[0])
        # self.orange_high_s.upload(orange_high[1])
        # self.orange_high_v.upload(orange_high[2])
        # blue_high= np.array(eval(self.config_ini.get('Camera', 'Blue_max')))
        # self.blue_high_h.upload(blue_high[0])
        # self.blue_high_s.upload(blue_high[1])
        # self.blue_high_v.upload(blue_high[2])
        # yellow_high = np.array(eval(self.config_ini.get('Camera', 'Yellow_max')))
        # self.yellow_high_h.upload(yellow_high[0])
        # self.yellow_high_s.upload(yellow_high[1])
        # self.yellow_high_v.upload(yellow_high[2])
        # orange_low = np.array(eval(self.config_ini.get('Camera', 'Orange_min')))
        # self.orange_low_h.upload(orange_low[0])
        # self.orange_low_s.upload(orange_low[1])
        # self.orange_low_v.upload(orange_low[2])
        # blue_low = np.array(eval(self.config_ini.get('Camera', 'Blue_min')))
        # self.blue_low_h.upload(blue_low[0])
        # self.blue_low_s.upload(blue_low[1])
        # self.blue_low_v.upload(blue_low[2])
        # yellow_low = np.array(eval(self.config_ini.get('Camera', 'Yellow_min')))
        # self.yellow_low_h.upload(yellow_low[0])
        # self.yellow_low_s.upload(yellow_low[1])
        # self.yellow_low_v.upload(yellow_low[2])
        
        self.orange_low_v = np.array(eval(self.config_ini.get('Camera', 'Orange_max')))
        self.yellow_low_v = cv2.cuda_GpuMat(np.array(eval(self.config_ini.get('Camera', 'Blue_max'))))
        self.blue_low_v = cv2.cuda_GpuMat(np.array(eval(self.config_ini.get('Camera', 'Yellow_max'))))
        self.orange_high_v = cv2.cuda_GpuMat(np.array(eval(self.config_ini.get('Camera', 'Orange_min'))))
        self.yellow_high_v = cv2.cuda_GpuMat(np.array(eval(self.config_ini.get('Camera', 'Blue_min'))))
        self.blue_high_v = cv2.cuda_GpuMat(np.array(eval(self.config_ini.get('Camera', 'Yellow_min'))))
        self.blue_rects = []
        self.yellow_rects = []
        self.orange_rects = []
        self.loopcount = 0
        self.color = {'orange':0, 'blue':1, 'yellow':2}
        
    

    def read_pos_data(self, orange_freq, blue_freq, yellow_freq, freq, raw=False, dposx = 0, dposy = 0):
        _, frame = self.capture.read()
        #hsv = cv2.cvtColor(frame[:,xmin:xmax], cv2.COLOR_BGR2HSV_FULL)
        self.gpu_src.upload(frame[:,xmin:xmax])
        self.gpu_dst = cv2.cuda.cvtColor(self.gpu_src, cv2.COLOR_BGR2HSV_FULL)
        self.mat_parts_h, self.mat_parts_s, self.mat_parts_v = cv2.cuda.split(self.gpu_dst)
        _, self.mat_parts_h_low = cv2.cuda.threshold(self.mat_parts_h, 125, 255, cv2.THRESH_BINARY)
        _, self.mat_parts_h_high = cv2.cuda.threshold(self.mat_parts_h,155, 255, cv2.THRESH_BINARY_INV)
        self.matparts_h = cv2.cuda.bitwise_and(self.mat_parts_h_low, self.mat_parts_h_high)
        
        _, self.mat_parts_s_low = cv2.cuda.threshold(self.mat_parts_s, 78, 255, cv2.THRESH_BINARY)
        _, self.mat_parts_s_high = cv2.cuda.threshold(self.mat_parts_s, 141, 255, cv2.THRESH_BINARY_INV)
        self.matparts_s = cv2.cuda.bitwise_and(self.mat_parts_s_low, self.mat_parts_s_high)

        _, self.mat_parts_v_low = cv2.cuda.threshold(self.mat_parts_v, 128, 255, cv2.THRESH_BINARY)
        _, self.mat_parts_v_high = cv2.cuda.threshold(self.mat_parts_v, 204, 255, cv2.THRESH_BINARY_INV)
        self.matparts_v = cv2.cuda.bitwise_and(self.mat_parts_v_low, self.mat_parts_v_high)
        
        self.tmp1 = cv2.cuda.bitwise_and(self.matparts_h, self.matparts_s)
        self.result = cv2.cuda.bitwise_and(self.tmp1, self.mat_parts_v)
        data = self.result.download()

        contours,_ =cv2.findContours(data, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rects = []
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(np.array(rect))
        self.orange_rects = rects
        # if self.loopcount in orange_freq:
        #     self.orange_rects = self.orange.find_rect_find_target_color(hsv)
        # elif self.loopcount in blue_freq:
        #     self.blue_rects = self.blue.find_rect_find_target_color(hsv)
        # elif self.loopcount in yellow_freq:
        #     self.yellow_rects = self.yellow.find_rect_find_target_color(hsv)
        # elif not orange_freq:
        #     self.orange_rects = self.orange.find_rect_find_target_color(hsv)
        # elif not blue_freq:
        #     self.blue_rects = self.blue.find_rect_find_target_color(hsv)
        # elif not yellow_freq:
        #     self.yellow_rects = self.yellow.find_rect_find_target_color(hsv)

        self.orange_rects.sort(key=(lambda x: x[2] * x[3]), reverse=True)
        # self.blue_rects.sort(key=(lambda x: x[2] * x[3]), reverse=True)
        # self.yellow_rects.sort(key=(lambda x: x[2] * x[3]), reverse=True)

        #self.orange_rects = sorted(self.orange_rects[:5], key=(lambda x: (x[0]-360 + x[2]/2)**2 + (x[1]-360 + x[3]/2)**2))
        #self.blue_rects = sorted(self.blue_rects[:5], key=(lambda x: (x[0]-360 + x[2]/2)**2 + (x[1]-360 + x[3]/2)**2))
        #self.yellow_rects = sorted(self.blue_rects[:5], key=(lambda x: (x[0]-360 + x[2]/2)**2 + (x[1]-360 + x[3]/2)**2))
        
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

        if raw:
            return self.orange_rects[:5], self.blue_rects[:5], self.yellow_rects[:5]
        
        if self.orange_rects:
            self.orange_rect = [int(self.orange_rects[0][0] + self.orange_rects[0][2] / 2 - 360 + dposx),
                                int(self.orange_rects[0][1] + self.orange_rects[0][3] / 2 - 360 + dposy),
                                int(self.orange_rects[0][2]), int(self.orange_rects[0][3])]
        else:
            self.orange_rect = []

        if self.blue_rects:
            blue_rects = np.array(self.blue_rects[:5])
            self.blue_rect = np.array(
                                [blue_rects[:,0] + blue_rects[:,2] / 2 - 360 + dposx,
                                 blue_rects[:,1] + blue_rects[:,3] / 2 - 360 + dposy,
                                 blue_rects[:,2], blue_rects[:,3]], np.int16)
        else:
            self.blue_rect = []

        if self.yellow_rects:
            yellow_rects = np.array(self.yellow_rects[:5])
            self.yellow_rect = np.array(
                                [yellow_rects[:,0] + yellow_rects[:,2] / 2 - 360 + dposx,
                                 yellow_rects[:,1] + yellow_rects[:,3] / 2 - 360 + dposy,
                                 yellow_rects[:,2], yellow_rects[:,3]], np.int16)
        else:
            self.yellow_rect = []

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
            
        
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
	main() 
