from re import L
import cv2
import numpy as np
from pandas import wide_to_long
from rplidar import RPLidar, RPLidarException
import rplidar
from multiprocessing import Process, Value, Array
import ctypes

class LiDAR():
    def __init__(self, *args):
        self.lidar = RPLidar('/dev/ttyUSB0')
        info = self.lidar.get_info()
        print(info)
        health = self.lidar.get_health()
        print(health)
        self.lidar.motor_speed = rplidar.MAX_MOTOR_PWM
        #self.lidar.motor_speed= 100
        self.data = self.lidar.iter_scans(max_buf_meas=False)
        self.line = []
        self.nowangle = 0
        self.length_wall=[0]*4
        self.abs_pos = [0,0]
        #print(args, np.array(args).shape)
        self.offset_pos = np.array(args)
        self.fix_pos = np.zeros(np.array(args).shape)

    def measure(self):
        try:
            nowdata = np.array(next(self.data))
            self.lidar.clean_input()
            #_ = next(self.data)
            for item in nowdata:
                if 268 < item[1] < 272:
                    return item[2]/10
            return 0.0
            #print(nowdata)

        except RPLidarException:
            self.lidar.connect()
            self.data = self.lidar.iter_scans()

    def lidar_process(self,state, length):
        while state.value:
            length.value = self.measure()

    def __del__(self):
        self.lidar.stop_motor()
        self.lidar.disconnect()

if __name__ == "__main__":
    lidar = LiDAR()
    length = Value('d', 0.0)
    state = Value(ctypes.c_bool, True)
    p = Process(target=lidar.lidar_process, args=(state, length))
    p.start()
    try:
        while True:
            print(length.value)
            #print(lidar.measure())
    except KeyboardInterrupt:
        state.value=False
        p.join()