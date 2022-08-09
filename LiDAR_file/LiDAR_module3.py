from pathlib import Path
import sys, os
#sys.path.append(str(Path(__file__).parent.parent) + "/Angle_Sensor")
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from Angle_Sensor import MPU9250 as MP
import cv2
import numpy as np
import math
import time
from pandas import wide_to_long
from rplidar import RPLidar, RPLidarException
import rplidar
from multiprocessing import Process, Value, Array
import ctypes

class LiDAR():
    def __init__(self, port='/dev/ttyUSB0', *args):
        #  lidar_init
        self.lidar = RPLidar(port)
        info = self.lidar.get_info()
        print(info)
        health = self.lidar.get_health()
        print(health)
        self.lidar.motor_speed = rplidar.MAX_MOTOR_PWM
        #self.lidar.motor_speed= 100
        #  menber valiables
        self.data = self.lidar.iter_scans(max_buf_meas=False)
        self.line = []
        self.pretime = 0
        self.nowangle = 0
        self.length_wall=[0]*4
        self.abs_pos = [0,0]
        #print(args, np.array(args).shape)
        self.offset_pos = np.array(args)
        self.fix_pos = np.zeros(np.array(args).shape)   

    def hough_lines(self, nowdata, rho, theta):
        Rx = np.int16(-1 * nowdata[:,2] * np.cos(np.radians(-1 * nowdata[:,1])))
        Ry = np.int16(nowdata[:,2] * np.sin(np.radians(nowdata[:,1])))
        hough = []
        
        rho_max = np.ceil(4000 / rho)
        rng = np.round(np.pi / theta)
        
        for i in np.arange(-rng, rng):
            rho2 = np.round((Rx * np.cos(i * theta) + Ry * np.sin(i * theta)) / rho)
            hist, _ = np.histogram(rho2, bins=np.arange(0, rho_max))
            hough.append(hist)
        return np.array(hough)
   
    def measure(self, nowangle):
        try:
            maps = np.zeros((500,500,3), np.uint8)
            gray = cv2.cvtColor(maps, cv2.COLOR_BGR2GRAY)
            #hough = hough_lines(np.array(next(data)), 0.5, np.pi/180)
            nowdata = np.array(next(self.data))
            #self.lidar.clean_input()
            #_ = next(self.data)
            dt1 = time.perf_counter() - self.pretime
            pretime1 = time.perf_counter()
            #nowdata = np.array(nowdata)
            Rx = np.int16(nowdata[:,2] * np.cos(np.radians(-(nowdata[:,1] + 180- nowangle))) / 10)
            Ry = np.int16(-1 * nowdata[:,2] * np.sin(np.radians(-(nowdata[:,1] + 180-nowangle))) / 10)
            #hsv[250, 250, 2] = 255 
            for Rx, Ry in zip(Rx, Ry):
                try:
                    gray[Ry+250,Rx+250] = 255
                    maps[Ry+250,Rx+250] = 255
                except IndexError:
                    pass
            gray = cv2.Canny(gray, 0, 0, apertureSize = 7)
            #gray_tmp = cv2.Laplacian(gray,cv2.CV_8U)
            #gray= cv2.convertScaleAbs(gray_tmp)
            lines = cv2.HoughLinesP(gray, rho=0.5, theta=np.pi/90, threshold=30, minLineLength=50, maxLineGap=500)
            #lines = cv2.HoughLines(gray, rho=0.5, theta=np.pi/90, threshold=25)
            cv2.waitKey(1)
            cv2.imshow("gray", gray)
            #map_fix = maps.copy()
            dt2 = time.perf_counter() - pretime1
            pretime2 = time.perf_counter()
            try:
                if type(lines) == np.ndarray:
                    self.line = lines
            except TypeError:
                pass  
            #self.abs_pos = [self.abs_pos[0]+1, self.abs_pos[1]+1]
            f_dists = []
            b_dists = []
            l_dists = []
            r_dists = []
            try:
                for item in self.line:
                    for x1, y1, x2, y2 in item:
                        a = y2-y1
                        b = x1-x2
                        c = x2*y1-x1*y2
                        e = a**2+b**2
                        k = -(a*250+b*250+c)/e
                        #d = (a*250+b*250+c)/math.sqrt(a**2+b**2)
                        d = abs(k)*math.sqrt(e)
                        if abs(y2 - y1) > abs(x2 - x1): 
                            if a * k < 0:
                                cv2.line(maps, (x1, y1), (x2, y2), (0,0,255), 2)
                                l_dists.append(d)
                                #print(f"Vert:{-d}")
                            else:
                                cv2.line(maps, (x1, y1), (x2, y2), (0, 100,255), 2)
                                r_dists.append(d)
                                #print(f"Vert:{d}")
                        if abs(y2 - y1) < abs(x2 - x1):
                            if b * k > 0:
                                cv2.line(maps, (x1, y1), (x2, y2), (255,0,0), 2)
                                b_dists.append(d)
                                #print(f"Hor:{-d}")
                            else:
                                cv2.line(maps, (x1, y1), (x2, y2), (255,255,0), 2)
                                f_dists.append(d)
                                #print(f"Hor:{d}")
                #print(type(self.line), self.line.shape)
                
                #for linep in line:
                    #x1, y1, x2, y2 = linep[0]
                    
                    #cv2.line(maps, (x1, y1), (x2, y2), (0,0,255), 1)
            except IndexError:
                pass
            #print(x_line[0], x_count[0])
            dt3 = time.perf_counter() - pretime2
            pretime3 = time.perf_counter()
            l_dist = 0
            r_dist = 0
            b_dist = 0
            f_dist = 0
            if l_dists:
                l_dist = sum(l_dists)/len(l_dists)
            if r_dists:
                r_dist = sum(r_dists)/len(r_dists)
            if b_dists:
                b_dist = sum(b_dists)/len(b_dists)
            if f_dists:
                f_dist = sum(f_dists)/len(f_dists)
            
            if l_dist == 0 and r_dist == 0:
                pass
            elif l_dist == 0:
                self.abs_pos[0] = 182 - r_dist
            elif r_dist == 0:
                self.abs_pos[0] = l_dist
            else: 
                self.abs_pos[0] = (l_dist + 182 - r_dist) / 2
                
            if self.abs_pos[0]<0 and 182<self.abs_pos[0]:
                self.abs_pos[0] = 0

            if b_dist == 0 and f_dist == 0:
                pass
            elif f_dist == 0:
                self.abs_pos[1] = 243 - b_dist
            elif b_dist == 0:
                self.abs_pos[1] = f_dist
            else: 
                self.abs_pos[1] = (f_dist + 243 - b_dist) / 2
                
            if self.abs_pos[1]<0 and 243<self.abs_pos[1]:
                self.abs_pos[1] = 0

            #print(f"{x_count[2]+y_count[2]} {self.abs_pos} {self.length_wall} {250-x_line[0], x_line[1]-250, 250-y_line[0],y_line[1]-250} ")
            #print(cx, cy)
            #cv2.circle(map_fix, (int(cx), int(cy)), 5, (230, 230, 230), -1)
            #cv2.circle(maps, (int(-100 * np.cos(np.radians(180)) + 250), int(100*np.sin(np.radians(180)) + 250)), 10, (255, 0, 0), -1)
            cv2.circle(maps, (int(250), int(250)), 10, (255, 0, 0), -1)

            cv2.imshow('Mask Data', maps)
            dt4 = time.perf_counter() - pretime3
            pretime4 = time.perf_counter()
            
            dt = time.perf_counter() - self.pretime
            self.pretime = time.perf_counter()
            #print(f"{dt:<6.4f} {1/dt:<5.2f} {dt1/dt:<5.1%} {dt2/dt:<5.1%} {dt3/dt:<5.1%} {dt4/dt:<5.1%}")
            #cv2.imshow('map_fix', map_fix)
            #cv2.imshow('lsd', mapl)
            
            
        except RPLidarException:
            self.lidar.connect()
            self.data = self.lidar.iter_scans()

    def lidar_process(self,state, angle, abspos):
        #print(1)
        while state.value:
            self.measure(angle.value)
            abspos[0] = int(self.abs_pos[0])
            abspos[1] = int(self.abs_pos[1])
            #print(self.abs_pos)

    def __del__(self):
        self.lidar.stop_motor()
        self.lidar.disconnect()

def readAngle(mpu9250, MAG_ANGLE, MAG_INI_ANGLE, preangle,RAW_MAG_ANGLE, angle_limited):
    #読み取り
    accXYZ, gyrXYZ = mpu9250.readAccGyro()
    magXYZ = mpu9250.readMag()
    temp = mpu9250.readTemp()
    
    #AK8963がデータを更新したら、角度を計算し、更新
    #print(accXYZ)
    
    if magXYZ[0] != 0 and magXYZ[1] != 0:
        RAW_MAG_ANGLE = mpu9250.culcFixMag(magXYZ)
        MAG_ANGLE = RAW_MAG_ANGLE - MAG_INI_ANGLE + angle_limited
        if MAG_ANGLE < -180.0:
            MAG_ANGLE += 360.0
        if MAG_ANGLE > 180.0:
            MAG_ANGLE -= 360.0
        #print(MAG_ANGLE)
        #pass
    
    #GYRO_ANGLE = mpu9250.culcGyroZ(gyrXYZ[2])
    delta_angle = mpu9250.fixGyroZ(gyrXYZ[2])
    GYRO_ANGLE = preangle + delta_angle
    #print(GYRO_ANGLE)
    
    GYRO_MAG_ANGLE = (GYRO_ANGLE * 0.97) + (MAG_ANGLE * 0.03)
    #if not (-175 < GYRO_MAG_ANGLE < 175) or not (-175 < MAG_ANGLE < 175):
    if ( abs(GYRO_MAG_ANGLE - MAG_ANGLE) > 30 ) or not (-177 < MAG_ANGLE < 177):
        GYRO_MAG_ANGLE = MAG_ANGLE
    #print(f'{GYRO_MAG_ANGLE:.3} : {GYRO_ANGLE:.3} : {MAG_ANGLE:.3} : {delta_angle:.3}')
    #print(GYRO_MAG_ANGLE, end=" ")
    return GYRO_MAG_ANGLE, MAG_ANGLE, RAW_MAG_ANGLE 

if __name__ == "__main__":
    angle_limited = 0
    angle = 0
    MAG_ANGLE = 0
    MAG_INI_ANGLE = 0
    ini_count = 0
    offset_ini_count = 0
    RAW_MAG_ANGLE = 0.0
    mpu9250 = MP.MPU9250(MP.MP9250_ADR)
    state = Value(ctypes.c_bool, True)
    lidar_angle = Value('d', 0.0)
    abspos = Array('i', [0,0])
    mpu9250.configMPU9250(gfsNo = MP.GFS_500, afsNo = MP.AFS_4G)
    # Full Scale : Gyro=500deg/sec, Acc=4g
    mpu9250.configAK8963(mode = MP.AK8963_MODE_C100HZ, mfs = MP.AK8963_BIT_16)
    
    lidar = LiDAR()
    pretime = time.perf_counter()
    p = Process(target=lidar.lidar_process, args=(state, lidar_angle, abspos))
    p.start()
    time.sleep(0.1)
    #time.sleep(0.1)
    limited_count=0
    while offset_ini_count < 20:
        magXYZ = mpu9250.readMag()
        if magXYZ[0] != 0 and magXYZ[1] != 0:
            offset_angle = mpu9250.culcFixMag(magXYZ)
            if offset_angle < -170 or offset_angle > 170:
                limited_count+=1
            offset_ini_count+=1
        time.sleep(0.015)
    if limited_count>3:
        angle_limited = 90
    while ini_count < 50:
        magXYZ = mpu9250.readMag()
        if magXYZ[0] != 0 and magXYZ[1] != 0:
            mag_offset_angle = mpu9250.culcFixMag(magXYZ) + angle_limited
            if mag_offset_angle < -180.0:
                mag_offset_angle += 360.0
            if mag_offset_angle > 180.0:
                mag_offset_angle -= 360.0
            MAG_INI_ANGLE += mag_offset_angle
            ini_count +=1
        time.sleep(0.015)
    MAG_INI_ANGLE /= 50
    
    
    try:
        while True:
            dt = time.perf_counter() - pretime
            pretime = time.perf_counter()
            angle, MAG_ANGLE, RAW_MAG_ANGLE = readAngle(mpu9250, MAG_ANGLE, MAG_INI_ANGLE, angle, RAW_MAG_ANGLE, angle_limited)
            lidar_angle.value = angle
            #lidar_angle.value = 0
            #print(f"{abspos[:]}, {angle:<10.3f}, {RAW_MAG_ANGLE:<10.3f}, {MAG_ANGLE:<10.3f}, {MAG_INI_ANGLE:.3}, {angle_limited}")
            abs_pos = abspos
            time.sleep(0.015)
            print(f"{dt:<6.4f} {1/dt:<5.2f}")
            #[x座標, y座標]の配列
    except KeyboardInterrupt:
        state.value=False
        p.join()
