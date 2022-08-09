from pathlib import Path
import sys, os
#sys.path.append(str(Path(__file__).parent.parent) + "/Angle_Sensor")
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from Angle_Sensor import MPU9250 as MP
import cv2
import numpy as np
import time
from pandas import wide_to_long
from rplidar import RPLidar, RPLidarException
import rplidar
from multiprocessing import Process, Value, Array
import ctypes

class LiDAR():
    def __init__(self, *args):
        #  lidar_init
        self.lidar = RPLidar('/dev/ttyUSB0')
        info = self.lidar.get_info()
        print(info)
        health = self.lidar.get_health()
        print(health)
        self.lidar.motor_speed = rplidar.MAX_MOTOR_PWM
        #self.lidar.motor_speed= 100
        #  menber valiables
        self.data = self.lidar.iter_scans(max_buf_meas=False)
        self.line = []
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
            self.lidar.clean_input()
            _ = next(self.data)
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
            #lines = cv2.HoughLinesP(gray, rho=1, theta=np.pi/360, threshold=5, minLineLength=5, maxLineGap=30)
            lines = cv2.HoughLines(gray, rho=0.5, theta=np.pi/90, threshold=25)
            cv2.waitKey(1)
            cv2.imshow("gray", gray)
            #map_fix = maps.copy()
            
            x_angle = 0
            x_count = [0, 0, 0]
            y_angle = 0
            y_count = [0, 0, 0]
            x_line = [0,0]
            y_line = [0,0]
            try:
                if type(lines) == np.ndarray:
                    self.line = lines
            except TypeError:
                pass  
            #self.abs_pos = [self.abs_pos[0]+1, self.abs_pos[1]+1]
            try:
                #print(type(self.line), self.line.shape)
                
                for rho, theta in self.line[0]:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 500*(-b))
                    y1 = int(y0 + 500*(a))
                    x2 = int(x0 - 500*(-b))
                    y2 = int(y0 - 500*(a))
                    """if x1 == -1000:
                        print(f"Yline : {y1} to {y2}")
                    elif y1 == 1000:
                        print(f"Xline : {x1} to {x2}")
                    """
                    #print(f"{rho:7} {np.degrees(theta):6.4}  {x1, y1, x2, y2}")
                    if theta < np.pi / 4 or 3 * np.pi / 4 <= theta:
                        cv2.line(maps, (x1, y1), (x2, y2), (0,0,255), 2)
                    elif np.pi / 4 <= theta < 3 * np.pi / 4:
                         cv2.line(maps, (x1, y1), (x2, y2), (255,0,0), 2)
                    """if np.cos(2*theta) > np.cos(np.pi / 4) :#or np.sin(2*theta) > np.sin(np.pi / 4):
                        cv2.line(maps, (x1, y1), (x2, y2), (0,0,255), 2)
                        #print(f"{1} {rho:7} {np.degrees(theta):6.4}  {x1, y1, x2, y2}")
                        #print(1,np.degrees(theta))
                        #u1 = (x1, y1)
                        #u2 = (x2, y2)
                        #u1 = self.rotation_o(u1, -theta)
                        #u2 = self.rotation_o(u2, -theta)
                        #print(f"{1} {u1} {u2} {x1, y1} {x2, y2}")
                        #xline = np.append(xline, np.array([[[x1, y1], [x2, y2]]]), axis=0)
                        #xtheta = np.append(xtheta, theta)
                        #cv2.line(map_fix, tuple(np.int16(u1)) , tuple(np.int16(u2)), (0,0,255), 2)
                        ux = x1 * np.cos(-theta) - y1 * np.sin(-theta)
                        #cv2.line(map_fix, (int(ux), -500) , (int(ux),500), (0,0,255), 2)
                        x_angle +=np.sin(2*theta)
                        if  -250 < ux - 250 < 0:
                            #print(ux)
                            x_line[0] += ux
                            x_count[0] +=1
                        elif 250 < ux  < 500 :
                            x_line[1] += ux
                            x_count[1] +=1 
                        x_count[2] +=1
                    elif np.cos(2*theta) < np.cos(3 * np.pi / 4):
                        cv2.line(maps, (x1, y1), (x2, y2), (0,255,0), 2)
                        uy = x1 * np.sin(np.pi / 2-theta) + y1 * np.cos(np.pi / 2 -theta)
                        #cv2.line(map_fix, (-500, int(uy)) , (500, int(uy)), (0,255,0), 2)
                        #print(2,np.degrees(theta))
                        #print(f"{y1} {uy}")
                        y_angle +=np.cos(2*theta)
                        if -250 < uy - 250 < 0:
                            y_line[0] += uy
                            y_count[0] +=1
                        elif 250 < uy < 500 :
                            y_line[1] += uy
                            y_count[1] +=1 
                        y_count[2] +=1
                    else:
                        if np.sin(2*theta) > np.sin(np.pi/4):
                            cv2.line(maps, (x1, y1), (x2, y2), (0,0,255), 2)
                            #print(f"{2} {rho:7} {np.degrees(theta):6.4}  {x1, y1, x2, y2}")
                            #u1 = (x1, y1)
                            #u2 = (x2, y2)
                            #u1 = self.rotation_o(u1, -theta)
                            #u2 = self.rotation_o(u2, -theta)
                            ux = x1 * np.cos(-theta) - y1 * np.sin(-theta)
                            #print(f"{2} {u1} {u2} {x1, y1} {x2, y2}")
                            #xline = np.append(xline, np.array([[[x1, y1], [x2, y2]]], axis=0))
                            #xtheta = np.append(xtheta, theta)
                            #cv2.line(map_fix, (int(ux), -500) , (int(ux),500), (0,0,255), 2)
                            #print(3,np.degrees(theta))
                            if -250 < ux - 250 < 0:
                                #print(ux)
                                x_line[0] += ux
                                x_count[0] +=1
                            elif 250 < ux < 500 :
                                x_line[1] += ux 
                                x_count[1] +=1
                            x_angle += np.sin(2*theta)
                            x_count[2] +=1
                        elif np.sin(2*theta) < np.sin(-np.pi / 4):
                            cv2.line(maps, (x1, y1), (x2, y2), (0,255,0), 2)
                            uy = x1 * np.sin(np.pi / 2-theta) + y1 * np.cos(np.pi / 2-theta)
                            #cv2.line(map_fix, (-500, int(uy)) , (500, int(uy)), (0,255,0), 2)
                            #print(4,np.degrees(theta))
                            cv2.line(maps, (x1, y1), (x2, y2), (255,0,0), 2)
                            y_angle +=np.cos(2*theta)
                            if -250 < uy - 250 < 0:
                                y_line[0] += uy
                                y_count[0] +=1
                            elif 250 < uy  < 500 :
                                y_line[1] += uy
                                y_count[1] +=1
                            y_count[2] +=1
                        """

                #for linep in line:
                    #x1, y1, x2, y2 = linep[0]
                    
                    #cv2.line(maps, (x1, y1), (x2, y2), (0,0,255), 1)
            except IndexError:
                pass
            #print(x_line[0], x_count[0])
            
            try:
                x_angle = np.arcsin(x_angle / x_count[2]) / 2
            except:
                x_angle=0
            try:
                y_angle = np.arccos(y_angle / y_count[2]) / 2
            except:
                y_angle=0
            for i in range(2):
                try:
                    x_line[i] = int(x_line[i]/x_count[i])
                    y_line[i] = int(y_line[i]/y_count[i])
                except:
                    x_line[i] =0
                    y_line[i] =0
                              
            
            #cv2.line(map_fix, (-500, y_line[0]) , (500, y_line[0]), (255,0,0), 2)
            #cv2.line(map_fix, (-500, y_line[1]) , (500, y_line[1]), (255,0,0), 2)
            #cv2.line(map_fix, (x_line[0], -500) , (x_line[0], 500), (255,0,0), 2)
            #cv2.line(map_fix, (x_line[1] ,-500) , (x_line[1], 500), (255,0,0), 2)
            #xline = self.rotation_c(xline, xtheta)
            #if x_angle < 0:
            #    x_angle += np.pi
            #print(f"{np.degrees(x_angle):6.4} {np.degrees(y_angle):6.4} data_end")
            #cv2.circle(maps, (250, 250), 5, (230, 230, 230), -1)
            cx = 250 * np.cos(-x_angle) - 250 * np.sin(-x_angle)
            cy = 250 * np.sin((np.pi/2-y_angle)) + 250 * np.cos((np.pi/2-y_angle))
            #print(f"{int(cx) - x_line[0]} {x_line[1] - int(cx)} { int(cy)-y_line[0]} {y_line[1] - int(cy)}")
            self.length_wall = [int(cx)-x_line[0], x_line[1]-int(cx), int(cy)-y_line[0], y_line[1] - int(cy)]
            if x_line[0] == 0 and x_line[1] == 0:
                pass
            elif x_line[0] == 0:
                self.abs_pos[0] = 182 - self.length_wall[1]
            elif x_line[1] == 0:
                self.abs_pos[0] = self.length_wall[0]
            else: 
                self.abs_pos[0] = (self.length_wall[0] + 182 - self.length_wall[1]) / 2

            if y_line[0] == 0 and y_line[1] == 0:
                pass
            elif y_line[0] == 0:
                self.abs_pos[1] = 243 - self.length_wall[3]
            elif y_line[1] == 0:
                self.abs_pos[1] = self.length_wall[2]
            else: 
                self.abs_pos[1] = (self.length_wall[2] + 243 - self.length_wall[3]) / 2

            #print(f"{x_count[2]+y_count[2]} {self.abs_pos} {self.length_wall} {250-x_line[0], x_line[1]-250, 250-y_line[0],y_line[1]-250} ")
            #print(cx, cy)
            #cv2.circle(map_fix, (int(cx), int(cy)), 5, (230, 230, 230), -1)
            cv2.circle(maps, (int(-100 * np.cos(np.radians(180)) + 250), int(100*np.sin(np.radians(180)) + 250)), 10, (255, 0, 0), -1)

            cv2.imshow('Mask Data', maps)
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

def readAngle(mpu9250, MAG_ANGLE, MAG_INI_ANGLE, preangle,RAW_MAG_ANGLE):
    #読み取り
    accXYZ, gyrXYZ = mpu9250.readAccGyro()
    magXYZ = mpu9250.readMag()
    temp = mpu9250.readTemp()
    
    #AK8963がデータを更新したら、角度を計算し、更新
    #print(accXYZ)
    
    if magXYZ[0] != 0 and magXYZ[1] != 0:
        RAW_MAG_ANGLE = mpu9250.culcFixMag(magXYZ)
        MAG_ANGLE = RAW_MAG_ANGLE - MAG_INI_ANGLE
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
    if ( abs(abs(GYRO_MAG_ANGLE) - abs(MAG_ANGLE)) > 30 ) or not (-177 < MAG_ANGLE < 177):
        GYRO_MAG_ANGLE = MAG_ANGLE
    #print(f'{GYRO_MAG_ANGLE:.3} : {GYRO_ANGLE:.3} : {MAG_ANGLE:.3} : {delta_angle:.3}')
    #print(GYRO_MAG_ANGLE, end=" ")
    return GYRO_MAG_ANGLE, MAG_ANGLE, RAW_MAG_ANGLE 

if __name__ == "__main__":
    
    angle = 0
    MAG_ANGLE = 0
    MAG_INI_ANGLE = 0
    ini_count = 0
    RAW_MAG_ANGLE = 0.0
    mpu9250 = MP.MPU9250(MP.MP9250_ADR)
    state = Value(ctypes.c_bool, True)
    lidar_angle = Value('d', 0.0)
    abspos = Array('i', [0,0])
    mpu9250.configMPU9250(gfsNo = MP.GFS_500, afsNo = MP.AFS_4G)
    # Full Scale : Gyro=500deg/sec, Acc=4g
    mpu9250.configAK8963(mode = MP.AK8963_MODE_C100HZ, mfs = MP.AK8963_BIT_16)
    pretime = time.perf_counter()
    """for i in range(20):
        magXYZ = mpu9250.readMag()
        if magXYZ[0] != 0 and magXYZ[1] != 0:
            _= mpu9250.culcFixMag(magXYZ)
        time.sleep(0.01)"""
    while ini_count < 50:
        magXYZ = mpu9250.readMag()
        if magXYZ[0] != 0 and magXYZ[1] != 0:
            MAG_INI_ANGLE += mpu9250.culcFixMag(magXYZ)
            ini_count +=1
        time.sleep(0.01)
    MAG_INI_ANGLE /= 50
    lidar = LiDAR()
    
    p = Process(target=lidar.lidar_process, args=(state, lidar_angle, abspos))
    p.start()
    try:
        while True:
            dt = time.perf_counter() - pretime
            pretime = time.perf_counter()
            angle, MAG_ANGLE, RAW_MAG_ANGLE = readAngle(mpu9250, MAG_ANGLE, MAG_INI_ANGLE, angle, RAW_MAG_ANGLE)
            lidar_angle.value = angle
            print(f"{abspos[:]}, {angle:10.3f}, {RAW_MAG_ANGLE:10.3f}, {MAG_ANGLE:10.3f}, {MAG_INI_ANGLE:.3}")
            abs_pos = abspos
            time.sleep(0.015)
            #[x座標, y座標]の配列
    except KeyboardInterrupt:
        state.value=False
        p.join()
