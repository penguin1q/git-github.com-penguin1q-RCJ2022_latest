import cv2
import numpy as np
from pandas import wide_to_long
from rplidar import RPLidar, RPLidarException
import rplidar
import threading


class LiDAR(threading.Thread):
    def __init__(self, *args):
        super(LiDAR, self).__init__()
        self.started = threading.Event()
        self.alive = True
        self.start()
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
            
            #cv2.imshow("gray", gray)
            #map_fix = maps.copy()
            '''grayl = gray.copy()
            linesl= self.LSD.detect(grayl)
            #linesl = linesl.tolist()
            mapl = maps.copy()
            try:
                for idx,l in enumerate(linesl):
                    x1,y1,x2,y2 = l[0][0],l[0][1],l[0][2],l[0][3]
                    # draw line
                    mapl = cv2.line(mapl, (x1,y1), (x2,y2), (0,0,255), 3)
                    print( f"{x1, y1, x2, y2}")

            except IndexError:
                pass'''
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
            try:
                #print(type(self.line), self.line.shape)
                for item in self.line:
                    #print(type(item), item.shape)
                    for rho, theta in item:
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
                        if np.cos(2*theta) > np.cos(np.pi / 4) :#or np.sin(2*theta) > np.sin(np.pi / 4):
                            #cv2.line(maps, (x1, y1), (x2, y2), (0,0,255), 2)
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
                            #cv2.line(maps, (x1, y1), (x2, y2), (0,255,0), 2)
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
                                #cv2.line(maps, (x1, y1), (x2, y2), (0,0,255), 2)
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
                                #cv2.line(maps, (x1, y1), (x2, y2), (0,255,0), 2)
                                uy = x1 * np.sin(np.pi / 2-theta) + y1 * np.cos(np.pi / 2-theta)
                                #cv2.line(map_fix, (-500, int(uy)) , (500, int(uy)), (0,255,0), 2)
                                #print(4,np.degrees(theta))
                                #cv2.line(maps, (x1, y1), (x2, y2), (255,0,0), 2)
                                y_angle +=np.cos(2*theta)
                                if -250 < uy - 250 < 0:
                                    y_line[0] += uy
                                    y_count[0] +=1
                                elif 250 < uy  < 500 :
                                    y_line[1] += uy
                                    y_count[1] +=1
                                y_count[2] +=1
                       

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
            #cv2.circle(maps, (int(-100 * np.cos(np.radians(180)) + 250), int(100*np.sin(np.radians(180)) + 250)), 10, (255, 0, 0), -1)

            #cv2.imshow('Mask Data', maps)
            #cv2.imshow('map_fix', map_fix)
            #cv2.imshow('lsd', mapl)
            
            
        except RPLidarException:
            self.lidar.connect()
            self.data = self.lidar.iter_scans()
            
    def run(self):
        self.started.wait()
        while self.alive:
            self.measure(self.nowangle)
            self.started.wait()
       
    def begin(self):
        self.started.set()

    def end(self):
        self.started.clear()

    def kill(self):
        self.started.set()
        self.alive = False
        self.join()

    def __del__(self):
        self.kill()
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
        
if __name__ == "__main__":
    pass
