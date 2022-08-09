from pathlib import Path
import sys, os
sys.path.append(str(Path(__file__).parent) + "/Camera_file")
from Camera_file.Camera_Module3 import Camera_Module
from Robot_tool_file.BasicAction_Module import BasicAction_Module
import Angle_Sensor.MPU9250 as MP
from Angle_Sensor.Angle_module import Angle_Module
import cv2
import serial
import time
import numpy as np
from LiDAR_file.LiDAR_module3 import LiDAR
from multiprocessing import Process, Value, Array
import ctypes
import Jetson.GPIO as GPIO
import pygame
import math

def main():
    #インスタンス作成
    camera = Camera_Module()
    mpu9250 = MP.MPU9250(MP.MP9250_ADR)
    angle_module = Angle_Module(True)
    lidar = LiDAR("/dev/ttyUSB0")
    ba = BasicAction_Module()
    
    #角度センサ初期設定
    mpu9250.configMPU9250(gfsNo = MP.GFS_500, afsNo = MP.AFS_4G)
    mpu9250.configAK8963(mode = MP.AK8963_MODE_C100HZ, mfs = MP.AK8963_BIT_16)
    
    #ライダー用変数
    lidar_state = Value(ctypes.c_bool, True)
    lidar_angle = Value('d', 0.0)
    abspos = Array('i', [0,0])
    maps = cv2.imread("LiDAR_file/2021rule.jpg")
    
    #pygame初期化
    pygame.init()
    joy = pygame.joystick.Joystick(0)  # ジョイスティック番号はjstest-gtkで確認しておく
    joy.init()
    
    
    #変数
    angle = 0
    sol = 0
    boll_hold = False
    pretime = 0
    state_data=0
    
    #GPIO初期化
    #GPIO.setmode(GPIO.BOARD)#なし
    #GPIO.setup(7, GPIO.IN)#なし
    
    #LiDAR測定開始
    p = Process(target=lidar.lidar_process, args=(lidar_state, lidar_angle, abspos))
    p.start()
    time.sleep(0.1)
    
    with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as ser:
        #with serial.Serial('/dev/ttyUSB1', 115200, timeout=1) as ser_ui:
        ser.reset_input_buffer()
        #ser_ui.reset_input_buffer()
        pretime = time.perf_counter()
        try:
            while True:
                Key = cv2.waitKey(1)
                if Key == 27:
                    break
                
                #マップコピー
                map_fix = maps.copy()
                
                #teensyからデータ受信
                serial_rec_data=[0]*5
                while ser.in_waiting > 5:
                    if bytes('A', 'utf-8') == ser.read():
                        for i in range(5):
                            serial_rec_data[i] = int.from_bytes(ser.read(), byteorder='big')
                #print(serial_rec_data, end="  ")
                
                #arduinoからデータ受信
                """while ser_ui.in_waiting > 0:
                    state_data = int.from_bytes(ser_ui.read(), byteorder='big')
                #print(f"{state_data:08b}")"""
                
                #角度算出
                nowtime = time.perf_counter()
                dt = nowtime - pretime
                pretime = nowtime
                angle = angle_module.readAngle(mpu9250)
                lidar_angle.value=angle

                #ジョイスティク イベント更新
                a = np.int16(joy.get_axis(0) * 255) #左右移動
                b = np.int16(joy.get_axis(1) * 255) #前後移動
                c = np.int16(joy.get_axis(3) * 180) #旋回
                #c = np.int16(joy.get_axis(2) * 180) #旋回(ELECOM)
                btn0 = joy.get_button(0) 
                btn1 = joy.get_button(1) 
                btn2 = joy.get_button(2) 
                btn3 = joy.get_button(3) 
                btn4 = joy.get_button(4)
                btn5 = joy.get_button(5)
                pygame.event.pump()  

                                
                #画像読み込み(ボールメイン)
                boll, A_goal, E_goal = camera.read_pos_data([], [1], [3], 10, dposx = -33, dposy = 16)
                out = np.array([a,b], np.int16)

                #中立点への移動
                abs_pos_cp = np.array(abspos, np.int16)
                if btn0 == 1 :
                    out = np.array(ba.move_pos(abs_pos_cp, [ 61,  70], dt), np.int16)
                elif btn1 == 1 :
                    out = np.array(ba.move_pos(abs_pos_cp, [121,  70], dt), np.int16)
                elif btn2 == 1 :
                    out = np.array(ba.move_pos(abs_pos_cp, [ 91, 122], dt), np.int16)
                elif btn3 == 1 :
                    out = np.array(ba.move_pos(abs_pos_cp, [ 61, 173], dt), np.int16)
                elif btn4 == 1 :
                    out = np.array(ba.move_pos(abs_pos_cp, [121, 173], dt), np.int16)
                    
                #角度制御
                anglepw = ba.angle_ctr(c, angle, dt, Kp=15, Ki=0, Kd=1)
                #anglepw=0
                   
                #オーバーフロー カットオフ
                #maxp = math.sqrt(out[0]**2 + out[1]**2)
                if abs(out[0]) > 280 or abs(out[1]) > 280:
                    maxp = max(abs(out[0]), abs(out[1]))
                    outx =  out[0] / maxp * 280
                    outy = out[1] / maxp * 280
                    out = np.array([outx, outy], np.int16)
                    
                print(f"{angle:>7.2f} {abspos[0]:>3} {abspos[1]:>3} {dt:<6.4f} {1/dt:<5.2f} {out}")# {boll[0]:>4} {boll[1]:>4}
                
                #teensyへデータ送信
                ByteData =spritInt16Data(out[0])
                ByteData +=spritInt16Data(out[1])
                ByteData +=spritInt16Data(anglepw)
                #ByteData +=list(0,0)
                ByteData +=[0,0]
                ByteData +=[1+2*sol, 0]
                #ByteData +=[1+2*sol]
                #print(ByteData)
                ser.write(bytes('A', 'utf-8'))
                for item in ByteData:
                    ser.write(bytes([item]))
                
                
                #座標表示
                cv2.circle(map_fix, tuple(np.int16(abspos)), 5, (230, 230, 230), -1)
                cv2.imshow("map",map_fix)
                

                #teensyからのデータフロー確認
                if ser.in_waiting > 40:
                    ser.reset_input_buffer()
        
            #強制終了緩衝材      
        except (KeyboardInterrupt, SystemExit):
            pass
    
        #teensyへ停止データ送信
        ByteData = spritInt16Data(0)
        ByteData += spritInt16Data(0)              
        ByteData += spritInt16Data(0)
        ByteData += spritInt16Data(0)
        ByteData += spritInt16Data(0)
        ser.write(bytes('A', 'utf-8'))
        for item in ByteData:
            ser.write(bytes([item]))
        ser.write(bytes([0]))
        ser.write(bytes([0]))
        
        #終了処理
        #GPIO.cleanup()
        lidar_state.value=False
        p.join()
   

#int16から2byteに変換         
def spritInt16Data(data):
    dataC = [0xff & data, 0xff & (data >> 8)]
    return dataC        

if __name__ == "__main__":
    main()
