from pathlib import Path
import sys, os
sys.path.append(str(Path(__file__).parent) + "/Camera_file")
sys.path.append(str(Path(__file__).parent) + "/Robot_tool_file")
from Camera_file.Camera_Module3 import Camera_Module
from Robot_tool_file.Offense_Module import Offense_Module
from BasicAction_Module import BasicAction_Module
import Angle_Sensor.MPU9250 as MP
import kento.Keeper as K
from Angle_Sensor.Angle_module import Angle_Module
import cv2
import serial
import time
import numpy as np
from LiDAR_file.LiDAR_module3 import LiDAR
from multiprocessing import Process, Value, Array
import ctypes
import Jetson.GPIO as GPIO

mygoal = "yellow"

def main():
    #lidar arduinoポート選択
    lidar_port = '/dev/ttyUSB0'
    arduino_port = '/dev/ttyUSB1'
    if len(sys.argv)>1:
        lidar_port = '/dev/ttyUSB1'
        arduino_port = '/dev/ttyUSB0'
    
    #インスタンス作成
    camera = Camera_Module()
    mpu9250 = MP.MPU9250(MP.MP9250_ADR)
    angle_module = Angle_Module(False)
    lidar = LiDAR(lidar_port)
    ba = BasicAction_Module()
    offense_module = Offense_Module()
    keeper = K.Keeper()
    
    #角度センサ初期設定
    mpu9250.configMPU9250(gfsNo = MP.GFS_500, afsNo = MP.AFS_4G)
    mpu9250.configAK8963(mode = MP.AK8963_MODE_C100HZ, mfs = MP.AK8963_BIT_16)
    
    #ライダー用変数
    lidar_state = Value(ctypes.c_bool, True)
    lidar_angle = Value('d', 0.0)
    abspos = Array('i', [0,0])
    maps = cv2.imread("LiDAR_file/2021rule.jpg")  
    
    #変数
    angle = 0
    sol = 0
    ball_hold = False
    pretime = 0
    prog_no = 0
    state = 0
    return_signal = 0
    state_data = 0
    ball_buf = []
    
    #GPIO初期化
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(11, GPIO.IN)
    
    #LiDAR測定開始
    p = Process(target=lidar.lidar_process, args=(lidar_state, lidar_angle, abspos))
    p.start()
    time.sleep(0.1)
    
    with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as ser:
        with serial.Serial(arduino_port, 115200, timeout=1) as ser_ui:
            ser.reset_input_buffer()
            ser_ui.reset_input_buffer()
            pretime = time.perf_counter()
            try:
                while True:
                    Key = cv2.waitKey(1)
                    if Key == 27:
                        break
                    
                    #マップコピー
                    map_fix = maps.copy()
                    
                    #ボールセンサ
                    """hold_sw = GPIO.input(11)
                    if hold_sw == 0:
                        ball_hold = True
                    else:
                        ball_hold=False"""
                    
                    #角度算出
                    nowtime = time.perf_counter()
                    dt = nowtime - pretime
                    pretime = nowtime
                    angle = angle_module.readAngle(mpu9250)
                    lidar_angle.value=angle
                    now_pos = np.array(abspos, np.int16)
                    if 255 < now_pos[0] or 255 < now_pos[1] or now_pos[0]<0 or now_pos[1]<0:
                        now_pos = np.array([0,0], np.int16)
        
                    #teensyからデータ受信
                    serial_rec_data=[0]*5
                    while ser.in_waiting > 5:
                        if bytes('A', 'utf-8') == ser.read():
                            for i in range(5):
                                serial_rec_data[i] = int.from_bytes(ser.read(), byteorder='big')
                                
                    #arduinoからデータ受信
                    while ser_ui.in_waiting > 0:
                        state_data = int.from_bytes(ser_ui.read(), byteorder='big')
                    prog_no       = 0b0000_0111 & state_data
                    state         = 1 if 0b0000_1000 & state_data else 0
                    return_signal = 1 if 0b0001_0000 & state_data else 0
                    #print(f"{state_data:08b} {prog_no} {state} {return_signal}")
                    
                    #画像読み込み(ボールメイン)
                    
                    if not ball_hold:
                        ball, A_goal, E_goal = camera.read_pos_data([], [1], [3], 10, dposx = 32, dposy = -18)
                        #if len(ball)>0:
                        #ball_buf.append(ball)
                        # if len(ball_buf)>5:
                        #    ball_buf.pop(0)
                    else:
                        if mygoal == "blue":
                            ball, A_goal, E_goal = camera.read_pos_data([1], [3], [], 10, dposx = 32, dposy = -18)
                        elif mygoal == "yellow":
                            ball, E_goal, A_goal = camera.read_pos_data([1], [], [3], 10, dposx = 32, dposy = -18)
                    
                    if len(ball)>0:        
                        if -4 <= ball[0]<=17 and -73<=ball[1]<=-50:
                            ball_hold = True
                        else :
                            ball_hold = False
                    else:
                        ball_hold=False
                        
                    out = np.array([0,0], np.int16)
                    
                    #0度方向へ角度制御
                    anglepw = ba.angle_ctr(0, angle, dt, Kp=8, Ki=0, Kd=1)
                    
                    
                    #戦術プログラム
                    #前向きオフェンス
                    if prog_no == 1:
                        if not ball_hold:
                            if len(ball)>0:
                                print(ball, 2)
                                out = offense_module.tracking_ball(ball, dt)
                                sol = 0
                            else:
                                out = offense_module.move_center(now_pos, dt)
                                sol = 0
                        else:
                            out, sol, _ =  offense_module.driver_less_offense(E_goal, angle, dt)
                            print("offense")
                            
                    #後ろ向きオフェンス
                    elif prog_no == 2:
                        if not ball_hold:
                            if len(ball)>0:
                                out = offense_module.tracking_ball(ball, dt)
                                sol = 0
                            else:
                                out = offense_module.move_center(now_pos, dt)
                                sol = 0
                        else:
                            out, sol = offense_module.forward_offense(E_goal)
                            
                    #キーパー
                    elif prog_no == 3:
                        print("keeper_move", end=" ")
                        #ボールがないときホームポジションに戻る
                        if not len(ball) > 0:
                            out = keeper.movePoint(now_pos, dt)
                            #print("out = {}".format(out))
                            #print(type(out))
    
                        #ボール保持時
                        elif ball_hold == True:    
                            out = keeper.movePoint(now_pos, dt, 91, 60, 220)
                            if now_pos[1] <= 60:
                                sol = 0
                                #print(1)
                                #print("out = {}".format(out))
                    
                        #通常の動き
                        else:
                            out = keeper.moveLeftRight(ball, now_pos, dt)
                            #print("out = {}".format(out))
                            
                    #escリセット
                    elif prog_no == 7 and state == 1:
                        esc_reset = 1
                        out=[0,0]
                        sol=0
                        anglepw=0
                        
                    else:
                        out=[0,0]
                        sol=0
                        angle_pw=0
                    
                    #オーバーフロー カットオフ
                    if abs(out[0]) > 270 or abs(out[1]) > 270:
                        maxp = max(abs(out[0]), abs(out[1]))
                        outx =  out[0] / maxp * 270
                        outy = out[1] / maxp * 270
                        out = np.array([outx, outy], np.int16)
                    
                    ball_size = 0
                    ballx = 0
                    bally = 0
                    if len(ball)>0:
                        ball_size = ball[2]*ball[3]
                        ballx = ball[0]
                        bally = ball[1]
                    else:
                        print("lost ", end=" ")
                    print(f"{ball_hold} ball[{ballx:>4}, {bally:>4}, {ball_size:>5}] angle:{angle:>7.2f} out[{out[0]:>4}, {out[1]:>4}] {prog_no}")
                    #print(f"{now_pos} {serial_rec_data}")
                        
                    #teensyへデータ送信
                    ByteData =spritInt16Data(out[0])
                    ByteData +=spritInt16Data(out[1])
                    ByteData +=spritInt16Data(anglepw)
                    ByteData += list(now_pos)
                    ByteData +=[state+2*sol, return_signal]
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
            ByteData += [0, 0]
            ByteData += [0, 0]
            ser.write(bytes('A', 'utf-8'))
            for item in ByteData:
                ser.write(bytes([item]))
            
            #終了処理
            GPIO.cleanup()
            lidar_state.value=False
            p.join()


#int16から2byteに変換         
def spritInt16Data(data):
    dataC = [0xff & data, 0xff & (data >> 8)]
    return dataC        

if __name__ == "__main__":
    main()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
