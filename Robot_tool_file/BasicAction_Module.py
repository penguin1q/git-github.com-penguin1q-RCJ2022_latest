import numpy as np
import math

class BasicAction_Module:
    def __init__(self):
        self.angle_preP = 0
        self.angle_I = 0
        self.move_preP = 0
    
    #角度コントロール(P,I,D)
    def angle_ctr(self, target, angle, dt, Kp=9, Ki=0, Kd=1, maxval=255):
        #P,I,D制御
        P = target - angle
        self.angle_I += P * dt
        D = (P - self.angle_preP) / dt
        self.angle_preP = P
        U = (Kp * P + Ki * self.angle_I + Kd * D) * -1
        
        #オーバーフロー カットオフ
        if abs(U) > maxval:
            U = np.sign(U) * maxval
        return np.int16(U)
    
    #指定地点への移動(P,D)
    def move_pos(self, now_pos, tar_pos, dt, Kp=5.8, Kd=0.95, E_range=64, maxval=270):
        #距離測定
        now_pos = np.array(now_pos)
        tar_pos = np.array(tar_pos)
        Px = (tar_pos[0] - now_pos[0])
        Py = (tar_pos[1] - now_pos[1])
        
        #誤差範囲内の場合停止
        if Px**2 + Py**2 < E_range:
            Px = Py = 0
        
        #P,D制御
        Px = np.sign(Px)*(abs(Px))
        Py = np.sign(Py)*(abs(Py))
        P = np.array([Px, Py])
        D = (P - self.move_preP) / dt
        self.move_preP = P
        out = P * Kp + D * Kd
        
        #近距離用の出力調整
        if 0<(abs(out[0]) + abs(out[1])) < 140:
            max_val = math.sqrt(out[0]**2 + out[1]**2)
            outx = int(out[0] / max_val * 140)
            outy = int(out[1] / max_val * 140) 
            out = [outx, outy]
            
        #出力制限
        if abs(out[0]) > maxval or abs(out[1]) > maxval:
            maxp = max(abs(out[0]), abs(out[1]))
            outx =  out[0] / maxp * maxval
            outy = out[1] / maxp * maxval
            out = [outx, outy]
        return np.array(out, np.int16)
