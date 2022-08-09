import numpy as np
import math


#2^3*3^2*5
#    3  *5  * 17
class Robot_moves:
    def __init__(self,
                 c_gain=[ 1, 3],
                 f_gain=[2.5, 2.5],
                 r_gain=[140, 120],
                 o_gain=[3, 4]):
        self.c_gain = c_gain
        self.f_gain = f_gain
        self.r_gain = r_gain
        self.o_gain = o_gain
        self.preP=0

    """[summary]
        boll tracking fanction.
        機体の前方方向にボールがあるならptpで移動
        逆に後方遠方にボールがあるなら、それもptpで移動
        それ以外(機体周辺のボール)はカージオイド
        [param]
        data: ボールの相対データ
        c_range: どこまでを前の範囲とするか
        c_gain: pid前進時のゲイン
        ygain: 後ろ方向に離れているとき、どこまでy方向にふくらませるか(中心との差)
        yr: ホール保持と中心との差
        [return]
        out: output(x,y)->ndarray
    """
    def tracking_ball(self, data, dt, c_range=[-0.80, -2.35],bygain=100, ygain=40, yr=30, f_gainD=0.2):
        if data:
            Theta = math.atan2((data[1] + yr), data[0])  #+ math.pi
            tx = 1
            ty = 1
            #print(f"theta[rad] = {Theta} theta = {math.degrees(Theta):.4}", end = "   ")
            #print(f"x[px] = {data[0]} y[px] = {data[1]} theta = {Theta:.4}", end = "   ")
            if c_range[0] > Theta > c_range[1]:
                """print("f range", end = "   ")
                outx = int(-self.f_gain[0] * data[0])
                outy = int(self.f_gain[1] * (data[1] + ygain))
                if abs(outx) > 255 and abs(outy) > 255:
                    max_val = max(abs(outx), abs(outy))
                    outx = int(outx / max_val * 250)
                    outy = int(outy / max_val * 250)
                elif 0 < (abs(outx) + abs(outy)) < 180:
                    max_val = max(abs(outx), abs(outy))
                    outx = int(outx / max_val * 180)
                    outy = int(outy / max_val * 180)
                out = np.array([outx, outy], np.int16)
                return out"""
                
                #print("f range", end = "   ")
                P = data[0]
                D = (P - self.preP) / dt
                self.preP = P
                outx = int(-(self.f_gain[0] * P + f_gainD * D))
                outy = int(self.f_gain[1] * (data[1] + ygain))
                if abs(outx) > 255 and abs(outy) > 255:
                    max_val = max(abs(outx), abs(outy))
                    outx = int(outx / max_val * 250)
                    outy = int(outy / max_val * 250)
                elif 0 < (abs(outx) + abs(outy)) < 180:
                    max_val = max(abs(outx), abs(outy))
                    outx = int(outx / max_val * 180)
                    outy = int(outy / max_val * 180)
                out = np.array([outx, outy], np.int16)
                return out
            
            elif ((data[0] < -60 or data[0] > 60) and data[1] > 50) or data[1] > 50:
                #print("back")
            #if (data[0] < -60 or data[0] > 60) and data[1] > 50:#デバック用_高速バック
                outx = int(-self.c_gain[0] * data[0])
                outy = int(self.c_gain[1] * (data[1] + bygain))
                if abs(outx) > 255 and abs(outy) > 255:
                    max_val = max(abs(outx), abs(outy))
                    outx = int(outx / max_val * 250)
                    outy = int(outy / max_val * 250)
                elif 0 < (abs(outx) + abs(outy)) < 140:
                    max_val = max(abs(outx), abs(outy))
                    outx = int(outx / max_val * 140)
                    outy = int(outy / max_val * 140)
                out = np.array([outx, outy], np.int16)
                #print(f"out = {out}", end = "   ")
                return out
                
            elif (data[0] < -60 or data[0] > 60):
                #print("side", end="   ")
                outx = int(-self.c_gain[1] * data[0])
                outy = int(self.c_gain[0] * (data[1] + bygain))
                if abs(outx) > 255 and abs(outy) > 255:
                    max_val = max(abs(outx), abs(outy))
                    outx = int(outx / max_val * 250)
                    outy = int(outy / max_val * 250)
                elif 0 < (abs(outx) + abs(outy)) < 140:
                    max_val = max(abs(outx), abs(outy))
                    outx = int(outx / max_val * 140)
                    outy = int(outy / max_val * 140)
                out = np.array([outx, outy], np.int16)
                #print(f"out = {out}", end = "   ")
                return out
                
            elif data[0] < 0:
            #if data[0] < 0:#デバック用_カージオイド
                ty = -1
                tx = -1
            else:
                ty = 1
                tx = 1
            #print("cald", end="   ")
            outx = int(-tx * self.r_gain[0] *
                       (-math.sin(Theta) + math.cos(2 * Theta)))
            outy = int(ty * self.r_gain[1] *
                       (math.cos(Theta) + math.sin(2 * Theta)))
            if 0 < (abs(outx) + abs(outy)) < 170:
                max_val = max(abs(outx), abs(outy))
                outx = int(outx / max_val * 170)
                outy = int(outy / max_val * 170)
            out = np.array([outx, outy], np.int16)
            return out

        else:
            outx = 0
            outy = 0
            out = np.array([outx, outy], np.int16)
            return out
    """[summary]
        lidarからの座標をもとに、指定の座標のポジションに移動させる
        [param]
        now_pos: 現在地座標[x,y]list or ndarray
        tar_ops: 目標座標[x, y]list or ndarray
        gain: 出力ゲイン
        range: 完璧な座標は無理なのでおよその範囲(半径cmの2乗)
        [return]
        out: 出力[x,y]->ndarray
    """
    def movepos(self, now_pos, tar_pos, gain = 3, range=400):
        now_pos = np.array(now_pos)
        tar_pos = np.array(tar_pos)
        error = (tar_pos - now_pos) * gain
        outx = error[0]
        outy = error[1]
        if error[0]**2 + error[1]**2 < range:
            return np.array([0,0])
        else:
            if 0 < (abs(outx) + abs(outy)) < 180:
                max_val = max(abs(outx), abs(outy))
                outx = int(outx / max_val * 180)
                outy = int(outy / max_val * 180)
            out = np.array([outx, outy], np.int16)
            return out

    def offence(self, data):
        
        sol = 0
        soo = False
        if data:   
            if (data[0] - abs(data[2]/2)<-60) and(data[0] + abs(data[2]/2)>40) and (0 > data[1] > -180):
                sol = 1
                soo = True
            outx = int(self.o_gain[0] * -data[0])
            outy = int(self.o_gain[1] * data[1])
            #print(f"x={[data[0] - abs(data[2]/2) , data[0] + abs(data[2]/2)]}, x={data[0]}, y={data[1]}, sol={soo}", end="   ")

            out = np.array([outx, outy], np.int16)
            return out, sol
        else:
            outx = 0
            outy = 0
            out = np.array([outx, outy], np.int16)
            return out, sol

