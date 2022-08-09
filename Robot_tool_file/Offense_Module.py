import numpy as np
import math
from pathlib import Path
import sys, os
from BasicAction_Module import BasicAction_Module

class Offense_Module:
    def __init__(self):
        self.ba = BasicAction_Module()
        self.preP = 0
    
    #ボール追跡
    def tracking_ball(self, ball, dt, c_range=[-0.80, -2.4], c_gain=[1, 3], 
                      bygain=120, ygain=40, f_gain=2.5, f_gainD=0.5,
                      yr=20, r_gain=[140, 120], b_range=-60, calg_range=[100, 100]):
                      
        #ボールが見える場合
        if ball:
            Theta = math.atan2((ball[1] + yr), ball[0])
            #print(f"{math.degrees(Theta):>6.4}", end=" ")
            tx = ty = 1
            if c_range[0] > Theta > c_range[1]:
                P = ball[0]
                D = (P - self.preP) / dt
                self.preP = P
                outx = int(-(f_gain * P + f_gainD * D))
                outy = int(f_gain * (ball[1] + ygain))
                if abs(outx) > 255 or abs(outy) > 255:
                    max_val = max(abs(outx), abs(outy))
                    outx = int(outx / max_val * 250)
                    outy = int(outy / max_val * 250)
                if 0 < (abs(outx) + abs(outy)) < 180:
                    max_val = math.sqrt(outx**2 + outy**2)
                    outx = int(outx / max_val * 180)
                    outy = int(outy / max_val * 180)
                out = np.array([outx, outy], np.int16)
                #print(f"front", end=" ")
                return out
                
            elif ball[1] > calg_range[1] or ((ball[0] < -calg_range[0] or ball[0] > calg_range[0]) and ball[1]> b_range):
                outx = int(-c_gain[0] * ball[0])
                outy = int(c_gain[1] * (ball[1] + bygain))
                if abs(outx) > 255 or abs(outy) > 255:
                    max_val = max(abs(outx), abs(outy))
                    outx = int(outx / max_val * 250)
                    outy = int(outy / max_val * 250)
                if 0 < (abs(outx) + abs(outy)) < 150:
                    max_val = max(abs(outx), abs(outy))
                    outx = int(outx / max_val * 150)
                    outy = int(outy / max_val * 150)
                out = np.array([outx, outy], np.int16)
                #print(f"back", end=" ")
                return out
                
            elif (ball[0] < -calg_range[0] or ball[0] > calg_range[0]):
                outx = int(-c_gain[1] * ball[0])
                outy = int(c_gain[0] * (ball[1] + bygain))
                if abs(outx) > 255 or abs(outy) > 255:
                    max_val = max(abs(outx), abs(outy))
                    outx = int(outx / max_val * 250)
                    outy = int(outy / max_val * 250)
                if 0 < (abs(outx) + abs(outy)) < 150:
                    max_val = max(abs(outx), abs(outy))
                    outx = int(outx / max_val * 150)
                    outy = int(outy / max_val * 150)
                out = np.array([outx, outy], np.int16)
                #print(f"side ", end=" ")
                return out
                
            elif ball[0] < 0:
                ty = tx = -1
            else:
                ty = tx = 1
            outx = int(-tx * r_gain[0] *
                       (-math.sin(Theta) + math.cos(2 * Theta)))
            outy = int(ty * r_gain[1] *
                       (math.cos(Theta) + math.sin(2 * Theta)))
            if 0 < (abs(outx) + abs(outy)) < 170:
                max_val = max(abs(outx), abs(outy))
                outx = int(outx / max_val * 170)
                outy = int(outy / max_val * 170)
            out = np.array([outx, outy], np.int16)
            #print(f"calg ", end=" ")
            return out
            
    def driver_less_offense(self, goal, now_angle, dt, o_gain=[3,4], Kp=6, Ki=0, Kd=1):
        sol = 0
        soo = False
        if len(goal)>0:
            Theta = 90 - math.degrees(math.atan2(goal[1], goal[0]))
            
            if goal:
                if (goal[0] - abs(goal[2]/2)<-60) and(goal[0] + abs(goal[2]/2)>40) and (0 > goal[1] > -180):
                    sol = 1
                outx = int(o_gain[0] * -goal[0])
                outy = int(o_gain[1] * goal[1])
                c = self.ba.angle_ctr(Theta, now_angle, dt, maxval=160)
                #print(f"x={[goal[0] - abs(goal[2]/2) , goal[0] + abs(goal[2]/2)]}, x={goal[0]}, y={goal[1]}, sol={soo}", end="   ")
                
                out = np.array([outx, outy], np.int16)
                return out, sol, c
            else:
                outx = 0
                outy = 0
                out = np.array([outx, outy], np.int16)
                return out, sol, c
        out=np.array([0,0],np.int16)
        c=0
        return out, sol, c
            
    def forward_offense(self, goal,o_gain=[3, 4]):
        
        sol = 0
        soo = False
        if goal:   
            if (goal[0] - abs(goal[2]/2)<-60) and(goal[0] + abs(goal[2]/2)>40) and (0 > goal[1] > -180):
                sol = 1
                outx = outy = 0
                out = np.array([outx, outy], np.int16)
                return out, sol
            outx = int(o_gain[0] * -goal[0])
            outy = int(o_gain[1] * goal[1])
            #print(f"x={[goal[0] - abs(goal[2]/2) , goal[0] + abs(goal[2]/2)]}, x={goal[0]}, y={goal[1]}, sol={soo}", end="   ")
            
            out = np.array([outx, outy], np.int16)
            return out, sol
        else:
            outx = 0
            outy = 0
            out = np.array([outx, outy], np.int16)
            return out, sol
            
    def backward_offense(self, goal, now_pos, now_angle, dt):
        sol=0
        out = np.array([0,0])
        c = 0
        #左攻め
        if 0<now_pos[0]<91:
            c = self.ba.angle_ctr(-170, now_angle, dt, maxval=160)
            
            #シューティングレンジ
            if now_pos[0]<50 and 35<=now_pos<=45:
                c = self.ba.angle_ctr(60, now_angle, dt, maxval=160)
                
            #Y方向移動
            if now_pos[0]<50:
                out = self.ba.move_pos(now_pos, [35, 40], dt, E_range=36, maxval=200)
            
            #X方向移動
            else:
                out = self.ba.move_pos(now_pos, [35, now_pos[1]], dt, E_range=36, maxval=200)
                
        #右攻め
        elif 91<=now_pos[0]<182:
            c = self.ba.angle_ctr(170, now_angle, dt, maxval=160)
            
            #シューティングレンジ
            if now_pos[0]>132 and 35<=now_pos<=45:
                c = self.ba.angle_ctr(-60, now_angle, dt, maxval=160)
                
            #Y方向移動
            if now_pos[0]>132:
                out = self.ba.move_pos(now_pos, [147, 40], dt, E_range=36, maxval=200)
                
            #X方向移動
            else:
                out = self.ba.move_pos(now_pos, [147, now_pos[1]], dt, E_range=36, maxval=200)
                
        return out, c, sol
        
    def move_center(self, now_pos, dt):
        out = np.array(self.ba.move_pos(now_pos, [ 91, 122], dt), np.int16)
        return out
            
            
            
                
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
