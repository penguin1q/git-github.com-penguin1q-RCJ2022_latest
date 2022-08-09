import numpy as np

#調整ゲイン
Kp_BallX = 7.0
Kd_BallX = 1.2
Kp_Pos = 12.0
Kd_Pos = 1.2

class Keeper:
    def __init__(self):
        #moveLeftRight用
        self.pre_bxP = 0
        self.pre_ayP = 0
        self.max_val = 270
        
        #movePoint用
        self.pre_aP = [0, 0]
        
    #x(左右)の出力の大きさを返す。ボールの正面に左右移動。上下移動は目標地点で一定。
    def moveLeftRight(self, ball, abs_pos, dt, target_y = 55):
        out = list()
        sol = 0
        #ball[0]の値はカメラが反転しているので、反転させる
        Px = -ball[0]
        Dx = (Px - self.pre_bxP) / dt
        self.pre_bxP = Px
        
        out.append(Kp_BallX * Px + Kd_BallX * Dx)
        if abs(out[0]) > self.max_val:
            #符号関数(sign)は正の値に対して1、負の値に対して-1、0に対して0を返す関数。
            out[0] = np.sign(out[0]) * self.max_val
        
      
        Py = (243 - abs_pos[1]) - target_y
        Dy = (Py - self.pre_ayP) / dt
        self.pre_ayP = Py
        
        out.append(Kp_Pos * Py + Kd_Pos * Dy)
        if abs(out[1]) > self.max_val:
            out[1] = np.sign(out[1]) * self.max_val
            
        #int16は符号あり16ビット整数型
        out = np.array(out, np.int16)
        
        return out
    
    #引数に座標を入れるとその絶対座標に移動(初期値は味方ゴール手前, 出力180),インスタンスを生成しなくても使える
    def movePoint(self, abs_pos, dt, target_x = 91, target_y = 243 - 55, er_rangeR = 8, max_val = 230): 
        out = list()
        
        Px = target_x - abs_pos[0]
        Py = target_y - abs_pos[1]
        Dx = (Px - self.pre_aP[0]) / dt
        Dy = (Py - self.pre_aP[1]) / dt
        self.pre_aP[0] = Px
        self.pre_aP[1] = Py
        
        #誤差範囲内なら半径8cm
        if Px**2 + Py**2 < er_rangeR**2:
            Px = Py = 0
        
        out.append(Kp_Pos * Px + Kd_Pos * Dx)
        out.append(Kp_Pos * Py + Kd_Pos * Dy)
        
        #オバーフロー対策
        if abs(out[0]) > abs(out[1]):
            #print("out[0] = {}".format(out[0]))
            if abs(out[0]) > max_val:
                c = max_val / abs(out[0])
                out[0] = out[0] * c
                out[1] = out[1] * c
                #print("a")
        elif abs(out[1]) > abs(out[0]):
            if abs(out[1]) > max_val:
                c = max_val / abs(out[1])
                out[0] = out[0] * c
                out[1] = out[1] * c
                #print("b")
        else:
            if abs(out[0]) > max_val and abs(out[1]) > max_val:
                out[0] = np.sign(out[0]) * max_val
                out[1] = np.sign(out[1]) * max_val
                
        out = np.array(out, np.int16)
        
        return out
        
       
if __name__ == '__main__':
    keeper = Keeper()
    #-360~360
    ball = []
    #y=0~182, y = 0~243
    abs_pos = [0, 220]
    hold = True
    sol = 0
    dt = 0.020  
   
    #ボールがないときホームポジションに戻る,ボールがないときは空のリストでくる,len()でlistの要素数を返す
    if not len(ball) > 0:
        out = keeper.movePoint(abs_pos, dt)
        print("out = {}".format(out))
        #print(type(out))
    
    #ボール保持時
    elif hold == True:    
        out = keeper.movePoint(abs_pos, dt, 91, 60, 220)
        if abs_pos[1] <= 60:
            sol = 1
        print("out = {}".format(out))
    
    #通常の動き
    else:
        out = keeper.moveLeftRight(ball, abs_pos, dt)
        print("out = {}".format(out))
