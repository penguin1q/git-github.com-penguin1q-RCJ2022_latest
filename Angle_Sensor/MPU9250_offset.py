# -*- coding: utf-8 -*-
import smbus
import time
import math
import numpy as np
import cv2
import configparser
import os
import errno
from pathlib import Path

I2C = smbus.SMBus(0)    # I2C smbus

MP9250_ADR = 0x68    # MPU9250 I2C slave address
AK8963_ADR = 0x0C    # AK8963 I2C slave address
DEVICE_ID = 0x71    ## Device id

# MPU-9250 Register Addresses
SMPLRT_DIV     = 0x19
CONFIG         = 0x1A
GYRO_CONFIG    = 0x1B
ACCEL_CONFIG   = 0x1C
ACCEL_CONFIG_2 = 0x1D
INT_PIN_CFG    = 0x37
INT_STATUS     = 0x3A
ACCEL_OUT      = 0x3B
TEMP_OUT       = 0x41
GYRO_OUT       = 0x43
PWR_MGMT_1     = 0x6B
WHO_AM_I       = 0x75

# Gyro Full Scale Index
GFS_250  = 0
GFS_500  = 1
GFS_1000 = 2
GFS_2000 = 3
GYR_FS = [0x00,0x08, 0x10, 0x18]        # ジャイロFSレンジのBit設定 Reg29[4:3]
GYR_RNG = [250.0, 500.0, 1000.0, 2000.0]    # ジャイロFSレンジ(単位=deg/sec)

# Accel Full Scale Index
AFS_2G   = 0
AFS_4G   = 1
AFS_8G   = 2
AFS_16G  = 3
ACC_FS = [0x00, 0x08, 0x10,0x18]    # 加速度FSレンジのBit設定 Reg28[4:3]
ACC_RNG = [2.0, 4.0, 8.0, 16.0]        # 加速度FSレンジ(単位=G)

# AK8963 Register Addresses
AK8963_ST1     = 0x02    # ステータス
AK8963_MAG_OUT = 0x03    # 測定データ
AK8963_CNTL1   = 0x0A    # コントロール
AK8963_ASAX    = 0x10    # 感度調整値

# CNTL1 Mode select
AK8963_MODE_DOWN   = 0x00    # パワーダウンモード
AK8963_MODE_ONE    = 0x01    # 単発測定モード
AK8963_MODE_C8HZ   = 0x02    # 連続測定モード１(8Hz)
AK8963_MODE_C100HZ = 0x06    # 連続測定モード２(100Hz)
AK8963_BIT_14      = 0x00    # 14bits output
AK8963_BIT_16      = 0x10    # 16bits output

#MAG_OFFSET
mx_min = None
mx_max = None
my_min = None
my_max = None
mx_offset = 0.0
my_offset = 0.0

#GyroZ_OFFSET and MAG_INI_DEG
gz_sum = 0.0
mag_deg_sum = 0.0
count = 0.0
gz_offset = 0.0
MAG_INI_DEG = 0.0

class MPU9250:
    def __init__(self, mpuAdr): #コンストラクタ
        self.mpuAdr = mpuAdr #0x68
        self.configMPU9250(gfsNo=GFS_250, afsNo=AFS_2G)    # Default
        self.configAK8963(mode=AK8963_MODE_C8HZ, mfs=AK8963_BIT_16)    # Default

    def searchDevice(self):    # Device Check
        who_am_I = I2C.read_byte_data(self.mpuAdr, WHO_AM_I)
        if(who_am_I == DEVICE_ID):
            return True
        else:
            return False

    def configMPU9250(self, gfsNo, afsNo):    # Configure MPU-9250
        if gfsNo < 0 or gfsNo > 3: #間違ったadressが指定されたら
            gfsNo = 3 #強制的に2000deg/sec
        self.gres = GYR_RNG[gfsNo] / 32767.0 #des/secへの変換に使用

        if afsNo < 0 or afsNo > 3: #上はジャイロ、下は加速度
            afsNo = 3 #ジャイロと同じことをしている
        self.ares = ACC_RNG[afsNo] / 32767.0

        I2C.write_byte_data(self.mpuAdr, PWR_MGMT_1, 0x00)    # sleep off
        time.sleep(0.1)
        I2C.write_byte_data(self.mpuAdr, PWR_MGMT_1, 0x01)    # auto select clock source
        time.sleep(0.1)
        I2C.write_byte_data(self.mpuAdr, CONFIG, 0x02)    # DLPF_CFG(Low Pass Filter 98Hz)
        I2C.write_byte_data(self.mpuAdr, SMPLRT_DIV, 0x04)    # sample rate divider
        I2C.write_byte_data(self.mpuAdr, GYRO_CONFIG, GYR_FS[gfsNo])    # gyro full scale select
        I2C.write_byte_data(self.mpuAdr, ACCEL_CONFIG, ACC_FS[afsNo])    # accel full scale select
        I2C.write_byte_data(self.mpuAdr, ACCEL_CONFIG_2, 0x02)    # A_DLPFCFG(Low Pass Filter 94Hz)
        I2C.write_byte_data(self.mpuAdr, INT_PIN_CFG, 0x02)    # BYPASS_EN
        time.sleep(0.1)

    def configAK8963(self, mode, mfs):    # Configure AK8963
        if mfs == AK8963_BIT_14: #単位変換で使う値を設定
            self.mres = 4800.0/8190.0
        else: #  mfs == AK8963_BIT_16:
            self.mres = 4800.0/32767.0

        I2C.write_byte_data(AK8963_ADR, AK8963_CNTL1, 0x00) #初期化
        time.sleep(0.01)
        I2C.write_byte_data(AK8963_ADR, AK8963_CNTL1, 0x0F)    # set read FuseROM mode
        time.sleep(0.01)
        data = I2C.read_i2c_block_data(AK8963_ADR, AK8963_ASAX, 3)    # read coef data
        self.magCoef = [0., 0., 0.] #各軸の感度調整データを読みこんでいる
        for ax in range(3):
            self.magCoef[ax] = (data[ax] - 128) / 256.0 + 1.0

        I2C.write_byte_data(AK8963_ADR, AK8963_CNTL1, 0x00)    # set power down mode
        time.sleep(0.01)
        I2C.write_byte_data(AK8963_ADR, AK8963_CNTL1, (mfs | mode))# set scale&continous mode
        time.sleep(0.01)

    def checkDataReady(self):
        dataReady = I2C.read_byte_data(self.mpuAdr, INT_STATUS)
        if dataReady & 0x01:
            return True
        else:
            return False

    # Read accelerometer and gyro
    def readAccGyro(self):
        data = I2C.read_i2c_block_data(self.mpuAdr, ACCEL_OUT, 14)
        # ACC=data[0:5], TEMP=data[6:7], GYRO=data[8:13] ---- MSB:LSB
        accData = [0., 0., 0.]
        for ax in range(3):    # set accelerometer data
            accData[ax] = self.byte2data(data[ax*2], data[ax*2+1])    # (MSB, LSB)
            accData[ax] = round(accData[ax] * self.ares, 3) #単位変換をし、小数点第三位にまとめる(四捨五入)
        gyrData = [0., 0., 0.]
        for ax in range(3):    # set gyro data
            gyrData[ax] = self.byte2data(data[8+ax*2], data[9+ax*2])    # (MSB, LSB)
            gyrData[ax] = round(gyrData[ax] * self.gres, 3)
        return accData, gyrData


    # Read mag
    def readMag(self):
        magData = [0., 0., 0.]
        dataReady = I2C.read_byte_data(AK8963_ADR, AK8963_ST1)
        if dataReady & 0x01 :    # Check data ready
            data = I2C.read_i2c_block_data(AK8963_ADR, AK8963_MAG_OUT, 7)
            # MAG=data[0:5] --- (LSB:MSB), OverFlow=data[6]
            if (data[6] & 0x08) != 0x08:    # Check overflow
                for ax in range(3):
                    magData[ax] = self.byte2data(data[ax*2+1], data[ax*2])    # (MSB, LSB)
                    magData[ax] = round(magData[ax] * self.mres * self.magCoef[ax], 3)
        return magData

    # Read temperature
    def readTemp(self):
        data = I2C.read_i2c_block_data(self.mpuAdr, TEMP_OUT, 2)
        tempData = self.byte2data(data[0], data[1])    # (MSB, LSB)
        tempData = round((tempData / 333.87 + 21.0), 3)
        return tempData

    def byte2data(self, byteMSB, byteLSB):
        value = (byteMSB << 8) | byteLSB #最上位(先頭)ビットの０，１で正負を判定。1なら負、０ならプラス
        value = (- (value & 0x8000)) | (value & 0x7FFF)    # 正負に直す
        #print(value)
        return value
    
    def culcMag(self, magXYZ):
        mag_angle = math.degrees(math.atan2(magXYZ[1] - my_offset, magXYZ[0] - mx_offset))
        return mag_angle
        
if __name__ == '__main__':
    mpu9250 = MPU9250(MP9250_ADR)        # 0x68
    if mpu9250.searchDevice():
        print("Who am I? OK")
    else:
        print("Who am I? Error")

    mpu9250.configMPU9250(gfsNo = GFS_500, afsNo = AFS_4G)
    # Full Scale : Gyro=500deg/sec, Acc=4g
    mpu9250.configAK8963(mode = AK8963_MODE_C100HZ, mfs = AK8963_BIT_16)
    # mode=Continous 100Hz, 16bit output
    
    while True:
        magXYZ = mpu9250.readMag()
        if magXYZ[0] != 0 and magXYZ[1] != 0:
            mx_min = magXYZ[0]
            mx_max = magXYZ[0]
            my_min = magXYZ[1]
            my_max = magXYZ[1]
            break
        time.sleep(0.003)
    
    while True:
        #img = cv2.imread("fruit_orange.png")
        img = np.zeros((100,100,3))
        cv2.imshow('image', img)
        magXYZ = mpu9250.readMag()
        if magXYZ[0] != 0 and magXYZ[1] != 0:
            if magXYZ[0] < mx_min:
                mx_min = magXYZ[0]
            if magXYZ[0] > mx_max:
                mx_max = magXYZ[0]
            if magXYZ[1] < my_min:
                my_min = magXYZ[1]
            if magXYZ[1] > my_max:
                my_max = magXYZ[1]
        print('mx_min:{}'.format(mx_min))
        print('mx_max:{}'.format(mx_max))
        print('my_min:{}'.format(my_min))
        print('my_max:{}'.format(my_max))
            
        # キー入力受付
        key = cv2.waitKey(1)
        print(key)
        # 終了キー（EnterかEscで終了）
        if (key == 13) or (key == 27):#or cnt == 200:
            print('break')
            break
        time.sleep(0.03)
            
    config_ini = configparser.ConfigParser()
    config_ini_path = str(Path(__file__).resolve().parent.parent) + '/config.ini'
    if not os.path.exists(config_ini_path):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), config_ini_path)
    config_ini.read(config_ini_path, encoding='utf-8')

    mx_offset = (mx_min + mx_max) / 2
    my_offset = (my_min + my_max) / 2
    print('\nmx_offset:{}'.format(mx_offset))
    print('my_offset:{}'.format(my_offset))
    print('mag_offset_completed')
    print('Dont Move!!')
    #time.sleep(5)
    config_ini.set("AngleSensor", "mx_offset", str(mx_offset))
    config_ini.set("AngleSensor", "my_offset", str(my_offset))
    cv2.destroyAllWindows()
    
    print('START GZ_OFFSET and GET_MAG_INI_DEG') 
    while count < 1000:
        img = cv2.imread("fruit_orange.png")
        #cv2.imshow('image', img)
        img = np.zeros((100,100,3))
        if not mpu9250.checkDataReady():
            #print("Data not ready")
            continue
        accXYZ, gyrXYZ = mpu9250.readAccGyro()
        magXYZ = mpu9250.readMag()
        gz_sum += gyrXYZ[2]
        mag_data = mpu9250.culcMag(magXYZ)
        mag_deg_sum += mag_data
        count += 1
        print(f"{count} : {mag_data}")
        time.sleep(0.01)
            
    gz_offset = gz_sum / count
    MAG_INI_DEG = mag_deg_sum / count
    
    print('\ngz_offset:{}'.format(gz_offset))
    print('MAG_INI_DEG:{}'.format(MAG_INI_DEG))
    print('Gyro_offset_completed')
    print('got MAG_INI_DEG')
    config_ini.set("AngleSensor", "gz_offset", str(gz_offset))
    config_ini.set("AngleSensor", "MAG_INI_DEG", str(MAG_INI_DEG))

    with open(config_ini_path, "w", encoding="utf-8") as configfile:
            config_ini.write(configfile, True)
    time.sleep(3)
    cv2.destroyAllWindows()
