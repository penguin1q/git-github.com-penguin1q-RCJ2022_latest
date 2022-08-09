import time
import configparser
import os
import errno
from pathlib import Path

class Angle_Module():
    def __init__(self, init_angle=True):
        self.ini_count=0
        self.MAG_INI_ANGLE_SUM=0
        self.limited_count=0
        self.angle_limited=0
        self.MAG_INI_ANGLE=0.0
        self.MAG_ANGLE = 0
        self.preangle=0
        self.config_ini = configparser.ConfigParser()
        self.config_ini_path = str(Path(__file__).resolve().parent.parent) + '/config.ini'
        if not os.path.exists(self.config_ini_path):
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), self.config_ini_path)
        self.config_ini.read(self.config_ini_path, encoding='utf-8')
        if init_angle == False:
            self.ini_count=100
            self.MAG_INI_ANGLE = float(self.config_ini.get("AngleSensor", "angle_ini_deg"))
            self.angle_limited = float(self.config_ini.get("AngleSensor", "angle_limited_deg"))
        
    def readAngle(self, mpu9250):
        #読み取り
        accXYZ, gyrXYZ = mpu9250.readAccGyro()
        magXYZ = mpu9250.readMag()
        temp = mpu9250.readTemp()
        
        #AK8963がデータを更新したら、角度を計算し、更新
        if magXYZ[0] != 0 and magXYZ[1] != 0:
            RAW_MAG_ANGLE = mpu9250.culcFixMag(magXYZ)
            self.MAG_ANGLE = RAW_MAG_ANGLE - self.MAG_INI_ANGLE + self.angle_limited
            if self.MAG_ANGLE < -180.0:
                self.MAG_ANGLE += 360.0
            if self.MAG_ANGLE > 180.0:
                self.MAG_ANGLE -= 360.0
            #起動時に初期角度を出す場合
            if self.ini_count < 19:
                if self.MAG_ANGLE < -160 or self.MAG_ANGLE > 160:
                    self.limited_count+=1
                print("limit_measure", end="  ")
                self.ini_count +=1
            elif self.ini_count == 19:
                if self.limited_count>3:
                    self.angle_limited = 90
                print(self.limited_count)
                self.ini_count +=1
            elif self.ini_count < 70:
                self.MAG_INI_ANGLE_SUM += self.MAG_ANGLE
                self.ini_count +=1
                print("fixing", end="  ")
            elif self.ini_count == 70:
                self.MAG_INI_ANGLE = self.MAG_INI_ANGLE_SUM / 50
                self.config_ini.set("AngleSensor", "angle_ini_deg", str(self.MAG_INI_ANGLE))
                self.config_ini.set("AngleSensor", "angle_limited_deg", str(self.angle_limited))
                with open(self.config_ini_path, "w", encoding="utf-8") as configfile:
                    self.config_ini.write(configfile, True)
                self.ini_count +=1
        
        #デルタ角算出        
        delta_angle = mpu9250.fixGyroZ(gyrXYZ[2])
        GYRO_ANGLE = self.preangle + delta_angle
        #相補フィルタ
        GYRO_MAG_ANGLE = (GYRO_ANGLE * 0.97) + (self.MAG_ANGLE * 0.03)
        #臨界点と角度が飛んだときの修正
        if ( abs(GYRO_MAG_ANGLE - self.MAG_ANGLE) > 40 ) or not (-177 < self.MAG_ANGLE < 177):
            GYRO_MAG_ANGLE = self.MAG_ANGLE
            
        self.preangle = GYRO_MAG_ANGLE 
        return GYRO_MAG_ANGLE
        
def main():
    import MPU9250 as MP
    mpu9250 = MP.MPU9250(MP.MP9250_ADR)
    angle_module = Angle_Module(True)
    mpu9250.configMPU9250(gfsNo = MP.GFS_500, afsNo = MP.AFS_4G)
    mpu9250.configAK8963(mode = MP.AK8963_MODE_C100HZ, mfs = MP.AK8963_BIT_16)
    while True:
        nowangle = angle_module.readAngle(mpu9250)
        time.sleep(0.016)
        print(f"{nowangle:>7.2f}")
        
            
if __name__ == "__main__":
    main()
