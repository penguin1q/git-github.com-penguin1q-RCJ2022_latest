# -*- coding: utf-8 -*-
import MPU9250 as MP

def main():
    #地磁気はグローバル変数にデータを保管する
    MAG_ANGLE = 0.0
    
    #コンストラクタの引数は順番に
    #(MP9250_ADR, mx_offset, my_offset, gz_offset, MAG_INI_DEG)
    mpu9250 = MP.MPU9250(MP.MP9250_ADR) #0x68
    
    #i2cで接続しているか、確認
    if mpu9250.searchDevice():
        print("Who am I? OK")
    else:
        print("Who am I? Error")

    #それぞれ（加速度、ジャイロ）レンジを設定
    mpu9250.configMPU9250(gfsNo = MP.GFS_500, afsNo = MP.AFS_4G)
    # Full Scale : Gyro=500deg/sec, Acc=4g
    mpu9250.configAK8963(mode = MP.AK8963_MODE_C100HZ, mfs = MP.AK8963_BIT_16)
    # mode=Continous 100Hz, 16bit output
                
    while(1):#ここから無限ループ
        
        #データが用意できていなかったら読み取りスキップ
        if not mpu9250.checkDataReady():
            #print("Data not ready")
            continue
        
        #読み取り
        accXYZ, gyrXYZ = mpu9250.readAccGyro()
        magXYZ = mpu9250.readMag()
        temp = mpu9250.readTemp()
        
        #AK8963がデータを更新したら、角度を計算し、更新
        if magXYZ[0] != 0 and magXYZ[1] != 0:
            MAG_ANGLE = mpu9250.culcFixMag(magXYZ)
            print(MAG_ANGLE)
            #pass
        
        GYRO_ANGLE = mpu9250.culcGyroZ(gyrXYZ[2])
        #print(GYRO_ANGLE)
        
        GYRO_MAG_ANGLE = GYRO_ANGLE * 0.95 + MAG_ANGLE * 0.05
        print(GYRO_MAG_ANGLE)
        
        #読み取り各軸生データ
        strAcc ="Acc = {:7.2f}, {:7.2f}, {:7.2f}, ".format(accXYZ[0], accXYZ[1], accXYZ[2])
        strGyro ="Gyro = {:7.2f}, {:7.2f}, {:7.2f}, ".format(gyrXYZ[0], gyrXYZ[1], gyrXYZ[2])
        strMag ="Mag = {:7.2f}, {:7.2f}, {:7.2f}, ".format(magXYZ[0], magXYZ[1], magXYZ[2])
        strTemp ="Temp = {:7.2f}".format(temp)
        #print(strAcc + strGyro + strMag + strTemp)

main()