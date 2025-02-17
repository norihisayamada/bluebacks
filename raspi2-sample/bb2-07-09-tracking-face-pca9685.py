# -*- coding: utf-8 -*-
import picamera
import picamera.array
import cv2
import math
from time import sleep
import smbus

(w, h)=(320, 240)

cascade_path =  "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml"
import os
if not os.path.isfile(cascade_path):
    cascade_path =  "/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml"
cascade = cv2.CascadeClassifier(cascade_path)

def resetPCA9685():
    bus.write_byte_data(address_pca9685, 0x00, 0x00)

def setPCA9685Freq(freq):
    freq = 0.9*freq # Arduinoのライブラリより
    prescaleval = 25000000.0    # 25MHz
    prescaleval /= 4096.0       # 12-bit
    prescaleval /= float(freq)
    prescaleval -= 1.0
    prescale = int(math.floor(prescaleval + 0.5))
    oldmode = bus.read_byte_data(address_pca9685, 0x00)
    newmode = (oldmode & 0x7F) | 0x10             # スリープモード
    bus.write_byte_data(address_pca9685, 0x00, newmode) # スリープモードへ
    bus.write_byte_data(address_pca9685, 0xFE, prescale) # プリスケーラーをセット
    bus.write_byte_data(address_pca9685, 0x00, oldmode)
    sleep(0.005)
    bus.write_byte_data(address_pca9685, 0x00, oldmode | 0xa1)

def setPCA9685Duty(channel, on, off):
    channelpos = 0x6 + 4*channel
    try:
        bus.write_i2c_block_data(address_pca9685, channelpos, [on&0xFF, on>>8, off&0xFF, off>>8] )
    except IOError:
        pass

def getPCA9685Duty(id, val):
    val_min = 0
    val_max = 4095
    servo_min = 143 # 50Hzで0.7ms
    servo_max = 410 # 50Hzで2.0ms  (中心は276)
    if id==1 :
        servo_min = 193 # 50Hzで0.95ms
        servo_max = 360 # 50Hzで1.8ms
    duty = (servo_min-servo_max)*(val-val_min)/(val_max-val_min) + servo_max
    # 一般的なサーボモーターはこちらを有効に
    #duty = (servo_max-servo_min)*(val-val_min)/(val_max-val_min) + servo_min
    if duty > servo_max:
        duty = servo_max
    if duty < servo_min:
        duty = servo_min
    return int(duty)

bus = smbus.SMBus(1)
address_pca9685 = 0x40

resetPCA9685()
setPCA9685Freq(50)
setPCA9685Duty(0, 0, 276)
setPCA9685Duty(1, 0, 276)

minsize = int(60*(w/640))
maxsize = int(300*(w/640))
wc = int(w/2)
hc = int(h/2)
prev_x = wc
prev_y = hc
prev_input_x = 2048
prev_input_y = 2048
# サーボモーターを回転させる量を決める定数
ratio_x =  640/w
ratio_y = -480/h
with picamera.PiCamera() as camera:
    with picamera.array.PiRGBArray(camera) as stream:
        camera.resolution = (w, h)
        camera.framerate = 15

        while True:
            # stream.arrayにBGRの順で映像データを格納
            camera.capture(stream, 'bgr', use_video_port=True)
            # 映像データをグレースケール画像grayに変換
            gray = cv2.cvtColor(stream.array, cv2.COLOR_BGR2GRAY)
            # grayから顔を探す
            facerect = cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=2, minSize=(minsize,minsize), maxSize=(maxsize,maxsize))

            if len(facerect) > 0:
                # 複数見つかった顔のうち、以前の顔の位置に最も近いものを探す
                mindist = w + h
                minindx = 0
                indx = 0
                for rect in facerect:
                    dist = math.fabs(rect[0]+rect[2]/2-prev_x) + math.fabs(rect[1]+rect[3]/2-prev_y)
                    if dist < mindist:
                        mindist = dist
                        minindx = indx
                    indx += 1

                # 現在の顔の位置
                face_x = facerect[minindx][0]+facerect[minindx][2]/2
                face_y = facerect[minindx][1]+facerect[minindx][3]/2

                # 元の画像(stream.array)上の、顔がある位置に赤い四角を描画
                cv2.rectangle(stream.array, tuple(facerect[minindx][0:2]),tuple(facerect[minindx][0:2]+facerect[minindx][2:4]), (0,0,255), thickness=2)

                dx = face_x-wc  # 左右中央からのずれ
                dy = face_y-hc  # 上下中央からのずれ

                duty0 = getPCA9685Duty(0, int(ratio_x*dx) + prev_input_x)
                setPCA9685Duty(0, 0, duty0)

                duty1 = getPCA9685Duty(1, int(ratio_y*dy) + prev_input_y)
                setPCA9685Duty(1, 0, duty1)

                # サーボモーターに対する入力値を更新
                prev_input_x = int(ratio_x*dx) + prev_input_x
                if prev_input_x > 4095:
                    prev_input_x = 4095
                if prev_input_x < 0:
                    prev_input_x = 0
                prev_input_y = int(ratio_y*dy) + prev_input_y
                if prev_input_y > 4095:
                    prev_input_y = 4095
                if prev_input_y < 0:
                    prev_input_y = 0

                # 以前の顔の位置を更新
                prev_x = face_x
                prev_y = face_y

            # stream.arrayをウインドウに表示
            cv2.imshow('frame', stream.array)

            # "q"を入力でアプリケーション終了
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # streamをリセット
            stream.seek(0)
            stream.truncate()

        cv2.destroyAllWindows()
