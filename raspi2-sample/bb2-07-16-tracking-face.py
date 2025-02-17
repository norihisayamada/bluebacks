# -*- coding: utf-8 -*-
import picamera
import picamera.array
import cv2
import math
import wiringpi
import pygame
import sys

(w, h)=(320, 240)

pygame.init()
size=(w, h)
screen = pygame.display.set_mode(size)

def pygame_imshow(array):
    b,g,r = cv2.split(array)
    rgb = cv2.merge([r,g,b])
    surface1 = pygame.surfarray.make_surface(rgb)       
    surface2 = pygame.transform.rotate(surface1, -90)
    surface3 = pygame.transform.flip(surface2, True, False)
    screen.blit(surface3, (0,0))
    pygame.display.flip()

cascade_path =  "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml"
import os
if not os.path.isfile(cascade_path):
    cascade_path =  "/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml"
cascade = cv2.CascadeClassifier(cascade_path)

def getServoDutyHw(id, val):
    val_min = 0
    val_max = 4095
    # デューティ比0%を0、100%を1024として数値を入力
    servo_min = 36   # 50Hz(周期20ms)、デューティ比3.5%: 3.5*1024/100=約36
    servo_max = 102  # 50Hz(周期20ms)、デューティ比10%: 10*1024/100=約102
    if id==1:
        servo_min = 53
        servo_max = 85
    duty = int((servo_min-servo_max)*(val-val_min)/(val_max-val_min) + servo_max)
    # 一般的なサーボモーターはこちらを有効に
    #duty = int((servo_max-servo_min)*(val-val_min)/(val_max-val_min) + servo_min)
    if duty > servo_max:
        duty = servo_max
    if duty < servo_min:
        duty = servo_min
    return duty

PWM0 = 18
PWM1 = 19

# wiringPiによるハードウェアPWM
wiringpi.wiringPiSetupGpio() # GPIO名で番号を指定する
wiringpi.pinMode(PWM0, wiringpi.GPIO.PWM_OUTPUT) # 左右方向のPWM出力を指定
wiringpi.pinMode(PWM1, wiringpi.GPIO.PWM_OUTPUT) # 上下方向のPWM出力を指定
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS) # 周波数を固定するための設定
wiringpi.pwmSetClock(375) # 50 Hz。18750/(周波数) の計算値に近い整数
# PWMのピン番号とデフォルトのパルス幅をデューティ100%を1024として指定。
# ここでは6.75%に対応する69を指定
wiringpi.pwmWrite(PWM0, 69)
wiringpi.pwmWrite(PWM1, 69)

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

                duty0 = getServoDutyHw(0, int(ratio_x*dx) + prev_input_x)
                wiringpi.pwmWrite(PWM0, duty0)

                duty1 = getServoDutyHw(1, int(ratio_y*dy) + prev_input_y)
                wiringpi.pwmWrite(PWM1, duty1)

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

            # pygameで画像を表示
            pygame_imshow(stream.array)

            # "q"を入力でアプリケーション終了
            for e in pygame.event.get():
                if e.type == pygame.KEYDOWN:
                    if e.key == pygame.K_q:
                        pygame.quit()
                        sys.exit()

            # streamをリセット
            stream.seek(0)
            stream.truncate()
