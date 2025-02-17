# -*- coding: utf-8 -*-
import picamera
import picamera.array
import cv2

(w, h)=(320, 240)

cascade_path =  "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml"
import os
if not os.path.isfile(cascade_path):
    cascade_path =  "/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml"
cascade = cv2.CascadeClassifier(cascade_path)

minsize = int(60*(w/640))
maxsize = int(300*(w/640))
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
                for rect in facerect:
                    # 元の画像(stream.array)の顔がある位置赤い四角を描画
                    # rect[0:2]:長方形の左上の座標, rect[2:4]:長方形の横と高さ
                    # rect[0:2]+rect[2:4]:長方形の右下の座標
                    cv2.rectangle(stream.array, tuple(rect[0:2]),tuple(rect[0:2]+rect[2:4]), (0,0,255), thickness=2)

            # stream.arrayをウインドウに表示
            cv2.imshow('frame', stream.array)

            # "q"を入力でアプリケーション終了
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # streamをリセット
            stream.seek(0)
            stream.truncate()

        cv2.destroyAllWindows()
