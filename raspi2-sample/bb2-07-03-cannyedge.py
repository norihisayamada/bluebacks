# -*- coding: utf-8 -*-
import picamera
import picamera.array
import cv2

(w, h)=(320, 240)

with picamera.PiCamera() as camera:
    with picamera.array.PiRGBArray(camera) as stream:
        camera.resolution = (w, h)
        camera.framerate = 15

        while True:
            # stream.arrayにBGRの順で映像データを格納
            camera.capture(stream, 'bgr', use_video_port=True)
            # 映像データをグレースケール画像grayに変換
            gray = cv2.cvtColor(stream.array, cv2.COLOR_BGR2GRAY)
            # Cannyフィルタを適用し、結果をedgeに格納
            edge = cv2.Canny(gray, 50, 100)
            # edgeをウインドウに表示
            cv2.imshow('frame', edge)

            # "q"を入力でアプリケーション終了
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # streamをリセット
            stream.seek(0)
            stream.truncate()

        cv2.destroyAllWindows()
