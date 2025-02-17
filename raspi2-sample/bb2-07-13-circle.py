# -*- coding: utf-8 -*-
import picamera
import picamera.array
import cv2
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

version = cv2.__version__.split(".")
CVversion = int(version[0])

blurval = min(int(16*(w/640)), 24)
blurval = blurval + (1 - blurval%2)
mincdist = int(100*(w/640))
mincradius = int(10*(w/640))
maxcradius = int(200*(w/640))
with picamera.PiCamera() as camera:
    with picamera.array.PiRGBArray(camera) as stream:
        camera.resolution = (w, h)
        camera.framerate = 15

        while True:
            # stream.arrayにBGRの順で映像データを格納
            camera.capture(stream, 'bgr', use_video_port=True)
            # 映像データをグレースケール画像grayに変換
            gray = cv2.cvtColor(stream.array, cv2.COLOR_BGR2GRAY)
            # ガウシアンぼかしを適用して、認識精度を上げる
            blur = cv2.GaussianBlur(gray, (blurval,blurval), 0)
            # ハフ変換を適用し、映像内の円を探す
            if CVversion == 2:
                circles = cv2.HoughCircles(blur, cv2.cv.CV_HOUGH_GRADIENT,
                      dp=1, minDist=mincdist, param1=120, param2=40,
                      minRadius=mincradius, maxRadius=maxcradius)
            else:
                circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT,
                      dp=1, minDist=mincdist, param1=120, param2=40,
                      minRadius=mincradius, maxRadius=maxcradius)

            if circles is not None:
                for c in circles[0]:
                    # 見つかった円の上に赤い円を元の映像(stream.array)上に描画
                    # c[0]:x座標, c[1]:y座標, c[2]:半径
                    cv2.circle(stream.array, (int(c[0]),int(c[1])), int(c[2]), (0,0,255), 2)

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
