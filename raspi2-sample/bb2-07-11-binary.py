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

with picamera.PiCamera() as camera:
    with picamera.array.PiRGBArray(camera) as stream:
        camera.resolution = (w, h)
        camera.framerate = 15

        while True:
            # stream.arrayにBGRの順で映像データを格納
            camera.capture(stream, 'bgr', use_video_port=True)
            # 映像データをグレースケール画像grayに変換
            gray = cv2.cvtColor(stream.array, cv2.COLOR_BGR2GRAY)
            # 画像の二値化 (閾値127)
            (ret, binary) = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
            # 画像の二値化 (大津の方法)
            #(ret, binary) = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            # retの表示
            print(ret)
            # pygameで画像を表示
            bin = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
            pygame_imshow(bin)

            # "q"を入力でアプリケーション終了
            for e in pygame.event.get():
                if e.type == pygame.KEYDOWN:
                    if e.key == pygame.K_q:
                        pygame.quit()
                        sys.exit()

            # streamをリセット
            stream.seek(0)
            stream.truncate()
