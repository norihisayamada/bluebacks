# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
from time import sleep
import smbus
import math

# MCP3208からSPI通信で12ビットのデジタル値を取得。0から7の8チャンネル使用可
def readadc(adcnum, clockpin, mosipin, misopin, cspin):
    if adcnum > 7 or adcnum < 0:
        return -1
    GPIO.output(cspin, GPIO.HIGH)
    GPIO.output(clockpin, GPIO.LOW)
    GPIO.output(cspin, GPIO.LOW)

    commandout = adcnum
    commandout |= 0x18  # スタートビット＋シングルエンドビット
    commandout <<= 3    # LSBから8ビット目を送信するようにする
    for i in range(5):
        # LSBから数えて8ビット目から4ビット目までを送信
        if commandout & 0x80:
            GPIO.output(mosipin, GPIO.HIGH)
        else:
            GPIO.output(mosipin, GPIO.LOW)
        commandout <<= 1
        GPIO.output(clockpin, GPIO.HIGH)
        GPIO.output(clockpin, GPIO.LOW)
    adcout = 0
    # 13ビット読む（ヌルビット＋12ビットデータ）
    for i in range(13):
        GPIO.output(clockpin, GPIO.HIGH)
        GPIO.output(clockpin, GPIO.LOW)
        adcout <<= 1
        if i>0 and GPIO.input(misopin)==GPIO.HIGH:
            adcout |= 0x1
    GPIO.output(cspin, GPIO.HIGH)
    return adcout

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
    return int(duty)

GPIO.setmode(GPIO.BCM)
# ピンの名前を変数として定義
SPICLK = 11
SPIMOSI = 10
SPIMISO = 9
SPICS = 8
# SPI通信用の入出力を定義
GPIO.setup(SPICLK, GPIO.OUT)
GPIO.setup(SPIMOSI, GPIO.OUT)
GPIO.setup(SPIMISO, GPIO.IN)
GPIO.setup(SPICS, GPIO.OUT)

bus = smbus.SMBus(1)
address_pca9685 = 0x40

resetPCA9685()
setPCA9685Freq(50)

adc_pin0 = 0
adc_pin1 = 1

try:
    while True:
        inputVal0 = readadc(adc_pin0, SPICLK, SPIMOSI, SPIMISO, SPICS)
        duty0 = getPCA9685Duty(0, inputVal0)
        setPCA9685Duty(0, 0, duty0)

        inputVal1 = readadc(adc_pin1, SPICLK, SPIMOSI, SPIMISO, SPICS)
        duty1 = getPCA9685Duty(1, inputVal1)
        setPCA9685Duty(1, 0, duty1)

        sleep(0.2)

except KeyboardInterrupt:
    pass

GPIO.cleanup()
