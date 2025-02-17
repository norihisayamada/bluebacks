# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
from time import sleep
import wiringpi

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
    return duty

GPIO.setmode(GPIO.BCM)
# ピンの名前を変数として定義(AD変換用)
SPICLK = 11
SPIMOSI = 10
SPIMISO = 9
SPICS = 8
# PWM用ピン
PWM0 = 18
PWM1 = 19

# SPI通信用の入出力を定義
GPIO.setup(SPICLK, GPIO.OUT)
GPIO.setup(SPIMOSI, GPIO.OUT)
GPIO.setup(SPIMISO, GPIO.IN)
GPIO.setup(SPICS, GPIO.OUT)

# wiringPiによるハードウェアPWM
wiringpi.wiringPiSetupGpio() # GPIO名で番号を指定する
wiringpi.pinMode(PWM0, wiringpi.GPIO.PWM_OUTPUT) # 左右方向PWM出力を指定
wiringpi.pinMode(PWM1, wiringpi.GPIO.PWM_OUTPUT) # 上下方向PWM出力を指定
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS) # 周波数を固定するための設定
wiringpi.pwmSetClock(375) # 50 Hz。18750/(周波数) の計算値に近い整数
# PWMのピン番号とデフォルトのパルス幅をデューティ100%を1024として指定
# ここでは6.75%に対応する69を指定
wiringpi.pwmWrite(PWM0, 69) 
wiringpi.pwmWrite(PWM1, 69) 

adc_pin0 = 0
adc_pin1 = 1

try:
    while True:
        inputVal0 = readadc(adc_pin0, SPICLK, SPIMOSI, SPIMISO, SPICS)
        duty0 = getServoDutyHw(0, inputVal0)
        wiringpi.pwmWrite(PWM0, duty0)

        inputVal1 = readadc(adc_pin1, SPICLK, SPIMOSI, SPIMISO, SPICS)
        duty1 = getServoDutyHw(1, inputVal1)
        wiringpi.pwmWrite(PWM1, duty1)

        sleep(0.2)

except KeyboardInterrupt:
    pass

GPIO.cleanup()
