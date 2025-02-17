import webiopi
import RPi.GPIO as GPIO
from time import sleep
import wiringpi

def getServoDutyHwForWebIOPi(id, val):
    val_min = 0
    val_max = 1
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

PWM0 = 18
PWM1 = 19

# 左右方向はwiringPiによるハードウェアPWMで
wiringpi.wiringPiSetupGpio() # GPIO名で番号を指定する
wiringpi.pinMode(PWM0, wiringpi.GPIO.PWM_OUTPUT) # 左右方向のPWM出力を指定
wiringpi.pinMode(PWM1, wiringpi.GPIO.PWM_OUTPUT) # 上下方向のPWM出力を指定
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS) # 周波数を固定するための設定
wiringpi.pwmSetClock(375) # 50 Hz。18750/(周波数) の計算値に近い整数
# PWMのピン番号とデフォルトのパルス幅をデューティ100%を1024として指定
# ここでは6.75%に対応する69を指定
wiringpi.pwmWrite(PWM0, 69) 
wiringpi.pwmWrite(PWM1, 69) 

# デバッグ出力を有効に
webiopi.setDebug()

# WebIOPiの起動時に呼ばれる関数
def setup():
    webiopi.debug("Script with macros - Setup")

# WebIOPiにより繰り返される関数
def loop():
    webiopi.sleep(5)

# WebIOPi終了時に呼ばれる関数
def destroy():
    webiopi.debug("Script with macros - Destroy")

@webiopi.macro
def setHwPWM(servoID, duty, commandID):
    id = int(servoID)
    duty = getServoDutyHwForWebIOPi(id, float(duty))
    if id==0:
        wiringpi.pwmWrite(PWM0,duty)
    else:
        wiringpi.pwmWrite(PWM1,duty)
