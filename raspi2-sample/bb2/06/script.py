import webiopi
from time import sleep
import smbus
import math

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

def getPCA9685DutyForWebIOPi(id, val):
    val_min = 0.0
    val_max = 1.0
    servo_min = 143 # 50Hzで0.7ms
    servo_max = 410 # 50Hzで2.0ms (中心は276)
    if id==1 :
        servo_min = 193 # 50Hzで0.95ms
        servo_max = 360 # 50Hzで1.8ms
    duty = (servo_min-servo_max)*(val-val_min)/(val_max-val_min) + servo_max
    # 一般的なサーボモーターはこちらを有効に
    #duty = (servo_max-servo_min)*(val-val_min)/(val_max-val_min) + servo_min
    return int(duty)

bus = smbus.SMBus(1)
address_pca9685 = 0x40

resetPCA9685()
setPCA9685Freq(50)

setPCA9685Duty(0, 0, 276)
setPCA9685Duty(1, 0, 276)

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
def setPCA9685PWM(servoID, duty, commandID):
    id = int(servoID)
    duty = getPCA9685DutyForWebIOPi(id, float(duty))
    setPCA9685Duty(id, 0, duty)

