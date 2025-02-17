import webiopi
import time
import subprocess

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

# 自作のマクロ。JavaScriptから呼ぶことができる
@webiopi.macro
def sendCommand(group, command):
    args = ['irsend', '-#', '1', 'SEND_ONCE', group, command]
    subprocess.Popen(args)

