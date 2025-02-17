# -*- coding: utf-8 -*-
import requests
import json

try:
    unicode # python2
    def uenc(str): return str.encode('utf-8')
    pass
except: # python3
    def uenc(str): return str
    pass

location = 130010  # 東京
proxies = None

# proxy環境下の方は下記を記述して有効に
#proxies = {
#        'http': 'プロキシサーバ名:ポート番号',
#        'https': 'プロキシサーバ名:ポート番号'
#}

weather_json = requests.get(
    'http://weather.livedoor.com/forecast/webservice/json/v1?city={0}'.format(location), proxies=proxies, timeout=5
).json()

print(uenc(json.dumps(weather_json, indent=2, ensure_ascii=False)))
