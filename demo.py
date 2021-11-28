import requests

# res = requests.get(url='https://api.binance.com/api/v3/ticker/24hr',timeout=10,proxies={'http':'127.0.0.1:10809','https':'127.0.0.1:10809'})
# print(res.content)
# print(res)



import requests
import urllib3
print(urllib3.__version__)
res = requests.get(url='https://api.binance.com/api/v3/ticker/24hr', timeout=10, proxies={'http': '127.0.0.1:10809', 'https': '127.0.0.1:10809'})
print(res.text)
