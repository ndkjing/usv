import json
import requests


def get_app_key(data, url, request_type='POST'):
    # 请求头设置
    payload_header = {
        # 'Content-Type': 'application/json',
        'Content-Type': 'application/x-www-form-urlencoded',

    }
    assert request_type in ['POST', 'GET']
    if request_type == 'POST':
        dump_json_data = json.dumps(data)
        return_data = requests.post(
            url=url, data=dump_json_data, headers=payload_header)
    else:
        return_data = requests.get(url=url)
    print('return_data.json()', return_data.json())
    return return_data.json().get('data').get('accessToken')

def get_url(serial_str):
    """

    """
    payload_header = {
        # 'Content-Type': 'application/json',
        'Content-Type': 'application/x-www-form-urlencoded',

    }
    app_key = get_app_key(data={'appKey': '1c7ea7dcea734a239a528fa458568f48', 'appSecret': '7efe513b44b4f81fc5cb97a7ab5afe55'},
                url='https://open.ys7.com/api/lapp/token/get?appKey=1c7ea7dcea734a239a528fa458568f48&appSecret=7efe513b44b4f81fc5cb97a7ab5afe55')
    url = 'https://open.ys7.com/api/lapp/v2/live/address/get?accessToken=%s&deviceSerial=%s&channelNo=1&protocol=3&quality=2'%(app_key,serial_str)
    return_data = requests.post(
        url=url, headers=payload_header)
    print('return_data.json()', return_data.json())
    return return_data.json().get('data').get('url')


if __name__ == '__main__':
    video_url = get_url(serial_str='F77671789')
    print('video_url',video_url)
