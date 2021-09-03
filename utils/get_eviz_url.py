import json
import time
import requests
from storage import save_data
import ship_code_config
# token_global_dict = {}  # {'设备序列号':[token ,过期时间]}


def get_access_toke(data, url, request_type='POST'):
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
    return return_data


def get_video_url(serial_str, token, protocol=2):
    payload_header = {
        # 'Content-Type': 'application/json',
        'Content-Type': 'application/x-www-form-urlencoded',
    }
    url = 'https://open.ys7.com/api/lapp/v2/live/address/get?accessToken=%s&deviceSerial=%s&channelNo=1&protocol=%s&quality=2' % (
        token, serial_str, protocol)
    return_data = requests.post(
        url=url, headers=payload_header)
    return return_data


def get_url(serial_str, protocol=2):
    """
    获取和验证播放地址
    流播放协议，1-ezopen、2-hls、3-rtmp、4-flv，默认为1
    """
    token_global_dict = {}
    save_token_dict = save_data.get_data(ship_code_config.save_token_path)
    if save_token_dict:
        token_global_dict.update(save_token_dict)
    url = None
    if token_global_dict.get(serial_str) is None or token_global_dict.get(serial_str)[1] < time.time():
        token_data = get_access_toke(
            data={'appKey': '1c7ea7dcea734a239a528fa458568f48', 'appSecret': '7efe513b44b4f81fc5cb97a7ab5afe55'},
            url='https://open.ys7.com/api/lapp/token/get?appKey=1c7ea7dcea734a239a528fa458568f48&appSecret=7efe513b44b4f81fc5cb97a7ab5afe55')
        if int(token_data.json().get('code')) == 200:
            access_token = token_data.json().get('data').get('accessToken')
            expire_time = token_data.json().get('data').get('expireTime')
            token_global_dict.update({serial_str: [access_token, float(expire_time)]})
            save_data.set_data(token_global_dict, ship_code_config.save_token_path)
            url_data = get_video_url(serial_str, access_token, protocol=protocol)
            # 不为字符串则代表返回其他错误码
            if int(url_data.json().get("code")) in [200, 201]:
                url = url_data.json().get("data").get("url")
            else:
                print({'get video_url error': url_data.json()})
    else:
        access_token = token_global_dict.get(serial_str)[0]
        url_data = get_video_url(serial_str, access_token, protocol=protocol)
        # 不为字符串则代表返回其他错误码
        if int(url_data.json().get("code")) in [200, 201]:
            url = url_data.json().get("data").get("url")
        else:
            print({'get video_url error': url_data.json()})
    return url


if __name__ == '__main__':
    video_url = get_url(serial_str='F77671789')
    print('video_url', video_url)
