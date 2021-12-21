import requests
from urllib3 import encode_multipart_formdata
import json
import os
import cv2

import config

headers = {
    'Content-Type': 'application/json',
}

def get_status(url, devicd_id):
    """
    :param url: 上传的服务器地址
    :param file_path: 文件路径
    :param file_name: 文件名称（上传到服务端文件即为这个名称， 不管原文件名称）
    :return: 服务器返回的内容
    """
    # 发送post请求
    try:
        url = url + "?deviceId=%s" % devicd_id
        response = requests.get(url=url, headers=headers)
        print('获取状态', response, response.content)
    except Exception as e:
        print('error', e)
        response = None
    if response:
        return_json = json.loads(response.content)
        print('return_json',return_json)
        if return_json.get("data"):
            return int(return_json.get("data").get("state"))


def send_status(url, data):
    """
    :param url: 上传的服务器地址
    :param file_path: 文件路径
    :param file_name: 文件名称（上传到服务端文件即为这个名称， 不管原文件名称）
    :return: 服务器返回的内容
    """
    # 发送post请求
    if isinstance(data, dict):
        dump_json_data = data
    else:
        dump_json_data = json.dumps(data)
    try:
        response = requests.post(
            url=url, params=dump_json_data, headers=headers, timeout=8)
        print('上传状态', response, response.content)
    except Exception as e:
        print('error', e)
        response=None
    if response:
        return_json = json.loads(response.content)
        if return_json.get("success"):
            return 1
    # return_data = json.loads(response.content)


if __name__ == "__main__":
    # response = post_file("http://192.168.8.26:8009/union/admin/uploadFile", "./webServer/demo.png")
    # print('res',response.content)
    import config
    from webServer import server_config
    r1 = get_status(url=server_config.http_get_ship_status,devicd_id="XXLJC4LCGSCSD1DA002")
    print('r1',r1)
    r2 = send_status(url=server_config.http_set_ship_status,data={"deviceId":'XXLJC4LCGSCSD1DA002',"state":"0"})
    print('r2', r2)