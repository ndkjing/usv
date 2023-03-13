import requests
from urllib3 import encode_multipart_formdata
import json
import os
import cv2

import server_config

headers = {
    'Content-Type': 'application/json',
}


def get_status(url, devicd_id):
    """
    :param url: 上传的服务器地址
    :return: 服务器返回的内容
    """
    # 发送post请求
    try:
        url = url + "?deviceId=%s" % devicd_id
        response = requests.get(url=url, headers=headers)
        # print('获取状态', response, response.content)
    except Exception as e:
        print('error', e)
        response = None
    if response:
        return_json = json.loads(response.content)
        if return_json.get("data"):
            return int(return_json.get("data").get("state"))


def send_status(url, data,token=None):
    """
    :param url: 上传的服务器地址
    :param data: 数据
    :return: 服务器返回的内容
    """
    # 请求头设置
    payload_header = {
        'Content-Type': 'application/json',
    }
    if token:
        payload_header.update({"token": token})
    # 发送post请求
    if isinstance(data, dict):
        dump_json_data = data
    else:
        dump_json_data = json.dumps(data)
    try:
        print('上传船状态数据', dump_json_data)
        response = requests.post(
            url=url, data=json.dumps(dump_json_data), headers=payload_header, timeout=8)
        print('上传状态返回', response, response.content)
    except Exception as e:
        print('error', e)
        return None
    if response:
        return_json = json.loads(response.content)
        if return_json.get("code") in [200,20000]:
            return 1  #正常上传
        elif return_json.get("code") == 401:
            return 2 # 重新获取token
    else:
        return None

# 发送数据到服务器http
def send_server_http_data(request_type, data, url, parm_type=1, token=None):
    """
    @param request_type:
    @param data:
    @param url:
    @param parm_type: 1 data 方式  2 params 方式
    @return:
    """
    try:
        # 请求头设置
        payload_header = {
            'Content-Type': 'application/json',
        }
        if token:
            payload_header.update({"token": token})
        assert request_type in ['POST', 'GET']
        if request_type == 'POST':
            if parm_type == 1:
                dump_json_data = json.dumps(data)
                return_data = requests.post(
                    url=url, data=dump_json_data, headers=payload_header, timeout=20)
            else:
                if isinstance(data, dict):
                    dump_json_data = data
                else:
                    dump_json_data = json.dumps(data)
                return_data = requests.post(
                    url=url, params=dump_json_data, headers=payload_header, timeout=20)
        else:
            if data:
                dump_json_data = json.dumps(data)
                return_data = requests.get(url=url, headers=payload_header, params=dump_json_data, timeout=20)
            else:
                return_data = requests.get(url=url, headers=payload_header, timeout=20)
            print('http返回数据', return_data)
        return return_data
    except Exception as e:
        print({'http请求报错': e})
        return None


def get_deep_date(request_type, data, url, parm_type=1, token=None):
    try:
        # 请求头设置
        payload_header = {
            'Content-Type': 'application/json',
        }
        if token:
            payload_header.update({"token": token})
        assert request_type in ['POST', 'GET']
        if request_type == 'POST':
            if parm_type == 1:
                dump_json_data = json.dumps(data)
                return_data = requests.post(
                    url=url, data=dump_json_data, headers=payload_header, timeout=20)
            else:
                if isinstance(data, dict):
                    dump_json_data = data
                else:
                    dump_json_data = json.dumps(data)
                return_data = requests.post(
                    url=url, params=dump_json_data, headers=payload_header, timeout=20)
        else:
            if data:
                dump_json_data = json.dumps(data)
                return_data = requests.get(url=url, headers=payload_header, params=dump_json_data, timeout=20)
            else:
                return_data = requests.get(url=url, headers=payload_header, timeout=20)
            print('http返回数据', return_data)
        return return_data
    except Exception as e:
        print({'http请求报错': e})
        return None

def send_deep_date(request_type, data, url, parm_type=1, token=None):
    try:
        # 请求头设置
        payload_header = {
            'Content-Type': 'application/json',
        }
        if token:
            payload_header.update({"token": token})
        assert request_type in ['POST', 'GET']
        if request_type == 'POST':
            if parm_type == 1:
                dump_json_data = json.dumps(data)
                return_data = requests.post(
                    url=url, data=dump_json_data, headers=payload_header, timeout=20)
            else:
                if isinstance(data, dict):
                    dump_json_data = data
                else:
                    dump_json_data = json.dumps(data)
                return_data = requests.post(
                    url=url, params=dump_json_data, headers=payload_header, timeout=20)
        else:
            if data:
                dump_json_data = json.dumps(data)
                return_data = requests.get(url=url, headers=payload_header, params=dump_json_data, timeout=20)
            else:
                return_data = requests.get(url=url, headers=payload_header, timeout=20)
            print('http返回数据  send_deep_date', return_data,json.loads(return_data.content))
        return return_data
    except Exception as e:
        print({'http请求报错': e})
        return None
if __name__ == "__main__":
    # response = post_file("http://192.168.8.26:8009/union/admin/uploadFile", "./webServer/demo.png")
    # print('res',response.content)
    import config

    # from webServer import server_config
    pass
    # get_status(url=server_config.http_get_ship_status, devicd_id="XXLJC4LCGSCSD1DA002")
    # send_status(url=server_config.http_set_ship_status, data={"deviceId": 'XXLJC4LCGSCSD1DA002', "state": "0"})
