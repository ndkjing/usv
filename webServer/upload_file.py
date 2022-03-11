import time

import requests
import json


def post_data(url, file, id=0):
    """
        :param url: 接口url
        :param file: 上传文件的路径
        :return:
        """
    files = {"file": open(file, "rb")}
    s = requests.session()
    if id == 1:
        r = s.post(url, params={"id": 1}, files=files, verify=False)
    else:
        r = s.post(url, files=files, verify=False)
    r_json = r.json()
    print('r', r_json)
    if r_json.get('success'):
        return r_json.get('data').get('picName')
    else:
        return False


def post_descip(data, url, request_type='POST'):
    # 请求头设置
    payload_header = {
        'Content-Type': 'application/json',
    }
    assert request_type in ['POST', 'GET']
    if request_type == 'POST':
        dump_json_data = json.dumps(data)
        return_data = requests.post(
            url=url, data=dump_json_data, headers=payload_header)
    else:
        return_data = requests.get(url=url)
    print('return_data.json()', return_data.json())


if __name__ == "__main__":
    ip_local = '192.168.8.26:8009'
    ip_xxl = 'ship.xxlun.com/'
    url_data = "http://%s/union/admin/uploadFile" % ip_local
    url_descpi = "http://%s/union/admin/xxl/data/monitoring" % ip_local
    # file = "weixin.jpg"
    file = "../a001.png"
    # file = "F:\downloads\SampleVideo_1280x720_5mb.mp4"
    save_name = post_data(url=url_data, file=file)
    time.sleep(10000)
    if save_name:
        print('save_name', save_name)
        send_data = {
            "details": "测试上传",
            "deviceId": "3c50f4c3-a9c1-4872-9f18-883af014380b",
            "fileUrl": save_name,
            "jwd": "114.000012,30.134500",
            "mapId": "1428633390148206594"
        }
        print('send_data', send_data)
        post_descip(data=send_data, url=url_descpi)
