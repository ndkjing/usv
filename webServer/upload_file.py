import requests
import json


def post_data(url, file):
    """
        :param url: 接口url
        :param file: 上传文件的路径
        :return:
        """
    files = {"file": open(file, "rb")}
    s = requests.session()
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
    url_data = "http://192.168.199.186:8009/union/admin/uploadFile"
    url_descpi = "http://192.168.199.186:8009/union/admin/xxl/data/monitoring"
    # file = "weixin.jpg"
    file = "demo.png"
    save_name = post_data(url=url_data, file=file)
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
