import time

import requests
from urllib3 import encode_multipart_formdata
import json
import os
import cv2

"""
用于保存抽水时上传数据
"""
# import config

# headers = {
#     'Content-Type': 'application/json',
# }
headers = {}


def get_img_url(url, data, token=None):
    m = 2
    img_url = None
    if m == 1:
        dump_json_data = json.dumps(data)
        return_data = requests.post(
            url=url, data=dump_json_data, headers=headers, timeout=8)
    else:
        if token:
            headers.update({'token': token})
        if isinstance(data, dict):
            dump_json_data = data
        else:
            dump_json_data = json.dumps(data)
        print('url', url, headers)
        try:
            return_data = requests.post(
                url=url, params=dump_json_data, headers=headers, timeout=8)
            json_data = json.loads(return_data.content)
            # print('请求图片地址返回数据：', json_data)
            # print('json_data', json_data)
            # print('json_data.get("success")', json_data.get("success"))
            if json_data or json_data.get("code") in [200, 20000]:
                if json_data.get("data"):
                    data_list = json_data.get("data").get('data')
                    return data_list
        except Exception as e:
            print('error', e)



def post_file(url, file_path, file_name=None, token=None):
    """
    :param url: 上传的服务器地址
    :param file_path: 文件路径
    :param file_name: 文件名称（上传到服务端文件即为这个名称， 不管原文件名称）
    :return: 服务器返回的内容
    """
    """

    # 读取文件内容
    file = open(file_path, "rb")
    file_content = file.read()
    file.close()
    # 准备请求体
    data = dict()
    # 处理文件名字
    if not file_name:
        file_name_list = file_path.rsplit("/", 1)  # 加入未给文件重新命名，使用文件原名称
        print(file_name_list)
        if len(file_name_list)>1:
            file_name = file_name_list[1]
        else:
            file_name = file_path
    data['file'] = (file_name, file_content)
    encode_data = encode_multipart_formdata(data)
    data = encode_data[0]
    headers['Content-Type'] = encode_data[1]
    # 发送post请求
    try:
        print(time.time(),'上传图片')
        response = requests.post(url=url, headers=headers, data=data)
    except Exception as e:
        print('error', e)
        response = None
    """
    headers = {}
    if token:
        headers.update({'token': token})
    try:
        print(time.time(), '上传图片')
        response = requests.post(url=url, headers=headers, files={'file': (file_path, open(file_path, "rb"))})
    except Exception as e:
        print('error', e)
        response = None
    print(time.time(), '上传图片response', json.loads(response.content))
    return_data = json.loads(response.content)
    if return_data or return_data.get("code") in [200, 20000]:
        if return_data.get("data"):
            server_save_img_path = return_data.get("data").get("picName")
        else:
            server_save_img_path = None
    else:
        server_save_img_path = None
    print('server_save_img_path', server_save_img_path)
    return server_save_img_path


def save_img(url, save_path):
    """
    保存图片
    @param url:
    @param save_path:
    @return:
    """
    print({'url': url})
    response = requests.get(url)
    # 获取的文本实际上是图片的二进制文本
    img = response.content
    # 将他拷贝到本地文件 w 写  b 二进制  wb代表写入二进制文本
    with open(save_path, 'wb') as f:
        f.write(img)


def add_img_info(save_path, add_info: []):
    """
    对图片添加水印
    @param save_path:
    @param add_info:
    @return:
    """
    if os.path.exists(save_path):
        img = cv2.imread(save_path)
        font = cv2.FONT_HERSHEY_SIMPLEX
        print(img.shape, )
        for index, data in enumerate(add_info):
            if index == 0:
                cv2.putText(img, 'gps:' + str(data), (img.shape[1] - 400, 50), font, 0.7, (0, 0, 200), 1, cv2.LINE_AA)
            if index == 1:
                cv2.putText(img, 'bottle:' + str(data), (img.shape[1] - 400, 80), font, 0.7, (0, 0, 200), 1,
                            cv2.LINE_AA)
            if index == 2:
                cv2.putText(img, 'deep:' + str(data) + 'm', (img.shape[1] - 400, 110), font, 0.7, (0, 0, 200), 1,
                            cv2.LINE_AA)
            if index == 3:
                cv2.putText(img, 'capacity:' + str(data) + 'ml', (img.shape[1] - 400, 140), font, 0.7, (0, 0, 200), 1,
                            cv2.LINE_AA)
            cv2.imwrite('image_text.jpg', img)


def all_throw_img(http_get_img_path, http_upload_img, ship_code, add_info=None):
    print(http_get_img_path, http_upload_img, ship_code, add_info)
    time1 = time.time()
    img_url = get_img_url(http_get_img_path, {"deviceId": ship_code})
    print(time.time() - time1)
    if img_url:
        img_path = 'temp.png'
        if add_info is None:
            add_info = [[114.123412, 31.112345], 1, 0.5, 5000]
        save_img(img_url, img_path)
        print(time.time() - time1)
        if os.path.exists(img_path):
            add_img_info(img_path, add_info=add_info)
            print(time.time() - time1)
            server_save_img_path = post_file(url=http_upload_img, file_path="image_text.jpg", file_name=None)
            print(time.time() - time1)
            return server_save_img_path


# 发送数据到服务器http
def send_server_http_data(request_type, data, url, token=None):
    # 请求头设置
    payloadHeader = {
        'Content-Type': 'application/json',
    }
    if token:
        payloadHeader.update({'token': token})
    assert request_type in ['POST', 'GET']
    if request_type == 'POST':
        dumpJsonData = json.dumps(data)
        return_data = requests.post(
            url=url, data=dumpJsonData, headers=payloadHeader)
    else:
        return_data = requests.get(url=url)
    return return_data


if __name__ == "__main__":
    import config

    img_url = get_img_url(config.http_get_img_path, {"deviceId": config.ship_code})
    if img_url:
        img_path = 'temp.png'
        add_info = [[114.123412, 31.112345], 1, 0.5, 5000]
        save_img(img_url, img_path)
        if os.path.exists(img_path):
            add_img_info(img_path, add_info=add_info)
            post_file(url=config.http_upload_img, file_path=img_path, file_name=None)
