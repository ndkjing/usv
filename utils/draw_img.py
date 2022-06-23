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

headers = {
    'Content-Type': 'application/json',
}


def get_img_url(url, data, token=None):
    m = 2
    img_url = None
    if m == 1:
        dump_json_data = json.dumps(data)
        headers_payload = {
            'Content-Type': 'application/json',
        }
        if token:
            headers_payload.update({"token": token})
        print('headers_payload', headers_payload)
        return_data = requests.post(
            url=url, data=dump_json_data, headers=headers_payload, timeout=8)
    else:
        if isinstance(data, dict):
            dump_json_data = data
        else:
            dump_json_data = json.dumps(data)
        headers_payload = {
            'Content-Type': 'application/json',
        }
        if token:
            headers_payload.update({"token": token})
        return_data = requests.post(
            url=url, params=dump_json_data, headers=headers_payload, timeout=8)
    json_data = json.loads(return_data.content)
    print('获取图片地址返回数据:', json_data)
    if json_data and json_data.get("code") == 200:
        img_url = json_data.get("data")
        if isinstance(img_url, str) and 'http' in img_url:
            return img_url


def post_file(url, file_path, file_name=None, token=None):
    """
    :param url: 上传的服务器地址
    :param file_path: 文件路径
    :param file_name: 文件名称（上传到服务端文件即为这个名称， 不管原文件名称）
    :return: 服务器返回的内容
    """
    try:
        headers_ = {}
        if token:
            headers_.update({"token": token})
        response = requests.post(url=url, headers=headers_, files={'file': (file_path, open(file_path, "rb"))})
        return_data = json.loads(response.content)
        print(time.time(), '上传图片response', return_data)
        if return_data and return_data.get("code") == 200:
            server_save_img_path = return_data.get("data").get("picName")
        else:
            server_save_img_path = None
        print('server_save_img_path', server_save_img_path)
        return server_save_img_path
    except Exception as e:
        print('error', e)


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


def all_throw_img(http_get_img_path, http_upload_img, ship_code, add_info=None, token=None):
    print(http_get_img_path, http_upload_img, ship_code, add_info)
    img_url = get_img_url(http_get_img_path, {"deviceId": ship_code}, token)  # type 0 采样获取图片  1报警获取图片
    if img_url:
        img_path = 'temp.png'
        if add_info is None:
            add_info = [[114.123412, 31.112345], 1, 0.5, 5000]
        save_img(img_url, img_path)
        if os.path.exists(img_path):
            add_img_info(img_path, add_info=add_info)
            server_save_img_path = post_file(url=http_upload_img, file_path="image_text.jpg", file_name=None,
                                             token=token)
            return server_save_img_path


if __name__ == "__main__":
    pass
    # img_url = get_img_url(config.http_get_img_path, {"deviceId": config.ship_code})
    # if img_url:
    #     img_path = 'temp.png'
    #     add_info = [[114.123412, 31.112345], 1, 0.5, 5000]
    #     save_img(img_url, img_path)
    #     if os.path.exists(img_path):
    #         add_img_info(img_path, add_info=add_info)
    #         post_file(url=config.http_upload_img, file_path=img_path, file_name=None)
