import time

import requests
import json
import os
import cv2

"""
摄像头检测到异常情况时上传图片和数据
"""
# import config
headers = {
    'Content-Type': 'application/json',
}


def upload_info(url, data, method=2, send_abnormal=False):
    img_url = None
    if method == 1:
        dump_json_data = json.dumps(data)
        return_data = requests.post(
            url=url, data=dump_json_data, headers=headers, timeout=8)
    else:
        if isinstance(data, dict):
            dump_json_data = data
        else:
            dump_json_data = json.dumps(data)
        return_data = requests.post(
            url=url, params=dump_json_data, headers=headers, timeout=8)
    print('return_data', return_data, return_data.content)
    json_data = json.loads(return_data.content)
    if json_data and json_data.get("success"):
        if send_abnormal:
            return True
        else:
            img_url = json_data.get("data").get("url")
    return img_url


def post_file(url, file_path, file_name=None):
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
    try:
        print(time.time(), '上传图片')
        response = requests.post(url=url, files={'file': (file_path, open(file_path, "rb"))})
    except Exception as e:
        print('error', e)
        response = None
    print(time.time(), '上传图片response', response, response.content)
    return_data = json.loads(response.content)
    if return_data and return_data.get("success"):
        server_save_img_path = return_data.get("data").get("picName")
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


def all_abnormal_img(http_upload_img, http_send_abnormal,ship_code, img_path,add_info=None):
    server_save_img_path = post_file(url=http_upload_img, file_path=img_path, file_name=None)
    if server_save_img_path:
        data = {
            "deviceId": ship_code,
            "content": "测试",
            "picName": server_save_img_path
        }
        is_success = upload_info(url=http_send_abnormal, method=1, data=data, send_abnormal=True)
        print('is_success', is_success)
        return is_success


if __name__ == "__main__":
    import config
    img_path = 'demo.png'
    # server_save_img_path = post_file(url=config.http_upload_img, file_path=img_path, file_name=None)
    all_abnormal_img(config.http_upload_img,config.http_send_abnormal,ship_code=config.ship_code,img_path=img_path)
