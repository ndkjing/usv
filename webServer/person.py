import cv2 as cv
import numpy as np
import time
import os,sys
sys.path.append(
    os.path.dirname(
        os.path.dirname(
            os.path.abspath(__file__))
        ))
import get_eviz_url
import upload_file


class DetectVideo:
    def __init__(self):
        self.last_save_time = None
        self.save_interval = 300  # 暂时设置保存间隔时间300秒
        self.max_save_interval = 5 * 60 * 60  # 暂时设置最大保存间隔时间5*60*60秒
        ip_local = '192.168.199.186:8009'
        ip_xxl = 'wuhanligong.xxlun.com'
        self.url_data = "http://%s/union/admin/uploadFile" % ip_xxl
        self.url_descpi = "http://%s/union/admin/xxl/data/monitoring" % ip_xxl

    @staticmethod
    def is_inside(o, i):
        ox, oy, ow, oh = o
        ix, iy, iw, ih = i
        return ox > ix and oy > iy and ox + ow < ix + iw and oy + oh < iy + ih

    @staticmethod
    def draw_person(image, person):
        x, y, w, h = person
        cv.rectangle(image, (x, y), (x + w, y + h), (255, 0, 255), 2)

    @staticmethod
    def detect_img(src=None):
        if isinstance(src, str):
            img = cv.imread("person.png")
        elif isinstance(src, np.array):
            img = src
        else:
            return
        DetectVideo.detect(img)

    @staticmethod
    def detect(img):
        hog = cv.HOGDescriptor()
        hog.setSVMDetector(cv.HOGDescriptor_getDefaultPeopleDetector())
        found, w = hog.detectMultiScale(img)
        found_filtered = []
        flag = False
        for ri, r in enumerate(found):
            for qi, q, in enumerate(found):
                if ri != qi and DetectVideo.is_inside(r, q):
                    break
                else:
                    found_filtered.append(r)
            for person in found_filtered:
                DetectVideo.draw_person(img, person)
                flag = True
        return flag
        # cv.imshow("people detection", img)
        # cv.waitKey(1)
        # cv.destroyAllWindows()

    def send_data(self, frame):
        cv.imwrite('demo.png', frame)
        self.last_save_time = time.time()

        # file = "weixin.jpg"
        file = "demo.png"
        save_name = upload_file.post_data(url=self.url_data, file=file)
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
            upload_file.post_descip(data=send_data, url=self.url_descpi)

    def detect_video(self, src=None, b_show=False):
        cap = cv.VideoCapture(src)
        while True:
            ret, frame = cap.read()
            if frame is None:
                continue
            b_person = self.detect(frame)
            # 如果检测到需要间隔一段时间上传数据，没有检测到也要过一段时间上传
            if b_person:
                if self.last_save_time is None:
                    self.send_data(frame)
                    self.last_save_time = time.time()
                elif time.time() - self.last_save_time > self.save_interval:
                    self.send_data(frame)
            else:
                if self.last_save_time is None:
                    self.last_save_time = time.time()
                elif time.time() - self.last_save_time > self.max_save_interval:
                    self.send_data(frame)
            if b_show:
                cv.imshow("people detection", frame)
                cv.waitKey(1)


if __name__ == '__main__':
    obj = DetectVideo()
    # detect_img(src="person.png")
    # 获取地址  (萤石云产品序列号)
    src_rtmp = get_eviz_url.get_url(serial_str='F77671789')
    # 检测视频流
    obj.detect_video(src=src_rtmp)
