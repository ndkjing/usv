import cv2
import numpy as np


class KalmanFilter:
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)


    def predict(self, coordX, coordY):
        ''' This function estimates the position of the object'''
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        x, y = int(predicted[0]), int(predicted[1])
        return x, y
kf = KalmanFilter()

img = cv2.imread(r"F:\downloads\Pysource-Kalman-filter\Pysource Kalman filter\blue_background.webp")

ball1_positions = [(50, 100), (100, 100), (150, 100), (200, 100), (250, 100), (300, 100), (350, 100), (400, 100), (450, 100)]

ball2_positions = [(4, 300), (61, 256), (116, 214), (170, 180), (225, 148), (279, 120), (332, 97),
         (383, 80), (434, 66), (484, 55), (535, 49), (586, 49), (634, 50),
         (683, 58), (731, 69), (778, 82), (824, 101), (870, 124), (917, 148),
         (962, 169), (1006, 212), (1051, 249), (1093, 290)]
i=0
pre_predict=[]
for pt in ball2_positions:
    cv2.circle(img, pt, 15, (0, 20, 220), -1)
    if i>15:
        predicted = kf.predict(pre_predict[0], pre_predict[1])
    else:
        predicted = kf.predict(pt[0], pt[1])
    i +=1
    pre_predict=predicted
    cv2.circle(img, predicted, 15, (20, 220, 0), 4)

# for i in range(10):
#     predicted = kf.predict(predicted[0], predicted[1])
#     cv2.circle(img, predicted, 15, (20, 220, 0), 4)
#
#     print(predicted)


cv2.imshow("Img", img)
cv2.waitKey(0)