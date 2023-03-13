import time

import cv2
import threading
import os

rtmp_video_urls = [
    'rtmp://rtmp01open.ys7.com:1935/v3/openlive/L09226983_1_2?expire=1705041929&id=535469719541526528&t=8c1b91102f511029bb2b48ed3102030ad7ceb6831ba4872c074090e6b0fd8a39&ev=100',
    'rtmp://rtmp01open.ys7.com:1935/v3/openlive/L09226986_1_2?expire=1705041953&id=535469818225098752&t=688cb6c32fdc682afb22b4c58b10a9dded03ec907c6042a80fe127509898aebe&ev=100',
    # 'rtmp://rtmp01open.ys7.com:1935/v3/openlive/L09226976_1_2?expire=1705041972&id=535469900391522304&t=afaa3cee22ec20b67ae0ce101d88f43e22ad8e8d5b0343f0d743b87d2b04e6e7&ev=100',
    # 'rtmp://rtmp01open.ys7.com:1935/v3/openlive/L09226970_1_2?expire=1705041988&id=535469967382941696&t=7f57405e1c54373e398263f0b9a8f88966a87f23f6cb00ce3b1322c9eddf04c5&ev=100'

]

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
save_video_path = os.path.join(root_path, 'videos')
if not os.path.exists(save_video_path):
    os.mkdir(save_video_path)
for i in range(len(rtmp_video_urls)):
    save_video_path_sub = os.path.join(save_video_path, str(i))
    if not os.path.exists(save_video_path_sub):
        os.mkdir(save_video_path_sub)

def save_video(index):
    print('index:',index)
    # index = int(index)
    cap = cv2.VideoCapture(rtmp_video_urls[index])
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    str_time = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
    save_path = os.path.join(os.path.join(save_video_path, str(index)),'%d_%s.avi'%(index,str_time))
    print('save_path',save_path)
    out = cv2.VideoWriter(save_path, fourcc, 30.0, (704, 576))  # 图像大小参数按（宽，高）一定得与写入帧大小一致
    start_save_time = None
    while True:
        ret, frame = cap.read()
        if start_save_time is None:
            start_save_time = time.time()
        if time.time()-start_save_time>30: # 重新保存录像
            out.release()
            start_save_time = time.time()
            print('resave')
            str_time = time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime())
            save_path = os.path.join(os.path.join(save_video_path, str(index)), '%d_%s.avi' % (index, str_time))
            print('save_path', save_path)
            out = cv2.VideoWriter(save_path, fourcc, 30.0, (704, 576))
        out.write(frame)
        # cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    out.release()
    cv2.destroyAllWindows()

t_list = []
for item in range(len(rtmp_video_urls)):
    t_list.append(threading.Thread(target=save_video,args=(item,)))

for t in t_list:
    t.start()