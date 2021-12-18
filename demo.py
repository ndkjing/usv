import requests
response = requests.get("https://ezviz-fastdfs-gateway.oss-cn-hangzhou.aliyuncs.com/1/capture/003cw4oR4El6u8Yp6bhrMmVbR6lzjAU.jpg?Expires=1639907840&OSSAccessKeyId=LTAIzI38nEHqg64n&Signature=HPj8ELSodrsBIn7Vm4bz0298zH0%3D")
# 获取的文本实际上是图片的二进制文本
img = response.content
# 将他拷贝到本地文件 w 写  b 二进制  wb代表写入二进制文本
with open('test.png', 'wb') as f:
    f.write(img)