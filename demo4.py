from scipy.stats import norm
mindis = -4.1
std = 1
a = norm.cdf(mindis, 0.0, std)
print('w,a,s,d 为前后左右，q为停止\n'
      'r,t 左右抽水泵\n'
      'y,u,i,o,p 摄像头舵机\n'
      'f  测距\n'
      ''
      '')