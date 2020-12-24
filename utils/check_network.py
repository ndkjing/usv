import time
from subprocess import run, PIPE
import config
cnt=0

def check_network():
    global cnt
    r = run('ping www.baidu.com',
            stdout=PIPE,
            stderr=PIPE,
            stdin=PIPE,
            shell=True)
    if r.returncode:
        # print('relogin 第{}次'.format(cnt))
        return False
    else:
        # print('正常联网')
        return True

if __name__ == '__main__':
    while 1:
        start_time = time.time()
        print("network: ", check_network())
        print('cost time:',time.time()-start_time)
        # time.sleep(config.check_network_interval)