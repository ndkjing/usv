import time
from subprocess import run, PIPE
import config
cnt=0

def check_network():
    r = run('ping www.baidu.com',
            stdout=PIPE,
            stderr=PIPE,
            stdin=PIPE,
            shell=True)
    if r.returncode:
        return False
    else:
        return True

if __name__ == '__main__':
    while 1:
        start_time = time.time()
        print("network: ", check_network())
        print('cost time:',time.time()-start_time)
        # time.sleep(config.check_network_interval)