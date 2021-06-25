import sys, os

sys.path.append(
    os.path.dirname(
        os.path.dirname(
            os.path.abspath(__file__))))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'drivers'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'externalConnect'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'messageBus'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'moveControl'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'statics'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'storage'))
sys.path.append(
    os.path.join(
        os.path.dirname(
            os.path.abspath(__file__)),
        'utils'))

import copy
from drivers import pi_main
from drivers import pi_softuart
from moveControl.pathTrack import simple_pid
import config
import math
import numpy as np
import threading
import pigpio
import json
import time
import tqdm


def nelder_mead(f, x_start,
                step=1.0, no_improve_thr=10e-6,
                no_improv_break=10, max_iter=0,
                alpha=1., gamma=2., rho=-0.5, sigma=0.5):
    '''
        @param f (function): function to optimize, must return a scalar score
            and operate over a numpy array of the same dimensions as x_start
        @param x_start (numpy array): initial position
        @param step (float): look-around radius in initial step
        @no_improv_thr,  no_improv_break (float, int): break after no_improv_break iterations with
            an improvement lower than no_improv_thr
        @max_iter (int): always break after this number of iterations.
            Set it to 0 to loop indefinitely.
        @alpha, gamma, rho, sigma (floats): parameters of the algorithm
            (see Wikipedia page for reference)
        return: tuple (best parameter array, best score)
    '''
    # init
    dim = len(x_start)
    prev_best = f(x_start)
    no_improv = 0
    res = [[x_start, prev_best]]

    for i in range(dim):
        x = copy.copy(x_start)
        x[i] = x[i] + step
        score = f(x)
        res.append([x, score])

    # simplex iter
    iters = 0
    while 1:
        print('x', x, 'res', res)
        res.sort(key=lambda x: x[1])
        best = res[0][1]

        # break after max_iter
        if max_iter and iters >= max_iter:
            return res[0]
        iters += 1

        # break after no_improv_break iterations with no improvement
        print('...best so far:', best)

        if best < prev_best - no_improve_thr:
            no_improv = 0
            prev_best = best
        else:
            no_improv += 1

        if no_improv >= no_improv_break:
            return res[0]

        # centroid
        x0 = [0.] * dim
        for tup in res[:-1]:
            for i, c in enumerate(tup[0]):
                x0[i] += c / (len(res) - 1)

        # reflection
        xr = x0 + alpha * (x0 - res[-1][0])
        rscore = f(xr)
        if res[0][1] <= rscore < res[-2][1]:
            del res[-1]
            res.append([xr, rscore])
            continue

        # expansion
        if rscore < res[0][1]:
            xe = x0 + gamma * (x0 - res[-1][0])
            escore = f(xe)
            if escore < rscore:
                del res[-1]
                res.append([xe, escore])
                continue
            else:
                del res[-1]
                res.append([xr, rscore])
                continue

        # contraction
        xc = x0 + rho * (x0 - res[-1][0])
        cscore = f(xc)
        if cscore < res[-1][1]:
            del res[-1]
            res.append([xc, cscore])
            continue

        # reduction
        x1 = res[0][0]
        nres = []
        for tup in res:
            redx = x1 + sigma * (tup[0] - x1)
            score = f(redx)
            nres.append([redx, score])
        res = nres


class AutoPidParameter:
    """
    自动求解pid参数
    """

    def __init__(self):
        self.kp = 0.8
        self.ki = 0
        self.kd = 0
        self.delta_kp = 0.1 * self.kp
        self.delta_ki = 0.1 * self.ki
        self.delta_kd = 0.1 * self.kd
        self.pid_obj = simple_pid.SimplePid()
        self.pi_main_obj = pi_main.PiMain()
        self.theta = 0
        self.start_theta = 0
        self.target_theta = 0
        self.theta_error_list = []
        # 总共测试次数
        self.loop_count = 100
        # 一个角度调节时间
        self.change_count = 20
        self.best_error = 180 * (self.change_count + 1)
        pi = pigpio.pi()
        self.compass_obj = pi_softuart.PiSoftuart(pi=pi, rx_pin=config.pin_compass_rx, tx_pin=config.pin_compass_tx,
                                                  baud=config.pin_compass_baud)

    def get_compass_data(self):
        while True:
            theta = self.compass_obj.read_compass()
            if theta:
                self.theta = theta

    def loop(self):
        nelder_mead(self.caluate_error, np.array([0., 0., 0.]))

    def caluate_error(self, x):
        config.kp = x[0]
        config.ki = x[1]
        config.kd = x[2]
        self.theta_error_list = []
        self.start_theta = self.pi_main_obj.theta
        if self.pi_main_obj.theta is None:
            self.pi_main_obj.theta = 0
        self.target_theta = (self.pi_main_obj.theta + 180) % 360
        for i in range(self.change_count):
            start_time = time.time()
            theta_error = self.target_theta - self.pi_main_obj.theta
            self.theta_error_list.append(abs(theta_error))
            if abs(theta_error) > 180:
                if theta_error > 0:
                    theta_error = theta_error - 360
                else:
                    theta_error = 360 + theta_error
            print('theta_error', theta_error)
            left_pwm, right_pwm = self.pid_obj.pid_pwm_2(distance=0,
                                                         theta_error=theta_error)

            self.pi_main_obj.set_pwm(left_pwm, right_pwm)
            time.sleep(0.3)
            print('epoch time:', time.time() - start_time)
        return sum(self.theta_error_list)


if __name__ == '__main__':
    auto_obj = AutoPidParameter()
    get_compass_data_thread = threading.Thread(target=auto_obj.pi_main_obj.get_compass_data)
    loop_change_pwm_thread = threading.Thread(target=auto_obj.pi_main_obj.loop_change_pwm)
    get_compass_data_thread.setDaemon(True)
    loop_change_pwm_thread.setDaemon(True)
    get_compass_data_thread.start()
    loop_change_pwm_thread.start()
    auto_obj.loop()
    # 1.00356692, 2.27904099, 0.30639995
    # except Exception as e:
    #     print('AutoPidParameter error ', e)
    #     auto_obj.pi_main_obj.stop()
