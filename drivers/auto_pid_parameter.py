from drivers import pi_main
from drivers import pi_softuart
from moveControl.pathTrack import simple_pid
import config

import threading
import pigpio
import json


class AutoPidParameter:
    """
    自动求解pid参数
    """

    def __init__(self):
        self.kp = 0.8
        self.ki = 0
        self.kd = 0
        self.delta_kp = 0.1*self.kp
        self.delta_ki = 0.1*self.ki
        self.delta_kd = 0.1*self.kd
        self.pid_obj = simple_pid.SimplePid()
        self.pi_main_obj = pi_main.PiMain()
        self.theta = None
        self.start_theta = None
        self.target_theta = None
        self.theta_error_list = []
        # 总共测试次数
        self.loop_count = 100
        # 一个角度调节时间
        self.change_count = 25
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
        while True:
            # for i in range(self.loop_count):
            config.kp, config.ki, config.kd = self.kp, self.ki, self.kd
            self.start_theta = self.theta
            self.target_theta = (self.theta + 180) % 360
            self.caluate_error()
            current_error = sum(self.theta_error_list)
            print('self.delta_parameters', self.delta_kp)
            if self.delta_kp + self.delta_kd < 0.01:
                with open('pid.json', 'w') as f:
                    json.dump({'pid': [self.kp, self.kd]}, f)
                break
            else:
                if current_error > self.best_error:
                    self.kp = self.kp * 1.1
                else:
                    self.best_error = current_error
                    self.kp = self.kp - 2 * self.delta_kp
                    self.start_theta = self.theta
                    self.target_theta = (self.theta + 180) % 360
                    self.caluate_error()
                    current_error = sum(self.theta_error_list)
                    if current_error > self.best_error:
                        self.kp = self.kp * 1.1
                    else:
                        self.best_error = current_error
                        self.kp = self.kp + self.delta_kp
                        self.delta_kp = self.delta_kp * 0.9
                    #
                    # if current_error > self.best_error:
                    #     self.kd = self.kd * 1.1
                    # else:
                    #     self.best_error = current_error
                    #     self.kd = self.kd - 2 * self.delta_kd
                    #     self.caluate_error()
                    #     current_error = sum(self.theta_error_list)
                    #     if current_error > self.best_error:
                    #         self.kd = self.kd * 1.1
                    #     else:
                    #         self.best_error = current_error
                    #         self.kd = self.kd + self.delta_kd
                    #         self.delta_kd = self.delta_kd * 0.9

    def caluate_error(self):
        self.theta_error_list = []
        for i in range(self.change_count):
            theta_error = self.target_theta - self.theta
            self.theta_error_list.append(abs(theta_error))
            if abs(theta_error) > 180:
                if theta_error > 0:
                    theta_error = theta_error - 360
                else:
                    theta_error = 360 + theta_error
            left_pwm, right_pwm = self.pid_obj.pid_pwm(distance=0,
                                                       theta_error=theta_error)
            self.pi_main_obj.set_pwm(left_pwm, right_pwm)

if __name__ == '__main__':
    auto_obj = AutoPidParameter()
    try:
        get_compass_data_thread = threading.Thread(target=auto_obj.get_compass_data)
        get_compass_data_thread.setDaemon(True)
        get_compass_data_thread.start()
        auto_obj.loop()
    except Exception as e:
        print('AutoPidParameter error ', e)
        auto_obj.pi_main_obj.stop()
