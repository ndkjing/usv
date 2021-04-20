import pygame
# pygame.joystick.init()
import threading
#初始化
pygame.init()
pygame.joystick.init()
clock = pygame.time.Clock()
print(pygame.joystick.get_count())
#车辆各个参数设置
car_max_speed=10  #车辆最大移动速度
car_max_turn=3   #车辆最大转向速度
camera_max_speed=10  #视角转移速度

done=False
left_speed=0  #车辆左半边轮子的速度
right_speed=0  #车辆右半边轮子的速度
vis_v_par=0 #摄像头舵机水平转向速度
vis_v_hor=0 #摄像头舵机垂直转向速度


def shoubin_thread():
    global left_speed
    global right_speed
    global vis_v_par
    global vis_v_hor
    #寻找电脑上的手柄数量，一般我只连了一个手柄（因为只有一个）
    joystick_count = pygame.joystick.get_count()
    for i in range(joystick_count):
        #读取对应设备
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        while (not done ):
            # 这句话很重要，这是保障实时读取手柄摇杆信息，反正不能去掉
            pygame.event.get()
            #获取摇杆数量
            axes = joystick.get_numaxes()
            print('摇杆数量',axes)
            ls=0
            rs=0
            vps = 0
            vhs = 0
            for i in range(axes):
                axis = joystick.get_axis(i)
                if i==0:
                    print("0 ",axis*car_max_turn)
                    ls+=axis*car_max_turn
                    rs-=axis*car_max_turn
                if i==1:
                    print("1 ",-axis*car_max_speed)
                    ls-=axis*car_max_speed
                    rs -= axis * car_max_speed
                if i==2:#上是负数
                    vps=axis*camera_max_speed
                    print("2 ", -axis * car_max_speed)
                if i == 3:  # 左是负数
                    vhs = -axis * camera_max_speed
                    print("3 ", -axis * car_max_speed)
                # print(i,axis)
            left_speed = ls
            right_speed = rs
            vis_v_par = vps
            vis_v_hor = vhs
            clock.tick(20)


def print_thread():
    while (not done):
        print('================')
        print('左轮速度 = ',left_speed,"  右轮速度 = ",right_speed)
        print('相机水平移动速度 = ',  vis_v_par, "  相机垂直移动速度 = ", vis_v_hor)


def get_shoubing_info():
    return left_speed,right_speed,vis_v_par,vis_v_hor


if __name__ == '__main__':
    t1 =threading.Thread(target=shoubin_thread)
    t1.start()
    t1.join()


