import time
import json

from messageBus import data_define
import config
from utils import draw_img


class ShipType:
    def __init__(self, ship_type):
        """

        @param ship_type:
        """
        self.ship_type = ship_type  # 抽水类型
        if ship_type == config.ShipType.water_detect:
            self.ship_obj = WaterDetect()
        elif ship_type == config.ShipType.multi_draw:
            self.ship_obj = MultiDraw()


class WaterDetect:
    def __init__(self):
        pass

    # 抽水
    def draw(self, data_manager_obj):
        """
        抽水
        """
        # 判断开关是否需要打开或者关闭
        if config.home_debug or not config.b_draw:
            if data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw:
                if data_manager_obj.draw_start_time is None:
                    data_manager_obj.draw_start_time = time.time()
                else:
                    data_manager_obj.dump_draw_time = config.draw_time - int(
                        time.time() - data_manager_obj.draw_start_time)
                    if time.time() - data_manager_obj.draw_start_time > config.draw_time:
                        data_manager_obj.dump_draw_time = 0
                        data_manager_obj.draw_start_time = None
                        data_manager_obj.b_draw_over_send_data = True
                        data_manager_obj.b_sampling = 2
                        data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 0
            else:
                data_manager_obj.draw_start_time = None
                data_manager_obj.dump_draw_time = 0
        else:
            # 开启了遥控器
            if data_manager_obj.pi_main_obj.b_start_remote:
                # 如果电脑控制抽水时候打开了遥控  将电脑抽水置位0
                if data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw:
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 0
                # 判断遥控器控制抽水
                # 正在抽水时不能让排水发送A0Z
                if data_manager_obj.pi_main_obj.remote_draw_status == 1:
                    if data_manager_obj.draw_start_time is None:
                        data_manager_obj.draw_start_time = time.time()
                    data_manager_obj.send_stc_data('A1Z')
                else:
                    if data_manager_obj.draw_start_time is not None:
                        # 超过抽水时间发送数据
                        data_manager_obj.dump_draw_time = config.draw_time - int(
                            time.time() - data_manager_obj.draw_start_time)
                        if time.time() - data_manager_obj.draw_start_time > config.draw_time:
                            data_manager_obj.dump_draw_time = 0
                            data_manager_obj.b_draw_over_send_data = True
                    # 将时间置空
                    data_manager_obj.draw_start_time = None
                    # 正在排水时不能发送结束抽水
                    if not data_manager_obj.pi_main_obj.remote_drain_status:
                        data_manager_obj.send_stc_data('A0Z')
                # 排水
                if data_manager_obj.pi_main_obj.remote_drain_status:
                    data_manager_obj.send_stc_data('A2Z')
                    time.sleep(0.1)
                else:
                    # 正在抽水时不能发送停止
                    if data_manager_obj.draw_start_time is None:
                        data_manager_obj.send_stc_data('A0Z')
                    time.sleep(0.1)
            # 没有开启遥控器
            else:
                # 判断是否抽水  点击抽水情况
                if data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw:
                    # 判断是否有杆子放下杆子
                    if config.b_control_deep:
                        data_manager_obj.pi_main_obj.set_draw_deep(config.min_deep_steer_pwm)
                        if data_manager_obj.pi_main_obj.draw_steer_pwm == data_manager_obj.pi_main_obj.target_draw_steer_pwm:
                            data_manager_obj.send_stc_data('A1Z')
                            # 如果有抽水杆需要先放到下面再计算开始时间
                            if data_manager_obj.draw_start_time is None:
                                data_manager_obj.draw_start_time = time.time()
                                # 触发一次停止
                                data_manager_obj.pi_main_obj.stop()
                            else:
                                # 超时中断抽水
                                data_manager_obj.dump_draw_time = config.draw_time - int(
                                    time.time() - data_manager_obj.draw_start_time)
                                if time.time() - data_manager_obj.draw_start_time > config.draw_time:
                                    data_manager_obj.dump_draw_time = 0
                                    data_manager_obj.b_draw_over_send_data = True
                                    data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 0
                                    data_manager_obj.is_need_drain = True
                                    data_manager_obj.b_sampling = 2
                                    data_manager_obj.send_stc_data('A0Z')
                    else:
                        data_manager_obj.send_stc_data('A1Z')
                        if data_manager_obj.draw_start_time is None:
                            data_manager_obj.draw_start_time = time.time()
                            # 触发一次停止
                            data_manager_obj.pi_main_obj.stop()
                        else:
                            # 超时中断抽水
                            data_manager_obj.dump_draw_time = config.draw_time - int(
                                time.time() - data_manager_obj.draw_start_time)  # 提示用户抽水剩余时间
                            if time.time() - data_manager_obj.draw_start_time > config.draw_time:
                                data_manager_obj.dump_draw_time = 0
                                data_manager_obj.b_draw_over_send_data = True
                                data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 0
                                data_manager_obj.is_need_drain = True  # 抽水完成需要排水
                                data_manager_obj.b_sampling = 2
                                data_manager_obj.send_stc_data('A0Z')
                else:
                    data_manager_obj.dump_draw_time = 0
                    # 没有在排水才能发送停止
                    if not data_manager_obj.is_auto_drain:
                        data_manager_obj.send_stc_data('A0Z')
                    # 没有抽水的情况下杆子都要收回来
                    if config.b_control_deep and data_manager_obj.drain_start_time is None:
                        data_manager_obj.pi_main_obj.set_draw_deep(config.max_deep_steer_pwm)
                    data_manager_obj.draw_start_time = None
                # 判断没有排水则先排水再收杆子
                if data_manager_obj.is_need_drain:
                    if data_manager_obj.drain_start_time is None:
                        data_manager_obj.drain_start_time = time.time()
                        data_manager_obj.send_stc_data('A2Z')
                        data_manager_obj.is_auto_drain = 1
                    else:
                        # 超时中断排水
                        if time.time() - data_manager_obj.drain_start_time > config.draw_time:
                            data_manager_obj.send_stc_data('A0Z')
                            data_manager_obj.is_auto_drain = 0
                            # 收回杆子
                            if config.b_control_deep:
                                data_manager_obj.pi_main_obj.set_draw_deep(config.max_deep_steer_pwm)
                            data_manager_obj.is_need_drain = False
                            data_manager_obj.drain_start_time = None

    # 预先存储任务   计算距离并排序
    def check_task(self, data_manager_obj):
        if data_manager_obj.server_data_obj.mqtt_send_get_obj.get_task == 1 and data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id:
            print("获取任务task_id", data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id)
            url = config.http_get_task + "?taskId=%s" % data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id
            return_data = data_manager_obj.server_data_obj.send_server_http_data('GET', '', url,
                                                                                 token=data_manager_obj.token)
            task_data_list = None
            if return_data:
                content_data = json.loads(return_data.content)
                print('获取任务数据', content_data)
                if not content_data.get('code'):
                    data_manager_obj.logger.info({'获取任务 GET请求失败': content_data})
                if content_data.get('data') and content_data.get('data').get('records') and len(
                        content_data.get('data').get('records')) == 1:
                    task_data = content_data.get('data').get('records')[0].get('task')
                    temp_task_data = content_data.get('data').get('records')[0].get('taskTem')
                    if content_data.get('data').get('records')[0].get('creator'):
                        data_manager_obj.creator = content_data.get('data').get('records')[0].get('creator')
                    if temp_task_data:
                        temp_task_data = json.loads(temp_task_data)
                    task_data = json.loads(task_data)
                    print('task_data', task_data)
                    print('temp_task_data', temp_task_data)
                    # 上次任务还没有完成继续任务
                    if temp_task_data and content_data.get('data').get('records')[0].get('planId'):
                        data_manager_obj.action_id = content_data.get('data').get('records')[0].get('planId')
                        data_manager_obj.server_data_obj.mqtt_send_get_obj.action_type = 3
                        task_data_list = temp_task_data
                    else:
                        task_data_list = task_data
            if not task_data_list:
                print('############ 没有任务数据')
                return
            data_manager_obj.server_data_obj.mqtt_send_get_obj.get_task = 0
            data_manager_obj.task_list = task_data_list
            data_manager_obj.sort_task_list = task_data_list
            data_manager_obj.has_task = 1

    # 任务
    def task(self, data_manager_obj):
        if len(data_manager_obj.sort_task_list) == 0:
            self.check_task(data_manager_obj)  # 检查是否需要发送预先存储任务
        # 有任务发送任务状态 更新任务为正在执行
        if data_manager_obj.has_task == 1:
            # 任务模式自己规划路径不再重新规划路径
            # 存放路径点和监测点
            path_planning_data = {"sampling_points": [],
                                  "path_points": []
                                  }
            for i in data_manager_obj.sort_task_list:
                if i.get("type") == 1:  # 检测点添加到监测点轨迹中
                    data_manager_obj.sample_index.append(1)
                else:
                    data_manager_obj.sample_index.append(0)
                path_planning_data.get("sampling_points").append(i.get("lnglat"))
                path_planning_data.get("path_points").append(i.get("lnglat"))
            data_manager_obj.send(method='mqtt',
                                  topic='path_planning_%s' % config.ship_code,
                                  data=path_planning_data,
                                  qos=0)
            print('mqtt任务经纬度数据', path_planning_data)
            print("task_id", data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id)
            data_manager_obj.has_task = 0
        # 更新剩余任务点 到点减一 所有点到达后设置任务状态为0
        if data_manager_obj.server_data_obj.mqtt_send_get_obj.cancel_action == 1:  # 取消行动
            print('data_manager_obj.server_data_obj.mqtt_send_get_obj.cancel_action',
                  data_manager_obj.server_data_obj.mqtt_send_get_obj.cancel_action)
            data_manager_obj.server_data_obj.mqtt_send_get_obj.cancel_action = 0
            data_manager_obj.server_data_obj.mqtt_send_get_obj.control_move_direction = -1
            update_plan_data = {"id": data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id,
                                # "taskTem": '[]',
                                "state": 0,
                                "deviceId": config.ship_code,
                                "planId": data_manager_obj.action_id
                                }
            print('更新任务消息', update_plan_data)
            data_manager_obj.sort_task_list = []
            return_data = data_manager_obj.server_data_obj.send_server_http_data('POST',
                                                                                 update_plan_data,
                                                                                 config.http_plan_update,
                                                                                 token=data_manager_obj.token)
            if return_data:
                content_data = json.loads(return_data.content)
                if content_data.get("code") != 200:
                    data_manager_obj.logger.error('更新任务失败')
                else:
                    data_manager_obj.logger.info({'更新任务': content_data})
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.cancel_action = 0
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id = ''
                    data_manager_obj.action_id = None
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.action_name = ""
        if data_manager_obj.is_plan_all_arrive:
            print('data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id',
                  data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id)
            print('data_manager_obj.is_plan_all_arrive', data_manager_obj.is_plan_all_arrive)
            update_plan_data = {"id": data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id,
                                "taskTem": '[]',
                                "state": 0,
                                "deviceId": config.ship_code,
                                "planId": ""
                                }
            print('更新任务消息', update_plan_data)
            data_manager_obj.sort_task_list = []
            return_data = data_manager_obj.server_data_obj.send_server_http_data('POST',
                                                                                 update_plan_data,
                                                                                 config.http_plan_update,
                                                                                 token=data_manager_obj.token)
            if return_data:
                content_data = json.loads(return_data.content)
                if content_data.get("code") != 200:
                    data_manager_obj.logger.error('更新任务失败')
                else:
                    data_manager_obj.logger.info({'更新任务': content_data})
                    data_manager_obj.is_need_update_plan = 0
                    data_manager_obj.is_plan_all_arrive = 0
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id = ''
                    data_manager_obj.action_id = None
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.action_name = ""
            data_manager_obj.server_data_obj.mqtt_send_get_obj.action_type = 2
        if data_manager_obj.is_need_update_plan == 1 and not data_manager_obj.is_plan_all_arrive and data_manager_obj.server_data_obj.mqtt_send_get_obj.sampling_points_status.count(
                0) > 0 and data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id:
            print('#################data_manager_obj.is_need_update_plan', data_manager_obj.is_need_update_plan)
            print('#################data_manager_obj.server_data_obj.mqtt_send_get_obj.sampling_points_status',
                  data_manager_obj.server_data_obj.mqtt_send_get_obj.sampling_points_status)
            if len(data_manager_obj.server_data_obj.mqtt_send_get_obj.sampling_points_status) > 0:
                index = data_manager_obj.server_data_obj.mqtt_send_get_obj.sampling_points_status.index(0)
                sampling_point_gps_list = []
                for i in range(index, len(data_manager_obj.server_data_obj.mqtt_send_get_obj.sampling_points_status)):
                    sampling_point_gps = data_manager_obj.server_data_obj.mqtt_send_get_obj.sampling_points[i]
                    sampling_point_gps_list.append(
                        {"lnglat": sampling_point_gps, "type": data_manager_obj.sample_index[i]})
            else:
                sampling_point_gps_list = []
            update_plan_data = {"id": data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id,
                                "taskTem": json.dumps(sampling_point_gps_list),
                                "state": 1,
                                "deviceId": config.ship_code,
                                "planId": data_manager_obj.action_id
                                }
            print('更新任务消息', update_plan_data)
            return_data = data_manager_obj.server_data_obj.send_server_http_data('POST',
                                                                                 update_plan_data,
                                                                                 config.http_plan_update,
                                                                                 token=data_manager_obj.token)
            if return_data:
                content_data = json.loads(return_data.content)
                if content_data.get("code") != 200:
                    data_manager_obj.logger.error('更新任务失败')
                else:
                    data_manager_obj.logger.info({'更新任务': content_data})
                    data_manager_obj.is_need_update_plan = 0

    # 上传数据
    def send_data(self, data_manager_obj):
        # 上传检测数据
        if data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code:
            data_manager_obj.data_define_obj.pool_code = data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code
        detect_data = data_manager_obj.data_define_obj.detect
        detect_data.update({'mapId': data_manager_obj.data_define_obj.pool_code})
        # 更新真实数据
        if not config.home_debug:
            mqtt_send_detect_data = data_define.fake_detect_data(detect_data)
            mqtt_send_detect_data['water'].update(data_manager_obj.pi_main_obj.water_data_dict)
        # 更新模拟数据
        else:
            mqtt_send_detect_data = data_define.fake_detect_data(detect_data)
        # 替换键
        for k_all, v_all in data_define.name_mappings.items():
            for old_key, new_key in v_all.items():
                pop_value = mqtt_send_detect_data[k_all].pop(old_key)
                mqtt_send_detect_data[k_all].update({new_key: pop_value})
        if data_manager_obj.b_draw_over_send_data and config.b_draw:
            # 添加经纬度
            mqtt_send_detect_data.update({'jwd': json.dumps(data_manager_obj.lng_lat)})
            mqtt_send_detect_data.update({'gjwd': json.dumps(data_manager_obj.gaode_lng_lat)})
            mqtt_send_detect_data.update(mqtt_send_detect_data.get('water'))
            if data_manager_obj.action_id:
                mqtt_send_detect_data.update({'planId': data_manager_obj.action_id})
            data_manager_obj.send(method='mqtt', topic='detect_data_%s' % config.ship_code, data=mqtt_send_detect_data,
                                  qos=0)
            if len(data_manager_obj.data_define_obj.pool_code) > 0:
                mqtt_send_detect_data.update({'mapId': data_manager_obj.data_define_obj.pool_code})
                return_data = data_manager_obj.server_data_obj.send_server_http_data('POST',
                                                                                     mqtt_send_detect_data,
                                                                                     config.http_data_save,
                                                                                     token=data_manager_obj.token)
                if return_data:
                    content_data = json.loads(return_data.content)
                    if not content_data.get("success") and content_data.get("code") not in [200, 20000]:
                        data_manager_obj.logger.error({'POST发送检测请求失败': content_data})
                    else:
                        # 发送结束改为False
                        data_manager_obj.b_draw_over_send_data = False
                    data_manager_obj.logger.info({"本地保存检测数据": mqtt_send_detect_data})


class MultiDraw:

    def __init__(self):
        pass

    # 立即抽水
    def draw_sub(self, b_draw, bottle_id, draw_deep, draw_time, data_manager_obj):
        """
        @param b_draw: 抽水
        @param bottle_id: 抽水瓶号
        @param draw_deep: 抽水深度
        @param draw_time: 抽水时间
        @return:
        """
        if config.home_debug:
            if b_draw:
                if data_manager_obj.draw_start_time is None:
                    data_manager_obj.draw_start_time = time.time()
                else:
                    # print("本次抽水总时间: %f 当前抽水时间: %f 最大抽水时间: %f" % (
                    #     draw_time, time.time() - data_manager_obj.draw_start_time, config.max_draw_time))
                    # 测试限制水满
                    # if data_manager_obj.bottle_draw_time_list[bottle_id - 1] > config.max_draw_time:
                    #     print('该瓶抽水已满不能再抽', data_manager_obj.bottle_draw_time_list[bottle_id - 1])
                    data_manager_obj.dump_draw_list = [draw_time - int(time.time() - data_manager_obj.draw_start_time),
                                                       draw_time]
                    if time.time() - data_manager_obj.draw_start_time > draw_time:
                        data_manager_obj.b_draw_over_send_data = True
                        data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 0
                        data_manager_obj.b_sampling = 2
                        data_manager_obj.draw_start_time = None
                        # data_manager_obj.bottle_draw_time_list[bottle_id - 1] += draw_time
                        time.sleep(0.1)
                        return True
            else:
                if data_manager_obj.draw_start_time is not None:
                    data_manager_obj.draw_start_time = None
        else:
            # 判断是否抽水  点击抽水情况
            if b_draw:
                if config.b_control_deep:
                    # 计算目标深度的pwm值
                    target_pwm = data_manager_obj.deep2pwm(draw_deep)
                    data_manager_obj.pi_main_obj.set_draw_deep(target_pwm)
                    print('############', target_pwm, data_manager_obj.pi_main_obj.draw_steer_pwm,
                          data_manager_obj.pi_main_obj.target_draw_steer_pwm)
                    if data_manager_obj.pi_main_obj.draw_steer_pwm != data_manager_obj.pi_main_obj.target_draw_steer_pwm:
                        return False
                if data_manager_obj.draw_start_time is None:
                    data_manager_obj.draw_start_time = time.time()
                    # 触发一次停止
                    # data_manager_obj.pi_main_obj.stop()
                print('##################################bottle_id', bottle_id)
                if bottle_id == 1:
                    data_manager_obj.send_stc_data('A1Z')
                elif bottle_id == 2:
                    data_manager_obj.send_stc_data('A3Z')
                elif bottle_id == 3:
                    data_manager_obj.send_stc_data('A4Z')
                elif bottle_id == 4:
                    data_manager_obj.send_stc_data('A5Z')
                # print("本次抽水总时间: %f 当前抽水时间: %f 最大抽水时间: %f" % (
                #     draw_time, time.time() - data_manager_obj.draw_start_time, config.max_draw_time))
                # 测试限制水满  改为不限制
                # if data_manager_obj.bottle_draw_time_list[bottle_id - 1] > config.max_draw_time:
                #     print('该瓶抽水已满不能再抽', data_manager_obj.bottle_draw_time_list[bottle_id - 1])
                # 超时中断抽水
                data_manager_obj.dump_draw_list = [draw_time - int(time.time() - data_manager_obj.draw_start_time),
                                                   draw_time]
                print('抽水时间', data_manager_obj.dump_draw_list, time.time() - data_manager_obj.draw_start_time,
                      data_manager_obj.draw_start_time)
                if time.time() - data_manager_obj.draw_start_time > draw_time:
                    return True
            else:
                # if data_manager_obj.is_stop_draw:
                #     print('停止抽水')
                data_manager_obj.send_stc_data('A0Z')
                # 没有抽水的情况下杆子都要收回来
                if config.b_control_deep:
                    data_manager_obj.pi_main_obj.set_draw_deep(config.max_deep_steer_pwm)
                # if data_manager_obj.draw_start_time is not None:
                #     data_manager_obj.draw_start_time = None

    # 判断怎么样抽水
    def draw(self, data_manager_obj):
        """
        抽水控制函数
        """
        # 调试模式 判断开关是否需要打开或者关闭
        if config.home_debug or not config.b_draw:
            # 前端发送抽水深度和抽水时间
            if data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw:
                if data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_bottle_id and \
                        data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_deep and \
                        data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity:
                    temp_draw_bottle_id = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_bottle_id
                    temp_draw_deep = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_deep
                    temp_draw_time = int(
                        60 * data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity / config.draw_speed)
                    data_manager_obj.current_draw_bottle = temp_draw_bottle_id
                    data_manager_obj.current_draw_deep = temp_draw_deep
                    data_manager_obj.current_draw_capacity = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity
                    b_finish_draw = self.draw_sub(True, temp_draw_bottle_id, temp_draw_deep, temp_draw_time,
                                                  data_manager_obj)
                    if b_finish_draw:
                        data_manager_obj.draw_over_bottle_info = [data_manager_obj.current_draw_bottle,
                                                                  data_manager_obj.current_draw_deep,
                                                                  data_manager_obj.current_draw_capacity]
            else:
                data_manager_obj.dump_draw_list = [0, 0]
            # 预先存储任务深度和水量
            if data_manager_obj.current_arriver_index == len(data_manager_obj.sort_task_done_list):
                return
            if data_manager_obj.current_arriver_index is not None and data_manager_obj.sort_task_done_list and \
                    data_manager_obj.sort_task_done_list[
                        data_manager_obj.current_arriver_index].count(0) > 0:  # 是否是使用预先存储任务
                data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 1
                data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_bottle_id = None
                data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_deep = None
                data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity = None
                index = data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index].index(0)
                temp_draw_bottle_id = \
                    data_manager_obj.sort_task_list[data_manager_obj.current_arriver_index].get("data")[index][0]
                temp_draw_deep = \
                    data_manager_obj.sort_task_list[data_manager_obj.current_arriver_index].get("data")[index][1]
                temp_bottle_amount = \
                    data_manager_obj.sort_task_list[data_manager_obj.current_arriver_index].get("data")[index][2]
                # 将存储的数据映射为真实深度和容量
                if temp_draw_deep == 10:
                    bottle_deep = 0.1
                elif temp_draw_deep == 20:
                    bottle_deep = 0.2
                elif temp_draw_deep == 30:
                    bottle_deep = 0.3
                elif temp_draw_deep == 40:
                    bottle_deep = 0.4
                else:
                    bottle_deep = 0.5
                if temp_bottle_amount == 10:
                    bottle_amount = 500
                elif temp_bottle_amount == 20:
                    bottle_amount = 1000
                elif temp_bottle_amount == 30:
                    bottle_amount = 2000
                elif temp_bottle_amount == 40:
                    bottle_amount = 3000
                elif temp_bottle_amount == 50:
                    bottle_amount = 4000
                else:
                    bottle_amount = 5000
                data_manager_obj.current_draw_bottle = temp_draw_bottle_id
                data_manager_obj.current_draw_deep = bottle_deep
                data_manager_obj.current_draw_capacity = bottle_amount
                print('index temp_draw_bottle_id,temp_draw_deep,temp_draw_time', index, temp_draw_bottle_id,
                      bottle_deep,
                      bottle_amount)
                temp_draw_time = int(60 * bottle_amount / config.draw_speed)  # 根据默认配置修改抽水时间
                is_finish_draw = self.draw_sub(True, temp_draw_bottle_id, data_manager_obj.current_draw_deep,
                                               temp_draw_time, data_manager_obj)
                if is_finish_draw:
                    data_manager_obj.is_need_update_plan = 1  # 抽完水后需要更新任务状态
                    data_manager_obj.dump_draw_list = [0, 0]
                    print(
                        'data_manager_obj.current_arriver_index index temp_draw_bottle_id,temp_draw_deep,temp_draw_time',
                        data_manager_obj.current_arriver_index, index, temp_draw_bottle_id,
                        temp_draw_deep, bottle_amount)
                    data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index][index] = 1
                    print('data_manager_obj.sort_task_done_list', data_manager_obj.current_arriver_index,
                          data_manager_obj.sort_task_done_list)
                    data_manager_obj.draw_over_bottle_info = [data_manager_obj.current_draw_bottle,
                                                              data_manager_obj.current_draw_deep,
                                                              data_manager_obj.current_draw_capacity]
            # elif data_manager_obj.current_arriver_index is not None and data_manager_obj.sort_task_done_list and data_manager_obj.sort_task_done_list[
            #     data_manager_obj.current_arriver_index].count(
            #     0) == 0:
            #     data_manager_obj.current_arriver_index += 1
        else:
            # 开启了遥控器
            if data_manager_obj.pi_main_obj.b_start_remote:
                # 判断遥控器控制抽水
                # 正在抽水时不能让排水发送A0Z
                if data_manager_obj.pi_main_obj.remote_draw_status == 1:
                    if not data_manager_obj.remote_draw_overtime:  # 判断没有超过抽水时间
                        if data_manager_obj.draw_start_time is None:
                            data_manager_obj.draw_start_time = time.time()
                        if data_manager_obj.pi_main_obj.remote_draw_status_0_1 == 1:
                            data_manager_obj.current_draw_bottle = 1
                            data_manager_obj.send_stc_data('A1Z')
                        elif data_manager_obj.pi_main_obj.remote_draw_status_0_1 == 2:
                            data_manager_obj.current_draw_bottle = 2
                            data_manager_obj.send_stc_data('A3Z')
                        elif data_manager_obj.pi_main_obj.remote_draw_status_2_3 == 3:
                            data_manager_obj.current_draw_bottle = 3
                            data_manager_obj.send_stc_data('A4Z')
                        elif data_manager_obj.pi_main_obj.remote_draw_status_2_3 == 4:
                            data_manager_obj.current_draw_bottle = 4
                            data_manager_obj.send_stc_data('A5Z')
                        print('data_manager_obj.current_draw_bottle', data_manager_obj.current_draw_bottle)
                        # 遥控器设置抽水深度和抽水时间
                        if data_manager_obj.pi_main_obj.current_draw_capacity:
                            temp_draw_time = int(
                                60 * data_manager_obj.pi_main_obj.current_draw_capacity / config.draw_speed)  # 暂时使用抽水时间位置设置为抽水容量
                        else:
                            temp_draw_time = int(60 * config.max_draw_capacity / config.draw_speed)  # 根据默认配置修改抽水时间
                        # 超过抽水时间停止抽水等待拨到0后再次拨到1才抽水
                        if time.time() - data_manager_obj.draw_start_time > temp_draw_time:
                            # data_manager_obj.bottle_draw_time_list[data_manager_obj.current_draw_bottle] += temp_draw_time
                            data_manager_obj.send_stc_data('A0Z')
                            data_manager_obj.b_sampling = 2  # 用于切换状态
                            data_manager_obj.b_draw_over_send_data = True  # 抽水超时发送数据
                            # 设置标志位为超时才停止抽水
                            data_manager_obj.remote_draw_overtime = 1
                            # 设置抽水完成信息
                            data_manager_obj.draw_over_bottle_info = [data_manager_obj.current_draw_bottle,
                                                                      config.draw_deep,
                                                                      config.max_draw_capacity]
                    else:
                        # 超时中断等待用户拨回拨杆
                        data_manager_obj.send_stc_data('A0Z')
                else:
                    if data_manager_obj.draw_start_time is not None:
                        data_manager_obj.bottle_draw_time_list[data_manager_obj.current_draw_bottle] += int(
                            time.time() - data_manager_obj.draw_start_time)
                        data_manager_obj.draw_start_time = None  # 将时间置空
                    if data_manager_obj.remote_draw_overtime:  # 当拨杆拨到0 后重新将抽水超时置空
                        data_manager_obj.remote_draw_overtime = 0
                    data_manager_obj.send_stc_data('A0Z')
            # 没有开启遥控器
            else:
                # 前端发送抽水深度和抽水时间
                if data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_bottle_id and \
                        data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_deep and \
                        data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity:
                    temp_draw_bottle_id = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_bottle_id
                    temp_draw_deep = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_deep
                    temp_draw_capacity = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity
                    temp_draw_time = int(
                        60 * data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity / config.draw_speed)
                    data_manager_obj.current_draw_bottle = temp_draw_bottle_id
                    data_manager_obj.current_draw_deep = temp_draw_deep
                    print('#################前端设置抽水瓶号 深度 容量:', temp_draw_bottle_id, temp_draw_deep, temp_draw_time)
                    data_manager_obj.current_draw_capacity = temp_draw_capacity
                    is_finish_draw = self.draw_sub(True, temp_draw_bottle_id, temp_draw_deep,
                                                   temp_draw_time, data_manager_obj)
                    if is_finish_draw:
                        data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_bottle_id = None
                        print('#####################is_finish_draw', is_finish_draw)
                        data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 0
                        data_manager_obj.b_sampling = 2
                        data_manager_obj.b_draw_over_send_data = True  # 抽水超时发送数据
                        data_manager_obj.draw_start_time = None
                        data_manager_obj.dump_draw_list = [0, 0]
                        data_manager_obj.draw_over_bottle_info = [data_manager_obj.current_draw_bottle,
                                                                  data_manager_obj.current_draw_deep,
                                                                  data_manager_obj.current_draw_capacity]
                elif not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and not data_manager_obj.sort_task_list:  # 前端没发送抽水且不是任务抽水
                    data_manager_obj.dump_draw_list = [0, 0]
                    self.draw_sub(False, 0, 0, 0, data_manager_obj)
                if data_manager_obj.current_arriver_index == len(data_manager_obj.sort_task_done_list):
                    return
                if data_manager_obj.current_arriver_index is not None:
                    print('到达任务点', data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index],
                          data_manager_obj.sort_task_done_list,
                          data_manager_obj.current_arriver_index)
                if data_manager_obj.current_arriver_index is not None and data_manager_obj.sort_task_done_list and \
                        data_manager_obj.sort_task_done_list[
                            data_manager_obj.current_arriver_index].count(0) > 0:  # 是否是使用预先存储任务
                    # data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 1
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_bottle_id = None
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_deep = None
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity = None
                    index = data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index].index(0)
                    temp_draw_bottle_id = \
                        data_manager_obj.sort_task_list[data_manager_obj.current_arriver_index].get("data")[index][0]
                    temp_draw_deep = \
                        data_manager_obj.sort_task_list[data_manager_obj.current_arriver_index].get("data")[index][1]
                    temp_bottle_amount = \
                        data_manager_obj.sort_task_list[data_manager_obj.current_arriver_index].get("data")[index][2]
                    data_manager_obj.pi_main_obj.stop()
                    # 将存储的数据映射为真实深度和容量
                    if temp_draw_deep == 10:
                        bottle_deep = 0.1
                    elif temp_draw_deep == 20:
                        bottle_deep = 0.2
                    elif temp_draw_deep == 30:
                        bottle_deep = 0.3
                    elif temp_draw_deep == 40:
                        bottle_deep = 0.4
                    else:
                        bottle_deep = config.draw_deep
                    if temp_bottle_amount == 10:
                        bottle_amount = 500
                    elif temp_bottle_amount == 20:
                        bottle_amount = 1000
                    elif temp_bottle_amount == 30:
                        bottle_amount = 2000
                    elif temp_bottle_amount == 40:
                        bottle_amount = 3000
                    elif temp_bottle_amount == 50:
                        bottle_amount = 4000
                    else:
                        bottle_amount = config.max_draw_capacity
                    data_manager_obj.current_draw_bottle = temp_draw_bottle_id
                    data_manager_obj.current_draw_deep = bottle_deep
                    data_manager_obj.current_draw_capacity = bottle_amount
                    print('index temp_draw_bottle_id,temp_draw_deep,temp_draw_time', index, temp_draw_bottle_id,
                          bottle_deep,
                          bottle_amount)
                    temp_draw_time = int(60 * bottle_amount / config.draw_speed)  # 根据默认配置修改抽水时间
                    is_finish_draw = self.draw_sub(True, temp_draw_bottle_id, bottle_deep, temp_draw_time,
                                                   data_manager_obj)
                    if is_finish_draw:
                        data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 0
                        data_manager_obj.draw_start_time = None
                        data_manager_obj.is_need_update_plan = 1  # 抽完水后需要更新任务状态
                        data_manager_obj.dump_draw_list = [0, 0]
                        data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index][index] = 1
                        print('data_manager_obj.sort_task_done_list', data_manager_obj.current_arriver_index,
                              data_manager_obj.sort_task_done_list)
                        data_manager_obj.draw_over_bottle_info = [data_manager_obj.current_draw_bottle,
                                                                  data_manager_obj.current_draw_deep,
                                                                  data_manager_obj.current_draw_capacity]
                        data_manager_obj.b_sampling = 2
                        data_manager_obj.b_draw_over_send_data = True  # 抽水超时发送数据
                # 前端没抽水 且任务模式当前点位全部抽完 则收回杆子
                # print(''',data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index])
                elif not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and data_manager_obj.current_arriver_index is not None and data_manager_obj.sort_task_done_list and sum(
                        data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index]) == len(
                    data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index]):
                    data_manager_obj.dump_draw_list = [0, 0]
                    self.draw_sub(False, 0, 0, 0, data_manager_obj)

    def check_task(self, data_manager_obj):
        if data_manager_obj.server_data_obj.mqtt_send_get_obj.get_task == 1 and data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id:
            print("获取任务task_id", data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id)
            url = config.http_get_task + "?taskId=%s" % data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id
            return_data = data_manager_obj.server_data_obj.send_server_http_data('GET', '', url,
                                                                                 token=data_manager_obj.token)
            task_data_list = []
            if return_data:
                content_data = json.loads(return_data.content)
                print('获取任务数据', content_data)
                if not content_data.get('code'):
                    data_manager_obj.logger.info({'获取任务 GET请求失败': content_data})
                if content_data.get('data') and content_data.get('data').get('records') and len(
                        content_data.get('data').get('records')) == 1:
                    ################ 解析采样数据
                    data_manager_obj.creator = content_data.get('data').get('records')[0].get('creator')
                    current_task_data = content_data.get('data').get('records')[0].get('task')
                    last_task_data = content_data.get('data').get('records')[0].get('taskTem')  # 上次剩余没完成任务数据
                    if last_task_data and json.loads(last_task_data):
                        task_data = json.loads(last_task_data)
                        data_manager_obj.server_data_obj.mqtt_send_get_obj.action_type = 3
                        data_manager_obj.action_id = content_data.get('data').get('records')[0].get('planId')
                    else:
                        task_data = json.loads(current_task_data)
                    print('task_data', task_data)
                    if task_data:
                        for task in task_data:
                            print('task', task)
                            temp_list = {}
                            # lng_lat_str = task.get("jwd")
                            # lng_lat = [float(i) for i in lng_lat_str.split(',')]
                            temp_list.update({"lnglat": task.get("lnglat")})
                            temp_list.update({"type": task.get("type")})
                            draw_info = []
                            if task.get("data"):
                                for bottle in task.get("data"):
                                    bottle_id = int(bottle.get("cabin"))
                                    bottle_deep = int(bottle.get("deep"))
                                    bottle_amount = int(bottle.get("amount"))
                                    if bottle_deep == 0 or bottle_amount == 0:
                                        continue
                                    draw_info.append((bottle_id, bottle_deep, bottle_amount))
                                temp_list.update({"data": draw_info})
                            task_data_list.append(temp_list)
            if not task_data_list:
                print('############ 没有任务数据')
                return
            data_manager_obj.server_data_obj.mqtt_send_get_obj.get_task = 0
            data_manager_obj.task_list = task_data_list
            data_manager_obj.sort_task_list = task_data_list
            data_manager_obj.has_task = 1
            print('排序任务数据data_manager_obj.task_list', data_manager_obj.task_list)

    # 任务
    def task(self, data_manager_obj):
        if len(data_manager_obj.sort_task_list) == 0:
            self.check_task(data_manager_obj)  # 检查是否需要发送预先存储任务
        # 有任务发送任务状态 更新任务为正在执行
        if data_manager_obj.has_task == 1:
            # 任务模式自己规划路径不再重新规划路径
            # 存放路径点和监测点
            path_planning_data = {"sampling_points": [],
                                  "path_points": []
                                  }
            # 带抽水任务列表
            data_manager_obj.sort_task_done_list = []  # 获取新任务清空原来数据
            data_manager_obj.current_arriver_index = None  # 获取新任务清空原来数据
            data_manager_obj.sample_index = []
            for i in data_manager_obj.sort_task_list:
                if i.get("type") == 1 and i.get("data"):  # 检测点添加到监测点轨迹中
                    data_manager_obj.sample_index.append(1)
                else:
                    data_manager_obj.sample_index.append(0)
                if i.get("data"):
                    data_manager_obj.sort_task_done_list.append([0] * len(i.get("data")))
                else:
                    data_manager_obj.sort_task_done_list.append([])
                path_planning_data.get("sampling_points").append(i.get("lnglat"))
                path_planning_data.get("path_points").append(i.get("lnglat"))
            data_manager_obj.send(method='mqtt',
                                  topic='path_planning_%s' % config.ship_code,
                                  data=path_planning_data,
                                  qos=0)
            print('mqtt任务经纬度数据', path_planning_data)
            print("task_id", data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id)
            data_manager_obj.has_task = 0
        if data_manager_obj.server_data_obj.mqtt_send_get_obj.cancel_action == 1 and not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw:  # 取消行动
            print('data_manager_obj.server_data_obj.mqtt_send_get_obj.cancel_action',
                  data_manager_obj.server_data_obj.mqtt_send_get_obj.cancel_action)
            data_manager_obj.server_data_obj.mqtt_send_get_obj.cancel_action = 0
            data_manager_obj.server_data_obj.mqtt_send_get_obj.control_move_direction = -1
            update_plan_data = {"id": data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id,
                                # "taskTem": '[]',
                                "state": 0,
                                "deviceId": config.ship_code,
                                "planId": data_manager_obj.action_id
                                }
            print('更新任务消息', update_plan_data)
            data_manager_obj.sort_task_list = []
            return_data = data_manager_obj.server_data_obj.send_server_http_data('POST',
                                                                                 update_plan_data,
                                                                                 config.http_plan_update,
                                                                                 token=data_manager_obj.token)
            if return_data:
                content_data = json.loads(return_data.content)
                if content_data.get("code") != 200:
                    data_manager_obj.logger.error('更新任务失败')
                else:
                    data_manager_obj.logger.info({'更新任务': content_data})
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.cancel_action = 0
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id = ''
                    data_manager_obj.action_id = None
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.action_name = ""
        if data_manager_obj.is_plan_all_arrive and not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw:
            print('data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id',
                  data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id)
            print('data_manager_obj.is_plan_all_arrive', data_manager_obj.is_plan_all_arrive)
            update_plan_data = {"id": data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id,
                                "taskTem": '[]',
                                "state": 0,
                                "deviceId": config.ship_code,
                                "planId": ""
                                }
            print('更新任务消息', update_plan_data)

            return_data = data_manager_obj.server_data_obj.send_server_http_data('POST',
                                                                                 update_plan_data,
                                                                                 config.http_plan_update,
                                                                                 token=data_manager_obj.token)
            if return_data:
                content_data = json.loads(return_data.content)
                if content_data.get("code") != 200:
                    data_manager_obj.logger.error('更新任务失败')
                else:
                    data_manager_obj.logger.info({'更新任务': content_data})
                    data_manager_obj.is_need_update_plan = 0
                    data_manager_obj.is_plan_all_arrive = 0
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id = ''
                    data_manager_obj.action_id = None
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.action_name = ""
                    data_manager_obj.sort_task_list = []
            data_manager_obj.server_data_obj.mqtt_send_get_obj.action_type = 2
        if data_manager_obj.is_need_update_plan == 1 and not data_manager_obj.is_plan_all_arrive and data_manager_obj.server_data_obj.mqtt_send_get_obj.sampling_points_status.count(
                0) > 0 and data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id and not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw:
            print('#################data_manager_obj.is_need_update_plan', data_manager_obj.is_need_update_plan)
            print('#################data_manager_obj.server_data_obj.mqtt_send_get_obj.sampling_points_status',
                  data_manager_obj.server_data_obj.mqtt_send_get_obj.sampling_points_status)
            if len(data_manager_obj.server_data_obj.mqtt_send_get_obj.sampling_points_status) > 0:
                index = data_manager_obj.server_data_obj.mqtt_send_get_obj.sampling_points_status.index(0)
                sampling_point_gps_list = []
                for i in range(index, len(data_manager_obj.server_data_obj.mqtt_send_get_obj.sampling_points_status)):
                    sampling_point_gps = data_manager_obj.server_data_obj.mqtt_send_get_obj.sampling_points[i]
                    dump_data_dict = {"lnglat": sampling_point_gps, "type": data_manager_obj.sample_index[i]}
                    data = []
                    if data_manager_obj.sort_task_list[i].get("data"):
                        for draw_item in data_manager_obj.sort_task_list[i].get("data"):
                            draw_item_dict = {}
                            draw_item_dict.update({"cabin": draw_item[0]})
                            draw_item_dict.update({"deep": draw_item[1]})
                            draw_item_dict.update({"amount": draw_item[2]})
                            data.append(draw_item_dict)
                    if data:
                        dump_data_dict.update({"data": data})
                    sampling_point_gps_list.append(dump_data_dict)

            else:
                sampling_point_gps_list = []
            update_plan_data = {"id": data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id,
                                "taskTem": json.dumps(sampling_point_gps_list),
                                "state": 1,
                                "deviceId": config.ship_code,
                                "planId": data_manager_obj.action_id
                                }
            print('更新任务消息', update_plan_data)
            return_data = data_manager_obj.server_data_obj.send_server_http_data('POST',
                                                                                 update_plan_data,
                                                                                 config.http_plan_update,
                                                                                 token=data_manager_obj.token)
            if return_data:
                content_data = json.loads(return_data.content)
                if content_data.get("code") != 200:
                    data_manager_obj.logger.error('更新任务失败')
                else:
                    data_manager_obj.logger.info({'更新任务': content_data})
                    data_manager_obj.is_need_update_plan = 0

    # 上传数据
    def send_data(self, data_manager_obj):
        if data_manager_obj.b_draw_over_send_data:
            print('data_manager_obj.b_draw_over_send_data', data_manager_obj.b_draw_over_send_data)
            if data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code:
                data_manager_obj.data_define_obj.pool_code = data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code
            draw_data = {}
            draw_data.update({'deviceId': config.ship_code})
            draw_data.update({'mapId': data_manager_obj.data_define_obj.pool_code})
            if len(data_manager_obj.draw_over_bottle_info) == 3:
                draw_data.update({"bottleNum": data_manager_obj.draw_over_bottle_info[0]})
                draw_data.update({"deep": data_manager_obj.draw_over_bottle_info[1]})
                draw_data.update({"capacity": data_manager_obj.draw_over_bottle_info[2]})
            else:
                draw_data.update({"capacity": '-1'})
                draw_data.update({"deep": '-1'})
                draw_data.update({"bottleNum": '-1'})
            # 添加经纬度
            draw_data.update({'jwd': json.dumps(data_manager_obj.lng_lat)})
            draw_data.update({'gjwd': json.dumps(data_manager_obj.gaode_lng_lat)})
            if data_manager_obj.action_id:
                draw_data.update({'planId': data_manager_obj.action_id})
                if data_manager_obj.creator:
                    draw_data.update({"creator": data_manager_obj.creator})
            if data_manager_obj.action_id:
                draw_data.update({'planId': data_manager_obj.action_id})
            data_manager_obj.send(method='mqtt', topic='draw_data_%s' % config.ship_code, data=draw_data,
                                  qos=0)
            # 添加到抽水列表中
            if data_manager_obj.gaode_lng_lat:
                data_manager_obj.draw_points_list.append(
                    [data_manager_obj.gaode_lng_lat[0], data_manager_obj.gaode_lng_lat[1],
                     data_manager_obj.current_draw_bottle, data_manager_obj.current_draw_deep,
                     data_manager_obj.current_draw_capacity])
            else:
                data_manager_obj.draw_points_list.append(
                    [1, 1, data_manager_obj.current_draw_bottle, data_manager_obj.current_draw_deep,
                     data_manager_obj.current_draw_capacity])
            # 发送到服务器
            if len(data_manager_obj.data_define_obj.pool_code) > 0:
                try:
                    # 上传图片给服务器
                    server_save_img_path = draw_img.all_throw_img(config.http_get_img_path,
                                                                  config.http_upload_img,
                                                                  config.ship_code,
                                                                  [draw_data['jwd'], draw_data['bottleNum'],
                                                                   draw_data['deep'], draw_data['capacity']],
                                                                  token=data_manager_obj.token)
                    # 请求图片成功添加图片路径 失败则不添加
                    if server_save_img_path:
                        draw_data.update({"pic": server_save_img_path})
                    print('draw_data', draw_data)
                    return_data = data_manager_obj.server_data_obj.send_server_http_data('POST',
                                                                                         draw_data,
                                                                                         config.http_draw_save,
                                                                                         token=data_manager_obj.token)
                    print('上传采样数据返回:', return_data)
                    if return_data:
                        content_data = json.loads(return_data.content)
                        if content_data.get("code") != 200:
                            data_manager_obj.logger.error({'发送采样数据失败': content_data})
                        else:
                            data_manager_obj.logger.info({"发送采样数据成功": draw_data})
                            # 发送结束改为False
                            data_manager_obj.b_draw_over_send_data = False
                except Exception as e:
                    data_manager_obj.logger.info({"发送采样数据error": e})
