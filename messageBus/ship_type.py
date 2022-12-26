import random
import time
import json

from messageBus import data_define
import config
from utils import draw_img
from utils import lng_lat_calculate
from utils import data_valid


class ShipType:
    def __init__(self, ship_id):
        """
        @param ship_type:
        """
        ship_code = 'XXLJC4LCGSCSD1DA%03d' % ship_id
        self.ship_code = ship_code
        self.ship_type = config.ship_code_type_dict.get(ship_code)
        if self.ship_type == config.ShipType.water_detect:
            self.ship_obj = WaterDetect(ship_id)
        elif self.ship_type == config.ShipType.multi_draw:
            self.ship_obj = MultiDraw(ship_id)
        elif self.ship_type == config.ShipType.adcp:
            self.ship_obj = Adcp(ship_id)
        elif self.ship_type == config.ShipType.multi_draw_detect:
            self.ship_obj = MultiDrawDetect(ship_id)
        elif self.ship_type == config.ShipType.multi_draw_detect_adcp:
            self.ship_obj = MultiDrawDetectAdcp(ship_id)
        else:
            print('错误船号ship_code', ship_code)


class WaterDetect:
    def __init__(self, ship_id):
        self.ship_id = ship_id
        self.ship_code = 'XXLJC4LCGSCSD1DA%03d' % ship_id

    # 立即抽水
    def draw_sub(self, b_draw, bottle_id, draw_deep, draw_capacity, data_manager_obj):
        """
        @param b_draw: 抽水
        @param bottle_id: 抽水瓶号
        @param draw_deep: 抽水深度
        @param draw_capacity: 抽水容量
        @param data_manager_obj: 数据管理对象
        @return:
        """
        # 判断是否抽水  点击抽水情况
        draw_scale = 1.0  # 抽水放大系数  不同船只抽水速度不一样
        if self.ship_id == 8:  # 8号船放大1.2倍
            draw_scale = 0.75
        if self.ship_id == 9:  # 9号船放大1.4倍
            draw_scale = 1.4
        if b_draw:
            send_data = 'S2,%d,%d,%dZ' % (bottle_id,
                                          int(draw_deep * 10),
                                          int(draw_scale * draw_capacity / 10))
            if data_manager_obj.pre_draw_info != send_data:
                data_manager_obj.pre_draw_info = send_data
                print('设置数据#########################')
                data_manager_obj.set_send_data(send_data, 2)
        else:
            send_data = 'S2,0,0,0Z'
            if data_manager_obj.pre_draw_info != send_data:
                data_manager_obj.pre_draw_info = send_data
                print('设置数据#########################')
                data_manager_obj.set_send_data(send_data, 2)

    # 判断怎么样抽水
    def draw(self, data_manager_obj):
        """
        抽水控制函数
        """
        # 前端发送抽水深度和抽水时间
        if data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw:
            data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_bottle_id = 5
            data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_deep = 0.5
            data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity = 1000
            temp_draw_bottle_id = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_bottle_id
            temp_draw_deep = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_deep
            temp_draw_capacity = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity
            data_manager_obj.current_draw_bottle = temp_draw_bottle_id
            data_manager_obj.current_draw_deep = temp_draw_deep
            data_manager_obj.current_draw_capacity = temp_draw_capacity
            # print('#################前端设置抽水瓶号 深度 容量:', temp_draw_bottle_id, temp_draw_deep, temp_draw_capacity)
            self.draw_sub(True, temp_draw_bottle_id, temp_draw_deep, temp_draw_capacity, data_manager_obj)
            # 收到32返回抽水结束
            if data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id) and \
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 4 and \
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[
                        0] == data_manager_obj.current_draw_bottle:
                print('抽水完成设置draw为0', data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id))
                data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 0
                data_manager_obj.draw_over_bottle_info = [data_manager_obj.current_draw_bottle,
                                                          data_manager_obj.current_draw_deep,
                                                          data_manager_obj.current_draw_capacity]
                if data_manager_obj.current_draw_bottle == 7:
                    data_manager_obj.b_draw_over_send_detect_data = 1
                else:
                    data_manager_obj.b_draw_over_send_data = 1
        # 前端没发送抽水且不是任务抽水
        elif not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and not data_manager_obj.sort_task_list:
            data_manager_obj.dump_draw_list = [0, 0]
            # print('结束抽水')
            self.draw_sub(False, 0, 0, 0, data_manager_obj)
        if data_manager_obj.current_arriver_index == len(data_manager_obj.sort_task_done_list):
            return
        # if data_manager_obj.current_arriver_index is not None:
        # print('到达任务点11111111',
        #       data_manager_obj.sort_task_done_list,
        #       data_manager_obj.current_arriver_index,)
        # print('到达任务点222222222222',data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index])
        if data_manager_obj.current_arriver_index is not None and data_manager_obj.sort_task_done_list and \
                data_manager_obj.sort_task_done_list[
                    data_manager_obj.current_arriver_index].count(0) > 0:  # 是否是使用预先存储任务
            # data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 1
            data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_bottle_id = None
            data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_deep = None
            data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity = None
            index = data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index].index(0)
            temp_draw_bottle_id = 5
            temp_draw_deep = 50
            temp_bottle_amount = 20
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
            self.draw_sub(True, temp_draw_bottle_id, bottle_deep, bottle_amount, data_manager_obj)
            print('data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)',
                  data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id))
            print('data_manager_obj.current_draw_bottle', data_manager_obj.current_draw_bottle)
            if data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id) and \
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 4 and \
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[
                        0] == data_manager_obj.current_draw_bottle:
                data_manager_obj.is_need_update_plan = 1  # 抽完水后需要更新任务状态
                data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index][index] = 1
                if data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index].count(0) == 0:
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 0
                print('##########任务抽水完成一个', data_manager_obj.current_arriver_index,
                      data_manager_obj.sort_task_done_list)
                data_manager_obj.draw_over_bottle_info = [data_manager_obj.current_draw_bottle,
                                                          data_manager_obj.current_draw_deep,
                                                          data_manager_obj.current_draw_capacity]
                if data_manager_obj.current_draw_bottle == 5:
                    data_manager_obj.b_draw_over_send_detect_data = 1
                else:
                    data_manager_obj.b_draw_over_send_data = 1
        # 前端没抽水 且任务模式当前点位全部抽完 则收回杆子
        elif not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and \
                data_manager_obj.current_arriver_index is not None and data_manager_obj.sort_task_done_list and \
                data_manager_obj.sort_task_done_list[
                    data_manager_obj.current_arriver_index].count(0) == 0:
            data_manager_obj.dump_draw_list = [0, 0]
            # print('任务停止抽水')
            # print('data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw',
            #       data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw)
            # print('data_manager_obj.is_need_update_plan', data_manager_obj.is_need_update_plan)
            # print('self.tcp_server_obj.ship_draw_dict.get(self.ship_id)',
            #       data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id))
            self.draw_sub(False, 0, 0, 0, data_manager_obj)
        # 更新倒计时抽水时间
        # print('22222222222',data_manager_obj.tcp_server_obj.ship_draw_dict,self.ship_id)
        if data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id):
            if data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 2:
                data_manager_obj.dump_draw_list = [
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[2],
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[3]]
            else:
                data_manager_obj.dump_draw_list = [0, 0]

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
            print('data_manager_obj.sort_task_list', data_manager_obj.sort_task_list)
            for i in data_manager_obj.sort_task_list:
                if i.get("type") == 1:  # 检测点添加到监测点轨迹中
                    data_manager_obj.sample_index.append(1)
                    data_manager_obj.sort_task_done_list.append([0])
                else:
                    data_manager_obj.sample_index.append(0)
                    data_manager_obj.sort_task_done_list.append([])
                path_planning_data.get("sampling_points").append(i.get("lnglat"))
                path_planning_data.get("path_points").append(i.get("lnglat"))
            data_manager_obj.send(method='mqtt',
                                  topic='path_planning_%s' % self.ship_code,
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
                                "deviceId": self.ship_code,
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
                                "deviceId": self.ship_code,
                                "planId": ""
                                }
            print('全部完成更新任务消息', update_plan_data)
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
                                "deviceId": self.ship_code,
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
        if data_manager_obj.b_draw_over_send_detect_data:
            if data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code:
                data_manager_obj.data_define_obj.pool_code = data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code
            detect_data = data_manager_obj.data_define_obj.detect
            detect_data.update({'mapId': data_manager_obj.data_define_obj.pool_code})
            detect_data.update({'deviceId': data_manager_obj.ship_code})
            # 更新真实数据
            mqtt_send_detect_data = data_define.fake_detect_data(detect_data)
            if self.ship_id in data_manager_obj.tcp_server_obj.ship_detect_data_dict:
                ec_data = data_manager_obj.tcp_server_obj.ship_detect_data_dict.get(self.ship_id)[3]
                ec_data = data_valid.valid_water_data(config.WaterType.EC, ec_data)
                mqtt_send_detect_data['water'].update({'EC': ec_data})
                do_data = data_manager_obj.tcp_server_obj.ship_detect_data_dict.get(self.ship_id)[2]
                do_data = data_valid.valid_water_data(config.WaterType.DO, do_data)
                mqtt_send_detect_data['water'].update({'DO': do_data})
                td_data = data_manager_obj.tcp_server_obj.ship_detect_data_dict.get(self.ship_id)[4]
                td_data = data_valid.valid_water_data(config.WaterType.TD, td_data)
                mqtt_send_detect_data['water'].update({'TD': td_data})
                ph_data = data_manager_obj.tcp_server_obj.ship_detect_data_dict.get(self.ship_id)[1]
                ph_data = data_valid.valid_water_data(config.WaterType.pH, ph_data)
                mqtt_send_detect_data['water'].update({'pH': ph_data})
                wt_data = data_manager_obj.tcp_server_obj.ship_detect_data_dict.get(self.ship_id)[0]
                wt_data = data_valid.valid_water_data(config.WaterType.wt, wt_data)
                mqtt_send_detect_data['water'].update({'wt': wt_data})
            # 替换键
            for k_all, v_all in data_define.name_mappings.items():
                for old_key, new_key in v_all.items():
                    pop_value = mqtt_send_detect_data[k_all].pop(old_key)
                    mqtt_send_detect_data[k_all].update({new_key: pop_value})
            # 添加经纬度
            mqtt_send_detect_data.update({'jwd': json.dumps(data_manager_obj.lng_lat)})
            mqtt_send_detect_data.update({'gjwd': json.dumps(data_manager_obj.gaode_lng_lat)})
            mqtt_send_detect_data.update(mqtt_send_detect_data.get('water'))
            if data_manager_obj.action_id:
                mqtt_send_detect_data.update({'planId': data_manager_obj.action_id})
            # if data_manager_obj.creator:
            #     mqtt_send_detect_data.update({"creator": data_manager_obj.creator})
            print('水质数据', mqtt_send_detect_data)
            data_manager_obj.send(method='mqtt', topic='detect_data_%s' % data_manager_obj.data_define_obj.ship_code,
                                  data=mqtt_send_detect_data,
                                  qos=0)
            if len(data_manager_obj.data_define_obj.pool_code) > 0:
                mqtt_send_detect_data.update({'mapId': data_manager_obj.data_define_obj.pool_code})
                return_data = data_manager_obj.server_data_obj.send_server_http_data('POST',
                                                                                     mqtt_send_detect_data,
                                                                                     config.http_data_save,
                                                                                     token=data_manager_obj.token)
                print('发送检测数据返回:', return_data, json.loads(return_data.content))
                if return_data:
                    content_data = json.loads(return_data.content)
                    if content_data.get("code") not in [200, 20000]:
                        data_manager_obj.logger.error({'POST发送检测请求失败': content_data})
                    else:
                        # 发送结束改为False
                        data_manager_obj.b_draw_over_send_detect_data = False
                    data_manager_obj.logger.info({"本地保存检测数据": mqtt_send_detect_data})
                    # if data_manager_obj.server_data_obj.mqtt_send_get_obj.scan_gap == 25:
                    #     data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 1


class MultiDraw:

    def __init__(self, ship_id):
        self.ship_id = ship_id
        self.ship_code = 'XXLJC4LCGSCSD1DA%03d' % ship_id

    # 立即抽水
    def draw_sub(self, b_draw, bottle_id, draw_deep, draw_capacity, data_manager_obj):
        """
        @param b_draw: 抽水
        @param bottle_id: 抽水瓶号
        @param draw_deep: 抽水深度
        @param draw_capacity: 抽水容量
        @param data_manager_obj: 数据管理对象
        @return:
        """
        # 判断是否抽水  点击抽水情况
        draw_scale = 1.0  # 抽水放大系数  不同船只抽水速度不一样
        if self.ship_id == 8:  # 8号船放大1.2倍
            draw_scale = 0.75
        if b_draw:
            # data_manager_obj.tcp_send_data = 'S2,%d,%d,%dZ' % (bottle_id,
            #                                                    int(draw_deep * 10),
            #                                                    int(draw_capacity / 10))
            send_data = 'S2,%d,%d,%dZ' % (bottle_id,
                                          int(draw_deep * 10),
                                          int(draw_scale * draw_capacity / 10))
            if data_manager_obj.pre_draw_info != send_data:
                data_manager_obj.pre_draw_info = send_data
                print('设置数据#########################')
                data_manager_obj.set_send_data(send_data, 2)
        else:
            # data_manager_obj.tcp_send_data = 'S2,0,0,0Z'
            send_data = 'S2,0,0,0Z'
            if data_manager_obj.pre_draw_info != send_data:
                data_manager_obj.pre_draw_info = send_data
                print('设置数据#########################')
                data_manager_obj.set_send_data(send_data, 2)

    # 立即抽水
    # def draw_sub(self, b_draw, bottle_id, draw_deep, draw_capacity, data_manager_obj):
    #     """
    #     @param b_draw: 抽水
    #     @param bottle_id: 抽水瓶号
    #     @param draw_deep: 抽水深度
    #     @param draw_capacity: 抽水容量
    #     @param data_manager_obj: 数据管理对象
    #     @return:
    #     """
    #     # 判断是否抽水  点击抽水情况
    #     # 判断是否抽水  点击抽水情况
    #     if b_draw:
    #         # data_manager_obj.tcp_send_data = 'S2,%d,%d,%dZ' % (bottle_id,
    #         #                                                    int(draw_deep * 10),
    #         #                                                    int(draw_capacity / 10))
    #         send_data = 'S2,%d,%d,%dZ' % (bottle_id,
    #                                       int(draw_deep * 10),
    #                                       int(draw_capacity / 10))
    #         data_manager_obj.set_send_data(send_data, 2)
    #     elif data_manager_obj.b_need_stop_draw:
    #         # data_manager_obj.tcp_send_data = 'S2,0,0,0Z'
    #         send_data = 'S2,0,0,0Z'
    #         data_manager_obj.set_send_data(send_data, 2)

    # 判断怎么样抽水
    def draw(self, data_manager_obj):
        """
        抽水控制函数
        """
        # 前端发送抽水深度和抽水时间
        if data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_bottle_id and \
                data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_deep and \
                data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity:
            temp_draw_bottle_id = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_bottle_id
            temp_draw_deep = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_deep
            temp_draw_capacity = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity
            data_manager_obj.current_draw_bottle = temp_draw_bottle_id
            data_manager_obj.current_draw_deep = temp_draw_deep
            data_manager_obj.current_draw_capacity = temp_draw_capacity
            # print('#################前端设置抽水瓶号 深度 容量:', temp_draw_bottle_id, temp_draw_deep, temp_draw_capacity)
            self.draw_sub(True, temp_draw_bottle_id, temp_draw_deep, temp_draw_capacity, data_manager_obj)
            # 收到32返回抽水结束
            if data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id) and \
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 4 and \
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[
                        0] == data_manager_obj.current_draw_bottle:
                print('抽水完成设置draw为0', data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id))
                data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 0
                data_manager_obj.draw_over_bottle_info = [data_manager_obj.current_draw_bottle,
                                                          data_manager_obj.current_draw_deep,
                                                          data_manager_obj.current_draw_capacity]
                if data_manager_obj.current_draw_bottle == 7:
                    data_manager_obj.b_draw_over_send_detect_data = 1
                else:
                    data_manager_obj.b_draw_over_send_data = 1
        # 前端没发送抽水且不是任务抽水
        elif not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and not data_manager_obj.sort_task_list:
            data_manager_obj.dump_draw_list = [0, 0]
            # print('结束抽水')
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
            data_manager_obj.b_need_stop_draw = 1
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
            self.draw_sub(True, temp_draw_bottle_id, bottle_deep, bottle_amount, data_manager_obj)
            print('data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)',
                  data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id))
            print('data_manager_obj.current_draw_bottle', data_manager_obj.current_draw_bottle)
            if data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id) and \
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 4 and \
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[
                        0] == data_manager_obj.current_draw_bottle:
                data_manager_obj.is_need_update_plan = 1  # 抽完水后需要更新任务状态
                data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index][index] = 1
                if data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index].count(0) == 0:
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 0
                print('##########任务抽水完成一个', data_manager_obj.current_arriver_index,
                      data_manager_obj.sort_task_done_list)
                data_manager_obj.draw_over_bottle_info = [data_manager_obj.current_draw_bottle,
                                                          data_manager_obj.current_draw_deep,
                                                          data_manager_obj.current_draw_capacity]
                if data_manager_obj.current_draw_bottle == 7:
                    data_manager_obj.b_draw_over_send_detect_data = 1
                else:
                    data_manager_obj.b_draw_over_send_data = 1
        # 前端没抽水 且任务模式当前点位全部抽完 则收回杆子
        # print(''',data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index])
        elif not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and \
                data_manager_obj.current_arriver_index is not None and data_manager_obj.sort_task_done_list and \
                data_manager_obj.sort_task_done_list[
                    data_manager_obj.current_arriver_index].count(0) == 0:
            data_manager_obj.dump_draw_list = [0, 0]
            print('任务停止抽水')
            print('data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw',
                  data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw)
            print('data_manager_obj.is_need_update_plan', data_manager_obj.is_need_update_plan)
            print('self.tcp_server_obj.ship_draw_dict.get(self.ship_id)',
                  data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id))
            self.draw_sub(False, 0, 0, 0, data_manager_obj)
        # 更新倒计时抽水时间
        if data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id):
            if data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 2:
                data_manager_obj.dump_draw_list = [
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[2],
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[3]]
            else:
                data_manager_obj.dump_draw_list = [0, 0]

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
                                  topic='path_planning_%s' % self.ship_code,
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
                                "deviceId": self.ship_code,
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
        if data_manager_obj.is_plan_all_arrive and not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and not data_manager_obj.b_draw_over_send_data:
            print('data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id',
                  data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id)
            print('data_manager_obj.is_plan_all_arrive', data_manager_obj.is_plan_all_arrive)
            update_plan_data = {"id": data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id,
                                "taskTem": '[]',
                                "state": 0,
                                "deviceId": self.ship_code,
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
                                "deviceId": self.ship_code,
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
            if data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code:
                data_manager_obj.data_define_obj.pool_code = data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code
            draw_data = {}
            draw_data.update({'deviceId': data_manager_obj.ship_code})
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
            # if data_manager_obj.creator:
            #     draw_data.update({"creator": data_manager_obj.creator})
            if data_manager_obj.action_id:
                draw_data.update({'planId': data_manager_obj.action_id})
            data_manager_obj.send(method='mqtt', topic='draw_data_%s' % self.ship_code, data=draw_data,
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
                                                                  self.ship_code,
                                                                  [draw_data['jwd'], draw_data['bottleNum'],
                                                                   draw_data['deep'], draw_data['capacity']],
                                                                  token=data_manager_obj.token)
                    # 请求图片成功添加图片路径 失败则不添加
                    if server_save_img_path:
                        draw_data.update({"pic": server_save_img_path})
                    # print('draw_data', draw_data)
                    return_data = data_manager_obj.server_data_obj.send_server_http_data('POST',
                                                                                         draw_data,
                                                                                         config.http_draw_save,
                                                                                         token=data_manager_obj.token)
                    # print('上传采样数据返回:', return_data)
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


class Adcp:
    def __init__(self, ship_id):
        self.save_deep = 0
        self.send_data_time = None
        self.send_data_lng_lat = None
        self.b_send_data = False
        self.ship_id = ship_id
        self.ship_code = 'XXLJC4LCGSCSD1DA%03d' % ship_id

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

    def draw(self, data_manager_obj):
        self.send_data(data_manager_obj)

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
                                  topic='path_planning_%s' % self.ship_code,
                                  data=path_planning_data,
                                  qos=0)
            print('mqtt任务经纬度数据', path_planning_data)
            print("task_id", data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id)
            data_manager_obj.has_task = 0
        if data_manager_obj.server_data_obj.mqtt_send_get_obj.cancel_action == 1 and not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw:  # 取消行动
            print('data_manager_obj.server_data_obj.mqtt_send_get_obj.cancel_action',
                  data_manager_obj.server_data_obj.mqtt_send_get_obj.cancel_action)
            draw_deep_data = {
                "action_type": 2,
                "action_id": data_manager_obj.action_id
            }
            # 发送话题绘图
            data_manager_obj.send(method='mqtt',
                                  topic='action_%s' % self.ship_code,
                                  data=draw_deep_data,
                                  qos=0)
            data_manager_obj.server_data_obj.mqtt_send_get_obj.cancel_action = 0
            data_manager_obj.server_data_obj.mqtt_send_get_obj.control_move_direction = -1
            update_plan_data = {"id": data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id,
                                # "taskTem": '[]',
                                "state": 0,
                                "deviceId": self.ship_code,
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
                                "deviceId": self.ship_code,
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
            if len(data_manager_obj.server_data_obj.mqtt_send_get_obj.sampling_points_status) > 0 and \
                    0 in data_manager_obj.server_data_obj.mqtt_send_get_obj.sampling_points_status:
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
                                "deviceId": self.ship_code,
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
        if not data_manager_obj.lng_lat:
            return
        if isinstance(self.send_data_lng_lat, list):
            delta_distance = lng_lat_calculate.distanceFromCoordinate(self.send_data_lng_lat[0],
                                                                      self.send_data_lng_lat[1],
                                                                      data_manager_obj.lng_lat[0],
                                                                      data_manager_obj.lng_lat[1],
                                                                      )
        else:
            delta_distance = 0
        if self.send_data_lng_lat is None or self.send_data_time is None:
            self.b_send_data = True
            self.send_data_lng_lat = data_manager_obj.lng_lat
            self.send_data_time = time.time()
            # print('初始测量')
        elif isinstance(self.send_data_lng_lat,
                        list) and delta_distance > data_manager_obj.server_data_obj.mqtt_send_get_obj.adcp_record_distance:
            self.b_send_data = True
            # print(delta_distance,'超过距离测量', data_manager_obj.server_data_obj.mqtt_send_get_obj.adcp_record_distance)
        # elif time.time() - self.send_data_time > data_manager_obj.server_data_obj.mqtt_send_get_obj.adcp_record_time:
        #     self.b_send_data = True
        # print(time.time(),'超出时间测量', data_manager_obj.server_data_obj.mqtt_send_get_obj.adcp_record_time)

        if self.b_send_data and data_manager_obj.deep != -1 and data_manager_obj.deep > 0.01:
            if data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code:
                data_manager_obj.data_define_obj.pool_code = data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code
            deep_data = {}
            deep_data.update({'deep': data_manager_obj.deep})
            deep_data.update({'deviceId': data_manager_obj.ship_code})
            deep_data.update({'mapId': data_manager_obj.data_define_obj.pool_code})
            # 添加经纬度
            deep_data.update({'jwd': json.dumps(data_manager_obj.lng_lat)})
            deep_data.update({'gjwd': json.dumps(data_manager_obj.gaode_lng_lat)})
            if data_manager_obj.action_id:
                deep_data.update({'planId': data_manager_obj.action_id})
            data_manager_obj.send(method='mqtt', topic='deep_data_%s' % data_manager_obj.ship_code,
                                  data={'deep': data_manager_obj.deep},
                                  qos=0)
            """
            {
            'deep':12,
            'deviceId':'XXLJC4LCGSCSD1DA007',
            'mapId':1539911190092759042,
            'jwd':[114.321321,31.312231]
            'gjwd':[114.121321,31.112231]
            }
            """
            # 发送到服务器
            if len(data_manager_obj.data_define_obj.pool_code) > 0:
                try:
                    # 没有数据 偏差距离不变
                    if not data_manager_obj.lng_lat or not data_manager_obj.server_data_obj.mqtt_send_get_obj.adcp:
                        return
                    print('深度数据', deep_data)
                    return_data = data_manager_obj.server_data_obj.send_server_http_data('POST',
                                                                                         deep_data,
                                                                                         config.http_deep_save,
                                                                                         token=data_manager_obj.token)
                    print('深度数据返回:', return_data)
                    self.save_deep = data_manager_obj.deep
                    self.send_data_lng_lat = data_manager_obj.lng_lat
                    self.send_data_time = time.time()
                    if return_data:
                        content_data = json.loads(return_data.content)
                        if content_data.get("code") != 200:
                            data_manager_obj.logger.error({'发送深度数据失败': content_data})
                        else:
                            data_manager_obj.logger.info({"发送深度数据成功": deep_data})
                            self.b_send_data = False
                except Exception as e:
                    data_manager_obj.logger.info({"发送深度数据error": e})


class MultiDrawDetect:

    def __init__(self, ship_id):
        self.ship_id = ship_id
        self.ship_code = 'XXLJC4LCGSCSD1DA%03d' % ship_id

    # 立即抽水
    def draw_sub(self, b_draw, bottle_id, draw_deep, draw_capacity, data_manager_obj):
        """
        @param b_draw: 抽水
        @param bottle_id: 抽水瓶号
        @param draw_deep: 抽水深度
        @param draw_capacity: 抽水容量
        @param data_manager_obj: 数据管理对象
        @return:
        """
        # 判断是否抽水  点击抽水情况
        draw_scale = 1.0  # 抽水放大系数  不同船只抽水速度不一样
        if self.ship_id == 8:  # 8号船放大1.2倍
            draw_scale = 0.79
        if b_draw:
            # data_manager_obj.tcp_send_data = 'S2,%d,%d,%dZ' % (bottle_id,
            #                                                    int(draw_deep * 10),
            #                                                    int(draw_capacity / 10))
            send_data = 'S2,%d,%d,%dZ' % (bottle_id,
                                          int(draw_deep * 10),
                                          int(draw_scale * draw_capacity / 10))
            if data_manager_obj.pre_draw_info != send_data:
                data_manager_obj.pre_draw_info = send_data
                print('设置数据#########################')
                data_manager_obj.set_send_data(send_data, 2)
        else:
            # data_manager_obj.tcp_send_data = 'S2,0,0,0Z'
            send_data = 'S2,0,0,0Z'
            if data_manager_obj.pre_draw_info != send_data:
                data_manager_obj.pre_draw_info = send_data
                print('设置数据#########################')
                data_manager_obj.set_send_data(send_data, 2)
        # else :
        #     # data_manager_obj.tcp_send_data = 'S2,0,0,0Z'
        #     send_data = 'S2,0,0,0Z'
        #     data_manager_obj.set_send_data(send_data, 2)

    # 判断怎么样抽水
    def draw(self, data_manager_obj):
        """
        抽水控制函数
        """
        # 前端发送抽水深度和抽水时间
        if data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_bottle_id and \
                data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_deep and \
                data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity:
            data_manager_obj.b_need_stop_draw = 1
            temp_draw_bottle_id = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_bottle_id
            temp_draw_deep = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_deep
            temp_draw_capacity = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity
            data_manager_obj.current_draw_bottle = temp_draw_bottle_id
            data_manager_obj.current_draw_deep = temp_draw_deep
            data_manager_obj.current_draw_capacity = temp_draw_capacity
            # print('#################前端设置抽水瓶号 深度 容量:', temp_draw_bottle_id, temp_draw_deep, temp_draw_capacity)
            self.draw_sub(True, temp_draw_bottle_id, temp_draw_deep, temp_draw_capacity, data_manager_obj)
            # 当抽水消息被确认后再判断抽水是否结束  收到32返回抽水结束
            if data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id) and \
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 4 and \
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[
                        0] == data_manager_obj.current_draw_bottle:
                print('抽水完成设置draw为0', data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id))
                data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 0
                data_manager_obj.draw_over_bottle_info = [data_manager_obj.current_draw_bottle,
                                                          data_manager_obj.current_draw_deep,
                                                          data_manager_obj.current_draw_capacity]
                if data_manager_obj.current_draw_bottle == 7:
                    data_manager_obj.b_draw_over_send_detect_data = 1
                else:
                    data_manager_obj.b_draw_over_send_data = 1
        # 前端没发送抽水且不是任务抽水
        elif not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and not data_manager_obj.sort_task_list:
            data_manager_obj.dump_draw_list = [0, 0]
            # print('结束抽水')
            self.draw_sub(False, 0, 0, 0, data_manager_obj)
        if data_manager_obj.current_arriver_index == len(data_manager_obj.sort_task_done_list):
            return
        # if data_manager_obj.current_arriver_index is not None:
        #     print('到达任务点', data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index],
        #           data_manager_obj.sort_task_done_list,
        #           data_manager_obj.current_arriver_index)
        if data_manager_obj.current_arriver_index is not None and data_manager_obj.sort_task_done_list and \
                data_manager_obj.sort_task_done_list[
                    data_manager_obj.current_arriver_index].count(0) > 0:  # 是否是使用预先存储任务
            # data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 1
            data_manager_obj.b_need_stop_draw = 1
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
            # print('index temp_draw_bottle_id,temp_draw_deep,temp_draw_time', index, temp_draw_bottle_id,
            #       bottle_deep,
            #       bottle_amount)
            self.draw_sub(True, temp_draw_bottle_id, bottle_deep, bottle_amount, data_manager_obj)
            # print('data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)',
            #       data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id))
            # print('data_manager_obj.current_draw_bottle', data_manager_obj.current_draw_bottle)
            if data_manager_obj.tcp_server_obj.ship_id_send_dict.get(self.ship_id)[
                2] == "" and data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id) and \
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 4 and \
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[
                        0] == data_manager_obj.current_draw_bottle:
                data_manager_obj.is_need_update_plan = 1  # 抽完水后需要更新任务状态
                data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index][index] = 1
                if data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index].count(0) == 0:
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 0
                # print('##########任务抽水完成一个', data_manager_obj.current_arriver_index,
                #       data_manager_obj.sort_task_done_list)
                data_manager_obj.draw_over_bottle_info = [data_manager_obj.current_draw_bottle,
                                                          data_manager_obj.current_draw_deep,
                                                          data_manager_obj.current_draw_capacity]
                if data_manager_obj.current_draw_bottle == 5:
                    data_manager_obj.b_draw_over_send_detect_data = 1
                else:
                    data_manager_obj.b_draw_over_send_data = 1
        # 前端没抽水 且任务模式当前点位全部抽完 则收回杆子
        # print(''',data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index])
        elif not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and \
                data_manager_obj.current_arriver_index is not None and data_manager_obj.sort_task_done_list and \
                data_manager_obj.sort_task_done_list[
                    data_manager_obj.current_arriver_index].count(0) == 0:
            data_manager_obj.dump_draw_list = [0, 0]
            # print('任务停止抽水')
            # print('data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw',
            #       data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw)
            # print('data_manager_obj.is_need_update_plan', data_manager_obj.is_need_update_plan)
            # print('self.tcp_server_obj.ship_draw_dict.get(self.ship_id)',
            #       data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id))
            self.draw_sub(False, 0, 0, 0, data_manager_obj)
        # 更新倒计时抽水时间
        if data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id):
            if data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 2:
                data_manager_obj.dump_draw_list = [
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[2],
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[3]]
            else:
                data_manager_obj.dump_draw_list = [0, 0]

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
                            temp_list.update({"lnglat": task.get("lnglat")})
                            temp_list.update({"type": task.get("type")})  # 0:路径点  1：采样点  2：采样检测点   3： 检测点
                            temp_list.update({"switchValue": task.get("switchValue")})  # True:采样检测点  False：路径点
                            draw_info = []
                            # 如果是检测点或者采样检测点就向数据中添加瓶号5
                            if task.get("switchValue"):
                                draw_info.append((5, 50, 20))
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
                                  topic='path_planning_%s' % self.ship_code,
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
                                "deviceId": self.ship_code,
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
        if data_manager_obj.is_plan_all_arrive and not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and not data_manager_obj.b_draw_over_send_data:
            print('data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id',
                  data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id)
            print('data_manager_obj.is_plan_all_arrive', data_manager_obj.is_plan_all_arrive)
            update_plan_data = {"id": data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id,
                                "taskTem": '[]',
                                "state": 0,
                                "deviceId": self.ship_code,
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
                                "deviceId": self.ship_code,
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
            if data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code:
                data_manager_obj.data_define_obj.pool_code = data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code
            draw_data = {}
            draw_data.update({'deviceId': data_manager_obj.ship_code})
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
            # if data_manager_obj.creator:
            #     draw_data.update({"creator": data_manager_obj.creator})
            if data_manager_obj.action_id:
                draw_data.update({'planId': data_manager_obj.action_id})
            data_manager_obj.send(method='mqtt', topic='draw_data_%s' % self.ship_code, data=draw_data,
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
                                                                  self.ship_code,
                                                                  [draw_data['jwd'], draw_data['bottleNum'],
                                                                   draw_data['deep'], draw_data['capacity']],
                                                                  token=data_manager_obj.token)
                    # 请求图片成功添加图片路径 失败则不添加
                    if server_save_img_path:
                        draw_data.update({"pic": server_save_img_path})
                    # print('draw_data', draw_data)
                    return_data = data_manager_obj.server_data_obj.send_server_http_data('POST',
                                                                                         draw_data,
                                                                                         config.http_draw_save,
                                                                                         token=data_manager_obj.token)
                    # print('上传采样数据返回:', return_data)
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
        if data_manager_obj.b_draw_over_send_detect_data:
            # 上传检测数据
            if data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code:
                data_manager_obj.data_define_obj.pool_code = data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code
            detect_data = data_manager_obj.data_define_obj.detect
            detect_data.update({'mapId': data_manager_obj.data_define_obj.pool_code})
            detect_data.update({'deviceId': data_manager_obj.ship_code})
            # 更新真实数据
            mqtt_send_detect_data = data_define.fake_detect_data(detect_data)
            if self.ship_id in data_manager_obj.tcp_server_obj.ship_detect_data_dict:
                ec_data = data_manager_obj.tcp_server_obj.ship_detect_data_dict.get(self.ship_id)[3]
                ec_data = data_valid.valid_water_data(config.WaterType.EC, ec_data)
                mqtt_send_detect_data['water'].update({'EC': ec_data})
                do_data = data_manager_obj.tcp_server_obj.ship_detect_data_dict.get(self.ship_id)[2]
                do_data = data_valid.valid_water_data(config.WaterType.DO, do_data)
                mqtt_send_detect_data['water'].update({'DO': do_data})
                td_data = data_manager_obj.tcp_server_obj.ship_detect_data_dict.get(self.ship_id)[4]
                td_data = data_valid.valid_water_data(config.WaterType.TD, td_data)
                mqtt_send_detect_data['water'].update({'TD': td_data})
                ph_data = data_manager_obj.tcp_server_obj.ship_detect_data_dict.get(self.ship_id)[1]
                ph_data = data_valid.valid_water_data(config.WaterType.pH, ph_data)
                mqtt_send_detect_data['water'].update({'pH': ph_data})
                wt_data = data_manager_obj.tcp_server_obj.ship_detect_data_dict.get(self.ship_id)[0]
                wt_data = data_valid.valid_water_data(config.WaterType.wt, wt_data)
                mqtt_send_detect_data['water'].update({'wt': wt_data})
            # 替换键
            for k_all, v_all in data_define.name_mappings.items():
                for old_key, new_key in v_all.items():
                    pop_value = mqtt_send_detect_data[k_all].pop(old_key)
                    mqtt_send_detect_data[k_all].update({new_key: pop_value})
            # 添加经纬度
            mqtt_send_detect_data.update({'jwd': json.dumps(data_manager_obj.lng_lat)})
            mqtt_send_detect_data.update({'gjwd': json.dumps(data_manager_obj.gaode_lng_lat)})
            mqtt_send_detect_data.update(mqtt_send_detect_data.get('water'))
            if data_manager_obj.action_id:
                mqtt_send_detect_data.update({'planId': data_manager_obj.action_id})
            # if data_manager_obj.creator:
            #     mqtt_send_detect_data.update({"creator": data_manager_obj.creator})
            data_manager_obj.send(method='mqtt', topic='detect_data_%s' % self.ship_code,
                                  data=mqtt_send_detect_data,
                                  qos=0)
            if len(data_manager_obj.data_define_obj.pool_code) > 0:
                mqtt_send_detect_data.update({'mapId': data_manager_obj.data_define_obj.pool_code})
                return_data = data_manager_obj.server_data_obj.send_server_http_data('POST',
                                                                                     mqtt_send_detect_data,
                                                                                     config.http_data_save,
                                                                                     token=data_manager_obj.token)
                print('发送检测数据返回:', return_data, json.loads(return_data.content))
                if return_data:
                    content_data = json.loads(return_data.content)
                    if not content_data.get("success") and content_data.get("code") not in [200, 20000]:
                        data_manager_obj.logger.error({'POST发送检测请求失败': content_data})
                    else:
                        # 发送结束改为False
                        data_manager_obj.b_draw_over_send_detect_data = False
                    data_manager_obj.logger.info({"本地保存检测数据": mqtt_send_detect_data})


class MultiDrawDetectAdcp:

    def __init__(self, ship_id):
        self.save_deep = 0
        self.send_data_time = None
        self.send_data_lng_lat = None
        self.b_send_data = False
        self.ship_id = ship_id
        self.ship_code = 'XXLJC4LCGSCSD1DA%03d' % ship_id

    # 立即抽水
    def draw_sub(self, b_draw, bottle_id, draw_deep, draw_capacity, data_manager_obj):
        """
        @param b_draw: 抽水
        @param bottle_id: 抽水瓶号
        @param draw_deep: 抽水深度
        @param draw_capacity: 抽水容量
        @param data_manager_obj: 数据管理对象
        @return:
        """
        # 判断是否抽水  点击抽水情况
        draw_scale = 1.0  # 抽水放大系数  不同船只抽水速度不一样
        if self.ship_id == 8:  # 8号船放大1.2倍
            draw_scale = 0.79
        if b_draw:
            # data_manager_obj.tcp_send_data = 'S2,%d,%d,%dZ' % (bottle_id,
            #                                                    int(draw_deep * 10),
            #                                                    int(draw_capacity / 10))
            send_data = 'S2,%d,%d,%dZ' % (bottle_id,
                                          int(draw_deep * 10),
                                          int(draw_scale * draw_capacity / 10))
            if data_manager_obj.pre_draw_info != send_data:
                data_manager_obj.pre_draw_info = send_data
                print('设置数据#########################')
                data_manager_obj.set_send_data(send_data, 2)
        else:
            # data_manager_obj.tcp_send_data = 'S2,0,0,0Z'
            send_data = 'S2,0,0,0Z'
            if data_manager_obj.pre_draw_info != send_data:
                data_manager_obj.pre_draw_info = send_data
                print('设置数据#########################')
                data_manager_obj.set_send_data(send_data, 2)
        # else :
        #     # data_manager_obj.tcp_send_data = 'S2,0,0,0Z'
        #     send_data = 'S2,0,0,0Z'
        #     data_manager_obj.set_send_data(send_data, 2)

    # 判断怎么样抽水
    def draw(self, data_manager_obj):
        """
        抽水控制函数
        """
        # 前端发送抽水深度和抽水时间
        if data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_bottle_id and \
                data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_deep and \
                data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity:
            data_manager_obj.b_need_stop_draw = 1
            temp_draw_bottle_id = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_bottle_id
            temp_draw_deep = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_deep
            temp_draw_capacity = data_manager_obj.server_data_obj.mqtt_send_get_obj.draw_capacity
            data_manager_obj.current_draw_bottle = temp_draw_bottle_id
            data_manager_obj.current_draw_deep = temp_draw_deep
            data_manager_obj.current_draw_capacity = temp_draw_capacity
            # print('#################前端设置抽水瓶号 深度 容量:', temp_draw_bottle_id, temp_draw_deep, temp_draw_capacity)
            self.draw_sub(True, temp_draw_bottle_id, temp_draw_deep, temp_draw_capacity, data_manager_obj)
            # 当抽水消息被确认后再判断抽水是否结束  收到32返回抽水结束
            if data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id) and \
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 4 and \
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[
                        0] == data_manager_obj.current_draw_bottle:
                print('抽水完成设置draw为0', data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id))
                data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 0
                data_manager_obj.draw_over_bottle_info = [data_manager_obj.current_draw_bottle,
                                                          data_manager_obj.current_draw_deep,
                                                          data_manager_obj.current_draw_capacity]
                if data_manager_obj.current_draw_bottle == 7:
                    data_manager_obj.b_draw_over_send_detect_data = 1
                else:
                    data_manager_obj.b_draw_over_send_data = 1
        # 前端没发送抽水且不是任务抽水
        elif not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and not data_manager_obj.sort_task_list:
            data_manager_obj.dump_draw_list = [0, 0]
            # print('结束抽水')
            self.draw_sub(False, 0, 0, 0, data_manager_obj)
        if data_manager_obj.current_arriver_index == len(data_manager_obj.sort_task_done_list):
            return
        # if data_manager_obj.current_arriver_index is not None:
        #     print('到达任务点', data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index],
        #           data_manager_obj.sort_task_done_list,
        #           data_manager_obj.current_arriver_index)
        if data_manager_obj.current_arriver_index is not None and data_manager_obj.sort_task_done_list and \
                data_manager_obj.sort_task_done_list[
                    data_manager_obj.current_arriver_index].count(0) > 0:  # 是否是使用预先存储任务
            # data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 1
            data_manager_obj.b_need_stop_draw = 1
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
            # print('index temp_draw_bottle_id,temp_draw_deep,temp_draw_time', index, temp_draw_bottle_id,
            #       bottle_deep,
            #       bottle_amount)
            self.draw_sub(True, temp_draw_bottle_id, bottle_deep, bottle_amount, data_manager_obj)
            # print('data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)',
            #       data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id))
            # print('data_manager_obj.current_draw_bottle', data_manager_obj.current_draw_bottle)
            if data_manager_obj.tcp_server_obj.ship_id_send_dict.get(self.ship_id)[
                2] == "" and data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id) and \
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 4 and \
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[
                        0] == data_manager_obj.current_draw_bottle:
                data_manager_obj.is_need_update_plan = 1  # 抽完水后需要更新任务状态
                data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index][index] = 1
                if data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index].count(0) == 0:
                    data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw = 0
                # print('##########任务抽水完成一个', data_manager_obj.current_arriver_index,
                #       data_manager_obj.sort_task_done_list)
                data_manager_obj.draw_over_bottle_info = [data_manager_obj.current_draw_bottle,
                                                          data_manager_obj.current_draw_deep,
                                                          data_manager_obj.current_draw_capacity]
                if data_manager_obj.current_draw_bottle == 5:
                    data_manager_obj.b_draw_over_send_detect_data = 1
                else:
                    data_manager_obj.b_draw_over_send_data = 1
        # 前端没抽水 且任务模式当前点位全部抽完 则收回杆子
        # print(''',data_manager_obj.sort_task_done_list[data_manager_obj.current_arriver_index])
        elif not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and \
                data_manager_obj.current_arriver_index is not None and data_manager_obj.sort_task_done_list and \
                data_manager_obj.sort_task_done_list[
                    data_manager_obj.current_arriver_index].count(0) == 0:
            data_manager_obj.dump_draw_list = [0, 0]
            # print('任务停止抽水')
            # print('data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw',
            #       data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw)
            # print('data_manager_obj.is_need_update_plan', data_manager_obj.is_need_update_plan)
            # print('self.tcp_server_obj.ship_draw_dict.get(self.ship_id)',
            #       data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id))
            self.draw_sub(False, 0, 0, 0, data_manager_obj)
        # 更新倒计时抽水时间
        if data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id):
            if data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[1] == 2:
                data_manager_obj.dump_draw_list = [
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[2],
                    data_manager_obj.tcp_server_obj.ship_draw_dict.get(self.ship_id)[3]]
            else:
                data_manager_obj.dump_draw_list = [0, 0]

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
                            temp_list.update({"lnglat": task.get("lnglat")})
                            temp_list.update({"type": task.get("type")})  # 0:路径点  1：采样点  2：采样检测点   3： 检测点
                            temp_list.update({"switchValue": task.get("switchValue")})  # True:采样检测点  False：路径点
                            draw_info = []
                            # 如果是检测点或者采样检测点就向数据中添加瓶号5
                            if task.get("switchValue"):
                                draw_info.append((5, 50, 20))
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
                                  topic='path_planning_%s' % self.ship_code,
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
                                "deviceId": self.ship_code,
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
        if data_manager_obj.is_plan_all_arrive and not data_manager_obj.server_data_obj.mqtt_send_get_obj.b_draw and not data_manager_obj.b_draw_over_send_data:
            print('data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id',
                  data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id)
            print('data_manager_obj.is_plan_all_arrive', data_manager_obj.is_plan_all_arrive)
            update_plan_data = {"id": data_manager_obj.server_data_obj.mqtt_send_get_obj.task_id,
                                "taskTem": '[]',
                                "state": 0,
                                "deviceId": self.ship_code,
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
                                "deviceId": self.ship_code,
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
        print('b_draw_over_send_detect_data', data_manager_obj.b_draw_over_send_detect_data)
        if data_manager_obj.b_draw_over_send_data:
            if data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code:
                data_manager_obj.data_define_obj.pool_code = data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code
            draw_data = {}
            draw_data.update({'deviceId': data_manager_obj.ship_code})
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
            # if data_manager_obj.creator:
            #     draw_data.update({"creator": data_manager_obj.creator})
            if data_manager_obj.action_id:
                draw_data.update({'planId': data_manager_obj.action_id})
            data_manager_obj.send(method='mqtt', topic='draw_data_%s' % self.ship_code, data=draw_data,
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
                                                                  self.ship_code,
                                                                  [draw_data['jwd'], draw_data['bottleNum'],
                                                                   draw_data['deep'], draw_data['capacity']],
                                                                  token=data_manager_obj.token)
                    # 请求图片成功添加图片路径 失败则不添加
                    if server_save_img_path:
                        draw_data.update({"pic": server_save_img_path})
                    # print('draw_data', draw_data)
                    return_data = data_manager_obj.server_data_obj.send_server_http_data('POST',
                                                                                         draw_data,
                                                                                         config.http_draw_save,
                                                                                         token=data_manager_obj.token)
                    # print('上传采样数据返回:', return_data)
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
        if data_manager_obj.b_draw_over_send_detect_data:
            # 上传检测数据
            if data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code:
                data_manager_obj.data_define_obj.pool_code = data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code
            detect_data = data_manager_obj.data_define_obj.detect
            detect_data.update({'mapId': data_manager_obj.data_define_obj.pool_code})
            detect_data.update({'deviceId': data_manager_obj.ship_code})
            # 更新真实数据
            mqtt_send_detect_data = data_define.fake_detect_data(detect_data)
            if self.ship_id in data_manager_obj.tcp_server_obj.ship_detect_data_dict:
                ec_data = data_manager_obj.tcp_server_obj.ship_detect_data_dict.get(self.ship_id)[3]
                ec_data = data_valid.valid_water_data(config.WaterType.EC, ec_data)
                mqtt_send_detect_data['water'].update({'EC': ec_data})
                do_data = data_manager_obj.tcp_server_obj.ship_detect_data_dict.get(self.ship_id)[2]
                do_data = data_valid.valid_water_data(config.WaterType.DO, do_data)
                mqtt_send_detect_data['water'].update({'DO': do_data})
                td_data = data_manager_obj.tcp_server_obj.ship_detect_data_dict.get(self.ship_id)[4]
                td_data = data_valid.valid_water_data(config.WaterType.TD, td_data)
                mqtt_send_detect_data['water'].update({'TD': td_data})
                ph_data = data_manager_obj.tcp_server_obj.ship_detect_data_dict.get(self.ship_id)[1]
                ph_data = data_valid.valid_water_data(config.WaterType.pH, ph_data)
                mqtt_send_detect_data['water'].update({'pH': ph_data})
                wt_data = data_manager_obj.tcp_server_obj.ship_detect_data_dict.get(self.ship_id)[0]
                wt_data = data_valid.valid_water_data(config.WaterType.wt, wt_data)
                mqtt_send_detect_data['water'].update({'wt': wt_data})
            # 替换键
            for k_all, v_all in data_define.name_mappings.items():
                for old_key, new_key in v_all.items():
                    pop_value = mqtt_send_detect_data[k_all].pop(old_key)
                    mqtt_send_detect_data[k_all].update({new_key: pop_value})
            # 添加经纬度
            mqtt_send_detect_data.update({'jwd': json.dumps(data_manager_obj.lng_lat)})
            mqtt_send_detect_data.update({'gjwd': json.dumps(data_manager_obj.gaode_lng_lat)})
            mqtt_send_detect_data.update(mqtt_send_detect_data.get('water'))
            if data_manager_obj.action_id:
                mqtt_send_detect_data.update({'planId': data_manager_obj.action_id})
            # if data_manager_obj.creator:
            #     mqtt_send_detect_data.update({"creator": data_manager_obj.creator})
            data_manager_obj.send(method='mqtt', topic='detect_data_%s' % self.ship_code,
                                  data=mqtt_send_detect_data,
                                  qos=0)
            if len(data_manager_obj.data_define_obj.pool_code) > 0:
                mqtt_send_detect_data.update({'mapId': data_manager_obj.data_define_obj.pool_code})
                return_data = data_manager_obj.server_data_obj.send_server_http_data('POST',
                                                                                     mqtt_send_detect_data,
                                                                                     config.http_data_save,
                                                                                     token=data_manager_obj.token)
                print('发送检测数据返回:', return_data, json.loads(return_data.content))
                if return_data:
                    content_data = json.loads(return_data.content)
                    if not content_data.get("success") and content_data.get("code") not in [200, 20000]:
                        data_manager_obj.logger.error({'POST发送检测请求失败': content_data})
                    else:
                        # 发送结束改为False
                        data_manager_obj.b_draw_over_send_detect_data = False
                    data_manager_obj.logger.info({"本地保存检测数据": mqtt_send_detect_data})
        if not data_manager_obj.lng_lat:
            return
        if isinstance(self.send_data_lng_lat, list):
            delta_distance = lng_lat_calculate.distanceFromCoordinate(self.send_data_lng_lat[0],
                                                                      self.send_data_lng_lat[1],
                                                                      data_manager_obj.lng_lat[0],
                                                                      data_manager_obj.lng_lat[1],
                                                                      )
        else:
            delta_distance = 0
        if self.send_data_lng_lat is None or self.send_data_time is None:
            self.b_send_data = True
            self.send_data_lng_lat = data_manager_obj.lng_lat
            self.send_data_time = time.time()
            # print('初始测量')
        elif isinstance(self.send_data_lng_lat,
                        list) and delta_distance > data_manager_obj.server_data_obj.mqtt_send_get_obj.adcp_record_distance:
            self.b_send_data = True
            # print(delta_distance,'超过距离测量', data_manager_obj.server_data_obj.mqtt_send_get_obj.adcp_record_distance)
        # elif time.time() - self.send_data_time > data_manager_obj.server_data_obj.mqtt_send_get_obj.adcp_record_time:
        #     self.b_send_data = True
        # print(time.time(),'超出时间测量', data_manager_obj.server_data_obj.mqtt_send_get_obj.adcp_record_time)

        if self.b_send_data and data_manager_obj.deep != -1 and data_manager_obj.deep > 0.01:
            if data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code:
                data_manager_obj.data_define_obj.pool_code = data_manager_obj.server_data_obj.mqtt_send_get_obj.pool_code
            deep_data = {}
            deep_data.update({'deep': data_manager_obj.deep})
            deep_data.update({'deviceId': data_manager_obj.ship_code})
            deep_data.update({'mapId': data_manager_obj.data_define_obj.pool_code})
            # 添加经纬度
            deep_data.update({'jwd': json.dumps(data_manager_obj.lng_lat)})
            deep_data.update({'gjwd': json.dumps(data_manager_obj.gaode_lng_lat)})
            if data_manager_obj.action_id:
                deep_data.update({'planId': data_manager_obj.action_id})
            data_manager_obj.send(method='mqtt', topic='deep_data_%s' % data_manager_obj.ship_code,
                                  data={'deep': data_manager_obj.deep},
                                  qos=0)
            """
            {
            'deep':12,
            'deviceId':'XXLJC4LCGSCSD1DA007',
            'mapId':1539911190092759042,
            'jwd':[114.321321,31.312231]
            'gjwd':[114.121321,31.112231]
            }
            """
            # 发送到服务器
            if len(data_manager_obj.data_define_obj.pool_code) > 0:
                try:
                    # 没有数据 偏差距离不变  暂时不适用判断
                    # if not data_manager_obj.lng_lat or not data_manager_obj.server_data_obj.mqtt_send_get_obj.adcp:
                    #     return
                    print('深度数据', deep_data)
                    return_data = data_manager_obj.server_data_obj.send_server_http_data('POST',
                                                                                         deep_data,
                                                                                         config.http_deep_save,
                                                                                         token=data_manager_obj.token)
                    print('深度数据返回:', return_data)
                    self.save_deep = data_manager_obj.deep
                    self.send_data_lng_lat = data_manager_obj.lng_lat
                    self.send_data_time = time.time()
                    if return_data:
                        content_data = json.loads(return_data.content)
                        if content_data.get("code") != 200:
                            data_manager_obj.logger.error({'发送深度数据失败': content_data})
                        else:
                            data_manager_obj.logger.info({"发送深度数据成功": deep_data})
                            self.b_send_data = False
                except Exception as e:
                    data_manager_obj.logger.info({"发送深度数据error": e})
