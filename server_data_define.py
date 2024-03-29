"""
定义服务器数据类型
"""
import server_config


class DataDefine:
    def __init__(self, ship_code):
        """
        数据定义对象
        """
        # 订阅话题
        self.topics = (
            ('pool_click_%s' % ship_code, 1),
            ('update_pool_click_%s' % ship_code, 1),
            ('path_confirm_%s' % ship_code, 0),
            ('user_lng_lat_%s' % ship_code, 0),
            ('pool_info_%s' % ship_code, 1),
            ('auto_lng_lat_%s' % ship_code, 1),
            ('path_planning_%s' % ship_code, 1),
            ('status_data_%s' % ship_code, 0),
            ('path_planning_confirm_%s' % ship_code, 0),
            ('distribution_map_%s' % ship_code, 0),
            ('token_%s' % ship_code, 0),
            ('alarm_picture_%s' % ship_code, 0),
            ('server_base_setting_%s' % ship_code, 0),
            ('action_%s' % ship_code, 0),
            ('delete_pool_%s' % ship_code, 0),  # 删除湖泊设置
        )
        self.pool_code = ''


if __name__ == '__main__':
    # 简单测试获取数据
    obj = DataDefine(server_config.ship_code_list[0])
