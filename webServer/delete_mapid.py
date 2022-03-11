import json
import os
import server_config
import copy
"""
使用指定船编号和湖泊id删除湖泊
"""

def delete_map(ship_code, mapid=None):
    save_map_path = os.path.join(server_config.save_map_dir, 'map_%s.json' % ship_code)
    with open(save_map_path, 'r') as fr:
        data = json.load(fr)
    write_data = copy.deepcopy(data)
    delete_index = None
    for i, v in enumerate(data['mapList']):
        if mapid == v["id"]:
            print('查找到id', v["id"])
            delete_index = i
            break
    if delete_index is not None:
        del write_data['mapList'][delete_index]
        with open(save_map_path, 'w') as fw:
            json.dump(write_data, fw)


if __name__ == '__main__':
    delete_map(ship_code='XXLJC4LCGSCSD1DA003', mapid='1456491488217112577')
