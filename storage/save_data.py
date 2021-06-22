import os
import sys
import json


# 设置数据
def set_data(save_data, save_path):
    if os.path.exists(save_path):
        try:
            with open(save_path, 'r') as f:
                current_save_data = json.load(f)
            with open(save_path, 'w') as f:
                current_save_data.update(save_data)
                json.dump(save_data, f)
        except Exception as e:
            os.remove(save_path)
    else:
        with open(save_path, 'w') as f:
            json.dump(save_data, f)


# 获取数据
def get_data(save_path):
    if os.path.exists(save_path):
        try:
            with open(save_path, 'r') as f:
                save_data = json.load(f)
            if isinstance(save_data, dict):
                return save_data
            else:
                return None
        except json.decoder.JSONDecodeError:
            os.remove(save_path)
            return None
    else:
        return None


if __name__ == '__main__':
    set_data({1:2},'test.json')
    print(get_data('test.json'))

