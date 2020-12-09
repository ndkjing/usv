import config

import json, os

# if not os.path.exists(config.local_map_data_path):
#     with open(config.local_map_data_path, 'w') as f:
#         json.dump({'2': 2}, f)
# else:
#     with open(config.local_map_data_path, 'r') as f:
#         data = json.load(f)
#         print(data)
#     with open(config.local_map_data_path, 'w') as f:
#         data.update({'1':1})
#         print(data)
#         json.dump(data, f)

# with open(config.local_map_data_path, 'r') as f:
#     data = json.load(f)
#     print(data)
# import utils.log as log
#
# logger = log.LogHandler('test_log')
# logger.info({"海带丝":123})


import numpy as np
with open('test.json' ,'w') as f:
    a = np.array([[1,2],[3,4]])
    l = a.tolist()
    print(type(l))
    json.dump({'1':l},f)

with open('test.json' ,'r') as f1:
    out_data = json.load(f1)
    print(out_data)
    print(type(out_data['1']))
