import requests
http_get_task = 'http://192.168.199.186:8009/union/admin/xxl/task/getOne'
http_update_task = 'http://192.168.199.186:8009/union/admin/xxl/task/upDataTask'
http_delete_task = 'http://192.168.199.186:8009/union/admin/xxl/task/delTask'
data1={"taskId": "1458307405703028737", "state": "1"}
import json
dump_json_data = data1
payload_header = {
            'Content-Type': 'application/json',
        }
print('data',data1)
return_data = requests.post(
                    url=http_update_task, params=dump_json_data, headers=payload_header)
print(return_data)
print(return_data.content)