import json
a = b'{ "move_direction": "0" }'
print(type(a))
str_a = a.decode('ascii')
print(str_a,type(str_a))
dict_a = json.loads(a)
print(dict_a,type(dict_a))

