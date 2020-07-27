print("hello")
print(0x200)
print(0x217)
print(0x200 + 0x017)
import os
import json

with open("/home/autodollyv2/can_bus_ws/src/config.json") as json_file: # open config.json
    config = json.load(json_file) # load configuration

node_id = int(config['dolly']['back']['node_id'],16)
address = 0x017
msg_id = node_id + address
print(msg_id)