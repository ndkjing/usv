# encoding: utf-8

import paho.mqtt.client as mqtt

host = "116.62.44.118"
port = 1883

import paho.mqtt.client as mqtt


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))


def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))

def pub():
    client = mqtt.Client(client_id="jing")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port, 600)
    client.publish('qqq', payload='Hello,EMQ!', qos=2)
    client.loop_start()

def sub():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port, 600)
    client.subscribe('qqq', qos=0)
    # client.loop_start()
    client.loop_forever()

if __name__ == '__main__':
    sub()