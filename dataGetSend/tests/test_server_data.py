from dataGetSend import server_data


import unittest



class TestDict(unittest.TestCase):

    def test_mqtt_send(self):
        obj = server_data.MqttSendGet()
        obj.subscribe_topic(topic='qqq')
        obj.publish_topic(topic='qqq', data={'12312': 'Hello,EMQ!'}, qos=2)
        obj.publish_topic(topic='qqq', data='32132', qos=2)
        obj.publish_topic(topic='qqq', data=3213, qos=2)
        obj.publish_topic(topic='qqq', data=[321,32], qos=2)

    def test_mqtt_get(self):
        print('test get')

    def test_http_send(self):
        pass


    def test_http_get(self):
        pass


if __name__ == '__main__':
    unittest.main()


