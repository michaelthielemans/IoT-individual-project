import time
from umqtt.simple import MQTTClient

server = "mqtt3.thingspeak.com"
ClientID = "BiEQFisZJBcjIRknCTgnIzU"
user = "BiEQFisZJBcjIRknCTgnIzU"
password = "6p0nWfWTnnRgF/qgpnIwcLl2"
topic = "channels/2791497/publish"
#msg = b'{"field1=123"}'
msg = b"field1=123"

def connect():
    print('Connecting to MQTT Broker "%s"' % server)
    client = MQTTClient(ClientID, server, 1883, user, password)
    try:
        client.connect()
        return client
    except Exception as e:
        print('Failed to connect to MQTT broker:', e)
        raise

def reconnect():
    print('Failed to connect to MQTT broker. Reconnecting...')
    time.sleep(5)
    client = connect()
    return client

try:
    client = connect()
except Exception:
    client = reconnect()

while True:
    try:
        print('Sending message %s on topic %s' % (msg, topic))
        client.publish(topic, msg)
        time.sleep(20)
    except Exception as e:
        print('Failed to publish message:', e)
        client = reconnect()