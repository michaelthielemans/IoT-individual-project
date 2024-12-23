import paho.mqtt.client as mqtt
from datetime import datetime
import time

# Define callback methods
def on_connect(mqttc, obj, flags, reason_code, properties):
    print("reason_code: "+str(reason_code))

def on_message(mqttc, obj, msg):
    print(msg.topic+" "+str(msg.qos)+" "+str(msg.payload))
    local_time = datetime.now()
    formatted_time = local_time.strftime("%Y-%m-%d %H:%M:%S")
    print("Formatted local time:", formatted_time)

def on_subscribe(mqttc, obj, mid, reason_code_list, properties):
    print("Subscribed: "+str(mid)+" "+str(reason_code_list))

def on_log(mqttc, obj, level, string):
    print(string)

#value = on_message.msg
# If you want to use a specific client id, use
# but note that the client id must be unique on the broker. Leaving the client
# id parameter empty will generate a random id for you.


mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="LzIIOwAuGSwaFzInGwQ4PCo")
mqttc.username_pw_set("LzIIOwAuGSwaFzInGwQ4PCo", "uPxk9Tg7VxhgULFqfIctn5q0")
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
# Uncomment to enable debug messages
mqttc.on_log = on_log
mqttc.connect("mqtt3.thingspeak.com", 1883, 60)
mqttc.subscribe("channels/2777434/subscribe", 0)
mqttc.loop_start()


try:
    print("Main thread loop is running. Press Ctrl+C to exit.")
    counter = 0
    while True:
        # Main thread's independent logic
        counter += 1
        print(f"Main loop running, counter: {counter}")
        time.sleep(20)  # Delay to simulate work
        mqttc.publish("channels/2777434/publish/fields/field1", counter)
except KeyboardInterrupt:
    print("Exiting program...")


mqttc.loop_stop()
mqttc.disconnect()