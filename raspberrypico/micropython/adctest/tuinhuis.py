import time
from umqtt.simple import MQTTClient
from machine import Pin, ADC, Timer, PWM
import network
import _thread

#----- pin definitions -----#
ADC_light_intensity = ADC(0)    # physical pin 31
potmeter_read_pin = ADC(2)      # physical pin 34
potmeter_activate_pin = Pin(15, Pin.OUT)
led1_meter = Pin(10, Pin.OUT)
led2_meter = Pin(11, Pin.OUT)
led3_meter = Pin(12, Pin.OUT)
led4_meter = Pin(13, Pin.OUT)
led_pwm = PWM(Pin(2))           # physical pin 4
led = Pin("LED", Pin.OUT)       #internal led

#----- Network variables-----#
SSID = "WIFI2G"
PASSWORD = "Thielemans"
wifi = network.WLAN(network.STA_IF)
wifi.active(True)

#----- variables -----#
timer = Timer()
# Set the PWM frequency.
led_pwm.freq(100)
required_light = 0
light_intensity = 0

#----- MQTT connection variables -----#
server = "mqtt3.thingspeak.com"
ClientID = "BiEQFisZJBcjIRknCTgnIzU"
user = "BiEQFisZJBcjIRknCTgnIzU"
password = "6p0nWfWTnnRgF/qgpnIwcLl2"
topic = "channels/2791497/publish"
payload = b"field1=0"


#----- Read input loop----#
def input_loop():
    global required_light
    global light_intensity
    global led_pwm
    duty = 1

    THRESHOLD_1 = 200
    THRESHOLD_2 = 65536 // 4
    THRESHOLD_3 = 65536 // 2
    THRESHOLD_4 = (65536 // 4) * 3

    while True:
        
        #----- read the light intensity -----#
        light_intensity_16bit = ADC_light_intensity.read_u16()
        light_intensity =  light_intensity_16bit/ 655
        
        # Activate the potentiometer
        potmeter_activate_pin.value(1)
        potmeter_value_16bit = potmeter_read_pin.read_u16()
        time.sleep(0.1)  # Small delay for stable reading
        potmeter_activate_pin.value(0)

        # Calculate required light (scaled down)
        required_light = potmeter_value_16bit // 655

        # Control LEDs based on thresholds
        led_states = [
            potmeter_value_16bit >= THRESHOLD_1,
            potmeter_value_16bit >= THRESHOLD_2,
            potmeter_value_16bit >= THRESHOLD_3,
            potmeter_value_16bit >= THRESHOLD_4,
        ]

        # Update LED states
        led1_meter.value(led_states[0])
        led2_meter.value(led_states[1])
        led3_meter.value(led_states[2])
        led4_meter.value(led_states[3])

        # if required_light > light_intensity:
        
        # # Scale brightness between 0 and 65535 (16-bit for PWM duty cycle)
        #     brightness = min((required_light - light_intensity) * 1000, 65535)  # Scale factor for demonstration

        # elif required_light < light_intensity:
        #         # Scale dimming based on the difference
        #     brightness = max(65535 - (light_intensity - required_light) * 1000, 0)  # Scale factor for demonstration
        # else:
        #     brightness = 32767  # Default medium brightness if value1 == value2

        #     # Set the PWM duty cycle
        # led_pwm.duty_u16(int(brightness))



        if potmeter_value_16bit > light_intensity_16bit :
            duty += 10
        elif potmeter_value_16bit < light_intensity_16bit:
            duty -= 10
        duty = max(0, min(duty, 65535))
        led_pwm.duty_u16(duty)
        print(str(potmeter_value_16bit) + "   " + str(light_intensity_16bit) + "     " +str(duty))
        
def print_terminal():
    print("The internal temperature : " + str(required_light))
    print("The light intensity : " + str(light_intensity) + " %")
    print("-----------------------------------------------------------")

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

#---- Establishing a wifi connection-----#
print("Connecting to Wi-Fi...")
wifi.connect(SSID, PASSWORD)

while not wifi.isconnected():
    print("Waiting for connection...")
    time.sleep(1)

print("Connected to Wi-Fi!")
print("IP address:", wifi.ifconfig()[0])  # Print the IP address

_thread.start_new_thread(input_loop, ())
#----- start a mqtt client connection-----#
# try:
#     client = connect()
# except Exception:
#     client = reconnect()

while True:
    
    print_terminal()

    # payload = "field1=" + str(light_intensity) + "&field2=" + str(required_light)
    # try:
    #     print('Sending message %s on topic %s' % (payload, topic))
    #     client.publish(topic, payload)
    # except Exception as e:
    #     print('Failed to publish message:', e)
    #     client = reconnect()

    time.sleep(20)