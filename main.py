import paho.mqtt.client as mqtt
from datetime import datetime
import time
import psutil
import wiringpi
import time
from bmp280 import BMP280
from smbus2 import SMBus
import threading
import json

#setup variables
main_loop_counter = 0
required_temp = 30
i2c_bus = SMBus(0)
bmp280_address = 0x76
bmp280 = BMP280(i2c_addr= bmp280_address, i2c_dev=i2c_bus)
interval = 15
button1_last_state = 0
button2_last_state = 0
button_debounce_time = 0.05
BH1750_I2C_ADDR = 0x23  # Default I2C address for BH1750
BH1750_POWER_ON = 0x01         # Power on the module
BH1750_CONTINUOUS_HIGH_RES_MODE = 0x10  # Continuous high-resolution mode



# Setup wiring pins
wiringpi.wiringPiSetup()
wiringpi.pinMode(2, 1) # Physical pin 7 -> PWM (not in use)
# wiringpi.pinMode(3, 1) # Physical pin 8 -> GPIO (not in use)
wiringpi.pinMode(4, 1) # Physical pin 10 as output -> LED 1
wiringpi.pinMode(5, 1) # Physical pin 11 as output -> LED 2
wiringpi.pinMode(6, 0) # Physical pin 12 as input -> Button 1 
wiringpi.pinMode(7, 0) # Physical pin 13 as input -> Button 2
wiringpi.pinMode(8, 1) # Physical pin 15 -> ultrasonic output (trigger)
wiringpi.pinMode(9, 0) # Physical pin 16 -> ulttrasonic input (echo)
wiringpi.pinMode(10, 1) # Physical pin 18 as output -> relay 1 : heater
wiringpi.pinMode(16, 1) # Physical pin 26 as output -> relay 2 : cooler

# define led blinking
def blink(led_pin):
    counter = 0
    while counter < 200:
        wiringpi.digitalWrite(led_pin, 1)
        time.sleep(0.1)
        wiringpi.digitalWrite(led_pin, 0)
        time.sleep(0.1)
        counter += 1

# Define callback methods for MQTT
def on_connect(mqttc, obj, flags, reason_code, properties):
    print("reason_code: "+str(reason_code))

def on_message(mqttc, obj, msg):
 
    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
    data = json.loads(msg.payload)
    
    # Extract field8 and field4 as integers
    field8_value = int(data.get('field8', 0))
    field4_value = int(float(data.get('field4', 0)))

    print(f"Field8 as an integer: {field8_value}")
    print(f"Field4 as an integer: {field4_value}")

    # Logic to handle heater and cooler
    if field8_value > field4_value:
        wiringpi.digitalWrite(10, 1)  # Turn on heater
        wiringpi.digitalWrite(16, 0)  # Turn off cooler
    elif field8_value < field4_value:
        wiringpi.digitalWrite(16, 1)  # Turn on cooler
        wiringpi.digitalWrite(10, 0)  # Turn off heater
    else:
        wiringpi.digitalWrite(10, 0)
        wiringpi.digitalWrite(16, 0)

def on_subscribe(mqttc, obj, mid, reason_code_list, properties):
    print("Subscribed: "+str(mid)+" "+str(reason_code_list))

def on_log(mqttc, obj, level, string):
    print(string)

def read_light():
    # Send power on command
    i2c_bus.write_byte(BH1750_I2C_ADDR, BH1750_POWER_ON)
    time.sleep(0.01)  # Allow sensor to wake up

    # Send command to start measurement in continuous mode
    i2c_bus.write_byte(BH1750_I2C_ADDR, BH1750_CONTINUOUS_HIGH_RES_MODE)
    time.sleep(0.2)  # Measurement delay (max 180ms)

    # Read 2 bytes of data (light intensity in lux)
    data = i2c_bus.read_i2c_block_data(BH1750_I2C_ADDR, 0x00, 2)
    lux = (data[0] << 8) | data[1]  # Combine the two bytes
    return lux

# Shared value
value = 0
lock = threading.Lock()  # To ensure thread-safe access to 'value'

# Debounce setup
debounce_time = 0.2  # 200 ms debounce time

# Button handler function

# def handle_buttons():
#     global value, last_button1_press, last_button2_press
#     debounce_time = 0.2  # 200 ms debounce

#     while True:
#         current_time = time.time()

#         # Button 1 logic
#         if wiringpi.digitalRead(6) == 1 and (current_time - last_button1_press) > debounce_time:
#             with lock:
#                 value += 1
#                 print(f"Button 1 pressed. Value increased to {value}.")
#             last_button1_press = current_time

#         # Button 2 logic
#         if wiringpi.digitalRead(7) == 1 and (current_time - last_button2_press) > debounce_time:
#             with lock:
#                 value -= 1
#                 print(f"Button 2 pressed. Value decreased to {value}.")
#             last_button2_press = current_time

#         time.sleep(0.05)  # Avoid busy-looping

def handle_buttons():
    global value
    button1_last_state = 0
    button2_last_state = 0

    while True:
        # Read button states
        button1 = wiringpi.digitalRead(6)
        button2 = wiringpi.digitalRead(7)

        if button1 and not button1_last_state:  # Button 1 pressed
            with lock:
                value += 1
                print(f"Button 1 pressed. Value increased to {value}.")
            wiringpi.digitalWrite(5, 1)  # Feedback: Turn on LED
            time.sleep(debounce_time)  # Debounce delay
            wiringpi.digitalWrite(5, 0)  # Turn off LED

        if button2 and not button2_last_state:  # Button 2 pressed
            with lock:
                value -= 1
                print(f"Button 2 pressed. Value decreased to {value}.")
            wiringpi.digitalWrite(5, 1)  # Feedback: Turn on LED
            time.sleep(debounce_time)  # Debounce delay
            wiringpi.digitalWrite(5, 0)  # Turn off LED

        # Update last state
        button1_last_state = button1
        button2_last_state = button2

        time.sleep(0.05)  # Small delay to avoid busy-looping

# Start button handler in a separate thread
button_thread = threading.Thread(target=handle_buttons, daemon=True)
button_thread.start()


#----- start mqtt connection ----#

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
    while True: # Main thread's independent logic
        
        #----- read buttons ----
        with lock:
            print(f"Current value: {value}")
        #-----  get sensordata ----- #
    
        # Trigger the ultrasonic pulse
        wiringpi.digitalWrite(8, 1)
        time.sleep(0.00001)  # Wait for 10 microseconds
        wiringpi.digitalWrite(8, 0)

        # Wait for the echo to go HIGH
        while wiringpi.digitalRead(9) == 0:
            pass
        signal_high = time.time()

        # Wait for the echo to go LOW
        while wiringpi.digitalRead(9) == 1:
            pass
        signal_low = time.time()

        # Calculate the time difference
        timepassed = signal_low - signal_high

        # Calculate the distance (speed of sound = 343 m/s or 34300 cm/s)
        distance = timepassed * 17000

        # Print the distance
        print(f"Distance: {distance:.2f} cm")

        # get the system performance data over 5 seconds:
        cpu_percent = psutil.cpu_percent(interval=5)
        ram_percent = psutil.virtual_memory().percent
        
        # get bmp280 sensor data
        bmp280_temp = bmp280.get_temperature()
        bmp280_pressure = bmp280.get_pressure()
        print("Temp: %4.1f, Pressure: %4.1f" % (bmp280_temp, bmp280_pressure))

        # get light level:
        light_level = read_light()
        print(f"Light Intensity: {light_level} lux")
        time.sleep(1)  # Delay between readings

        # Publish it to thingspeak:
        main_loop_counter += 1
        print(f"Main loop running, counter: {main_loop_counter}")
        payload = "field1=" + str(cpu_percent) + "&field2=" + str(ram_percent) + "&field3=" + str(distance) + "&field4=" + str(bmp280_temp) + "&field5=" + str(bmp280_pressure) + "&field6=" + str(light_level) + "&field7=" + str("50") + "&field8=" + str(value)
        mqttc.publish("channels/2777434/publish", payload)
        
        blink(4)

except KeyboardInterrupt:
    print("Exiting program...")

mqttc.loop_stop()
mqttc.disconnect()