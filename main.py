import paho.mqtt.client as mqtt
import time
import psutil
import wiringpi
import time
from bmp280 import BMP280
from smbus2 import SMBus
import threading
import json
from evdev import InputDevice, categorize, ecodes

#setup variables
i2c_bus = SMBus(0)
bmp280_address = 0x76
bmp280 = BMP280(i2c_addr= bmp280_address, i2c_dev=i2c_bus)
button1_last_state = 0
button2_last_state = 0
IR_device_path = '/dev/input/event6'   # Replace '/dev/input/event6' with your IR device path
distance = 0.0
distance_thread_lock = threading.Lock()
required_temp = 20
required_temp_lock = threading.Lock()  # To ensure thread-safe access to 'value'
motion_detector = False
motion_detected_lock = threading.Lock()

# Setup wiring pins
wiringpi.wiringPiSetup()
wiringpi.pinMode(3, 0) # Physical pin 8 as input -> motion detector
wiringpi.pinMode(4, 1) # Physical pin 10 as output -> RED LED 1
wiringpi.pinMode(5, 1) # Physical pin 11 as output -> RGB LED RED
wiringpi.pinMode(6, 0) # Physical pin 12 as input -> Button 1 
wiringpi.pinMode(7, 0) # Physical pin 13 as input -> Button 2
wiringpi.pinMode(8, 1) # Physical pin 15 -> ultrasonic output (trigger)
wiringpi.pinMode(9, 0) # Physical pin 16 -> ulttrasonic input (echo)
wiringpi.pinMode(10, 1) # Physical pin 18 -> as output RGB LED GREEN
wiringpi.pinMode(13, 1) # Physical pin 19 -> as output RGB LED BLUE
wiringpi.pinMode(15, 1) # Physical pin 24 as output -> relay 1 : cooler
wiringpi.pinMode(16, 1) # Physical pin 26 as output -> relay 2 : heater

def handle_buttons():
    global required_temp
    button1_last_state = 0
    button2_last_state = 0
    # debounce_time = 0.2  # 200 ms debounce time 

    while True:
        # Read button states
        button1 = wiringpi.digitalRead(6)
        button2 = wiringpi.digitalRead(7)

        if button1 and not button1_last_state:  # Button 1 pressed
            with required_temp_lock:
                required_temp += 1
                print(f"Button 1 pressed. Value increased to {required_temp}.")
            blink_led_fast(5)

        if button2 and not button2_last_state:  # Button 2 pressed
            with required_temp_lock:
                required_temp -= 1
                print(f"Button 2 pressed. Value decreased to {required_temp}.")
            blink_led_fast(13)

        # Update last state
        button1_last_state = button1
        button2_last_state = button2

        time.sleep(0.05)  # Small delay to avoid busy-looping

def blink_led_fast(led_pin):
    wiringpi.digitalWrite(led_pin, 1)
    time.sleep(0.01)
    wiringpi.digitalWrite(led_pin, 0)
    time.sleep(0.01)

def monitor_traffic():
    prev_bytes_sent = 0
    try:
        print("Monitoring network traffic in a separate thread.")
        while True:
            # Get current traffic stats
            current_bytes_sent = psutil.net_io_counters().bytes_sent

            # Calculate traffic differences
            sent_diff = current_bytes_sent - prev_bytes_sent

            # Update previous stats
            prev_bytes_sent = current_bytes_sent

            # Blink LED if there is traffic
            if sent_diff > 500:
                #print(f"Traffic detected: Sent: {sent_diff} bytes")
                blink_led_fast(4)

            time.sleep(0.01)  # Check every 500ms
    except KeyboardInterrupt:
        print("Exiting network monitor thread...")
        wiringpi.digitalWrite(4, 0)  # Turn off LED before exiting

def ir_listener():
    global required_temp
    device = InputDevice(IR_device_path)
    print(f"Listening for IR codes on {IR_device_path}...")

    for event in device.read_loop():
        # Filter for scan codes (press events only)
        if event.type == ecodes.EV_MSC:
            if event.value == 7:
                with required_temp_lock:
                    required_temp += 0.5
                    blink_led_fast(5)
            elif event.value == 21:
                with required_temp_lock:
                    required_temp -= 0.5
                    blink_led_fast(13)
        print(f"required temp set to : {required_temp}")

def motion_detection():
    global motion_detected
    while True:
        if wiringpi.digitalRead(3) == 1:
            motion_detected = True
            blink_led_fast(10)
        else:
            motion_detected = False
        time.sleep(0.2)

def on_connect(mqttc, obj, flags, reason_code, properties):
    print("reason_code: "+str(reason_code))

def on_message(mqttc, obj, msg):
    global required_temp

    print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
    data = json.loads(msg.payload)
    
    # Extract field7 as integers
    with required_temp_lock:
        required_temp = int(float(data.get('field6', 0)))

def on_subscribe(mqttc, obj, mid, reason_code_list, properties):
    print("Subscribed: "+str(mid)+" "+str(reason_code_list))

def on_log(mqttc, obj, level, string):
    print(string)

def measure_distance():
    global distance
    while True:
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

        timepassed = signal_low - signal_high

        # Calculate the distance (speed of sound = 343 m/s or 34300 cm/s)
        new_distance = timepassed * 17000

        # Safely update the shared distance variable
        with distance_thread_lock:
            distance = new_distance

        # Sleep briefly to prevent excessive CPU usage
        time.sleep(0.1)


# start the dedicated thread for the network detection
traffic_thread = threading.Thread(target=monitor_traffic, daemon=True)
traffic_thread.start()

# Start button handler in a separate thread
button_thread = threading.Thread(target=handle_buttons, daemon=True)
button_thread.start()

# start the dedicated thread for the network detection
motion_detect_thread = threading.Thread(target=motion_detection, daemon=True)
motion_detect_thread.start()

# Start IR listener in a separate thread
ir_thread = threading.Thread(target=ir_listener, daemon=True)
ir_thread.start()

# Start the ultrasonic measurement thread
distance_thread = threading.Thread(target=measure_distance, daemon=True)
distance_thread.start()

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
    while True: # Main thread's

        # pull all metrics and put into a variable
        cpu_percent_publish = psutil.cpu_percent(interval=5)
        ram_percent_publish = psutil.virtual_memory().percent
        bmp280_temp_publish = bmp280.get_temperature()
        print(f"+++--- The current temp = {bmp280_temp_publish} ---+++")
        bmp280_pressure_publish = bmp280.get_pressure()
        print(f"+++--- The current pressure = {bmp280_pressure_publish} ---+++")
        with required_temp_lock:
            required_temp_publish = required_temp
        with distance_thread_lock:
            distance_publish = distance
        with motion_detected_lock:
            motion_detected_publish = motion_detected
        print(f"+++--- The current distance = {required_temp_publish} ---+++")

        # Publish it to thingspeak:
        payload = "field1=" + str(cpu_percent_publish) + "&field2=" + str(ram_percent_publish) + "&field3=" + str(distance_publish) + "&field4=" + str(bmp280_temp_publish) + "&field5=" + str(bmp280_pressure_publish) + "&field6=" + str(required_temp_publish)
        
        mqttc.publish("channels/2777434/publish", payload)
        
        if required_temp > (bmp280_temp_publish + 0.5):
            wiringpi.digitalWrite(16,0)
            wiringpi.digitalWrite(15,1)
        elif required_temp < (bmp280_temp_publish - 0.5):
            wiringpi.digitalWrite(15,0)
            wiringpi.digitalWrite(16,1)
        else:
            wiringpi.digitalWrite(15,1)
            wiringpi.digitalWrite(16,1)

        time.sleep(16)

except FileNotFoundError:
    print(f"Device not found: {IR_device_path}. Check your IR device path.")
except PermissionError:
    print("Permission denied. Try running the script with 'sudo'.")
except KeyboardInterrupt:
    print("Exiting program...")

mqttc.loop_stop()
mqttc.disconnect()