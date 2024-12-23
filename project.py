import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt
import psutil
import wiringpi
import time


# Setup wiring pins
wiringpi.wiringPiSetup()
wiringpi.pinMode(2, 1)
wiringpi.pinMode(3, 1)
wiringpi.pinMode(8, 1)  # Set pin 8 as output (trigger)
wiringpi.pinMode(9, 0)  # Set pin 9 as input (echo)

while (True):
    
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


    # get the system performance data over 20 seconds.
    cpu_percent = psutil.cpu_percent(interval=5)
    ram_percent = psutil.virtual_memory().percent

    # build the payload string.
    payload = "field1=" + str(cpu_percent) + "&field2=" + str(ram_percent) + "&field3="+str(distance)
    time.sleep(20)





    #----- attempt to publish this data over mqtt to thingspeak -----#
    try:
        print ("Writing Payload = ", payload," to host: ", mqtt_host, " clientID= ", mqtt_client_ID, " User ", mqtt_username, " PWD ", mqtt_password)
        publish.single(topic, payload, hostname=mqtt_host, transport=t_transport, port=t_port, client_id=mqtt_client_ID, auth={'username':mqtt_username,'password':mqtt_password})
    except Exception as e:
        print (e)
    except KeyboardInterrupt:
        print("Measurement stopped by user")
