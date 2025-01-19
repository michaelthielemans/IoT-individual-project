from machine import Pin, ADC, Timer, PWM
import utime
import time
import network
import socket

sensor_temp = ADC(4)
ADC_light_intensity = ADC(0)
sysvolt = ADC(29).read_u16()
conversion_factor = 3.3 / (65535)
led = Pin("LED", Pin.OUT)
timer = Timer()
pwm = PWM(Pin(2))
# Set the PWM frequency.
pwm.freq(1000)

def tick(timer):
    global led
    led.toggle()

while True:
    reading = sensor_temp.read_u16() * conversion_factor
    value_light_intensity = ADC_light_intensity.read_u16()
    # The temperature sensor measures the Vbe voltage of a biased bipolar diode, connected tothe fifth ADC channel
    # Typically, Vbe = 0.706V at 27 degrees C, with a slope of -1.721mV (0.001721) per degree.
    percentage_light_intensity = value_light_intensity / 650
    temperature = 27 - (reading - 0.706)/0.001721
    print(value_light_intensity)
    print(str(percentage_light_intensity) + " %")
    print("-----------")
    #print(f"potentiometer = {potentiometer}")
    #print(f"system voltage = {sysvolt}")
    utime.sleep(1)
    if percentage_light_intensity < 50:
        led.value(1)
    else:
        led.value(0)

    # Fade the LED in and out a few times.
    duty = 0
    direction = 1
    for _ in range(8 * 256):
        duty += direction
        if duty > 255:
            duty = 255
            direction = -1
        elif duty < 0:
            duty = 0
            direction = 1

        pwm.duty_u16(duty * duty)
        time.sleep(0.01)