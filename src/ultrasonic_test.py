import RPi.GPIO as GPIO
import time

TRIG = 21
ECHO = 20
MAX_TIMEOUT = 0.05

GPIO.setmode(GPIO.BCM)

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

GPIO.output(TRIG, False)
print("Waiting for sensor to settle...")
time.sleep(2)

try:
    while True:
        print("distance measurement in progress")
        
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)
        
        pulse_start = time.time()
        pulse_end = time.time()
        
        timeout_start = time.time()
        while GPIO.input(ECHO) == 0:
            pulse_start = time.time()
            if time.time() - timeout_start > MAX_TIMEOUT:
                pulse_start = None
                break
            
        if pulse_start is not None:
            while GPIO.input(ECHO) == 1:
                pulse_end = time.time()
                if time.time() - pulse_start > MAX_TIMEOUT:
                    pulse_end = None
                    break
        
        if pulse_start is not None and pulse_end is not None:
            pulse_duration = pulse_end - pulse_start
            
            distance = pulse_duration * 17150
            
            distance = round(distance, 2)
            
            print("Distance:", distance, "cm")
        else:
            print("Measurement timed out. Check sensor and connections.")
        
        time.sleep(1)

except KeyboardInterrupt:
    print("Measurement stopped by User")
    GPIO.cleanup()