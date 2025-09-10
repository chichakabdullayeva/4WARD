import RPi.GPIO as GPIO
import time

S2_PIN = 23
S3_PIN = 24
OUT_PIN = 25
LED_PIN = 17


RED_CALIBRATION = 145
GREEN_CALIBRATION = 200
BLUE_CALIBRATION = 200
TOLERANCE = 30

GPIO.setmode(GPIO.BCM)
GPIO.setup(S2_PIN, GPIO.OUT)
GPIO.setup(S3_PIN, GPIO.OUT)
GPIO.setup(OUT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LED_PIN, GPIO.OUT)

def get_frequency(s2_state, s3_state):
    GPIO.output(S2_PIN, s2_state)
    GPIO.output(S3_PIN, s3_state)
    
    start_time = time.time()
    
    pulse_count = 0
    while time.time() - start_time < 0.1:
        if GPIO.input(OUT_PIN) == 0:
            pulse_count += 1
            while GPIO.input(OUT_PIN) == 0:
                pass
    
    frequency = pulse_count
    return frequency

try:
    print("Color sensor ready. Point at a colored object...")
    GPIO.output(LED_PIN, GPIO.HIGH)

    while True:
        red_frequency = get_frequency(GPIO.LOW, GPIO.LOW)
        green_frequency = get_frequency(GPIO.HIGH, GPIO.HIGH)
        blue_frequency = get_frequency(GPIO.LOW, GPIO.HIGH)

        normalized_red = (red_frequency / RED_CALIBRATION) * 255
        normalized_green = (green_frequency / GREEN_CALIBRATION) * 255
        normalized_blue = (blue_frequency / BLUE_CALIBRATION) * 255
        
        print(f"Raw: R={red_frequency} G={green_frequency} B={blue_frequency}")
        print(f"Normalized: R={normalized_red:.2f} G={normalized_green:.2f} B={normalized_blue:.2f}")

      
        if normalized_blue > normalized_red and normalized_blue > normalized_green and normalized_blue > 150:
            print("Detected Color: BLUE")
        elif normalized_red > normalized_blue and normalized_green > normalized_blue and normalized_red > 150 and normalized_green > 100:
            print("Detected Color: ORANGE")
        else:
            print("Detected Color: UNKNOWN")

        time.sleep(0.2)

except KeyboardInterrupt:
    GPIO.output(LED_PIN, GPIO.LOW)
    GPIO.cleanup()
    print("Program stopped.")