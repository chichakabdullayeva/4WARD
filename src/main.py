import RPi.GPIO as GPIO
import time

# GPIO pins for the servo and ultrasonic sensors
SERVO_PIN = 18
FRONT_TRIG = 21
FRONT_ECHO = 20
LEFT_TRIG = 27
LEFT_ECHO = 17
RIGHT_TRIG = 4
RIGHT_ECHO = 23

# State constants
STATE_CENTER = 0
STATE_LEFT = 1
STATE_RIGHT = 2

# Filter settings
FILTER_SAMPLES = 5
left_history = [0] * FILTER_SAMPLES
right_history = [0] * FILTER_SAMPLES

# Setup GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Setup servo
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)

# Setup ultrasonic sensors
def setup_ultrasonic_sensor(trig_pin, echo_pin):
    GPIO.setup(trig_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)
    GPIO.output(trig_pin, False)
    time.sleep(0.2)

setup_ultrasonic_sensor(FRONT_TRIG, FRONT_ECHO)
setup_ultrasonic_sensor(LEFT_TRIG, LEFT_ECHO)
setup_ultrasonic_sensor(RIGHT_TRIG, RIGHT_ECHO)

# Function to get distance from a single sensor
def get_distance(trig_pin, echo_pin, samples=5):
    values = []
    for _ in range(samples):
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(trig_pin, False)
        
        pulse_start = time.time()
        pulse_end = time.time()
        
        timeout = time.time()
        while GPIO.input(echo_pin) == 0:
            pulse_start = time.time()
            if time.time() - timeout > 0.02:
                return None

        timeout = time.time()
        while GPIO.input(echo_pin) == 1:
            pulse_end = time.time()
            if time.time() - timeout > 0.02:
                return None
        
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        if distance > 0:
            values.append(distance)
        time.sleep(0.01)

    return round(sum(values) / len(values), 2) if values else None

# Servo control function (smooth movement)
current_angle = 0
def set_angle(target_angle):
    global current_angle
    step = 2 if target_angle > current_angle else -2
    while current_angle != target_angle:
        current_angle += step
        if (step > 0 and current_angle > target_angle) or (step < 0 and current_angle < target_angle):
            current_angle = target_angle
        duty_cycle = (current_angle + 90) / 18.0 + 2
        duty_cycle = max(2, min(12, duty_cycle))
        pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.02)

try:
    set_angle(0)
    current_state = STATE_CENTER
    print("System starting...")

    while True:
        # Get and filter readings from sensors
        left_dist = get_distance(LEFT_TRIG, LEFT_ECHO)
        right_dist = get_distance(RIGHT_TRIG, RIGHT_ECHO)

        # Update history
        if left_dist is not None:
            left_history.append(left_dist)
            left_history.pop(0)
        if right_dist is not None:
            right_history.append(right_dist)
            right_history.pop(0)
            
        # Get average filtered distances
        avg_left_dist = sum(left_history) / FILTER_SAMPLES
        avg_right_dist = sum(right_history) / FILTER_SAMPLES
        
        print(f"Filtered Distances: Left={avg_left_dist}cm, Right={avg_right_dist}cm")

        # Decision logic with hysteresis
        if current_state == STATE_CENTER:
            if avg_left_dist < 15 and avg_left_dist > 0:
                print("Obstacle on the left, turning right.")
                set_angle(20)
                current_state = STATE_RIGHT
            elif avg_right_dist < 15 and avg_right_dist > 0:
                print("Obstacle on the right, turning left.")
                set_angle(-20)
                current_state = STATE_LEFT
            else:
                print("Path is clear, holding center.")

        elif current_state == STATE_RIGHT:
            if avg_left_dist > 20:
                print("Left path is clear, returning to center.")
                set_angle(0)
                current_state = STATE_CENTER
            else:
                print("Still turning right.")
        
        elif current_state == STATE_LEFT:
            if avg_right_dist > 20:
                print("Right path is clear, returning to center.")
                set_angle(0)
                current_state = STATE_CENTER
            else:
                print("Still turning left.")
        
        time.sleep(0.2)

except KeyboardInterrupt:
    print("Program terminated by user.")
finally:
    pwm.stop()
    GPIO.cleanup()