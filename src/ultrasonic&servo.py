import RPi.GPIO as GPIO
import time
import cv2
import numpy as np

SERVO_PIN = 18
FRONT_TRIG = 21
FRONT_ECHO = 20
LEFT_TRIG = 27
LEFT_ECHO = 17
RIGHT_TRIG = 4
RIGHT_ECHO = 23

MOTOR_PWM = 13
MOTOR_IN1 = 24
MOTOR_IN2 = 25

STATE_CENTER = 0
STATE_LEFT = 1
STATE_RIGHT = 2

FILTER_SAMPLES = 5
left_history = [0] * FILTER_SAMPLES
right_history = [0] * FILTER_SAMPLES

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm_servo = GPIO.PWM(SERVO_PIN, 50)
pwm_servo.start(0)

GPIO.setup(MOTOR_IN1, GPIO.OUT)
GPIO.setup(MOTOR_IN2, GPIO.OUT)
GPIO.setup(MOTOR_PWM, GPIO.OUT)
pwm_motor = GPIO.PWM(MOTOR_PWM, 100)
pwm_motor.start(0)

def setup_ultrasonic_sensor(trig_pin, echo_pin):
    GPIO.setup(trig_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)
    GPIO.output(trig_pin, False)
    time.sleep(0.2)

setup_ultrasonic_sensor(FRONT_TRIG, FRONT_ECHO)
setup_ultrasonic_sensor(LEFT_TRIG, LEFT_ECHO)
setup_ultrasonic_sensor(RIGHT_TRIG, RIGHT_ECHO)

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
        pwm_servo.ChangeDutyCycle(duty_cycle)
        time.sleep(0.02)

def move_forward():
    GPIO.output(MOTOR_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(60)

def stop_motor():
    GPIO.output(MOTOR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(0)
    
try:
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise IOError("Cannot open webcam")
except Exception as e:
    print(f"Error opening camera: {e}")
    cap = None

GREEN_LOWER = np.array([35, 50, 50])
GREEN_UPPER = np.array([85, 255, 255])
RED_LOWER1 = np.array([0, 50, 50])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 50, 50])
RED_UPPER2 = np.array([180, 255, 255])

def detect_color(frame, lower_bound, upper_bound):
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area > 500:
            x, y, w, h = cv2.boundingRect(largest_contour)
            return (x, y, w, h), area
    return None, 0

try:
    set_angle(0)
    current_state = STATE_CENTER
    print("System starting...")
    move_forward()

    while True:
        if cap:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame.")
                continue

            green_box, green_area = detect_color(frame, GREEN_LOWER, GREEN_UPPER)
            
            mask1 = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), RED_LOWER1, RED_UPPER1)
            mask2 = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), RED_UPPER2, RED_UPPER2)
            red_mask = cv2.add(mask1, mask2)
            
            red_contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            red_area = 0
            red_box = None
            if red_contours:
                largest_red_contour = max(red_contours, key=cv2.contourArea)
                red_area = cv2.contourArea(largest_red_contour)
                if red_area > 500:
                    red_box = cv2.boundingRect(largest_red_contour)
                    
            if green_area > red_area and green_box:
                print("Green object detected, turning left.")
                set_angle(-20)
                current_state = STATE_LEFT
                time.sleep(1)
                continue
            
            if red_area > green_area and red_box:
                print("Red object detected, turning right.")
                set_angle(20)
                current_state = STATE_RIGHT
                time.sleep(1)
                continue
        
        left_dist = get_distance(LEFT_TRIG, LEFT_ECHO)
        right_dist = get_distance(RIGHT_TRIG, RIGHT_ECHO)
        
        if left_dist is not None:
            left_history.append(left_dist)
            left_history.pop(0)
        if right_dist is not None:
            right_history.append(right_dist)
            right_history.pop(0)
        
        avg_left_dist = sum(left_history) / FILTER_SAMPLES
        avg_right_dist = sum(right_history) / FILTER_SAMPLES

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
                pass
        
        elif current_state == STATE_LEFT:
            if avg_right_dist > 20:
                print("Right path is clear, returning to center.")
                set_angle(0)
                current_state = STATE_CENTER
            else:
                pass
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program terminated by user.")
finally:
    if cap:
        cap.release()
    stop_motor()
    pwm_servo.stop()
    pwm_motor.stop()
    GPIO.cleanup()