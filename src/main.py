import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
import subprocess

SERVO_PIN = 18
FRONT_TRIG = 4
FRONT_ECHO = 23
LEFT_TRIG = 21
LEFT_ECHO = 20
RIGHT_TRIG = 27
RIGHT_ECHO = 17

MOTOR_PWM = 13
MOTOR_IN1 = 24
MOTOR_IN2 = 25

S0_PIN = 22
S1_PIN = 10
S2_PIN = 9
S3_PIN = 11
OUT_PIN = 8

RED_CALIBRATION = 145
GREEN_CALIBRATION = 200
BLUE_CALIBRATION = 200

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

GPIO.setup(S0_PIN, GPIO.OUT)
GPIO.setup(S1_PIN, GPIO.OUT)
GPIO.setup(S2_PIN, GPIO.OUT)
GPIO.setup(S3_PIN, GPIO.OUT)
GPIO.setup(OUT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

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

current_pwm_duty = 8.61
def set_servo_pwm(target_pwm_duty):
    global current_pwm_duty
    
    target_pwm_duty = max(5.89, min(11.33, target_pwm_duty))
    
    step = 0.1 if target_pwm_duty > current_pwm_duty else -0.1
    while abs(current_pwm_duty - target_pwm_duty) > 0.01:
        current_pwm_duty += step
        current_pwm_duty = max(5.89, min(11.33, current_pwm_duty))
        
        pwm_servo.ChangeDutyCycle(current_pwm_duty)
        time.sleep(0.02)
    
    current_pwm_duty = target_pwm_duty
    pwm_servo.ChangeDutyCycle(current_pwm_duty)


def move_forward():
    GPIO.output(MOTOR_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(60)

def stop_motor():
    GPIO.output(MOTOR_IN1, GPIO.LOW)
    GPIO.output(MOTOR_IN2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(0)

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

cmd = ["rpicam-vid", "--width", "320", "--height", "240", "--codec", "mjpeg", "--timeout", "0", "--framerate", "30", "-o", "-"]
proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
bytes_buffer = b""

frame_width = 320
frame_center = frame_width / 2
MIN_OBJECT_AREA = 2000
FRONT_AVOIDANCE_DISTANCE = 5

consecutive_count = 0
last_detected_color = None
lap_count = 0
last_count_time = 0

try:
    set_servo_pwm(8.61)
    print("System is starting...")
    move_forward()

    while True:
        # Step 1: Check for front obstacles first (highest priority)
        front_dist = get_distance(FRONT_TRIG, FRONT_ECHO)
        if front_dist is not None and front_dist < FRONT_AVOIDANCE_DISTANCE:
            print(f"Front obstacle detected at {front_dist}cm. Stopping.")
            stop_motor()
            set_servo_pwm(8.61)
            time.sleep(0.5)
            continue # Skip to the next loop iteration

        # Step 2: Check for colored objects for avoidance (second priority)
        # Process camera frame
        bytes_buffer += proc.stdout.read(1024)
        a = bytes_buffer.find(b'\xff\xd8')
        b = bytes_buffer.find(b'\xff\xd9')
        if a != -1 and b != -1:
            jpg = bytes_buffer[a:b+2]
            bytes_buffer = bytes_buffer[b+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Color detection masks
            lower_green = np.array([45, 100, 100])
            upper_green = np.array([65, 255, 255])
            lower_red1 = np.array([170, 120, 120])
            upper_red1 = np.array([179, 255, 255])
            lower_red2 = np.array([0, 120, 120])
            upper_red2 = np.array([10, 255, 255])
            
            mask_green = cv2.inRange(hsv, lower_green, upper_green)
            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            
            kernel = np.ones((5, 5), np.uint8)
            mask_green = cv2.erode(mask_green, kernel, iterations=1)
            mask_green = cv2.dilate(mask_green, kernel, iterations=1)
            mask_red = cv2.erode(mask_red, kernel, iterations=1)
            mask_red = cv2.dilate(mask_red, kernel, iterations=1)
            
            green_area, red_area = 0, 0
            
            contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours_green:
                largest_green_contour = max(contours_green, key=cv2.contourArea)
                green_area = cv2.contourArea(largest_green_contour)
            
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours_red:
                largest_red_contour = max(contours_red, key=cv2.contourArea)
                red_area = cv2.contourArea(largest_red_contour)
            
            # Action based on color detection
            if green_area > MIN_OBJECT_AREA:
                print("Green object detected. Turning right.")
                set_servo_pwm(10.83) # Turn right
            elif red_area > MIN_OBJECT_AREA:
                print("Red object detected. Turning left.")
                set_servo_pwm(5.89) # Turn left
            else:
                set_servo_pwm(8.61) # Go straight
                
            cv2.imshow("Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # Step 3: Check for side obstacles if no colored object is in view (third priority)
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

        if avg_left_dist < 15 and avg_left_dist > 0:
            set_servo_pwm(10.83) # Turn right
        elif avg_right_dist < 15 and avg_right_dist > 0:
            set_servo_pwm(5.89) # Turn left
        else:
            set_servo_pwm(8.61) # Go straight
        
        # Step 4: Check color sensor for laps (lowest priority)
        current_time = time.time()
        if current_time - last_count_time < 2:
            time.sleep(0.1)
            continue
        
        red_frequency = get_frequency(GPIO.LOW, GPIO.LOW)
        green_frequency = get_frequency(GPIO.HIGH, GPIO.HIGH)
        blue_frequency = get_frequency(GPIO.LOW, GPIO.HIGH)
        
        normalized_red = (red_frequency / RED_CALIBRATION) * 255
        normalized_green = (green_frequency / GREEN_CALIBRATION) * 255
        normalized_blue = (blue_frequency / BLUE_CALIBRATION) * 255
        
        current_color = "UNKNOWN"
        if normalized_blue > normalized_red and normalized_blue > normalized_green and normalized_blue > 150:
            current_color = "BLUE"
        elif normalized_red > normalized_blue and normalized_green > normalized_blue and normalized_red > 150 and normalized_green > 100:
            current_color = "ORANGE"

        if current_color == last_detected_color:
            consecutive_count += 1
        else:
            last_detected_color = current_color
            consecutive_count = 1

        if current_color != "UNKNOWN" and consecutive_count >= 12:
            lap_count += 1
            print(f"Color ({current_color}) detected 12 times in a row! Lap: {lap_count}")
            consecutive_count = 0
            last_detected_color = None
            last_count_time = current_time

        if lap_count >= 3:
            print("Task completed. Laps completed: 3. System is shutting down.")
            break

        time.sleep(0.1)

finally:
    proc.terminate()
    cv2.destroyAllWindows()
    stop_motor()
    pwm_servo.stop()
    pwm_motor.stop()
    GPIO.cleanup()