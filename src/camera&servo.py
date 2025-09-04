import cv2
import numpy as np
import subprocess
import RPi.GPIO as GPIO
import time

SERVO_PIN = 18
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)

def set_angle(angle):
    duty_cycle = (angle + 90) / 18.0 + 2
    pwm.ChangeDutyCycle(duty_cycle)

# Command to start the Raspberry Pi camera with MJPEG streaming
cmd = ["rpicam-vid", "--width", "320", "--height", "240", "--codec", "mjpeg", "--timeout", "0", "--framerate", "30", "-o", "-"]
proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
bytes_buffer = b""

frame_width = 320
frame_center = frame_width / 2

# PD controller coefficients. Adjust these to reduce oscillations.
kp = 0.1  # Proportional coefficient (P)
kd = 0.05 # Differential coefficient (D)

# Variables for the PD controller
last_error = 0

# Minimum area threshold for detecting an object
MIN_OBJECT_AREA = 2000 

try:
    set_angle(0)
    while True:
        bytes_buffer += proc.stdout.read(1024)
        a = bytes_buffer.find(b'\xff\xd8')
        b = bytes_buffer.find(b'\xff\xd9')
        if a != -1 and b != -1:
            jpg = bytes_buffer[a:b+2]
            bytes_buffer = bytes_buffer[b+2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define HSV ranges for green and red colors
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
            
            # Apply morphological operations to reduce noise
            kernel = np.ones((5, 5), np.uint8)
            mask_green = cv2.erode(mask_green, kernel, iterations=1)
            mask_green = cv2.dilate(mask_green, kernel, iterations=1)
            mask_red = cv2.erode(mask_red, kernel, iterations=1)
            mask_red = cv2.dilate(mask_red, kernel, iterations=1)
            
            green_area, red_area = 0, 0
            target_center_x = None

            # Find contours for green objects
            contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours_green:
                largest_green_contour = max(contours_green, key=cv2.contourArea)
                green_area = cv2.contourArea(largest_green_contour)
                if green_area > MIN_OBJECT_AREA:
                    M = cv2.moments(largest_green_contour)
                    if M["m00"] != 0:
                        target_center_x = int(M["m10"] / M["m00"])
            
            # Find contours for red objects
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours_red:
                largest_red_contour = max(contours_red, key=cv2.contourArea)
                red_area = cv2.contourArea(largest_red_contour)
                if red_area > MIN_OBJECT_AREA and red_area > green_area:
                    M = cv2.moments(largest_red_contour)
                    if M["m00"] != 0:
                        target_center_x = int(M["m10"] / M["m00"])

            direction_text = "No Sign Detected: Center"
            
            if target_center_x is not None:
                error = frame_center - target_center_x
                
                # Calculate differential component
                derivative = error - last_error
                
                # Calculate angle using PD controller
                angle_change = -(kp * error + kd * derivative)

                # Update the last error
                last_error = error

                # Limit the angle change to prevent excessive movement
                if angle_change > 15:
                    angle_change = 15
                elif angle_change < -15:
                    angle_change = -15
                
                set_angle(angle_change)

                if green_area > red_area:
                    direction_text = f"Green: {int(angle_change)} degrees"
                else:
                    direction_text = f"Red: {int(angle_change)} degrees"
            else:
                set_angle(0)
                last_error = 0 # Reset error if object is lost
                direction_text = "No Sign Detected: Center"

            cv2.putText(frame, direction_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow("Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

finally:
    proc.terminate()
    cv2.destroyAllWindows()
    pwm.stop()
    GPIO.cleanup()
