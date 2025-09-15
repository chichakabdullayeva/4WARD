import RPi.GPIO as GPIO
import time

# Ultrasonic sensor pins
FRONT_TRIG = 4
FRONT_ECHO = 23
LEFT_TRIG = 21
LEFT_ECHO = 20
RIGHT_TRIG = 27
RIGHT_ECHO = 17

# Servo pin
SERVO_PIN = 18

# Motor driver pins
MOTOR_IN1 = 24
MOTOR_IN2 = 25
MOTOR_PWM = 13
STBY_PIN = 22

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

class TB6612MotorDriver:
    def __init__(self, in1_pin, in2_pin, pwm_pin, stby_pin, pwm_freq=1000):
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.pwm_pin = pwm_pin
        self.stby_pin = stby_pin
        
        # Setup motor GPIO pins
        GPIO.setup([self.in1_pin, self.in2_pin, self.stby_pin], GPIO.OUT)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        
        # Initialize PWM
        self.pwm = GPIO.PWM(self.pwm_pin, pwm_freq)
        self.pwm.start(0)
        
        # Enable driver (standby off)
        GPIO.output(self.stby_pin, GPIO.HIGH)
        print("Motor driver initialized")
    
    def forward(self, speed):
        speed = max(0, min(100, speed))
        GPIO.output(self.in1_pin, GPIO.HIGH)
        GPIO.output(self.in2_pin, GPIO.LOW)
        self.pwm.ChangeDutyCycle(speed)
        print(f"Motor moving forward at {speed}% speed")
    
    def backward(self, speed):
        speed = max(0, min(100, speed))
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(speed)
        print(f"Motor moving backward at {speed}% speed")
    
    def stop(self):
        GPIO.output(self.in1_pin, GPIO.HIGH)
        GPIO.output(self.in2_pin, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(0)
        print("Motor stopped (brake)")
    
    def coast(self):
        GPIO.output(self.in1_pin, GPIO.LOW)
        GPIO.output(self.in2_pin, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
        print("Motor coasting")
    
    def cleanup(self):
        self.pwm.stop()

def setup_ultrasonic_sensor(trig_pin, echo_pin):
    GPIO.setup(trig_pin, GPIO.OUT)
    GPIO.setup(echo_pin, GPIO.IN)
    GPIO.output(trig_pin, False)
    time.sleep(0.2)
    print(f"Sensor on pins TRIG={trig_pin}, ECHO={echo_pin} configured.")

def get_distance(trig_pin, echo_pin, samples=3):
    distances = []
    for _ in range(samples):
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(trig_pin, False)
        
        pulse_start = time.time()
        pulse_end = time.time()
        
        timeout = time.time()
        while GPIO.input(echo_pin) == 0:
            pulse_start = time.time()
            if time.time() - timeout > 0.04:
                return None
        
        timeout = time.time()
        while GPIO.input(echo_pin) == 1:
            pulse_end = time.time()
            if time.time() - timeout > 0.04:
                return None
        
        pulse_duration = pulse_end - pulse_start
        distance_cm = pulse_duration * 17150
        
        if distance_cm > 0:
            distances.append(distance_cm)
            
        time.sleep(0.02)
    
    if distances:
        return round(sum(distances) / len(distances), 2)
    else:
        return None


def set_servo_position(servo_pwm, position):
    """
    Set servo to specific position
    position: 'left', 'center', 'right'
    """
    if position == 'left':
        duty_cycle = 5.5   # 0 degrees (left)
    elif position == 'center':
        duty_cycle = 8.5   # 90 degrees (center)
    elif position == 'right':
        duty_cycle = 12.5  # 180 degrees (right)
    else:
        duty_cycle = 8.5   # default to center
    
    servo_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.3)  # Give servo time to move
    print(f"Servo moved to {position} position (duty cycle: {duty_cycle})")

try:
    # Setup sensors
    setup_ultrasonic_sensor(FRONT_TRIG, FRONT_ECHO)
    setup_ultrasonic_sensor(LEFT_TRIG, LEFT_ECHO)
    setup_ultrasonic_sensor(RIGHT_TRIG, RIGHT_ECHO)
    
    # Setup servo
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    servo_pwm = GPIO.PWM(SERVO_PIN, 50)
    servo_pwm.start(8.5)  # Start at center position
    print("Servo initialized at center position")
    
    # Initialize motor
    motor = TB6612MotorDriver(MOTOR_IN1, MOTOR_IN2, MOTOR_PWM, STBY_PIN)
    
    print("Robot control started... Press Ctrl+C to exit.")
    
    while True:
        left_dist = get_distance(LEFT_TRIG, LEFT_ECHO)
        right_dist = get_distance(RIGHT_TRIG, RIGHT_ECHO)
        front_dist = get_distance(FRONT_TRIG, FRONT_ECHO)
        
        print("-" * 50)
        print(f"Distances - Left: {left_dist}, Right: {right_dist}, Front: {front_dist}")
        
        # Obstacle avoidance logic with motor control
        if front_dist is not None and front_dist <= 10:
            # Obstacle in front - stop and decide direction
            motor.stop()
            set_servo_position(servo_pwm, 'center')
            print("Obstacle ahead! Stopping motor.")
            
            # Check left and right for best path
            if left_dist is not None and right_dist is not None:
                if left_dist > right_dist and left_dist > 10:
                    set_servo_position(servo_pwm, 'left')
                    motor.backward(30)  # Reverse a bit
                    time.sleep(0.5)
                    motor.stop()
                    print("Turning left")
                elif right_dist > 20:
                    set_servo_position(servo_pwm, 'right')
                    motor.backward(30)  # Reverse a bit
                    time.sleep(0.5)
                    motor.stop()
                    print("Turning right")
                else:
                    motor.backward(40)  # Back up if both sides blocked
                    print("Backing up - both sides blocked")
            else:
                motor.backward(40)
                print("Backing up - sensor read error")
                
        elif left_dist is not None and left_dist <= 10:
            # Obstacle on left - turn servo right to avoid it
            set_servo_position(servo_pwm, 'right')
            motor.forward(40)
            print(f"Left obstacle at {left_dist:.2f} cm. Steering right.")
            
        elif right_dist is not None and right_dist <= 10:
            # Obstacle on right - turn servo left to avoid it
            set_servo_position(servo_pwm, 'left')
            motor.forward(40)
            print(f"Right obstacle at {right_dist:.2f} cm. Steering left.")
            
        else:
            # Path clear - move forward with centered servo
            set_servo_position(servo_pwm, 'center')
            motor.forward(50)
            if front_dist is not None:
                print(f"Path clear. Moving forward. Front distance: {front_dist:.2f} cm")
            else:
                print("Path clear. Moving forward. Front sensor error.")
        
        time.sleep(0.3)

except KeyboardInterrupt:
    print("\nProgram terminated by user.")
    
finally:
    # Cleanup everything
    if 'motor' in locals():
        motor.stop()
        motor.cleanup()
    if 'servo_pwm' in locals():
        servo_pwm.stop()
    GPIO.cleanup()
    print("All components cleaned up.")