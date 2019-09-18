# Demo time (meeting room 3047)
# Libraries
import time
import RPi.GPIO as GPIO
from adafruit_motorkit import MotorKit
import Adafruit_CharLCD as LCD

# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# Motor kit config
# motor1 = LEFT; forward = -1
# motor2 = RIGHT; forward = 1
kit = MotorKit()

# LCD config
lcd_rs        = 22
lcd_en        = 17
lcd_d4        = 26
lcd_d5        = 19
lcd_d6        = 13
lcd_d7        = 6
lcd_backlight = 4
lcd_columns   = 16
lcd_rows      = 2
lcd = LCD.Adafruit_CharLCD(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7,
                           lcd_columns, lcd_rows, lcd_backlight)

# Ultrasonic sensor config
GPIO_TRIGGER = 20
GPIO_ECHO = 21
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

# Optical sensor config
opSense1 = 16 # LEFT sensor
GPIO.setup(opSense1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
opSense2 = 12 # RIGHT sensor
GPIO.setup(opSense2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
opSense3 = 25 # FRONT sensor
GPIO.setup(opSense3, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Buzzer config
buzzer = 23
GPIO.setup(buzzer, GPIO.OUT)
buzzerPWM = GPIO.PWM(buzzer, 400)
buzzerPWM.start(0)

# Motor kit helper functions
# Speed from -1 -> 1; 0 = stop
def forward(speed):
    kit.motor1.throttle = -speed
    kit.motor2.throttle = speed

def backward(speed):
    kit.motor1.throttle = speed
    kit.motor2.throttle = -speed

def stop():
    kit.motor1.throttle = 0
    kit.motor2.throttle = 0

def turnLeft(speed):
    kit.motor1.throttle = speed/2
    kit.motor2.throttle = speed/2
    
    
def turnRight(speed):
    kit.motor1.throttle = -speed/2
    kit.motor2.throttle = -speed/2

# Optical sensor helper functions
# 0 = WHITE; 1 = BLACK
def color(sensor):
    if GPIO.input(sensor) == GPIO.LOW:
        #print("Detected WHITE")
        #print("______________")
        retColor = 0
    else:
        #print("Detected BLACK")
        #print("______________")
        retColor = 1
    return retColor

# Ultrasonic sensor helper functions    
# This function calculates the distance measured by the ultrasonic sensor 
def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    StartTime = time.time()
    StopTime = time.time()
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        pass
    StartTime = time.time()
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        pass
    StopTime = time.time()
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 1000000) / 58.3 
    return distance


# Auto function
# This function lets the robot follow a black path. This is the main functionality.
def auto_run():
    while True:
        # read optical sensors
        left = color(opSense1)
        right = color(opSense2)
        front = color(opSense3)
        if front == 1 and not (left == 0 and right == 0):
                # detected BLACK, move forward
                forward(0.5)
                lcd.clear()
                lcd.message('Forward!')
                time.sleep(0.001)
        else:
            # did not detect BLACK, decide what to do
            if left == 1 and right == 0:
                # turn left
                stop()
                lcd.clear()
                lcd.message('Turn Left')
                turnLeft(1.7)
                time.sleep(0.005)
            elif left == 0 and right == 1:
                # turn right
                stop()
                lcd.clear()
                lcd.message('Turn Right')
                turnRight(1.7)
                time.sleep(0.005)
            elif left == 1 and right == 1:
                # turn forward if both side sensors detect BLACK
                forward(0.5)
                lcd.clear()
                lcd.message('Forward!')
                time.sleep(0.001)
            elif left == 0 and right == 0:
                # all sensors are off: now move forward 3 cm and check again
                forward(0.3)
                time.sleep(0.4)
              
                # checking if both sensors are detecting black               
                if color(opSense1) == 1 or color(opSense2) == 1:
                    continue
                    
                # if there is a gap in the track, it will turn right and left to find the black tape   
                time.sleep(0.3)
                flag = 0
                if color(opSense1) == 0 and color(opSense2) == 0:
                    for i in range (0,20,1):
                        turnRight(1.5)
                        time.sleep(0.01)
                        stop()
                        if color(opSense3) == 1:
                            stop()
                            flag = 1
                            break
                    stop()
                    if flag == 0:
                        for i in range (0,40,1):
                            turnLeft(1.5)
                            time.sleep(0.01)
                            stop()
                            if color(opSense3) == 1:
                                flag = 1
                                stop()
                                break
                        stop()
              
                    # to stop the function when the track ends and displays message on the LCD
                    if flag == 0:
                        # reached end of track
                        lcd.clear()
                        lcd.message("STOP")
                        stop()
                        while(color(opSense1) == 0 and color(opSense2) == 0):
                            pass
                    else: 
                        continue

# moves the robot depending on WASD movement code.
def move(dir):
    if (dir == 'w'):
        lcd.clear()
        lcd.message('Moving forward')
        forward(speed)
    elif (dir == 'a'):
        lcd.clear()
        lcd.message('Turning left')
        turnLeft(speed*1.5)
    elif (dir == 's'):
        lcd.clear()
        lcd.message('Moving backward')
        backward(speed)
    elif (dir == 'd'):
        lcd.clear()
        lcd.message('Turning right')
        turnRight(speed*1.5)

# roam mode for the robot. In this mode, the robot moves forward until it
# encounters an obstacle. It will try to maneuver away from the obstacle.
def roam():
    while True:
        # move forward for 0.5 seconds
        lcd.clear()
        lcd.message('Roaming...')
        forward(0.3)
        time.sleep(0.5)
        # poll distance and decide what to do
        dist = distance()
        if dist < 20:
            # detected obstacle
            stop()
            time.sleep(0.5)
            # decide randomly (depending on distance) whether to turn right or left
            leftTurn = (round(dist) % 2 == 0)
            # turn until no obstacle is found
            while dist < 20:
                if (leftTurn):
                    turnLeft(1)
                    lcd.clear()
                    lcd.message('Roaming...')
                    lcd.message('\n turn Left')
                else:
                    turnRight(1)
                    lcd.clear()  
                    lcd.message('Roaming...')
                    lcd.message('\n turn Right')
                time.sleep(0.5)
                dist = distance()

# Main function
if __name__ == '__main__':
    
    # LCD test
    lcd.message('Hello!')
    time.sleep(2)
    lcd.clear()
    
    # main loop
    while True:
        # stop robot and wait for command
        stop()
        lcd.clear()
        lcd.message("Waiting for\ncommand")
        input_ = input("Enter command: auto, roam, [WASD][duration], exit:")
        if (input_ == "auto"):
            # enable auto mode, main functionality.
            try:
                auto_run()
            except KeyboardInterrupt:
                print("\n")
        elif(len(input_) == 0 or input_.isspace()): 
            print("No command, retry")
            continue
            
        elif (input_.split()[0].lower() in ('w', 'a', 's', 'd')):
            # parse movement command and move manually
            move_cmd = input_.split()
            dir = move_cmd[0]
            speed = 0.5
            if (len(move_cmd) == 1):
                duration = 1.0
            elif (len(move_cmd) == 2):
                try:
                    duration = float(move_cmd[1])
                except ValueError:
                    print("Invalid duration")
                    continue
            
            # poll distance and estimate if there is enough space to move
            dist = distance()
            
            if dist < duration * 22.0: # If too close to an object, stop
                lcd.clear()
                lcd.message('Object ahead:\n %.1f cm' % dist)
                print('Cannot move: Object ahead at {:.1f} cm, lower than expected movement distance {:.1f} cm'.format(dist, duration*22.0))
                # sound an error message using the buzzer
                if(dir == 'w'):
                    stop()
                    buzzerPWM.ChangeDutyCycle(50)
                    time.sleep(0.3)
                    buzzerPWM.ChangeDutyCycle(0)   
                    time.sleep(0.1)
                    buzzerPWM.ChangeDutyCycle(50)
                    time.sleep(0.5)
                    buzzerPWM.ChangeDutyCycle(0)  
                else:
                    move(dir)
                    try:
                        time.sleep(duration)
                    except KeyboardInterrupt:
                        print("\n")
            else:
                move(dir)
                try:
                    time.sleep(duration)
                except KeyboardInterrupt:
                    print("\n")
        
        elif (input_ == "roam"):
            # enable roam functionality
            print("Roaming...")
            try:
                roam()
            except KeyboardInterrupt:
                print("\n")
        
        elif (input_ == "exit"):
            # quit program
            print('Quitting program...')
            break
            
        else:
            print("Unrecognized command")
            
         

GPIO.cleanup()
