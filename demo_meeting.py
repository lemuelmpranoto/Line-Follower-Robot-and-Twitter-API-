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

"""
# Buzzer config
buzzer = 23
GPIO.setup(buzzer, GPIO.OUT)
buzzerPWM = GPIO.PWM(buzzer, 400)
buzzerPWM.start(0)
"""

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





# Main function
if __name__ == '__main__':
    try:
        """
        # Motor test
        forward(0.5)
        time.sleep(0.5)
        turnRight(0.65)
        time.sleep(1)
        stop()
        turnLeft(0.65)
        time.sleep(2)
        stop()
        """
    
        # LCD test
        lcd.message('Hello!')
        time.sleep(2)
        lcd.clear()
        
        while True:
            left = color(opSense1)
            right = color(opSense2)
            front = color(opSense3)
            print( left, right, front)
            if front == 1 and not (left == 0 and right == 0): # BLACK
                    forward(0.5)
                    lcd.clear()
                    lcd.message('Forward!')
                    time.sleep(0.001)
            else:
                if left == 1 and right == 0:
                    stop()
                    lcd.clear()
                    lcd.message('Turn Left')
                    turnLeft(1.5)
                    time.sleep(0.005)
                elif left == 0 and right == 1:
                    stop()
                    lcd.clear()
                    lcd.message('Turn Right')
                    turnRight(1.5)
                    time.sleep(0.005)
                elif left == 1 and right == 1:
                    forward(0.5)
                    lcd.clear()
                    lcd.message('Forward!')
                    time.sleep(0.001)
                elif left == 0 and right == 0:
                    forward(0.4)
                    time.sleep(0.4)
                    if color(opSense1) == 1 or color(opSense2) == 1:
                        continue
                    time.sleep(0.3)
                    if color(opSense1) == 0 and color(opSense2) == 0:
                        lcd.clear()
                        lcd.message("STOP")
                        stop()
                        while(color(opSense1) == 0 and color(opSense2) == 0):
                            pass
        
        # while True: 
            # if color(opSense3) == 1 and (color(opSense1) == 1 or color(opSense2) == 1): # BLACK
                # forward(0.4)
                # lcd.clear()
                # lcd.message('Forward!')
                # time.sleep(0.005)
            
            # elif color(opSense1) == 1 and color(opSense2) == 0 and color(opSense3) == 0: # LEFT BLACK - RIGHT WHITE
                # stop()
                # lcd.clear()
                # lcd.message('Turn Left')
                #while(color(opSense2) == 0 and color(opSense1) == 1 and color(opSense3) == 0):
                # turnLeft(1.5)
                # time.sleep(0.003)
                    
            # elif color(opSense1) == 0 and color(opSense2) == 1 and color(opSense3) == 0: # LEFT WHITE - RIGHT BLACK
                # stop()
                # lcd.clear()
                # lcd.message('Turn Right')
                #while(color(opSense1) == 0 and color(opSense2) == 1 and color(opSense3) == 0):
                # turnRight(1.5)
                # time.sleep(0.003)
                    
            # if color(opSense1) == 0 and color(opSense2) == 0: # both WHITE - GAP or STOP
                # forward(0.3)
                # time.sleep(0.5)
                # if color(opSense1) == 0 and color(opSense2) == 0:
                    # lcd.clear()
                    # lcd.message('STOP')
                    # stop()
                    # break
            
                
        
        """
            # Ultrasonic sensor test
            dist = distance()
            print ("Measured Distance = %.1f cm" % dist)
            time.sleep(1)   
            
            if dist < 5: # If too close to an object, stop
                stop()
                lcd.clear()
                lcd.message('Object ahead:\n %.1f cm' % dist) 
                # Buzzer test
                buzzerPWM.ChangeDutyCycle(50)
                time.sleep(0.5)
                buzzerPWM.ChangeDutyCycle(0)
        """
           
            
    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print('\n')
        print("Measurement stopped by User")
        stop()

GPIO.cleanup()