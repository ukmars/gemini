# simple wall follower for the Gemini version of UKMARSbot

from machine import Pin, ADC, PWM, UART
import time
import os
# These are the pin connection settings for the main board and mezzanine using the Pi Pico
leftMezzLED = Pin(12,Pin.OUT) # The LED marked D1 on the left side of the board
rightMezzLED = Pin(13,Pin.OUT) # The LED marked D2 on the right side of the board
onBoardLED = Pin("LED", Pin.OUT) # The small red LED on the Pico processor board
leftFwd = PWM(Pin(2))
leftFwd.freq(2000)
leftRev = PWM(Pin(3))
leftRev.freq(2000)
rightFwd = PWM(Pin(4))
rightFwd.freq(2000)
rightRev = PWM(Pin(5))
rightRev.freq(2000)
leftButton = Pin(15, Pin.IN, Pin.PULL_UP) # The tactile button switch marked SW1
rightButton = Pin(14, Pin.IN, Pin.PULL_UP) # The tactile button switch marked SW2

# These are the pin connection settings for use with the wall sensor board
# phototransistor sensor pins
leftSensor = ADC(28) # input from the left wall sensor
rightSensor = ADC(26) # input from the right wall sensor
frontSensor = ADC(27) # input from the front wall sensor
#Triggers for LEDs
sidesEmitter = Pin(22,Pin.OUT) # switches on the 2 side facing wall illumination LEDs
frontEmitter = Pin(21,Pin.OUT) # switches on the forward facing wall illumination LEDs
# These are the indicator LEDs on the wall sensor board
leftSensorLED = Pin(20,Pin.OUT) # indicator LED for when left wall seen
centreSensorLED = Pin(19,Pin.OUT) # indicator LED for when front wall seen
rightSensorLED = Pin(18,Pin.OUT) # indicator LED for when right wall seen

leftSensorLED.value(1)
centreSensorLED.value(1)
rightSensorLED.value(1)
time.sleep(5)
leftSensorLED.value(0)
centreSensorLED.value(0)
rightSensorLED.value(0)
# Variables
leftval = 3000
rightval = 3000
frontval = 6000

leftRev.duty_u16(0)
rightRev.duty_u16(0)

def readSensors():
    #Values are derived by subtracting the lit value of a sensor from the unlit value 
    #The UKMARS wall sensor board gives high value readings for low incident light and low value readings for high incident light 
    global leftSensorValue, rightSensorValue, frontSensorValue
    global leftSensorLit, rightSensorLit, frontSensorLit
    global leftSensorUnlit, rightSensorUnlit, frontSensorUnlit
    global leftval, rightval, frontval

    leftSensorUnlit = leftSensor.read_u16()
    rightSensorUnlit = rightSensor.read_u16()
    sidesEmitter.value(1)
    time.sleep_us(75)
    leftSensorLit = leftSensor.read_u16()
    rightSensorLit = rightSensor.read_u16()
    sidesEmitter.value(0)
    
    frontSensorUnlit = frontSensor.read_u16()    
    frontEmitter.value(1)
    time.sleep_us(75)
    frontSensorLit = frontSensor.read_u16()
    frontEmitter.value(0)
    time.sleep_us(75)

    leftSensorValue = (leftSensorLit - leftSensorUnlit) # 
    rightSensorValue = (rightSensorLit- rightSensorUnlit) # 
    frontSensorValue = (frontSensorLit- frontSensorUnlit)
    
    if leftSensorValue > leftval:
        leftSensorLED.value(1)
    else:
        leftSensorLED.value(0)
    if rightSensorValue > rightval:
        rightSensorLED.value(1)
    else:
        rightSensorLED.value(0)   
    if frontSensorValue > frontval:
        centreSensorLED.value(1)
    else:
        centreSensorLED.value(0)       
        

def wallfollow():
    global leftSensorValue, rightSensorValue, frontSensorValue
    global leftWall, rightWall,frontWall,basespeed
    pfactor = 0.4
    dfactor = 20
    preverr = 0
    difference = 0
    readSensors() # get initial side wall values
    leftInit = leftSensorValue
    rightInit = rightSensorValue
    basespeed = 12000
    maxspeed = 65535
    leftspeed = basespeed
    rightspeed = basespeed
    leftFwd.duty_u16(maxspeed)  # Set left forward max speed
    leftRev.duty_u16(maxspeed - leftspeed)      # and reverse speed to max speed - desired speed
    rightFwd.duty_u16(maxspeed) # Set right forward max speed
    rightRev.duty_u16(maxspeed - rightspeed)     # and reverse speed to max speed - desired speed
    while(True):
        readSensors() #read the sensors
        checkwalls() # see if walls are presents
        if frontWall == True:
            turnright()
            #stop()
        difference = leftSensorValue - leftInit # get difference from initial wall value
        dterm = int((difference - preverr) * dfactor)
        error = int(difference * pfactor) # proportional factor
        leftspeed = basespeed + error + dterm
        rightspeed = basespeed - error - dterm
        # check that speed values have not gone out of range
        if (leftspeed > 65500):
            leftspeed = 65500
        if (leftspeed < 1000):
            leftspeed = 1000
        if (rightspeed > 65500):
            rightspeed = 65500
        if (rightspeed < 1000):
            rightspeed = 1000
        #print( leftSensorValue,  rightSensorValue, difference, leftspeed, rightspeed)
        #time.sleep(0.5)
        # light mezzanine LEDs to say which side of line we are on
        if (difference > 0):
            leftMezzLED.on()
            rightMezzLED.off()
        else:
            leftMezzLED.off()
            rightMezzLED.on()
        leftFwd.duty_u16(maxspeed)  # Set left forward speed
        leftRev.duty_u16(maxspeed - leftspeed) 
        rightFwd.duty_u16(maxspeed) # Set right forward speed
        rightRev.duty_u16(maxspeed - rightspeed)
        preverr = difference

def checkwalls():
    global leftSensorValue, rightSensorValue, frontSensorValue
    global leftWall, rightWall,frontWall, leftval, rightval,frontval
    if leftSensorValue > leftval:
        leftWall = 1
    else:
        leftwall = 0
    if rightSensorValue > rightval:
        rightWall = 1
    else:
        rightwall = 0
    if frontSensorValue > frontval:
        frontWall = 1
    else:
        frontWall = 0

def stop():
    maxspeed = 65535
    leftFwd.duty_u16(maxspeed)  # Set left forward speed
    leftRev.duty_u16(maxspeed) 
    rightFwd.duty_u16(maxspeed) # Set right forward speed
    rightRev.duty_u16(maxspeed)
    if True:
        leftMezzLED.on()
        time.sleep(0.25)
        leftMezzLED.off()
        time.sleep(0.25)
            
def turnright():
    global basespeed
    maxspeed = 65535
    rightspeed = int(basespeed / 3)
    leftFwd.duty_u16(maxspeed)  # Set left forward speed
    leftRev.duty_u16(maxspeed - basespeed) 
    rightFwd.duty_u16(maxspeed) # Set right forward speed
    rightRev.duty_u16(maxspeed - rightspeed)
    time.sleep(0.5)
    
    
    
    
def photoshow():
    global leftSensorValue, rightSensorValue, frontSensorValue
    global leftSensorLit, rightSensorLit, frontSensorLit
    global leftSensorUnlit, rightSensorUnlit, frontSensorUnlit
    while True:
        readSensors()
        print("diff", leftSensorValue,  rightSensorValue, frontSensorValue, "Lit", leftSensorLit, rightSensorLit, frontSensorLit,"Unlit",leftSensorUnlit, rightSensorUnlit, frontSensorUnlit)
        time.sleep(0.5)

def motortest():
    maxspeed = 65535
    speed = 25000
    reverse = 15000
    # run motors forward and backwards and light the LEDs then stop motors
    print("motortest")
    while True:      
        leftMezzLED.on()         # Switch on the left mezzanine LED
        leftFwd.duty_u16(speed)  # Set left forward speed to 15000
        leftRev.duty_u16(maxspeed-speed)      # and reverse speed to zero
        rightFwd.duty_u16(speed) # Set right forward speed to 15000
        rightRev.duty_u16(maxspeed-speed)     # and reverse speed to zero
        time.sleep(5)            # Wait for 5 seconds
        leftMezzLED.off()        # Switch off the left mezzanine LED
        rightMezzLED.on()        # Switch on the right mezzanine LED
        leftRev.duty_u16(reverse)  # Set left reverse speed to 15000
        leftFwd.duty_u16(maxspeed-reverse)      # and forward speed to zero
        rightRev.duty_u16(reverse) # Set left reverse speed to 15000
        rightFwd.duty_u16(maxspeed-reverse)     # and forward speed to zero
        time.sleep(5)            # Wait for 5 seconds
        leftMezzLED.off()        # Switch off the left mezzanine LED
        rightMezzLED.off()       # Switch off the right mezzanine LED
        leftFwd.duty_u16(0)      # Set left forward speed to zero
        leftRev.duty_u16(0)      # and reverse speed to zero
        rightFwd.duty_u16(0)     # Set right forward speed to zero
        rightRev.duty_u16(0)     # and reverse speed to zero 
        time.sleep(1) 

# code executed when program starts
wallfollow()
#photoshow()
#motortest()