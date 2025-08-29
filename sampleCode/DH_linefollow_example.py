# simple line follower for the Gemini version of UKMARSbot

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

# These are the pin connection settings for use with the line sensor board
# phototransistor sensor pins
leftSensor = ADC(28) # left front line sensor 
rightSensor = ADC(26) # right front line sensor
radiusSensor = ADC(27) # left side radius change marker sensor
startSensor = ADC(27) # right side start/stop marker sensor
#Triggers for LEDs
emitter = Pin(22,Pin.OUT) # trigger for front and right side start/stop LED
radiusEmitter = Pin(18,Pin.OUT) # trigger for left side radius change LED
# These are the indicator LEDs on the line sensor board
leftSensorLED = Pin(21,Pin.OUT) # left indicator LED
centreSensorLED = Pin(20,Pin.OUT) # centre indicator LED
rightSensorLED = Pin(19,Pin.OUT) # right indicator LED

# Variables

def readSensors():
    #Values are derived by subtracting the lit value of a sensor from the unlit value 
    #Unlit raw readings close to 65000 indicate good separation from ambient light
    #The radius and start/finish sensors are multiplexed onto the same ADC channel using separate emitters
    #The UKMARS line follower sensor board gives high value readings for low incident light and low value readings for high incident light
    
    global leftSensorValue, rightSensorValue, radiusSensorValue, startSensorValue
    global leftSensorUnlit, rightSensorUnlit, radiusSensorUnlit, startSensorUnlit


    #leftSensorUnlit = leftSensor.read_u16()
    #rightSensorUnlit = rightSensor.read_u16()
    #startSensorUnlit = startSensor.read_u16()
    #radiusSensorUnlit = radiusSensor.read_u16()
    
    emitter.value(1)
    #time.sleep_us(75)
    leftSensorLit = leftSensor.read_u16()
    rightSensorLit = rightSensor.read_u16()
    #startSensorLit = startSensor.read_u16()
    #emitter.value(0)
    
    #radiusEmitter.value(1)
    #time.sleep_us(75)
    #radiusSensorLit = radiusSensor.read_u16()
    #radiusEmitter.value(0)
    #time.sleep_us(75)

    leftSensorValue = (leftSensorLit) # - leftSensorLit)
    rightSensorValue = (rightSensorLit) # - rightSensorLit)
    #startSensorValue = (startSensorUnlit - startSensorLit)
    #radiusSensorValue = (radiusSensorUnlit - radiusSensorLit)

def linefollow():
    global leftSensorValue, rightSensorValue, radiusSensorValue, startSensorValue
    global leftFrontLow, leftFrontHigh,rightFrontLow, rightFrontHigh
    leftFrontLow = rightFrontLow = 35000
    leftFrontHigh = rightFrontHigh = 35000
    pfactor = 0.4
    dfactor = 20
    preverr = 0
    difference = 0
    basespeed = 11500 
    leftspeed = basespeed
    rightspeed = basespeed
    leftFwd.duty_u16(leftspeed)  # Set left forward speed
    leftRev.duty_u16(0)      # and reverse speed to zero
    rightFwd.duty_u16(rightspeed) # Set right forward speed
    rightRev.duty_u16(0)     # and reverse speed to zero
    while(True):
        readSensors() #read the sensors
        highlow() # capture lowest and highest sensor values
        side =  0
        leftSensorTrigger = leftFrontHigh - ((leftFrontLow + leftFrontHigh) / 4)
        if leftSensorValue > leftSensorTrigger : # going off the line to left
            side = 1
            leftspeed = int(basespeed * 1.2)
            rightspeed = int(basespeed / 2)
        rightSensorTrigger = rightFrontHigh - ((rightFrontLow + rightFrontHigh) / 4)
        if rightSensorValue > rightSensorTrigger : # going off the line to right
            side = 2
            leftspeed = int(basespeed /2)
            rightspeed = int(basespeed * 1.2)        
        if side == 0:    
            difference = leftSensorValue - rightSensorValue # get difference between front sensors
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
        leftFwd.duty_u16(leftspeed)  # Set left forward speed
        rightFwd.duty_u16(rightspeed) # Set right forward speed
        preverr = difference

def highlow():
    global leftSensorValue, rightSensorValue, radiusSensorValue, startSensorValue
    global leftFrontLow, leftFrontHigh,rightFrontLow, rightFrontHigh
    if leftSensorValue > leftFrontHigh:
        leftFrontHigh = leftSensorValue
    if leftSensorValue < leftFrontLow:
        leftFrontLow = leftSensorValue
    if rightSensorValue > rightFrontHigh:
        rightFrontHigh = rightSensorValue
    if rightSensorValue < rightFrontLow:
        rightFrontLow = rightSensorValue

def photoshow():
    global leftSensorValue, rightSensorValue, radiusSensorValue, startSensorValue
    while True:
        readSensors()
        difference = leftSensorValue - rightSensorValue
        print(leftSensorValue,  rightSensorValue, difference)
        time.sleep(0.5)

def motortest():
    # run motors forward and backwards and light the LEDs then stop motors
    leftMezzLED.on()         # Switch on the left mezzanine LED
    leftFwd.duty_u16(15000)  # Set left forward speed to 15000
    leftRev.duty_u16(0)      # and reverse speed to zero
    rightFwd.duty_u16(15000) # Set right forward speed to 15000
    rightRev.duty_u16(0)     # and reverse speed to zero
    time.sleep(5)            # Wait for 5 seconds
    leftMezzLED.off()        # Switch off the left mezzanine LED
    rightMezzLED.on()        # Switch on the right mezzanine LED
    leftRev.duty_u16(15000)  # Set left reverse speed to 15000
    leftFwd.duty_u16(0)      # and forward speed to zero
    rightRev.duty_u16(15000) # Set left reverse speed to 15000
    rightFwd.duty_u16(0)     # and forward speed to zero
    time.sleep(5)            # Wait for 5 seconds
    leftMezzLED.off()        # Switch off the left mezzanine LED
    rightMezzLED.off()       # Switch off the right mezzanine LED
    leftFwd.duty_u16(0)      # Set left forward speed to zero
    leftRev.duty_u16(0)      # and reverse speed to zero
    rightFwd.duty_u16(0)     # Set right forward speed to zero
    rightRev.duty_u16(0)     # and reverse speed to zero 


# code exeecuted when program starts
linefollow()
#photoshow()
#motortest()