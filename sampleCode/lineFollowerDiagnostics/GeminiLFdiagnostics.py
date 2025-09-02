# Diagnostic code to verify the correct build of a UKMARS Gemini platform with line follower sensor using Pi Pico
# Updated for V2 board configuration
#
#Author:	Ian Butterworth
#Created:	29 April 2025
#Updated:	11 August 2025
#
#Connecting an HC05/HC06 bluetooth module to J2 on the mezzanine processor board will provide wireless reporting
#of the values measured to a serial monitor eg Putty or mobile phone
#
#The test phases are:
#
#0- flash the onboard LED							when done press the left button SW1
#1- flash the Gemini indicators and emitters		when done press the right button SW2
#2- read and display the sensor values				when done press the left button SW1
#3- read and display the encoder values				when done press the right button SW2
#4- move the robot forwards then backwards			automatically returns to test phase 0
#
#This programm requires diagnosticEncoders.py
#------------
#MIT License

#Copyright (c) 2025 Ian Butterworth

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.

from machine import Pin,ADC,PWM
import time
from diagnosticEncoders import Encoders
import os
from machine import UART
uart=UART(0, 115200)	#Select baudrate for bluetooth connection

leftsensor  = ADC(28)
rightsensor = ADC(26)
radiussensor = ADC(27)
startsensor = ADC(27)
emitter = Pin(22,Pin.OUT)
radiusEmitter = Pin(18,Pin.OUT)

onBoardLED = Pin("LED", Pin.OUT)
leftRedLED = Pin(21,Pin.OUT)		#Sensor
centreAmberLED = Pin(20,Pin.OUT)	#Sensor
rightGreenLED = Pin(19,Pin.OUT)	#Sensor
leftWhiteLED = Pin(12,Pin.OUT)	#Mezz
rightBlueLED = Pin(13,Pin.OUT)	#Mezz

leftRev = PWM(Pin(3))
leftRev.freq(2000)
leftFwd = PWM(Pin(2))
leftFwd.freq(2000)
rightRev = PWM(Pin(5))
rightRev.freq(2000)
rightFwd = PWM(Pin(4))
rightFwd.freq(2000)
leftButton = Pin(15, Pin.IN, Pin.PULL_UP)
rightButton = Pin(14, Pin.IN, Pin.PULL_UP)
encoders = Encoders()
#leftQuadA = Pin(9,Pin.IN)	#Not needed, pins are assigned in the encoders object
#leftQuadB = Pin(8,Pin.IN)
#rightQuadA = Pin(7,Pin.IN)
#rightQuadB = Pin(6,Pin.IN)

#Global variables to make readings available after the update function has run
leftSensorUnlit = 0
rightSensorUnlit = 0
radiusSensorUnlit = 0
startSensorUnlit =0
leftSensorValue = 0
rightSensorValue = 0
radiusSensorValue = 0
startSensorValue =0


def readSensors():
    #Values are derived by subtracting the lit value of a sensor from the unlit value and dividing by 512
    #to give a value in the range 0-127.
    #Unlit raw readings close to 65000 indicate good separation from ambient light
    #The variable detectedValue controls the illumination of inicator LEDs or each of the sensors so moving
    #the robot around over lines and markers will show detection
    #The radius and start/finish sensors are multiplexed onto the same ADC channel using separate emitters
    
    global leftSensorValue, rightSensorValue, radiusSensorValue, startSensorValue
    global leftSensorUnlit, rightSensorUnlit, radiusSensorUnlit, startSensorUnlit
    detectedValue = 64	#50% of saturated brightness
    emitter.value(0)
    radiusEmitter.value(0)
    leftSensorUnlit = leftsensor.read_u16()
    rightSensorUnlit = rightsensor.read_u16()
    startSensorUnlit = startsensor.read_u16()
    radiusSensorUnlit = radiussensor.read_u16()
    
    emitter.value(1)
    time.sleep_us(15)
    leftSensorLit = leftsensor.read_u16()
    rightSensorLit = rightsensor.read_u16()
    startSensorLit = startsensor.read_u16()
    emitter.value(0)
    
    radiusEmitter.value(1)
    time.sleep_us(15)
    radiusSensorLit = radiussensor.read_u16()
    radiusEmitter.value(0)
    
    leftSensorValue = (leftSensorUnlit - leftSensorLit) // 512	# limits range to 127
    rightSensorValue = (rightSensorUnlit - rightSensorLit) //512
    startSensorValue = (startSensorUnlit - startSensorLit) //512
    radiusSensorValue = (radiusSensorUnlit - radiusSensorLit) //512
    
    if leftSensorValue > detectedValue:
        leftRedLED.value(1)
    else:
        leftRedLED.value(0)
    if rightSensorValue > detectedValue:
        rightGreenLED.value(1)
    else:
        rightGreenLED.value(0)
    if startSensorValue > detectedValue:
        rightBlueLED.value(1)
    else:
        rightBlueLED.value(0)
    if radiusSensorValue > detectedValue:
        leftWhiteLED.value(1)
    else:
        leftWhiteLED.value(0)


#Set the motor pwm duty cycle from required speed expressed as a percentage 0-100
def leftMotor(speed):
    if speed < 0:
        speed = -speed
        if speed > 100:
            speed = 100
        dutyCycle = speed * 655
        leftFwd.duty_u16(0)
        leftRev.duty_u16(int(dutyCycle))
    else:
        if speed > 100:
            speed = 100
        dutyCycle = speed * 655
        leftRev.duty_u16(0)
        leftFwd.duty_u16(int(dutyCycle))

def rightMotor(speed):
    if speed < 0:
        speed = -speed
        if speed > 100:
            speed = 100
        dutyCycle = speed * 655
        rightFwd.duty_u16(0)
        rightRev.duty_u16(int(dutyCycle))
    else:
        if speed > 100:
            speed = 100
        dutyCycle = speed * 655
        rightRev.duty_u16(0)
        rightFwd.duty_u16(int(dutyCycle))

def stopMotors():       
    rightRev.duty_u16(0)
    rightFwd.duty_u16(0)
    leftRev.duty_u16(0)
    leftFwd.duty_u16(0)
    

def sequenceLEDs():
    #Cycle round flashing all the indicator LEDs and the emitters in turn
    #The emitters are pulsed as they are overdriven on a duty cycle
    period_ms = 200
    leftRedLED.value(1)
    time.sleep_ms(period_ms)
    leftRedLED.value(0)
    centreAmberLED.value(1)
    time.sleep_ms(period_ms)
    centreAmberLED.value(0)
    rightGreenLED.value(1)
    time.sleep_ms(period_ms)
    rightGreenLED.value(0)
    leftWhiteLED.value(1)
    time.sleep_ms(period_ms)
    leftWhiteLED.value(0)
    rightBlueLED.value(1)
    time.sleep_ms(period_ms)
    rightBlueLED.value(0)
    for pulses in range(0,40):
        emitter.value(1)
        time.sleep_us(15)
        emitter.value(0)
        time.sleep_ms(2)
    time.sleep_ms(period_ms)
    for pulses in range(0,40):
        radiusEmitter.value(1)
        time.sleep_us(15)
        radiusEmitter.value(0)
        time.sleep_ms(2)
    time.sleep_ms(period_ms)
    
def allOff():
    emitter.value(0)
    radiusEmitter.value(0)
    leftRedLED.value(0)
    centreAmberLED.value(0)
    rightGreenLED.value(0)
    leftWhiteLED.value(0)
    rightBlueLED.value(0)

#............................................
#
#Main Program starts here
#
stopMotors()
emitter.value(0)
radiusEmitter.value(0)
displayCount = 500
leftCount = 0
rightCount = 0
diagnosticLoopCount = 0

while True:
#Flash the on board LED until the left button is pressed
#This is a good time to establish a serial connection if using an HC05/06 bluetooth module
    while(leftButton.value() == 1):
        onBoardLED.value(1)
        time.sleep(0.1)
        onBoardLED.value(0)
        time.sleep(0.1)
    if diagnosticLoopCount == 0:
        uart.write('Hello\n\r')
        print('Hello')
    else:
        uart.write('Hello again\n\r')
        print('Hello again')
    diagnosticLoopCount += 1

#Now flash the Gemini LEDs and emitters until the right button is pressed
    while(rightButton.value() == 1):
        sequenceLEDs()
    allOff()
    
#Now read and display the sensors until the left button is pressed
    while(leftButton.value() == 1):
        readSensors()
        displayCount += 1
        if displayCount > 500:
            print(radiusSensorUnlit,leftSensorUnlit,rightSensorUnlit,startSensorUnlit,radiusSensorValue,leftSensorValue,rightSensorValue,startSensorValue)
            uart.write(str(radiusSensorUnlit))
            uart.write('    ')
            uart.write(str(leftSensorUnlit))
            uart.write('    ')
            uart.write(str(rightSensorUnlit))
            uart.write('    ')
            uart.write(str(startSensorUnlit))
            uart.write('    ')
            uart.write(str(radiusSensorValue))
            uart.write('    ')
            uart.write(str(leftSensorValue))
            uart.write('    ')
            uart.write(str(rightSensorValue))
            uart.write('    ')
            uart.write(str(startSensorValue))
            uart.write(chr(10))
            uart.write(chr(13))
            displayCount = 0
        time.sleep_ms(2)
    allOff()
    
#Now read and display the encoders until the right button is pressed
    displayCount = 500
    leftCount, rightCount = encoders.get_counts(True)
    while(rightButton.value() == 1):
        leftCount, rightCount = encoders.get_counts(False)
        displayCount += 1
        if displayCount > 500:
            print(leftCount,rightCount)
            uart.write(str(leftCount))
            uart.write('    ')
            uart.write(str(rightCount))
            uart.write(chr(10))
            uart.write(chr(13))
            displayCount = 0
        time.sleep_ms(2)


#Now move the robot forwards then backwards
#If the robot is only being powered by the USB connector lift it off the surface to allow the motors to work!
    rightMotor(35)
    leftMotor(35)
    time.sleep_ms(500)
    stopMotors()
    time.sleep_ms(500)
    rightMotor(-35)
    leftMotor(-35)
    time.sleep_ms(500)
    stopMotors()
    time.sleep_ms(500)
