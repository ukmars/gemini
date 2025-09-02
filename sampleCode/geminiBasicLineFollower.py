# * Project:    UKMARS Gemini
# * File:       GeminiLineFollower.py
# * Updated for V2 board configuration
#
# * Author:     Ian Butterworth
# * Created:    21 May 2025
# * Updated:	22 August 2025
# *
# * Description:
# *     Simple example line following code for the UKMARS Gemini platform with PiPico.
# *     This code does not require that encoders are available.
# *     Logging information is broadcast on UART0 for transmission over bluetooth connected to J2.
# *     It is not necessary to have the bluetooth module installed the robot will run but 
# *     without the bluetooth serial link, logging information will not be available.
# *
#  * -----
#  * MIT License
#  *
#  * Copyright (c) 2025 Ian Butterworth
#  *
#  * Permission is hereby granted, free of charge, to any person obtaining a copy of
#  * this software and associated documentation files (the "Software"), to deal in
#  * the Software without restriction, including without limitation the rights to
#  * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
#  * of the Software, and to permit persons to whom the Software is furnished to do
#  * so, subject to the following conditions:
#  *
#  * The above copyright notice and this permission notice shall be included in all
#  * copies or substantial portions of the Software.
#  *
#  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  * SOFTWARE.


from machine import Pin,ADC,PWM,UART
import time
import os

uart=UART(0, 115200)	#Select baudrate for bluetooth connection - choose the fastest reliable value
                        #This code outputs calibration data and running logging information over UART0
leftsensor  = ADC(28)
rightsensor = ADC(26)
radiussensor = ADC(27)
startsensor = ADC(27)
emitter = Pin(22,Pin.OUT)
radiusEmitter = Pin(18,Pin.OUT)

onBoardLED = Pin("LED", Pin.OUT)
leftSensorLED = Pin(21,Pin.OUT)
centreSensorLED = Pin(20,Pin.OUT)
rightSensorLED = Pin(19,Pin.OUT)
leftMezzLED = Pin(12,Pin.OUT)
rightMezzLED = Pin(13,Pin.OUT)

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


WAITING_CALIBRATION = 1 #Robot states
CALIBRATING = 2
WAITING_START_PB = 3
PRE_START_MARKER = 4
RUNNING = 5
STOPPING = 6
HALTED = 7

robotState = WAITING_CALIBRATION
loopCounter = 0 #Used to tag logged data and detect any lack of continuity in the logging
loopPeriod = 4  #Delay between control loop iterations in mS - control loop frequency will be lower due to the time taken to execute the code consider aligning with Pico clock
error = 0       #Delay should be as small as possible but give enough time for the logging data transmission over bluetooth
oldError = 0
change = 0
kp    = 1.5
kd = 1500       #differential constant defined as the multiplier of change in error per second
kdLoop = kd * loopPeriod / 1000 #The actual constant applied in the loop needs to scale for the duration of the loop
speed = 25      #between 0 and 100
pErrorTerm = 0
dErrorTerm = 0
fallOffError = 80	#Error to apply if line is lost on one side
fallOffLevel = 10   #Sensor reading(%) below which line is considered lost

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
    #Values are derived by subtracting the lit value of a sensor from the unlit value 
    #Unlit raw readings close to 65000 indicate good separation from ambient light
    #The radius and start/finish sensors are multiplexed onto the same ADC channel using separate emitters
    #The UKMARS line follower sensor board gives high value readings for low incident light and low value readings for high incident light
    
    global leftSensorValue, rightSensorValue, radiusSensorValue, startSensorValue
    global leftSensorUnlit, rightSensorUnlit, radiusSensorUnlit, startSensorUnlit

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

    leftSensorValue = (leftSensorUnlit - leftSensorLit)
    rightSensorValue = (rightSensorUnlit - rightSensorLit)
    startSensorValue = (startSensorUnlit - startSensorLit)
    radiusSensorValue = (radiusSensorUnlit - radiusSensorLit)
    
#Set the motor pwm duty cycle from required speed expressed as a percentage 0-100
def leftMotor(speed):
    if speed < 0:
        speed = -speed
        if speed > 100:
            speed = 100
        dutyCycle = speed * 655
        leftFwd.duty_u16(65535 - int(dutyCycle))
        leftRev.duty_u16(65535)
    else:
        if speed > 100:
            speed = 100
        dutyCycle = speed * 655
        leftRev.duty_u16(65535 - int(dutyCycle))
        leftFwd.duty_u16(65535)

def rightMotor(speed):
    if speed < 0:
        speed = -speed
        if speed > 100:
            speed = 100
        dutyCycle = speed * 655
        rightFwd.duty_u16(65535 - int(dutyCycle))
        rightRev.duty_u16(65535)
    else:
        if speed > 100:
            speed = 100
        dutyCycle = speed * 655
        rightRev.duty_u16(65535 - int(dutyCycle))
        rightFwd.duty_u16(65535)

def stopMotors():       
    rightRev.duty_u16(65535)
    rightFwd.duty_u16(65535)
    leftRev.duty_u16(65535)
    leftFwd.duty_u16(65535)

#--------------------------------------------
#calibration caps and collars
leftMax = 0
leftMin = 65535
rightMax = 0
rightMin = 65535
radiusMax = 0
radiusMin = 65535
startMax = 0
startMin = 65535

radiusLeadingEdge = 0
radiusTrailingEdge = 0
startLeadingEdge = 0
startTrailingEdge = 0

#update the maximum and minimum reflected values
def calibrateSensors():
    global leftMin,leftMax,rightMin,rightMax,radiusMin,radiusMax,startMin,startMax
    global leftSensorValue, rightSensorValue, radiusSensorValue, startSensorValue
    if leftSensorValue > leftMax: leftMax = leftSensorValue
    if leftSensorValue < leftMin: leftMin = leftSensorValue
    if rightSensorValue > rightMax: rightMax = rightSensorValue
    if rightSensorValue < rightMin: rightMin = rightSensorValue
    if radiusSensorValue > radiusMax: radiusMax = radiusSensorValue
    if radiusSensorValue < radiusMin: radiusMin = radiusSensorValue
    if startSensorValue > startMax: startMax = startSensorValue
    if startSensorValue < startMin: startMin = startSensorValue

#global flags for marker detection
sfLeadingEdgeDetected = False
sfTrailingEdgeDetected = False
sfTrigger = False
radiusLeadingEdgeDetected = False
radiusTrailingEdgeDetected = False
radiusTrigger = False
crossoverDetected = False

def initialiseMarkerFlags():
    global sfLeadingEdgeDetected,sfTrailingEdgeDetected,sfTrigger
    global radiusLeadingEdgeDetected,radiusTrailingEdgeDetected,radiusTrigger,crossoverDetected
    sfLeadingEdgeDetected = False
    sfTrailingEdgeDetected = False
    sfTrigger = False
    radiusLeadingEdgeDetected = False
    radiusTrailingEdgeDetected = False
    radiusTrigger = False
    crossoverDetected = False

def checkMarkers():
    global sfLeadingEdgeDetected,sfTrailingEdgeDetected,sfTrigger
    global radiusLeadingEdgeDetected,radiusTrailingEdgeDetected,radiusTrigger,crossoverDetected
    global radiusLeadingEdge,radiusTrailingEdge,startLeadingEdge,startTrailingEdge
    global radiusSensorValue, startSensorValue

    if (sfLeadingEdgeDetected):                                     # already detected leading edge of start/finish marker
        if(radiusLeadingEdgeDetected):
            crossoverDetected = True
        if (startSensorValue < startTrailingEdge):                  # trailing edge detected
            sfTrailingEdgeDetected = True
            if (crossoverDetected):                                 # ignore crossovers
                sfLeadingEdgeDetected = False
                sfTrailingEdgeDetected = False
                sfTrigger = False
                if (not radiusLeadingEdgeDetected):                 # if the radius detection has been cleared clear the crossover flag
                    crossoverDetected = False                       # otherwise leave it set to allow radius to ignore crossover and clear 
            else:
                sfTrigger = True                                    # trailing edge detected and no crossover 
                sfLeadingEdgeDetected = False
                sfTrailingEdgeDetected = False
        else:
            pass                                                    #Leading edge detected and sensor above trailing edge threshold so do nothing
    else:                                               # No SF leading edge previously detected so check for leading edge
        if (startSensorValue > startLeadingEdge):
            sfLeadingEdgeDetected = True
            if (radiusLeadingEdgeDetected):
                crossoverDetected = True;                           # radius leading edge also detected so must be a crossover
        else:                                           #No leading edge detected so do nothing
            pass

    if (radiusLeadingEdgeDetected):                                 # already detected leading edge of radius marker
        if(sfLeadingEdgeDetected):
            crossoverDetected = True
        if (radiusSensorValue < radiusTrailingEdge):                # trailing edge detected
            radiusTrailingEdgeDetected = True
            if (crossoverDetected):
                radiusLeadingEdgeDetected = False
                radiusTrailingEdgeDetected = False
                radiusTrigger = False
                if (not sfLeadingEdgeDetected):                     # if the start finish detection has been cleared clear the crossover flag
                    crossoverDetected = False                       # otherwise leave it set to allow start finish to ignore crossover and clear
            else:
                radiusTrigger = True                                # trailing edge detected and no crossover so register radius
                radiusLeadingEdgeDetected = False
                radiusTrailingEdgeDetected = False
        else:                                                       #Leading edge detected and sensor above trailing edge threshold so do nothing
            pass
    else:                                               #No Radius leading edge already detected so check for leading edge
        if (radiusSensorValue > radiusLeadingEdge):
            radiusLeadingEdgeDetected = True
            if (sfLeadingEdgeDetected):
                crossoverDetected = True;                           # s/f leading edge also detected so must be a crossover


def displayCalibrationHeader():
    uart.write('loopCounter, leftMin, leftMax, rightMin, rightMax, radiusMin, radiusMax, startMin, startMax')
    uart.write(chr(10))
    uart.write(chr(13))

def displayCalibration():
    global loopCounter,leftMin,leftMax,rightMin,rightMax,radiusMin,radiusMax,startMin,startMax
    uart.write(str(loopCounter))
    uart.write(',')
    uart.write(str(leftMin))
    uart.write(',')
    uart.write(str(leftMax))
    uart.write(',')
    uart.write(str(rightMin))
    uart.write(',')
    uart.write(str(rightMax))
    uart.write(',')
    uart.write(str(radiusMin))
    uart.write(',')
    uart.write(str(radiusMax))
    uart.write(',')
    uart.write(str(startMin))
    uart.write(',')
    uart.write(str(startMax))
    uart.write(chr(10))
    uart.write(chr(13))

def displayLogging():
    global loopCounter,lsensorCal,rsensorCal,error,change,pErrorTerm,dErrorTerm,leftSpeed,rightSpeed
    uart.write(str(loopCounter))
    uart.write(',')
    uart.write(str(round(lsensorCal,2)))
    uart.write(',')
    uart.write(str(round(rsensorCal,2)))            
    uart.write(',')
    uart.write(str(round(error,2)))
    uart.write(',')
    uart.write(str(round(change,2)))
    uart.write(',')
    uart.write(str(round(pErrorTerm,2)))
    uart.write(',')
    uart.write(str(round(dErrorTerm,2)))
    uart.write(',')
    uart.write(str(round(leftSpeed,2)))
    uart.write(',')
    uart.write(str(round(rightSpeed,2)))
    uart.write(chr(10))
    uart.write(chr(13))

def greeting():
    global speed, kp, kd, kdLoop
    uart.write('Gemini Basic Line Follower : Hello\n\r')
    uart.write('speed=')
    uart.write(str(speed))
    uart.write('  kp=')
    uart.write(str(kp))
    uart.write('  kd=')
    uart.write(str(kd))
    uart.write('  kdLoop=')
    uart.write(str(kdLoop))
    uart.write(chr(10))
    uart.write(chr(13))

def displayThresholds():
    global radiusLeadingEdge,radiusTrailingEdge,startLeadingEdge,startTrailingEdge
    uart.write('radiusLeading=')
    uart.write(str(radiusLeadingEdge))
    uart.write('  radiusTrailing=')
    uart.write(str(radiusTrailingEdge))
    uart.write('  startLeading=')
    uart.write(str(startLeadingEdge))
    uart.write('  startTrailing=')
    uart.write(str(startTrailingEdge))
    uart.write(chr(10))
    uart.write(chr(13))

#--------------------------------------------
#Main Program
robotCalibrated = False
while (True):
    robotRunning = False
    stopMotors()
    if not robotCalibrated:
        while (rightButton.value() == 1):	#Slow Blink LED until Right button pressed
        #This is a good time to establish a serial connection if using an HC05/06 bluetooth module
            onBoardLED.value(1)
            time.sleep(0.3)
            onBoardLED.value(0)
            time.sleep(0.3)

        rightMezzLED.value(1)   #signal button detected with LED
        robotState = CALIBRATING
        greeting()
        uart.write('Calibrating robot')
        uart.write(chr(10))
        uart.write(chr(13))
        displayCalibrationHeader()
        time.sleep_ms(250)
        rightMezzLED.value(0)

        startCount = loopCounter
        leftMotor(15)
        rightMotor(-15)
        while((loopCounter - startCount) < 200): 
            loopCounter +=1
            readSensors()
            calibrateSensors()
            displayCalibration()
            time.sleep_ms(5)
        stopMotors()
        radiusLeadingEdge = radiusMin + ((radiusMax-radiusMin)/2)   #Marker leading and trailing edge thresholds for marker detection
        radiusTrailingEdge = radiusMin + ((radiusMax-radiusMin)/3)  #with hysteresis
        startLeadingEdge = startMin + ((startMax-startMin)/2)
        startTrailingEdge = startMin + ((startMax-startMin)/3)
        displayThresholds()    
        time.sleep_ms(10)
        robotCalibrated = True

    #use left button to start
    robotState = WAITING_START_PB
    while (leftButton.value() == 1):	#Fast Blink LED until Left button pressed
        onBoardLED.value(1)
        time.sleep(0.1)
        onBoardLED.value(0)
        time.sleep(0.1)
    leftMezzLED.value(1)    #signal button detected with LED
    leftSensorLED.value(0)  #clear LEDs used to signal significant steering adjustment
    rightSensorLED.value(0)
    uart.write('Starting robot')
    uart.write(chr(10))
    uart.write(chr(13))
    robotRunning = True
    robotState = PRE_START_MARKER
    error = 0
    oldError = 0
    change = 0
    pErrorTerm = 0
    dErrorTerm = 0
    uart.write('loop, left, right, error, change, pTerm, dTerm, leftSpeed, rightSpeed')
    uart.write(chr(10))
    uart.write(chr(13))
    time.sleep_ms(250)
    leftMezzLED.value(0)
    markerToggle = False

    while robotRunning:
        readSensors()
        checkMarkers()
        if(sfTrigger):
            markerToggle = not markerToggle
            rightMezzLED.value(markerToggle)
            sfTrigger = False
            if(robotState == PRE_START_MARKER):
                robotState =  RUNNING
            else:
                robotState =  STOPPING
                startCount = loopCounter
        if(radiusTrigger):
            markerToggle = not markerToggle
            rightMezzLED.value(markerToggle)
            radiusTrigger = False

        if(robotState == STOPPING):
            if((loopCounter - startCount) < 10):    #run on past the finsih marker
                leftMezzLED.value(1)
            else:
                robotState =  HALTED
                leftMezzLED.value(0)
                robotRunning = False

        rsensorCal = (rightSensorValue - rightMin)*100 / (rightMax - rightMin)
        lsensorCal = (leftSensorValue - leftMin)*100 / (leftMax - leftMin)

        #detect loosing the line
        if lsensorCal > fallOffLevel or rsensorCal > fallOffLevel: #on the line
            loopCounter +=1
            if lsensorCal < fallOffLevel or rsensorCal < fallOffLevel:
                if oldError < 0:
                    error = -fallOffError
                else:
                    error = fallOffError
            else:
                error = lsensorCal - rsensorCal
            pErrorTerm = error * kp
            change = error - oldError
            oldError = error
            dErrorTerm= change * kdLoop
            leftSpeed = speed - pErrorTerm - dErrorTerm
            rightSpeed = speed + pErrorTerm + dErrorTerm
            displayLogging()
            leftMotor(leftSpeed)
            rightMotor(rightSpeed)
            if((leftSpeed - rightSpeed) > 10):
                leftSensorLED.value(1)
                rightSensorLED.value(0)
            else:
                if((rightSpeed - leftSpeed) > 10):
                    leftSensorLED.value(0)
                    rightSensorLED.value(1)
                else:
                    leftSensorLED.value(0)
                    rightSensorLED.value(0)

        else:#off the line
            robotRunning = False
        if(leftButton.value() == 0):    #left button stops the run and goes back to fast blinking
            robotRunning = False
        time.sleep_ms(loopPeriod)
    robotState = HALTED
    uart.write('Robot halted')
    uart.write(chr(10))
    uart.write(chr(13))
    stopMotors()            #stop immediately
    time.sleep(1)      #allow time to come off push button
