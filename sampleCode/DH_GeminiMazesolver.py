# code for Gemini maze solver  v7 - 27 Sept 2026
# File saved on PC as geminimazesolver
# version with full size maze and centre at cell 135 
# Running micro-python on raspberry pico on standard Gemini UKMARSbot
# In Thonny on windows PC  - From Run, Select Interpreter then MicroPython raspberry Pi Pico
# 50:1 gear motors with magnetic encoders from pi hut 18cm = about 4900 pulses on sum of 4 encoders
#
# Use the buttons to select the program then switch on the power
# It uses 2 LEDs on gemini board plus the PCB red LED to show selected program
# If plugged into the PC it will show the currently selected switch value
# The program number is displayed as 3 bits of binary
# Select program on 4 way switch in binary  LEDS
# Press PCB button to run the selected program
# - prog 0 = maze solver in 16 x 16 maze    Red & Green
# - prog 1 = run set of moves test          Red
# - prog 2 = phototest,                     Green
# - prog 3 = mazesolver in 6 x 3 maze       None
# - prog 4 = calibration - run this first in cell with wheels on cell boundary with walls on side and ahead
#                                           Red & Green & Pico LED
# - prog 5 = motortest and encoder values,
#            18cm = 2145, 1 cm = 119 pulses Red & Pico LED
# - prog 6 = flash LEDs forever test        Green & Pico LED
# = prog 7 = mazesolver for 6X3 home maze   Pico LED
#            Stops at the end of each move
# - prog 8 - wallfollower with debug on     Pico LED
# - prog 9 - switch test display            Pico LED
# - prog10 - encoder test                   Pico LED
# - prog11 = route check & run tests        Pico LED
# - prog 12 - run saved route
#
# Robot pauses for 2 seconds at the end of each move if in debug mode
# Changed flood to stop when reaching the mouse position, not when it gets to the start
# Added test for optimal route when returned to start cell
# v2 put encoder adjustment to keep straight in goahead when no walls seen
# v3 stop one wheel on left and right turn
# v4 amend motor drive to use maxspeed and 65536 - desired speed for better braking control
# v5 Don't add walls if we have been in this cell before
# v6 still not fixed fast run back from centre
# to fix :
# Back up after turn around at centre cell if wall present to back into
# Run back from centre on fast run not correct.
#       Going too far after turnaround in centre on fast run and not stopping back at start cell
# Better checking of position in cell when doing a long straight run of cells
#    - check alignment from posts or when going past any walls not present
#    - check position when wall seen ahead
# When the actual maze route is [1, 1, 3, 3, 2, 2, 1, 1]
#      the maze fast route is calculated by routecheck to be  [1, 1, 3, 3, 1, 2, 1, 1, 0]
#      It gets the ist left turn(2) wrong and goes ahead (1) instead into a wall
#  *****************************************************

from machine import Pin, ADC, PWM, UART
import time
import os
import sys

uart=UART(0, 115200)
#import ubluetooth # for Bluetooth BLE use
# Global constants

WIDTH = 16 # is 16 in full size maze
HEIGHT = 16 # is 16 in full size maze
TABLEWIDTH = 16
TABLEHEIGHT = 16
START = 0 # the start cell number
MIDDLE = 135 # int((TABLEWIDTH * HEIGHT /2) + (WIDTH / 2)) # middle of full size maze
NORTH = 1
EAST = 2
SOUTH = 4
WEST = 8
VISITED = 16
EXPLORE = 1
RUNMAZE = 3
AHEAD = 1
LEFT = 2
RIGHT = 3
AROUND = 4
PRESENT = 1            # item is known to be present
NOTPRESENT = 0         # item is known to be not present
UNKNOWN = 2            # state of item is unknown
OUT = 0                # going OUT from start to centre
BACK = 1               # coming BACK to start from centre
YES = 1
NO = 0
# 275 total pulses = approx 1cm
cm18 = 4950 # one cell 18cm
cm13 = 3265 # 13.6cm total encoder count - back wall to end of 1st cell
cm9 = 2475 # 9cm - half a cell - total encoder count
cm10 = 2750 # 10cm
cm6p5 = 1787 # 6.5cm total encoder count
cm2 = 5500
cm4 = 1100 # was 1100
cm5 = 1375
cm3p25 = 893 # 3.25cm total encoder count
cm3 = 825 # 3cm total encoder count
deg90R = 1700 # 90 degree right turn
deg90L = 1600 # 90 degree left turn
deg180 = 3250 # 180 degree turn
debug = 0
wait = 0
frontwallseen = 500    # value to be above for LF if front wall seen from cell boundary
currentcell = 0
w2sec = 0 # flag to wait 2 seconds after each move
turnedleft = 0
mode = 1 # mazesolver mode by default
adjustment = 0
maxspeed = 65535 # maximum motor PWM speed for reverse entry
# define size of maze cells - extra row added at top and one to right for processing at edge of maze

numcells = (TABLEWIDTH * (TABLEHEIGHT + 1)) + 10

# set up and initialise arrays for walls and flood values in maze
walls = [0]*numcells # array of cell items initialised to zero starting at cell 0
#walls[0] = 99   example of setting an indexed value 
maze = [0]*numcells # array that holds flood values for maze cells
# cells are numbers from left to right then upwards from start cell as zero
# to check bits in a byte use walls[n] & NORTH to check the north wall bit, walls[n] & EAST for next bit etc
proclist = [0]*numcells # array that holds the list of cells to be processed next by the flood routine
celllist = [0]*numcells # array that holds list of cells as we progress
celllistno = 0
moves = [0]*numcells # array that holds the list of moves made at each time we get to a cell boundary
movesno = 0 # pointer to move next being made
fmoves = [0]*numcells # array that holds the list of moves for the fast run
fmovesno = 0
heads = [0]*numcells # array that holds the list of headings for the run
headsno = 0
# pin definitions
# Set pins for digital outputs on ESP32
# pin definitions. Sets pins for digital outputs on ESP32-S3
onBoardLED = Pin("LED", Pin.OUT)
leftSensorLED = Pin(20,Pin.OUT)
centreSensorLED = Pin(19,Pin.OUT)
rightSensorLED = Pin(18,Pin.OUT)
leftMezzLED = Pin(12,Pin.OUT)
rightMezzLED = Pin(13,Pin.OUT)

# Define analogue inputs used on line sensors - chose one of the next 2 sets of 4 defines
# For KES Python through hole sensor board use these 4 
Frontsense = ADC(27)   #  sees front wall
Lsidesense = ADC(28)   #  Sees left wallr
Rsidesense = ADC(26)   #  Sees right wall

leftRev = PWM(Pin(3))
leftRev.freq(2000)
leftFwd = PWM(Pin(2))
leftFwd.freq(2000)
rightRev = PWM(Pin(4))
rightRev.freq(2000)
rightFwd = PWM(Pin(5))
rightFwd.freq(2000)
leftButton = Pin(15, Pin.IN, Pin.PULL_UP)
rightButton = Pin(14, Pin.IN, Pin.PULL_UP)

emitter1 = Pin(22, Pin.OUT) # Trigger for LEDs on sensor board changed to GPIO number
emitter2 = Pin(21, Pin.OUT) # Trigger for LEDs on sensor board changed to GPIO number


Rightenc1 = Pin(6, Pin.IN) # right encoder
Rightenc2 = Pin(7, Pin.IN) # right encoder
Leftenc1 = Pin(8, Pin.IN) # left encoder
Leftenc2 = Pin(9, Pin.IN) # left encoder

# set both motors to stopped using Sign-Magnitude drive with inverted inputs, or a "slow decay" braking mode. 
leftRev.duty_u16(maxspeed)
leftFwd.duty_u16(maxspeed)
rightRev.duty_u16(maxspeed)
rightFwd.duty_u16(maxspeed)


def backup():
    global basespeed, currdist, mode, turnedleft, maxspeed, cm4
    rightFwd.duty_u16(maxspeed - 15000) # set motors to reverse
    rightRev.duty_u16(maxspeed)
    leftFwd.duty_u16(maxspeed - 15000)
    leftRev.duty_u16(maxspeed)
    time.sleep (1)
    rightRev.duty_u16(maxspeed) # set motors to stopped
    rightFwd.duty_u16(maxspeed)
    leftRev.duty_u16(maxspeed)
    leftFwd.duty_u16(maxspeed)
    currdist = 0
    goahead(cm4,basespeed) # otherwise go to centre of the cell
    return


def buttonwait():  # wait for the tactile button on the PCB board to be pressed
    global swvalue, btnvalue
    print ("waiting for button press")
    readswitches()
    setting = 0
    while (setting == 0): # Check button 
        readswitches()
        setting = btnvalue
    return

def calibrate(): # get mouse aligned in centre of start cell and save sensor initial values
    global leftside, rightside, front, lsinit, rsinit, frontinit
    global ls, ff, rs, frontwallseen
    lsinit = rsinit = frontinit = 0
    time.sleep(.1)
    photoread()
    readswitches()
    while(btnvalue == 0):  # until rear button is pressed
        photoread()       # read photo sensors
        #print((leftside),(front),(rightside))
        if (ls == rs): # if central put both LEDs on
            leftMezzLED.value(1) # switch on sensor1 LED on sensor board
            rightMezzLED.value(1) # switch on sensor2 LED on sensor board
        if (ls > rs):  # put LEDs on to indicate if left or right of centre
            leftMezzLED.value(0) # switch on sensor1 LED on sensor board
            rightMezzLED.value(1) # switch on sensor2 LED on sensor board
        if (ls < rs):
            leftMezzLED.value(1) # switch on sensor1 LED on sensor board
            rightMezzLED.value(0) # switch on sensor2 LED on sensor board
        readswitches()  # read switch, save values when rear button pressed
    lsinit = ls     # save calibrated values of the 4 sensors
    rsinit = rs
    frontinit = ff
    leftside = ls
    rightside = rs
    front = ff
    writecalibration() # write calibration values to calibration files
    if (debug == 1):
        print ("calibrated", lsinit, frontinit, rsinit)
    if (debug == 1):
        print ("calibrated", ls, ff, rs)

def checklrspeed():
    global lspeed, rspeed
    if lspeed > 60000:
        lspeed = 60000
    if rspeed > 60000:
        rspeed = 60000
    if lspeed < 10:
        lspeed = 10
    if rspeed < 10:
        rspeed = 10
    #if (debug == 1):
        #print ("checkspeed", leftspeed, rightspeed)
    return

def checkspeed():
    global leftspeed, rightspeed
    if leftspeed > 60000:
        leftspeed = 60000
    if rightspeed > 60000:
        rightspeed = 60000
    if leftspeed < 10:
        leftspeed = 10
    if rightspeed < 10:
        rightspeed = 10
    #if (debug == 1):
        #print ("checkspeed", leftspeed, rightspeed)
    return

def checkwalls():
    # put LEDs on if walls seen
    global debug, ls, lsinit,rs, rsinit, ff, finit, leftwall, rightwall, frontwall
    if (ls > (lsinit / 6)):
        leftMezzLED.value(True)        
        leftwall = PRESENT
    else:
        leftMezzLED.value(False)
        leftwall = NOTPRESENT
    if (rs > (rsinit / 6)):
        rightMezzLED.value(True)
        rightwall = PRESENT
    else:
        rightMezzLED.value(False)
        rightwall = NOTPRESENT
        
    #if ((ff  > (5 * (frontinit)):
    #if  (ff > frontwallseen): # see front wall with both sensors
    if  (ff > finit):    
        onBoardLED.value(True)
        frontwall = PRESENT
    else:
        onBoardLED.value(False)
        frontwall = NOTPRESENT
    #if (debug == 1):
        #print ("checkwalls ", leftwall, rightwall, frontwall)

def encoderreset():
    global l1count, l2count, r1count, r2count 
    l1count = 0
    l2count = 0
    r1count = 0
    r2count = 0
    
def encodertest():     # 18 cm movement should give just over 212 counts on l1 and r1 and 106 on l2 and r2
    global l1count, l2count, r1count, r2count, maxspeed     # making 636 total to move 1 cell forward (318 on each wheel)
    # going backwards the counts still increase
    basespeed = 15000
    leftRev.duty_u16(maxspeed - basespeed)
    leftFwd.duty_u16(maxspeed) 
    rightRev.duty_u16(maxspeed - basespeed)
    rightFwd.duty_u16(maxspeed)
    encoderreset()     # set encoder counts back to zero
    while True:
        print((l1count), (l2count), (r1count), (r2count))
        time.sleep(.5)
    return


def flashLEDs(n):   #
    i = 0
    while (i < n):
        leftMezzLED.value(True) # Green LED
        rightMezzLED.value(True)
        if (n > 90):
            onBoardLED.value(True) # set onboard PCB LEd if flashing forever
        time.sleep(0.5)
        leftMezzLED.value(False)
        rightMezzLED.value(False)
        onBoardLED.value(False)
        time.sleep(0.5)
        i = i + 1
        if (i >90):
            i = 90 # this keeps it in teh loop forever  if n is > 90



def floodclear(): # clear the flood table
    global numcells    
    for x in range(256):
        maze[x] = numcells
@micropython.native # run this routine in native mode (lower level code)
def floodmaze(strt,fin):   # flood the maze from the strt cell to the fin cell
    global maze, walls, floodfail, debug, numcells, currentcell
    floodstart = time.ticks_ms() # get time now
    floodclear()           # clear the flood table to all 283
    floodcleared = time.ticks_ms() # get time now
    floodcleared = floodcleared - floodstart
    flooded = 0            # set flag to not finished flooding yet
    floodfail = 0          # flag to show if flood failed to complete to end point
    curr = strt            # current cell being processed
    floodval = 0
    maze[strt] = 1         # set start cell flood value to one
    n = 0                  # index for processing list array of cells to say where to add to end of list
    nxt = 0                # pointer to the first unprocessed item on the list
    while (flooded == 0):
        fval = maze[curr]  # get current value of current cell
        if ((walls[curr] & SOUTH) == 0):     # is there a gap to the SOUTH of current cell
            if (maze[curr - TABLEWIDTH] == numcells):
                maze[curr - TABLEWIDTH] = fval + 1    # set flood value in this cell
                proclist[n] = curr-TABLEWIDTH         # save flood cell for future processing
                n = n + 1                        # update processing list number
                if (proclist[n-1] == currentcell):       # check if finished flooding
                    flooded = 1                  # set flag to stop loop
        if ((walls[curr] & EAST) == 0):      # is there a gap to the EAST of current cell
            if (maze[curr + 1] == numcells):
                maze[curr + 1] = fval + 1        # set flood value in this cell
                proclist[n] = curr + 1           # save flood cell for future processing
                n = n + 1                        # update processing list number
                if (proclist[n-1] == currentcell):           # check if finished flooding
                    flooded = 1                      # set flag to stop loop
        if ((walls[curr] & NORTH) == 0):     # is there a gap to the NORTH of current cell
            if (maze[curr + TABLEWIDTH] == numcells):
                maze[curr + TABLEWIDTH] = fval + 1    # set flood value in this cell
                proclist[n] = curr + TABLEWIDTH       # save flood cell for future processing
                n = n + 1                        # update processing list number
                if (proclist[n-1] == currentcell):           # check if finished flooding
                       flooded = 1                      # set flag to stop loop
        if ((walls[curr] & WEST) == 0):      # is there a gap to the WEST of current cell
            if (maze[curr - 1] == numcells):
                maze[curr - 1] = fval + 1        # set flood value in this cell
                proclist[n] = curr - 1           # save flood cell for future processing
                n = n + 1                        # update processing list number
                if (proclist[n-1] == currentcell):       # check if finished flooding
                    flooded = 1                  # set flag to stop loop
        #print (proclist[n-1] , fin)
        #print (strt, fin, nxt, n, proclist)
        
        curr = proclist[nxt]                 # get the location of the next cell to process
        nxt = nxt + 1                        # point to next item to process on the list
        
        if (nxt > n):                        # check if flood unable to continue as no more cells accessible
            floodfail = 1                     # set flood failure status flag
            flooded = 1 # stop  the flooding loop
            if (debug == 1):
                print (strt, fin, nxt, n, proclist)
    return                                    # return 
    
def goahead(dist,endspeed): # dist is in encoder pulses i.e. around 4950 total of 4 counts for 180mm
    global l1count, l2count, r1count, r2count, currdist, leftwall, rightwall, frontwall, lsinit, rsinit, ls , rs
    global leftspeed, rightspeed, basespeed, wait, moves, movesno, w2sec, numcells, adjustment, maxspeed
    global frontwallseen
    #if (movesno < numcells):
        #moves[movesno] = 1 # Log a go ahead in the moves array
        #movesno = movesno + 1
    encoderreset()
    leftRev.duty_u16(maxspeed - basespeed)
    leftFwd.duty_u16(maxspeed) # ensure that we start with both motors going at basespeed
    rightRev.duty_u16(maxspeed - basespeed)
    rightFwd.duty_u16(maxspeed)
    godist = dist - currdist # distance to go less already done
    startcount = l1count + l2count + r1count + r2count # vaue of counts when we call this procedure
    gonesum = 0
    gone = 0
    pfactor = 1
    nowspeed = basespeed
    prevdiff = 0
    #numloops = 0
    #time1 = time.ticks_us()
    while (gone < godist):
        #numloops =  numloops + 1
        gone = l1count + l2count + r1count + r2count
        photoread()
        showLEDwalls()
        lsidediff = (ls - lsinit) * 3
        rsidediff = (rs - rsinit) * 3
        dterm = (lsidediff - prevdiff) * 3
        if (lsidediff > 40000):
            lsidediff = 40000
        if (lsidediff < -40000):
            lsidediff = - -40000
        #print(dist, gonesum, l1count, l2count, r1count, r2count, startdist, startcount)
        #  we can track walls if present in this cell for the first half of cell only
        #if ((gonesum + currdist) < 1800):
        if (ff < frontwallseen): # if front wall not present use side walls to adjust posn
                # if wall to left and no wall ahead keep away from left or right walls
            if(ls > lsinit):  # too close to left wall
                adjustment = lsidediff
            if((ls < lsinit) and (rs > rsinit)):  # not too close to left wall but too close to right wall
                adjustment = -rsidediff
        if ((ls < lsinit) and (rs < rsinit)):
            adjustment = 20*((r1count + r2count) - (l1count + l2count)) # use encoder differences to keep straight
            #adjustment = 0    # previous code to keep straight on        
        if (adjustment >= 0):
            leftspeed = int(basespeed)
            rightspeed = int(basespeed - adjustment)
        if (adjustment < 0):
            leftspeed = int(basespeed + adjustment)
            rightspeed = int(basespeed)
        checkspeed()
        leftRev.duty_u16(maxspeed - leftspeed)
        leftFwd.duty_u16(maxspeed) 
        rightRev.duty_u16(maxspeed - rightspeed)
        rightFwd.duty_u16(maxspeed)
        prevdiff = lsidediff
    #time2 = time.ticks_us()
    #time0 = (time2 - time1) / numloops
    #print (" time", time0, "numloops", numloops)
    # check here if wall was seen but is no longer, reset encoder counts to 1310
    # if left wall present: but wall no longer seen
    #     if (ls < (lsinit/3))
    
    if (wait == 1):        
        pause() # stop and wait for 2 seconds then restart at basespeed
    # rest of stub - to be written later to adjust to required end speed 
    return

def goonecellahead(): # go forward one cell
    global heading, currentcell,ff,frontwallseen, basespeed, currdist, w2sec, cm9
    global movesno, numcells, moves
    #if (movesno < numcells):
        #moves[movesno] = 1 # Log a go ahead in the moves array
        #movesno = movesno + 1
    currdist = 0
    if (debug == 1):
        print ("go ahead 1 cell in cell no", currentcell)
    goahead(cm9, basespeed) # move 1/2 cell forward
    if (ff > frontwallseen): # seeing front wall not spotted at cell boundary 
            # now record the wall in the lines below
            if (heading == NORTH):
                walls[currentcell] = walls[currentcell] | NORTH         # record front wall
                walls[currentcell + TABLEWIDTH] = walls[currentcell + TABLEWIDTH] | SOUTH  
            if (heading == SOUTH):
                walls[currentcell] = walls[currentcell] | SOUTH         # record front wall
                if(currentcell > TABLEWIDTH):
                    walls[currentcell - TABLEWIDTH] = walls[currentcell - TABLEWIDTH] | NORTH           
            if (heading == EAST):
                walls[currentcell] = walls[currentcell] | EAST         # record front wall
                walls[currentcell + 1] = walls[currentcell + 1] | WEST  
            if (heading == WEST):
                walls[currentcell] = walls[currentcell] | WEST         # record front wall
                walls[currentcell - 1] = walls[currentcell - 1] | EAST  
            turnaround(0)    # turn around immediately as already in centre of cell
            goahead(cm9, basespeed) # move 1/2 cell back to start of cell
            
    else:
        frontwall = NOTPRESENT
        goahead(cm9, basespeed)   #move 1/2 cell forward 
        if (heading == NORTH):
            currentcell = currentcell + TABLEWIDTH # update cell pointer as we move into next cell
        if (heading == SOUTH):
            currentcell = currentcell - TABLEWIDTH # update cell pointer as we move into next cell
        if (heading == EAST):
            currentcell = currentcell + 1 # update cell pointer as we move into next cell
        if (heading == WEST):
            currentcell = currentcell - 1 # update cell pointer as we move into next cell
        # heading stays the same
    return


def halt():      # halt and flash LEDs continuously
    global maxspeed
    leftRev.duty_u16(maxspeed)
    leftFwd.duty_u16(maxspeed) 
    rightRev.duty_u16(maxspeed)
    rightFwd.duty_u16(maxspeed)
    print ("halt called")
    flashLEDs(99) # flash LEDs forever

def LEDtest():
    while True:
        leftSensorLED.value(True)
        print ("leftLED")
        time.sleep(5)
        leftSensorLED.value(False)
        centreSensorLED.value(True)
        print ("centreLED")
        time.sleep(5)
        centreSensorLED.value(False)
        rightSensorLED.value(True)
        print ("rightLED")
        time.sleep(5)
        rightSensorLED.value(False)
        onBoardLED.value(True)
        print ("onboardLED")
        time.sleep(5)
        onBoardLED.value(False)
        leftMezzLED.value(True)
        print ("leftMezzLED")
        time.sleep(5)
        leftMezzLED.value(False)
        rightMezzLED.value(True)
        print ("rightMezzLED")
        time.sleep(5)
        rightMezzLED.value(False)

def leftcount1(): # left encoder count 2 routines
    global l1count
    l1count = l1count+1
    return
def leftcount2():
    global l2count
    l2count = l2count+1
    return

def motortest():      #
    global l1count, l2count, r1count, r2count, maxspeed
    # test going ahead a fixed distance
    basespeed = 14000
    leftRev.duty_u16(maxspeed - basespeed)
    leftFwd.duty_u16(maxspeed) 
    rightRev.duty_u16(maxspeed - basespeed)
    rightFwd.duty_u16(maxspeed)
    encsum = 0
    l1count = l2count = r1count = r2count = 0
    while (encsum < 9800): # 2 cells - 36cm
        encsum = l1count + l2count + r1count + r2count
        print((l1count), (l2count), (r1count), (r2count))
        #time.sleep(.5)
    halt()
    # alternative motor test
    basespeed = 14000    
    while (True):
        leftMezzLED.value(True)
        leftRev.duty_u16(maxspeed - basespeed)
        leftFwd.duty_u16(maxspeed) 
        rightRev.duty_u16(maxspeed - basespeed)
        rightFwd.duty_u16(maxspeed)
        basespeed = basespeed + 500
        time.sleep(1)
        leftMezzLED.value(False)
        time.sleep(1)
    
    
def nowtime():
    global curr_msec
    curr_msec = round(time.time()*1000)
    
def pause(): # wait a few milliseconds at critical times
    global leftspeed, rightspeed, maxspeed
    stop()
    time.sleep_ms(2)
    leftRev.duty_u16(maxspeed - leftspeed)
    leftFwd.duty_u16(maxspeed) # reset to moving forward at basespeed
    rightRev.duty_u16(maxspeed - rightspeed)
    rightFwd.duty_u16(maxspeed) # reset to moving forward at basespeed

def pause1(): # wait a second or two at the end of each move
    global leftspeed, rightspeed, maxspeed
    stop()
    time.sleep(1)
    leftRev.duty_u16(maxspeed - leftspeed)
    leftFwd.duty_u16(maxspeed) # reset to moving forward at basespeed
    rightRev.duty_u16(maxspeed - rightspeed)
    rightFwd.duty_u16(maxspeed) # reset to moving forward at basespeed

def photoread():
    #Values are derived by subtracting the lit value of a sensor from the unlit value 
    #Unlit raw readings close to 65000 indicate good separation from ambient light
    #The radius and start/finish sensors are multiplexed onto the same ADC channel using separate emitters
    #The UKMARS line follower sensor board gives high value readings for low incident light and low value readings for high incident light
    
    global leftSensorValue, rightSensorValue, frontSensorValue
    global leftSensorUnlit, rightSensorUnlit, frontSensorUnlit
    global leftSensorLit, rightSensorLit, frontSensorLit
    global ls, ff, rs 
    emitter1.value(0)
    emitter2.value(0)
    
    leftSensorUnlit = Lsidesense.read_u16()
    rightSensorUnlit = Rsidesense.read_u16()   
    emitter1.value(1)
    time.sleep_us(15)
    leftSensorLit = Lsidesense.read_u16()
    rightSensorLit = Rsidesense.read_u16()
    emitter1.value(0)
    frontSensorUnlit = Frontsense.read_u16() 
    emitter2.value(1)
    time.sleep_us(15)
    frontSensorLit = Frontsense.read_u16()    
    emitter2.value(0)
    ls = (leftSensorLit - leftSensorUnlit)
    rs = (rightSensorLit - rightSensorUnlit)
    ff = (frontSensorLit - frontSensorUnlit)
    if (ls < 0):
        ls = 0
    if (rs < 0):
        rs = 0    
    if (ff < 0):
        ff = 0
    emitter1.value(0)    
    emitter2.value(0) 

def phototest(): # Show walls seen on LEDs. Left wall yellow, Right wall green, front wall red
    global leftSensorValue, rightSensorValue, frontSensorValue
    global leftSensorUnlit, rightSensorUnlit, frontSensorUnlit
    global leftSensorLit, rightSensorLit, frontSensorLit
    global leftside, front, rightside, ls, ff, rs, frontwallseen
    global lsinit, rsinit, frontinit, finit
    while True:
        lsinit = rsinit = frontinit = 0
        readcalibration()    # read wall calibration values in
        frontwallseen = int(finit * 0.5)    # value to be above if front wall seen from cell boundary
        photoread()
        if (ls > (lsinit / 3)):
           leftSensorLED.value(True)
        else:
           leftSensorLED.value(False)
        if (rs > (rsinit / 3)):
            rightSensorLED.value(True)
        else:
            rightSensorLED.value(False)
        if  (ff > frontwallseen): # see front wall with both sensors
            centreSensorLED.value(True) # red LED on main PCB
        else:
            centreSensorLED.value(False)
        print ("calibration file values read in", lsinit, finit, rsinit) 
        print("  ls",ls,"ff",ff,"rs",rs)
        print("unlit", leftSensorUnlit, frontSensorUnlit, rightSensorUnlit)
        print("  lit", leftSensorLit, frontSensorLit, rightSensorLit)
        time.sleep(.5)
    return



def readcalibration(): # returns the calibration values
    global finit, lsinit, rsinit
    f1 = open("calibration1.txt", "r")
    front = f1.read()
    finit = int(front)
    f2 = open("calibration2.txt", "r")
    leftside = f2.read()
    lsinit = int(leftside)
    f3 = open("calibration3.txt", "r")
    rightside = f3.read()
    rsinit = int(rightside)

def readswitches(): # Gives btnvalue and swvalue set to 1 if the L or R button pressed
    global swvalue, btnvalue, switchvoltage
    swvalue = leftButton.value()
    btnvalue = rightButton.value()
    
def rightcount1():
    global r1count
    r1count = r1count+1
    return
def rightcount2():
    global r2count
    r2count = r2count+1
    return

def routecheck(): # checks that we have visited all of the cells on the flood route to the centre
    # if we have visited all the cells we should have the optimal route and "visits" will be True
    global heading, currentcell, visits, wait, move, fmoves, fmovesno, fnummoves
    currentcell = 0
    fmovesno = 0 # point to first item in fast moves array
    heading = NORTH
    visits = True # flag to say that all cells seen so far have had a visit
    while (currentcell != MIDDLE):
        if ((walls[currentcell] & VISITED) == False): # if not been in this cell set the visited flag to false
            visits = False
        wherenext()
        hdg = heading
        fmoves[fmovesno] = move # store the move in the fast moves array
        fmovesno = fmovesno + 1
        if (move == AHEAD):
            if (hdg == NORTH):
                currentcell = currentcell + TABLEWIDTH
            if (hdg == SOUTH):
                currentcell = currentcell - TABLEWIDTH
            if (hdg == EAST):
                currentcell = currentcell + 1
            if (hdg == WEST):
                currentcell = currentcell - 1
            heading = hdg
        if (move == RIGHT):
            if (hdg == NORTH):
                currentcell = currentcell + 1
                heading = EAST
            if (hdg == SOUTH):
                currentcell = currentcell - 1
                heading = WEST
            if (hdg == EAST):
                currentcell = currentcell - TABLEWIDTH
                heading = SOUTH
            if (hdg == WEST):
                currentcell = currentcell + TABLEWIDTH
                heading = NORTH
        if (move == LEFT):
            if (hdg == NORTH):
                currentcell = currentcell - 1
                heading = WEST
            if (hdg == SOUTH):
                currentcell = currentcell + 1
                heading = EAST
            if (hdg == EAST):
                currentcell = currentcell + TABLEWIDTH
                heading = NORTH
            if (hdg == WEST):
                currentcell = currentcell - TABLEWIDTH
                heading = SOUTH
    fnummoves =  fmovesno # save number of moves in the run
    print ("fnummoves", fnummoves)
    savefastmoves() # save the calculated fast moves list
    if (debug == 1):
        print(fmoves)
    return

def routerun(): # run the list of moves for a fast run to the centre from the edge of the first cell
    global fmoves, fmovesno, fnummoves, basespeed, currdist, cm6p5, cm18, cm9
    print ("start fast run")
    #print ("nummoves", nummoves, "moves", moves)
    #stop()
    goahead(cm6p5, 0) # go from middle to edge of start cell 
    currdist = 0
    runmoves = fmoves  #copy moves list to runmoves so not overwritten during run
    runmovesno = 1
    encoderreset()
    while (runmovesno <= fnummoves-1):
        if (runmoves[runmovesno] == AHEAD):
            goahead(cm18,basespeed) # move 1 cell distance forward
        if (runmoves[runmovesno] == RIGHT):
            turnright()
        if (runmoves[runmovesno] == LEFT):
            turnleft(0)
        if (runmoves[runmovesno] == AROUND):
            turnaround(0)            
        runmovesno = runmovesno + 1 # point at next move
        #pause1()
    goahead(cm9, 0) # go to centre of goal cell 
    return

def routetest():
    global fmoves, movesno, fnummoves, basespeed, leftspeed, rightspeed
    readcalibration() # just to set up init values
    basespeed = 18000
    leftspeed = basespeed
    rightspeed = basespeed
    fnummoves = 4 # set to one more than number of moves to make
    # start the bot on a cell boundary
    # It will stop half a cell after the requested moves
    fmoves[1] = 1 # 1 = ahead left = 2 right = 3
    fmoves[2] = 2 # 
    fmoves[3] = 1 # ahead
    fmoves[4] = 1 # ahead
    fmoves[5] = 1 # ahead
    time.sleep(1)
    #print ("moves set", fmoves)
    routerun() # runs from fmoves[1]
    print("route run")
    halt()

def runback(): # run the list of moves back to the start
    global heading, currentcell, currdist, visits, wait, move, moves, movesno, fnummoves, basespeed, cm2, cm9, cm18
    print ("start runback. Number of fast moves =", fnummoves)
    currdist = 0
    fmovesno = 0
    # Point to the actual last index of the array (nummoves - 1)
    fmovesnumber = fnummoves - 1 
    basespeed = 18000 # runback speed = 18000
    while (fmovesno < (fnummoves - 1)): # 
        #print ("fastmove",fmoves[fmovesnumber], "fnumbe", fmovesnumber) 
        if (fmoves[fmovesnumber] == AHEAD):
            #print("goahead CM18")
            goahead(cm18,basespeed) # move 1 cell distance forward
        elif (fmoves[fmovesnumber] == RIGHT): # move turns reversed on way back
            #print("turnleft")
            turnleft(0)
        elif (fmoves[fmovesnumber] == LEFT): # move turns reversed on way back
            #print("turnright")
            turnright()
        elif (fmoves[fmovesnumber] == AROUND): # ignore the turn around at the end of the moves list
            fmovesno = fmovesno
            #print ("round")
        fmovesno = fmovesno + 1  # increase number of moves made
        fmovesnumber = fmovesnumber - 1 # point at next move
        #print ("Fmoves number", fmovesnumber)
    goahead(cm9,basespeed) # go to centre of start cell
    turnaround(0) # turn around in start cell, backup and go to centre
    return

def runsaved():
    global btnvalue, swvalue, walls, fmoves, fnummoves, basespeed
    global lfinit, rfinit, lsinit, rsinit, cm4, cm13, currdist
    #read in walls and fast route then run it at speed set on switches
    flashLEDs(6) # confirm we are running prog 12 by 6 flashes
    readswitches()
    while (btnvalue == 0): # wait for button press after switches set and read
        readswitches()
    # read in saved walls
    readcalibration() # get init values
    walldata = []
    with open('savedwalls.txt') as f:
        text = f.read()
    items = text.split(',')
    for item in items:
        item = item.strip()
        # Skip empty or non‑numeric items
        if item.isdigit():
            walldata.append(int(item))
    # Ensure exactly 256 items
    walldata = walldata[:256]
    walls = walldata
    #print (walls)
    # read in saved route
    nummoves = 0
    routedata = []
    with open('savedmoves.txt') as route:
        text = route.read()
    items = text.split(',')
    for item in items:
        item = item.strip() # Skip empty or non‑numeric items
        if item.isdigit():
            routedata.append(int(item))
            nummoves = nummoves + 1
    # Ensure exactly 256 items
    routedata = routedata[:256]
    fmoves = routedata
    print ("nummoves", nummoves)
    fnummoves = nummoves #  number of moves
    #print (routedata)
    #print (fmoves)
    basespeed = 18000 # ***************** modify with number of left buttons pressed
    flashLEDs(1)
    time.sleep(1)
    currdist = 0
    goahead(cm4,basespeed) # go to get axles at 1st cell boundary. finish at basespeed
    routerun()
    stop()
    flashLEDs(3)
    turnaround(0)
    #goahead(cm2,basespeed) # go ahead to get axles at 1st cell boundary. finish at basespeed
    runback()
    halt()

def savemaze(): # saves maze, walls and moves data when called
    global movecount, numcells, currentcell, heading, celllist, celllistno, heads, headsno
    n = 0
    filedata = "savedmaze.txt"
    fmdata = open(filedata, "w") # open file
    while (n < numcells):
        number = str(maze[n])
        fmdata.write(number + ", ")
        n = n + 1
    fmdata.close()
    n = 0
    filedata = "savedwalls.txt"
    fwdata = open(filedata, "w") # open file
    while (n < numcells):
        number = str(walls[n])
        fwdata.write(number + ", ")
        n = n + 1
    fwdata.close()
    n = 0
    filedata = "savedmoves.txt"
    fdata = open(filedata, "w") # open file
    while (n < (movecount + 1)):
        number = str(moves[n])
        fdata.write(number + ", ")
        n = n + 1
    fdata.close()
    
    n = 0
    filedata = "savedheadings.txt"
    fdata = open(filedata, "w") # open file
    while (n < (headsno + 1)):
        number = str(heads[n])
        fdata.write(number + ", ")
        n = n + 1
    fdata.close()

    n = 0
    filedata = "savedcellnos.txt" # save list of cells visited
    fldata = open(filedata, "w") # open file
    while (n < (celllistno + 1)):
        number = str(celllist[n])
        fldata.write(number + ", ")
        n = n + 1
    fldata.close()

    # halt()
def savefastmoves(): # saves fast moves data when called
    global fnummoves, fnumcells
    n = 0
    filedata = "savedfastmoves.txt"
    fdata = open(filedata, "w") # open file
    while (n < (fnummoves + 1)):
        number = str(fmoves[n])
        fdata.write(number + ", ")
        n = n + 1
    fdata.close()
    # halt()

def selectprogram(): # select program to run using press buttons
    # left button increses program to run. Right button runs that program
    global progno, swvalue, btnvalue, wait, msize, debug, MIDDLE, HEIGHT, WIDTH, w2sec, maxspeed 
    # set both motors to stopped
    leftRev.duty_u16(maxspeed)
    leftFwd.duty_u16(maxspeed)
    rightRev.duty_u16(maxspeed)
    rightFwd.duty_u16(maxspeed)
    readswitches()
    print ("swiches", swvalue, btnvalue)
    progno = 0
    while (btnvalue == True): # wait for PCB board button to be pressed
        time.sleep(0.1) # 
        readswitches()
        if (swvalue == False):
            progno = progno + 1
        if progno == 0: # 
            leftMezzLED.value (True) #green & red LED
            rightMezzLED.value (True)
            onBoardLED.value (False) # PCB LED off
        if progno == 1: # 3rd level indent this line
            leftMezzLED.value (True) # green LED only
            rightMezzLED.value (False)
            onBoardLED.value (False) # PCB LED off
        if progno == 2: # 3rd level indent this line
            leftMezzLED.value (False) # red LED only
            rightMezzLED.value (True)
            onBoardLED.value (False) # PCB LED off
        if progno == 3: # 3rd level indent this line
            leftMezzLED.value (False) # no LEDs
            rightMezzLED.value (False)
            onBoardLED.value (False) # PCB LED off
        if progno == 4: # 3rd level indent this line
            leftMezzLED.value (True)# both LEDs
            rightMezzLED.value (True)
            onBoardLED.value (True) # PCB LED on
        if progno == 5: # 3rd level indent this line
            leftMezzLED.value (True) # green LED
            rightMezzLED.value (False)
            onBoardLED.value (True) # PCB LED on
        if progno == 6: # 3rd level indent this line
            leftMezzLED.value (False) # Yellow LED
            rightMezzLED.value (True)
            onBoardLED.value (True) # PCB LED on
        if progno == 7: # 3rd level indent this line
            leftMezzLED.value (False) # no LED
            rightMezzLED.value (False)
            onBoardLED.value (True) # PCB LED on
        if progno == 8: # 3rd level indent this line
            leftMezzLED.value (False) # no LED
            rightMezzLED.value (False)
            onBoardLED.value (False) # PCB LED on
        if progno == 9: # 3rd level indent this line
            leftMezzLED.value (True) # green LED only
            rightMezzLED.value (False)
            onBoardLED.value (False) # PCB LED off
        if progno == 10: # 3rd level indent this line
            leftMezzLED.value (False) # red LED only
            rightMezzLED.value (True)
            onBoardLED.value (False) # PCB LED off
        if progno == 11: # 3rd level indent this line
            leftMezzLED.value (False) # no LEDs
            rightMezzLED.value (False)
            onBoardLED.value (False) # PCB LED off
        if progno == 12: # 3rd level indent this line
            leftMezzLED.value (True)# both LEDs
            rightMezzLED.value (True)
            onBoardLED.value (True) # PCB LED on
        print ("progno", progno)
        time.sleep(0.2)
    # This code runs the selected program with any variables being set
    if (progno == 0): # both LEDs
        print ("mazesolve") # l
        mazesolve()
    if (progno == 1): # Yellow LED
        print ("routetest")
        #debug = 1
        routetest()             
    if (progno == 2):# Red LED
        print ("Phototest")
        phototest()
    if (progno == 3): # No LEDs
        print ("mazesolve home 6x3 maze")
        msize = 18     # Home maze size
        WIDTH = 3
        HEIGHT = 6
        MIDDLE = 66 #  top right 5th cell of home maze
        wait = 0
        w2sec = 0 # wait after end of every move
        mazesolve()
    if (progno == 4): # Both LEDs + PCB one
        print ("wall calibration")
        debug = 1
        calibrate()
        halt()
    if (progno == 5): # Yellow LED plus PCB one
        print ("motor test") # go ahead 5 cells then stop
        motortest()
    if (progno == 6): # Red LED plus PCB one
        print ("Flash LEDs")
        flashLEDs(99) # flash LEDs forever
    if (progno == 7): # no LEDs but PCB one
        print ("mazesolver home 6x3 maze")
        wait = 1 # pauses 2 msec at various critical points
        debug = 0
        w2sec = 0 # pauses 2 sec at various critical point
        msize = 18     # Home maze size
        WIDTH = 3
        HEIGHT = 6
        MIDDLE = 17 #   66 = top right 5th cell of home maze
        mazesolve()
    if (progno == 8): # no LEDS
        print ("Wall follow with debug") # l
        debug = 1
        wallfollow()
    if (progno == 9): # No LEDs        
        print ("switchtests")
        switchtest()
    if (progno == 10): # No LEDs
        print ("encoder test")
        encodertest()
    if (progno == 11): # No LEDs
        print ("runcheck test")
        debug = 1
        floodmaze(MIDDLE,START)
        routecheck()
    if (progno == 12): # No LEDs
        print ("run saved route")
        debug = 0
        runsaved()

def setoutsidewalls():
    for x in range(WIDTH):    # does range 0 to 15 when WIDTH = 16
        y = TABLEWIDTH * (HEIGHT - 1) + x
        walls[y] = walls[y] | NORTH    # set top (NORTH) walls
        walls[x] = walls[x] | SOUTH    # set bottom (SOUTH) walls
    for x in range(HEIGHT):
        y = (x * TABLEHEIGHT) + WIDTH - 1        
        walls[y] = walls[y] | EAST     # set right (EAST) walls      
        y = x * TABLEWIDTH
        walls[y] = walls[y] | WEST     # set left (WEST) walls
    # set wall to east of start cell    
    walls[0] = walls[0] | EAST
    walls[1] = walls[1] | WEST
    walls[0] = walls[0] | VISITED
    #showwalls()    print (walls)
    
def setwalls():  # sets left, right and front walls seen when at start of cell boundary
    global ls, ff, rs, heading, lsinit, rsinit, finit, currentcell
    global frontwallseen, leftwall, rightwall, frontwall
    leftwall = rightwall = frontwall = NOTPRESENT
    if (ls > (lsinit / 3)):
        leftwall = PRESENT
    if (rs > (rsinit / 3)):
        rightwall = PRESENT
    if (ff > frontwallseen):
        frontwall = PRESENT
    if (walls[currentcell] >= VISITED):# dont set wall if been in cell before
        return
    if (heading == NORTH):
        if (leftwall == PRESENT):
            walls[currentcell] = walls[currentcell] | WEST         # record left wall
            if (currentcell > 0):
                walls[currentcell - 1] = walls[currentcell - 1] | EAST  # record right wall in cell to left of current cell
        if (rightwall == PRESENT):
            walls[currentcell] = walls[currentcell] | EAST         # record right wall
            walls[currentcell + 1] = walls[currentcell + 1] | WEST # record left wall in cell to right of current cell
        if (frontwall == PRESENT):
            walls[currentcell] = walls[currentcell] | NORTH         # record front wall
            walls[currentcell + TABLEWIDTH] = walls[currentcell + TABLEWIDTH] | SOUTH  
    if (heading == SOUTH):
        if (leftwall == PRESENT):
            #print (currentcell)
            walls[currentcell] = walls[currentcell] | EAST         # record left wall
            walls[currentcell + 1] = walls[currentcell + 1] | WEST  # record right wall in cell to left of current cell
        if (rightwall == PRESENT):
            walls[currentcell] = walls[currentcell] | WEST         # record right wall
            walls[currentcell - 1] = walls[currentcell - 1] | EAST # record left wall in cell to right of current cell
        if (frontwall == PRESENT):
            walls[currentcell] = walls[currentcell] | SOUTH         # record front wall
            if(currentcell > TABLEWIDTH):
                walls[currentcell - TABLEWIDTH] = walls[currentcell - TABLEWIDTH] | NORTH           
    if (heading == EAST):
        if (leftwall == PRESENT):
            #print (currentcell)
            walls[currentcell] = walls[currentcell] | NORTH         # record left wall
            walls[currentcell + TABLEWIDTH] = walls[currentcell + TABLEWIDTH] | SOUTH  # record right wall in cell to left of current cell
        if (rightwall == PRESENT):
            walls[currentcell] = walls[currentcell] | SOUTH         # record right wall
            if(currentcell >= TABLEWIDTH):
                walls[currentcell - TABLEWIDTH] = walls[currentcell - TABLEWIDTH] | NORTH # record left wall in cell to right of current cell          
        if (frontwall == PRESENT):
            walls[currentcell] = walls[currentcell] | EAST         # record front wall
            walls[currentcell + 1] = walls[currentcell + 1] | WEST  
    if (heading == WEST):
        if (leftwall == PRESENT): 
            #print (currentcell)
            walls[currentcell] = walls[currentcell] | SOUTH         # record left wall
            if (currentcell >= 0):
                walls[currentcell - TABLEWIDTH] = walls[currentcell - TABLEWIDTH] | NORTH  # record right wall in cell to left of current cell
        if (rightwall == PRESENT):
            walls[currentcell] = walls[currentcell] | NORTH         # record right wall
            walls[currentcell + TABLEWIDTH] = walls[currentcell + TABLEWIDTH] | SOUTH # record left wall in cell to right of current cell           
        if (frontwall == PRESENT):
            walls[currentcell] = walls[currentcell] | WEST         # record front wall
            walls[currentcell - 1] = walls[currentcell - 1] | EAST  
    walls[currentcell] = walls[currentcell] | VISITED         # mark cell as visited after putting in the walls seen
 
def showflood():
    x = ((HEIGHT-1) * TABLEWIDTH)
    y = HEIGHT - 1
    if (debug == 1):
        print("Flood table")
        while (x >= 0):
            if (y < 10):
                space = "  "
            else:
                space = " "            
            print (y,space, maze[x],maze[x+1],maze[x+2],maze[x+3],maze[x+4],maze[x+5],maze[x+6],maze[x+7],maze[x+8],maze[x+9],maze[x+10],maze[x+11],maze[x+12],maze[x+13],maze[x+14],maze[x+15])
            x = x - TABLEWIDTH
            y = y - 1
            
def showLEDwalls():
    global ls,rs,ff,lsinit,rsinit,frontwallseen
    # put LEDs on if walls seen
    if (ls > (lsinit / 3)):
        leftSensorLED.value(True) # green LED on sensor board
    else:
        leftSensorLED.value(False)
    if (rs > (rsinit / 3)):
        rightSensorLED.value(True) # red LED on sensor board
    else:
        rightSensorLED.value(False)    
    if  ((ff > frontwallseen) ): # see front wall with both sensors
        centreSensorLED(True) # red LED on main PCB
    else:
        centreSensorLED(False)

def showwalls():
    x = ((HEIGHT-1) * TABLEWIDTH) 
    y = HEIGHT - 1
    if (debug == 1):
        print("Maze walls")
        while (x >= 0):
            if (y < 10):
                space = "  "
            else:
                space = " "
            print (y,space, walls[x],walls[x+1],walls[x+2],walls[x+3],walls[x+4],walls[x+5],walls[x+6],walls[x+7],walls[x+8],walls[x+9],walls[x+10],walls[x+11],walls[x+12],walls[x+13],walls[x+14],walls[x+15])
            x = x - TABLEWIDTH
            y = y - 1


def stop():
    global maxspeed
    leftRev.duty_u16(maxspeed)
    leftFwd.duty_u16(maxspeed) # set to stop
    rightRev.duty_u16(maxspeed)
    rightFwd.duty_u16(maxspeed) # set to stop
    
def switchtest(): # reads 4 way switch and single button values
    global swvalue, btnvalue, switchvoltage
    i = 1
    while True:
        readswitches() # returns vaue 0 to 12 or 16 for button
        print (i, "switchvoltage", switchvoltage, "swvalue", swvalue, "btnvalue", btnvalue)
        i = i + 1
        time.sleep(0.5)
        
def turnaround(fwdfirst):
    global heading, currentcell, debug, moves, movesno, numcells, maxspeed
    global l1count, l2count, r1count, r2count, currdist, leftwall, rightwall, frontwall, lsinit, rsinit
    global leftspeed, rightspeed, basespeed, wait, lspeed, rspeed, mode, deg180, cm4, cm13, backupok, ls , rs
    global NORTH, EAST, SOUTH, WEST
    #if (movesno < numcells):
       # moves[movesno] = 4 # Log a turn round in the moves array
       # movesno = movesno + 1
    if (debug == 1):
        print ("turnaround in cell no", currentcell)
    if (fwdfirst > 0):
        goahead (fwdfirst, basespeed) # go forward to requested amount while following walls
    # stop motors, wait a fraction of a second
    leftspeed = 0
    rightspeed = 0
    leftRev.duty_u16(maxspeed - leftspeed)
    leftFwd.duty_u16(maxspeed) # 
    rightRev.duty_u16(maxspeed - rightspeed)
    rightFwd.duty_u16(maxspeed)
    photoread()
    time.sleep_ms(100)
    #print ("spin round 180 degrees")
    if (ff > frontwallseen): # checking for wall ahead before turn round
        backupok = YES
    else:
        backupok = NO
    if (currentcell == MIDDLE):
        backupok = NO
    # spin round on the spot 180 degrees
    encoderreset()       # reset encoder counts to zero
    encsum = l1count + l2count + r1count + r2count
    leftspeed = basespeed
    rightspeed = basespeed
    leftRev.duty_u16(maxspeed - leftspeed)
    leftFwd.duty_u16(maxspeed) # 
    rightRev.duty_u16(maxspeed) # set the right motor direction to reverse
    rightFwd.duty_u16(maxspeed - rightspeed)
    gonesum = l1count + l2count + r1count + r2count  # number of encoder counts at start of turn
    dist = deg180 # 180 degree turn
    # make sure right motor is turning at exactly the same amount as the left one for distance required 
    turnfactor = 5 # amount to increase speed of motors if not turning at proper speeds
    while (gonesum < dist):
        photoread()
        leftencs = l1count + l2count  # sum of left encoder counts 
        rightencs = r1count + r2count # sum of right encoder counts
        turnerr = (leftencs - rightencs) * turnfactor
        lspeed = leftspeed + turnerr
        rspeed = rightspeed - turnerr
        checklrspeed()
        leftRev.duty_u16(maxspeed - lspeed)
        leftFwd.duty_u16(maxspeed) # 
        rightRev.duty_u16(maxspeed) # set the right motor direction to reverse
        rightFwd.duty_u16(maxspeed - rspeed)
        gonesum = l1count + l2count + r1count + r2count  # number of encoder counts since start of turn
        time.sleep(0.001)
    leftspeed = 0
    rightspeed = 0
    leftRev.duty_u16(maxspeed - leftspeed)
    leftFwd.duty_u16(maxspeed) # 
    rightRev.duty_u16(maxspeed - rightspeed)
    rightFwd.duty_u16(maxspeed) # # set the right motor direction back to forward
    if(wait == 1): # pause if in wait mode
        pause()
        #print("backup", backupok)
    if(backupok == YES): # is there a wall to back up to?
        backup()   # back up to rear wall and then go to middle of cell
    head = heading # get current heading
    if (head == NORTH):
        heading = SOUTH
    if (head == EAST):
        heading = WEST
    if (head == WEST):
        heading = EAST
    if (head == SOUTH):
        heading = NORTH
    photoread()   # check presence of walls on either side after turn round
    if (ls > (lsinit / 3)):
        leftwall = PRESENT
    else:
        leftwall = NOTPRESENT        
    if (rs > (rsinit / 3)):
        rightwall = PRESENT
    else:
        rightwall = NOTPRESENT
    if(w2sec == 1): # pause if in wait mode
        pause1()
    leftRev.duty_u16(maxspeed - leftspeed)
    leftFwd.duty_u16(maxspeed) # 
    rightRev.duty_u16(maxspeed - rightspeed) # set the direction back to fwd
    rightFwd.duty_u16(maxspeed)
    # go forward to cell boundary checking not too close to either side wall and adjust if needed
    currdist = 0
    if(backupok == YES):
        goahead(cm4, basespeed) # go to cell boundary
    #pause() # temp test
    if(w2sec == 1): # pause if in wait mode
        pause1()
    
    if (heading == NORTH):
        currentcell = currentcell + TABLEWIDTH # update cell pointer as we move into next cell
    if (heading == SOUTH):
        currentcell = currentcell - TABLEWIDTH # update cell pointer as we move into next cell
    if (heading == EAST):
        currentcell = currentcell + 1 # update cell pointer as we move into next cell
    if (heading == WEST):
        currentcell = currentcell - 1 # update cell pointer as we move into next cell  
    return

def turnleft(dis): # Go ahead dis encoder pulses then turn left
    global heading, basespeed, currentcell, wait, progno, debug, mode, numcells
    global l1count, l2count, r1count, r2count, currdist, leftwall, rightwall, frontwall, lsinit, rsinit, ls , rs
    global leftspeed, rightspeed, basespeed, debug, lspeed, rspeed, moves, movesno, turnedleft, cm3, cm4, cm5, deg90L
    #if (movesno < numcells):
        #moves[movesno] = 2 # Log a left turn in the moves array
        #movesno = movesno + 1
    encoderreset()   
    leftMezzLED.value(False) # switch off left wall seen LED
    #if (debug == 1):
        #print ("turnleft in cell no", currentcell)
    gonesum = 0
    currdist = 0 # go straight ahead before making the turn
    goahead (cm5, basespeed) # 4cm forward
    # set up wheel speeds for the turn
    leftspeed = 0
    rightspeed = basespeed
    #checkspeed()
    leftRev.duty_u16(maxspeed)
    leftFwd.duty_u16(maxspeed) 
    rightRev.duty_u16(maxspeed - rightspeed)
    rightFwd.duty_u16(maxspeed)
    gonesum = 0
    dist = deg90L #turn 90 degrees left
    encoderreset() # reset encoder counts 
    while (gonesum < dist):
        photoread()
        if (ls > lsinit * 2): # seeing wall on left
            gonesum = dist # stop loop
        rightencs = r1count + r2count # sum of right encoder counts
        gonesum = rightencs  # number of right encoder counts since start of turn
    encoderreset() # reset encoder counts 
    #time.sleep(0.001)
    goahead (cm5, basespeed)
    turnedleft = 1
    # update cell number 
    if (heading == NORTH):
        currentcell = currentcell - 1 # update cell pointer as we move into next cell
    if (heading == SOUTH):
        currentcell = currentcell + 1 # update cell pointer as we move into next cell
    if (heading == EAST):
        currentcell = currentcell + TABLEWIDTH # update cell pointer as we move into next cell
    if (heading == WEST):
        currentcell = currentcell - TABLEWIDTH # update cell pointer as we move into next cell
     # update heading
    headin = heading
    if (headin == NORTH):
        heading = WEST
    if (headin == EAST):
        heading = NORTH
    if (headin == WEST):
        heading = SOUTH
    if (headin == SOUTH):
        heading = EAST
    if (wait == 1):
        pause()   
    return
    
def turnright():
    global heading, basespeed, currentcell, wait, progno, debug, moves, movesno, mode, numcells, maxspeed
    global l1count, l2count, r1count, r2count, currdist, leftwall, rightwall, frontwall, lsinit, rsinit, ls , rs
    global leftspeed, rightspeed, basespeed, debug, lspeed, rspeed, cm3, cm4, cm5, deg90R
    #if (movesno < numcells):
        #moves[movesno] = 3 # Log a right turn in the moves array
        #movesno = movesno + 1
    startcount = l1count + l2count # vaue of left encoder counts when we call this procedure
    encoderreset()
    if (debug == 1):
        print ("turnright in cell no", currentcell)
        leftMezzLED.value(False)
        rightMezzLED.value(True)
        onBoardLED.value(False)
        # halt()
    currdist = 0
    dist = deg90R
    goahead (cm5, basespeed)    
    leftspeed = basespeed
    leftRev.duty_u16(maxspeed - leftspeed)
    leftFwd.duty_u16(maxspeed) 
    rightRev.duty_u16(maxspeed)
    rightFwd.duty_u16(maxspeed)
    gonesum = 0
    encoderreset() # reset encoder counts 
    while (gonesum < dist):
        photoread()
        leftencs = l1count + l2count # sum of left encoder counts
        rightencs = r1count + r2count # sum of right encoder counts 
        gonesum = leftencs  # number of left encoder counts since start of turn
        #time.sleep(0.001)
    leftspeed = (basespeed)
    rightspeed = (basespeed)
    leftRev.duty_u16(maxspeed - leftspeed)
    leftFwd.duty_u16(maxspeed) # reset to moving forward at basespeed
    rightRev.duty_u16(maxspeed - rightspeed)
    rightFwd.duty_u16(maxspeed) # reset to moving forward at basespeed
    goahead (cm4, basespeed)    
    #  change current cell number
    if (heading == NORTH):
        currentcell = currentcell + 1 # update cell pointer as we move into next cell
    if (heading == SOUTH):
        currentcell = currentcell - 1 # update cell pointer as we move into next cell
    if (heading == EAST):
        currentcell = currentcell - TABLEWIDTH # update cell pointer as we move into next cell
    if (heading == WEST):
        currentcell = currentcell + TABLEWIDTH # update cell pointer as we move into next cell
    #  change heading
    headin = heading
    if (headin == NORTH):
        heading = EAST
    if (headin == EAST):
        heading = SOUTH
    if (headin == WEST):
        heading = NORTH
    if (headin == SOUTH):
        heading = WEST
    if (wait == 1):    
        pause()
    return

def wallahead(): # Wall ahead seen in wallfollower mode
    global ff, leftwall, rightwall, debug
    global finit, lsinit, rsinit, currdist
    if (debug == 1):
        print("wallahead rtn started", finit)
        pause()
    onBoardLED.value(True)  # switch on wall ahead seen   
    # check in here if no wall to right
    # if so, we are in a dead end and need to spin 180 degrees
    turned = False
    while (ff > finit/2): # i.e. front wall seen
        photoread()
        checkwalls()
        if (debug == 1):
            pause()
            print(ff, finit)
        if(leftwall == NOTPRESENT):
             turned =  True
             currdist = 0
             #goahead(250,0)
             turnleft(0)
             ff = finit # code to get out of the loop
        if(rightwall == NOTPRESENT):
            turned =  True
            currdist = 0
            #goahead(250,0)
            turnright()
            rf = rfinit # code to get out of the loop 
        if (turned == False):
            currdist = 0
            goahead(350,0) # go closer to front wall before turn round
            turnaround(0)
        onBoardLED.value(False)  # switch off wall ahead seen
    return


def walltest():
    global basespeed, currdist, lsinit, rsinit, ls, rs
    calibrate()
    photoread()
    
    currdist = 0
    basespeed = 5000
    goahead (3816, 5000)
    halt()
    

def wherenext():      # decide whether to go ahead or turn
    # Logic
    # get flood number for current cell we are moving into
    # If no wall to right get flood number of wall to right
    # if lower than lowest so far set into lowest flood and save move needed to go to it
    # If no wall to the left get flood number for wall to the left
    # if left cell flood number is less than current and less than right cell number store cell number as next
    # If no wall ahead check flood number for cell ahead and if lower than others set that as where to go
    # if there were walls all round action is turn round and set previous cell as new current cell no
    # Note that we may be moving in any direction so right or left or ahead cells may be in any direction depending on the heading
    #
    global heading, currentcell, maze, walls, move, TABLEWIDTH, NORTH, EAST, SOUTH, WEST
    global AHEAD, RIGHT, LEFT, AROUND
    currflood = maze[currentcell]   # get flood number for cell we are moving in to
    lowestflood = currflood         # field to hold best flood number of adjacent cells
    
    if (heading == NORTH):
        nextcell = currentcell + TABLEWIDTH              # next cell is one above
        if (walls[currentcell] & NORTH == 0):            # check if no wall ahead
            if (maze[nextcell] < lowestflood):           # see if lower flood in that cell
                lowestflood = maze[nextcell]             # if so, save lowest flood no
                move = AHEAD                             # save move we need to make
        nextcell = currentcell + 1                       # next cell is one to right
        if (walls[currentcell] & EAST == 0):
            if (maze[nextcell] < lowestflood):
                lowestflood = maze[nextcell]
                move = RIGHT
        nextcell = currentcell - 1                       # next cell is one to left
        if (walls[currentcell] & WEST == 0):
            if (maze[nextcell] < lowestflood):
                lowestflood = maze[nextcell]
                move = LEFT
        nextcell = currentcell - TABLEWIDTH                   # next cell is behind us 
        if (walls[currentcell] & SOUTH == 0):
            if (maze[nextcell] < lowestflood):
                lowestflood = maze[nextcell]
                move = AROUND               
    if (heading == SOUTH):
        nextcell = currentcell - TABLEWIDTH                   # next cell is one below us
        if (walls[currentcell] & SOUTH == 0):
            if (maze[nextcell] < lowestflood):
                lowestflood = maze[nextcell]
                move = AHEAD
        nextcell = currentcell + 1                       # next cell is one to the left
        if (walls[currentcell] & EAST == 0):
            if (maze[nextcell] < lowestflood):
                lowestflood = maze[nextcell]
                move = LEFT
        nextcell = currentcell - 1                       # next cell is one to the right
        if (walls[currentcell] & WEST == 0):
            if (maze[nextcell] < lowestflood):
                lowestflood = maze[nextcell]
                move = RIGHT
        nextcell = currentcell + TABLEWIDTH                   # next cell is behind us
        if (walls[currentcell] & NORTH == 0):
            if (maze[nextcell] < lowestflood):
                lowestflood = maze[nextcell]
                move = AROUND      
    if (heading == EAST):
        nextcell = currentcell + 1                        # next cell is ahead
        if (walls[currentcell] & EAST == 0):
            if (maze[nextcell] < lowestflood):
                lowestflood = maze[nextcell]
                move = AHEAD
        nextcell = currentcell + TABLEWIDTH                   # next cell is one to the left
        if (walls[currentcell] & NORTH == 0):
            if (maze[nextcell] < lowestflood):
                lowestflood = maze[nextcell]
                move = LEFT
        nextcell = currentcell - TABLEWIDTH                    # next cell is one to the right
        if (walls[currentcell] & SOUTH == 0):
            if (maze[nextcell] < lowestflood):
                lowestflood = maze[nextcell]
                move = RIGHT
        nextcell = currentcell - 1                         # next cell is one behind us
        if (walls[currentcell] & WEST == 0):
            if (maze[nextcell] < lowestflood):
                lowestflood = maze[nextcell]
                move = AROUND  

    if (heading == WEST):
        nextcell = currentcell - 1                        # next cell is ahead
        if (walls[currentcell] & WEST == 0):
            if (maze[nextcell] < lowestflood):
                lowestflood = maze[nextcell]
                move = AHEAD
        nextcell = currentcell - TABLEWIDTH                   # next cell is one to the left
        if (walls[currentcell] & SOUTH == 0):
            if (maze[nextcell] < lowestflood):
                lowestflood = maze[nextcell]
                move = LEFT
        nextcell = currentcell + TABLEWIDTH                    # next cell is one to the right
        if (walls[currentcell] & NORTH == 0):
            if (maze[nextcell] < lowestflood):
                lowestflood = maze[nextcell]
                move = RIGHT
        nextcell = currentcell + 1                         # next cell is one behind us
        if (walls[currentcell] & EAST == 0):
            if (maze[nextcell] < lowestflood):
                lowestflood = maze[nextcell]
                move = AROUND  
    return

def writecalibration():
    global front, leftside, rightside
    f1 = open("calibration1.txt","w")
    f1.write(str(front))
    f1.close()    
    f2 = open("calibration2.txt","w")
    f2.write(str(leftside))
    f2.close()
    f3 = open("calibration3.txt","w")
    f3.write(str(rightside))
    f3.close()

def mazesolve():
    global front, leftspeed, rightspeed, heading, currentcell, currdist, swvalue, visits, maxspeed
    global basespeed, leftwall, rightwall, frontwall, move, msize, movecount, route, state, MIDDLE, HEIGHT, WIDTH
    global lsinit, rsinit, finit, mode, w2sec, frontwallseen, cm2, cm4, cm6p5, cm9, cm10, cm13, backupok
    global celllist, celllistno, heads, headsno, START, OUT, BACK
    mode = 1 # 0 for wallfollow, 1 for mazesolver
    setoutsidewalls()  # set up outside walls in the maze
    showwalls() # test code line
    readcalibration()    # read wall calibration values into lsinit, rsinit and finit
    leftSensorLED.value(True)
    rightSensorLED.value(True)
    onBoardLED.value(False)
    time.sleep (0.5) # wait while hand gets clear of the button that starts it
    leftSensorLED.value(False)
    rightSensorLED.value(False)
    onBoardLED.value(True)
    time.sleep (1)    
    basespeed = 18000    # standard running speed
    currentspeed = 0    # expected current speed
    leftspeed = basespeed
    rightspeed = basespeed
    frontwallseen = int(finit * 0.5)    # value to be above for FF if front wall seen from cell boundary
   # value to be above for RF if front wall seen from cell boundary
    print("Fseen", frontwallseen, "Finit", finit)
    side = 0
    adjustment = 0
    prevdiff = 0
    currentcell = 0     # number of the cell we are in
    celllistno = 0
    heading = NORTH     # direction the mouse is pointing
    state = EXPLORE     # state that is controlling what we do
    route = OUT          # go - OUT from start or BACK to start
    runs = 1
    leftwall = rightwall = PRESENT
    frontwall = NOTPRESENT
    #buttonwait()        # wait for start button pressed
    time.sleep(1)
    backup()            # reverse back to wall in start cell
    encoderreset()
    leftRev.duty_u16(maxspeed - leftspeed)
    leftFwd.duty_u16(maxspeed) # reset to moving forward at basespeed
    rightRev.duty_u16(maxspeed - rightspeed)
    rightFwd.duty_u16(maxspeed) # reset to moving forward at basespeed
    currdist = 0
    goahead(cm9,basespeed) # go from middle to get axles at 1st cell boundary. finish at basespeed
    currentcell = TABLEWIDTH
    if (wait == 1):
        pause()
    movecount = 0
    movesno = 0 # point to first item in moves array
    # main loop that is executed every time we go to a cell boundary
    while state == EXPLORE:     # move loop for explore
        movecount = movecount + 1
        #if (movecount == 3): # test code to save maze and walls after number of moves
            #savemaze()
            #halt()
        photoread() # read the wall sensors at cell boundary
        setwalls() # set the left, right and front wall presence flags for the cell we are just going into
        showLEDwalls() # put LEDs on if walls seen   
        stop() # stop motors while doing the flood - usually 10 to 40 msec        
        if (route == OUT):
            floodmaze(MIDDLE, currentcell) # flood from middle cell to current cell
        if (route == BACK):
            floodmaze(START, currentcell) # flood from start cell to current cell
            #savemaze() # test code to check flood is correct
            #halt()
        #if (movecount == 10): # test code to save maze and walls after number of moves
            #stop()
            #savemaze()
            #halt()
        # restart motors after halt during flood
        leftRev.duty_u16(maxspeed - leftspeed)
        leftFwd.duty_u16(maxspeed) # reset to moving forward at basespeed
        rightRev.duty_u16(maxspeed - rightspeed)
        rightFwd.duty_u16(maxspeed) # reset to moving forward at basespeed

        if (floodfail == 1):
            print ("Flood failed", MIDDLE, START, currentcell)
            showflood()
            showwalls()
            savemaze()
            halt() # then halt
        #print ("celllistno", celllistno)    
        if (celllistno < numcells):
            celllist[celllistno] = currentcell # log the current cell we are in
            celllistno = celllistno + 1
            heads[headsno] = heading # log headings 
            headsno = headsno + 1
        
        wherenext()  # get direction that we want to go in move field
        #print ("move", move)
        if (w2sec == 1):
            pause1()
        if (move == AHEAD):
            if (movesno < numcells):
                moves[movesno] = 1 # Log a turn round in the moves array
                movesno = movesno + 1
            currdist = 0 # starting at beginning of cell
            goonecellahead()
        if (move == RIGHT):
            if (movesno < numcells):
                moves[movesno] = 3 # Log a right turn in the moves array
                movesno = movesno + 1
            turnright()
        if (move == LEFT):
            if (movesno < numcells):
                moves[movesno] = 2 # Log a left turn in the moves array
                movesno = movesno + 1
            turnleft(0)
        if (move == AROUND):
            if (movesno < numcells):
                moves[movesno] = 4 # Log a turn round in the moves array
                movesno = movesno + 1
            turnaround(cm4) 
       
        # This section determines what we do when we get to the boundary of the centre cell
        if ((route == OUT) and (currentcell == MIDDLE)):  # Reached middle cell
            leftRev.duty_u16(maxspeed)
            leftFwd.duty_u16(maxspeed) # set to stopped
            rightRev.duty_u16(maxspeed)
            rightFwd.duty_u16(maxspeed) # set to stopped

            if (w2sec == 1):
                pause1()
            photoread()
            setwalls() # set the left, right and front wall presence flags for the cell we are just going into
            showLEDwalls() # set LEDs on to show the walls
            goahead(cm10,basespeed) # move to centre of middle cell
            stop()    # stop motors
            savemaze() # save maze, walls and moves list
            flashLEDs(4) # # flash LEDs 4 times then carry on
            turnaround(0) # turnround on the spot in centre of cell and update heading
            stop()
            time.sleep(1)
            goahead(cm10,basespeed) # move to edge of middle/end cell
            route = BACK   # reset route direction after reached middle/end cell
            if (w2sec == 1):
                pause1()
        # This section determines what we do when we get back to the start cell
        if ((route == BACK) and (currentcell == START)):  # Reached start cell again
            goahead(cm9,basespeed) # move to middle of start cell
            leftspeed = basespeed
            rightspeed = basespeed
            pause()
            turnaround(0) # in start cell then backup and go fwd to cell boundary
            print ("back at start cell")
            route = OUT # reset to head towards the centre
            # are we ready to do a direct run to the centre now?
            stop()
            floodmaze(MIDDLE,START) # flood from middle cell to start cell
            routecheck() # check if we have an optimum route, and create the moves list
            #if "visits" is True we have an  optimal route, else do another explore run
            if (visits == False):
                flashLEDs(4) # we need to do a search again
                movecount = 0
                movesno = 0
                currentcell = TABLEWIDTH
                heading = NORTH
            if (visits == True): # found optimal route so do a fast run
                state = RUNMAZE

            #savemaze() # save maze, walls and moves list
                
    # get here when we stop exploring as we are ready to run the maze

    while True:
        #stop()
        print("ready to do fast run ")
        #buttonwait() # wait for button press to restart after setting speed on 4 way switch
        #time.sleep(1)
        #readswitches()
        basespeed = basespeed + 1000 # increase speed
        leftspeed = basespeed
        rightspeed = basespeed
        currdist = 0
        routerun()
        stop()
        flashLEDs(3)
        turnaround(0)
        if (backupok == NO):            
            goahead(cm6p5,basespeed) # go ahead to get axles at 1st cell boundary. finish at basespeed
        else:
            goahead(cm5,basespeed)
        runback()

           
    # should not get here
    showwalls()
    stop()
    halt()    

# maze solver routines :
# calibrate - get mouse aligned centrally between the walls
# setwalls - read sensors when at cell boundary and set flags to say which walls seen if any
# floodmaze - flood maze from 1st cell number to 2nd cell number
# moveahead - go specified distance forward and at specified end speed whilst following walls if present
# checkroute - See if all the cells on the current flood route have been visited - so we know optimal
# runroute - run a sequence of cells at a given speed
# setaheadwall - set flag for wall seen ahead when at centre of cell
# setsidewalls  set flags for the side walls seen as we go into that cell
# triggerstart - detect pulse in front of mouse o start it once against back wall
# turnleft - smooth turn left from middle of cell to middle of cell
# turnright - smooth turn right from middle of cell to middle of cell
# turnaround  - stop and then spin 180 degrees, and if present back up to rear wall after spin
# wherenext - decide when in cell centre whether to continue ahead or turn

        # Logic for explore to centre
        # start with axles at cell boundary        
        # Read and set Left & Right Walls
        # do flood, then choose adjacent cell with lowest flood number (Left,Right or Ahead) and no wall in the way
        # If left or right cell is lowest do left or right turn to cell boundary
        # if ahead is lowest, move to centre of cell and check if wall ahead. Set wall ahead if it is there.
        # If no wall ahead go forward to cell boundary
        # if wall ahead, stop in centre then spin turn to lowest flood number cell at side or back.
        #    then go to cell boundary
        # Update current cell number
        # Go round loop again



    
    
    
# ------- End of definitions -----------------------------------------------------

onBoardLED.value(0)     # switch off LED on PCB
leftSensorLED.value(0) # switch off left sensor LED on sensor board
centreSensorLED.value(0) # switch off centre sensor2 LED on sensor board
rightSensorLED.value(0) # switch off right sensor LED on sensor board
leftMezzLED.value(0) # switch off left Mezzanine LED
rightMezzLED.value(0)# switch off right Mezzanine LED

l1count = l2count = r1count = r2count = 0 #reset the encoder counts

progno = 0
swvalue = 0
heading = 1


# configure irq callback for encoders
Leftenc1.irq(lambda p:leftcount1()) 
Leftenc2.irq(lambda p:leftcount2())
Rightenc1.irq(lambda p:rightcount1())
Rightenc2.irq(lambda p:rightcount2())

# test code for conversion
#selectprogram()
#motortest()
#LEDtest()
#phototest()
#mazesolve()
#basespeed = 15000
#front = leftside = rightside = 1000
#writecalibration()
#leftRev.duty_u16(maxspeed - bsespeed)
#leftFwd.duty_u16(maxspeed) 
#rightRev.duty_u16(maxspeed - basespeed)
#rightFwd.duty_u16(maxspeed)



# -------------------------------------------------
debug = 0          # if debug is set  to 1 it will print and wait at various points
wait = 0           # if wait is set  to 1 it will wait 2msec at various points
msize = 256

try:
    selectprogram()
except Exception as e:
    # Open a file in append mode to log the error
    with open("error_log.txt", "a") as f:
        f.write("\n====================================\n")
        f.write(f"Crash Log (Uptime seconds): {time.time()}\n")
        f.write("====================================\n")        
        # 2. Write the exact traceback lines
        sys.print_exception(e, f)        
        # 3. Force the ESP32 to write data from RAM cache straight to the physical flash
        f.flush() 
    # 4. Give the hardware a brief moment to finish file writing operations
    time.sleep(1)
    f.close
# Reboot the device to recover
print("Crash saved.")
halt()

#selectprogram()    # select and run the selected program



leftMezzLED.value(0) # switch off sensor1 LED on wall folllower board
rightMezzLED.value(0) # switch off sensor2 LED on wall folllower board
leftRev.duty_u16(maxspeed)
leftFwd.duty_u16(maxspeed) # stop motors
rightRev.duty_u16(maxspeed)
rightFwd.duty_u16(maxspeed) # stop motors

