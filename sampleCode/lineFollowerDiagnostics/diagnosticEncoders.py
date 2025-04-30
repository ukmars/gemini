#Very simple encoder class to return counts to the line follower diagnostic program
#Uses a quadrature encoder counter implemented in PIO from:
#https://github.com/raspberrypi/pico-examples/blob/master/pio/quadrature_encoder/quadrature_encoder.pio
#Encoder pin assignments and rotataion directions are hard coded in the class definition
#
#

import machine
import utime
import rp2
from machine import Pin
import array

class PIOQuadratureCounter:
    
    @rp2.asm_pio(autopush=False, autopull=False)
    
    def counter():
        jmp("update")    # 00 -> 00
        jmp("decrement") # 00 -> 01
        jmp("increment") # 00 -> 10
        jmp("update")    # 00 -> 11
        
        jmp("increment") # 01 -> 00
        jmp("update")    # 01 -> 01
        jmp("update")    # 01 -> 10
        jmp("decrement") # 01 -> 11
        
        jmp("decrement") # 10 -> 00
        jmp("update")    # 10 -> 01
        jmp("update")    # 10 -> 10
        jmp("increment") # 10 -> 11
        
        jmp("update")    # 11 -> 00
        jmp("increment") # 11 -> 01
        label("decrement")#11 -> 10
        jmp(y_dec, "update")
        label("update")  # 11 -> 11
        
        wrap_target()
        set(x, 0)
        pull(noblock) # check for request
        
        mov(x, osr)
        mov(osr, isr) # OSR = previous pins
        jmp(not_x, "sample_pins")
        
        # report data
        mov(isr, y)
        push()
        
        label("sample_pins")
        mov(isr, null)
        in_(osr, 2)
        in_(pins, 2)
        mov(pc, isr) # jump to <previous>:<current>
        
        label("increment")
        mov(x, invert(y))
        jmp(x_dec, "increment2")
        label("increment2")
        mov(y, invert(x))
        wrap()
        nop() # We have to fill up the program space or Micropython won't put it at the
        nop() # beginning of the program, and the jump table doesn't work.
        nop()

    def __init__(self, pio1, pin1, pin2):
        if pin2 != pin1 + 1:
            raise Exception("pin2 must be pin1 + 1")
        
        Pin(pin1, Pin.IN, Pin.PULL_UP)
        Pin(pin2, Pin.IN, Pin.PULL_UP)
        self.sm1 = rp2.StateMachine(pio1, self.counter, freq=125000000, in_base=machine.Pin(pin1))
        self.sm1.active(1)
        self.buf = array.array('i', [0])
    
    def read(self):
        self.sm1.put(1)

        # For efficiency, here we use an undocumented feature of
        # StateMachine to read a SIGNED integer value without
        # allocating a new object.
        self.sm1.get(self.buf)
        return self.buf[0]


class Encoders:
    def __init__(self):
        self.left = PIOQuadratureCounter(0, 8, 9)
        self.right = PIOQuadratureCounter(1, 6, 7)

        # Zero counts
        self.left_offset = 0
        self.right_offset = 0
        self.get_counts(reset = True)

    def get_counts(self, reset = False):
        # Sign on left encoders is reversed from what
        # the PIO counter gives us.
        left = -self.left.read() - self.left_offset
        right = self.right.read() - self.right_offset

        if reset:
            self.left_offset += left
            self.right_offset += right

        return left , right
