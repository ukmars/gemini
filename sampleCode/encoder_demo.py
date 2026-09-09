# UKMARS Gemini Encoder Demo
# Oct 29 2025
# Prints the left and right motor encoder values

from machine import Pin
from time import sleep

# Define pins
# You may need to swap the numbers if one encoder is negative when going forward
left_a = Pin(8, Pin.IN, Pin.PULL_UP)
left_b = Pin(9, Pin.IN, Pin.PULL_UP)

right_a = Pin(7, Pin.IN, Pin.PULL_UP)
right_b = Pin(6, Pin.IN, Pin.PULL_UP)


# Global state
last_left_a = left_a.value()
last_right_a = left_a.value()

left_counts = 0
right_counts = 0

def left_encoder_callback(pin):
    global left_counts, last_left_a
    a = left_a.value()
    b = left_b.value()
    
    if a != last_left_a:
        # Check direction based on B when A changes
        if a == b:
            left_counts -= 1
        else:
            left_counts += 1
        last_left_a = a
        
def right_encoder_callback(pin):
    global right_counts, last_right_a
    a = right_a.value()
    b = right_b.value()
    
    if a != last_right_a:
        # Check direction based on B when A changes
        if a == b:
            right_counts -= 1
        else:
            right_counts += 1
        last_right_a = a


# Attach interrupts
left_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=left_encoder_callback)
right_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=right_encoder_callback)

# Main loop
while True:
    print(f"Left: {left_counts} Right: {right_counts}")
    sleep(0.1)
