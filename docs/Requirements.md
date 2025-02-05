# Requirements and Design Choices for Gemini

## Brief

This document states the core requirements and end offering that Gemini will provide.

## Main board

Will be the same size and dimensions of UKMARSBOT

## Processor Support

The primary Processor with be the Raspberry Pi RP2040, other processors can be supported with additional mezanine boards, the development of these boards is for robot builders to pursue.

## Mezzanine boards

Initially we will be designing a Mezzanine board for the PiPico/PiPicoW

Meazanine boards will achieve connection with the main chassis board with 20mm extented male headers. 0ptionally stackable female headers from the chassis board can be used with standard male headers on the mezanine board.
The mezanine board will be mounted on the rearmost pair of motor bracket bolts and the rearmost pair of sensor mounting bolts.
## Sensors

### Wall Sensor


### Line Sensor


## Motors

N20 extended motor shaft DC motors with side mount encoders. Gear ratios to be selected by the builder. Recommended ratios 50:1, 30:1 or 20:1. Higher ratios give better resolution and easier control, lower ratios give highr top speed.

[Motor and Encoders](https://shop.pimoroni.com/products/micro-metal-motor-encoder?variant=39888423354451)
or 
https://shop.pimoroni.com/products/micro-metal-motor-encoder?variant=39888423354451

The above encoders are preferred as they can route the cable forwards without adding height to the motor assembly. Other encoders with cables emerging at the top may fit withing the headroom above the motor, builders will need to confirm this if choosing alternative encoders.
The connection to the encoders are JST-SH, if possible pads for corresponding JST-SH sockets will be included in the chassis board. 0.1inch x 6 connectors will be included on the chassis board. 

## Motor Controller
We have decided to use the DRV8833 chipset on a Switch/HW-627 module board which available from multiple suppliers on Amazon or eBay.
Example supplier (no recommendation) https://www.ebay.co.uk/itm/285548428267?itmmeta=01JKATES6WY164KBCSWANRJMRK&hash=item427c033beb:g:P4EAAOSwL0RlSjV8
Example supplier (no recommendation) https://www.ebay.co.uk/itm/195060438441?itmmeta=01JKATBT9NRDGMP47MV4P07C40&hash=item2d6a8215a9:g:ymYAAOSwSLZig2RE

## Motor Mounts
Pololu motor mounts or equivalent can be used. 3d Printed mounts will offer a better solution using standard M3 or M2.5 bolts with standoffs to mount the mezanine board.
## Batteries

The design will accommodate a PP3 battery form factor. Builders may use any PP3 battery they prefer, or any other battery that will fit withing the form factor of a PP3 battery and presents a voltage no greater than 9v.
The battery connection to the chassis board will be 0.1inch x2. A polarised connector can be used but is not mandatory so a polarity protection diode is required on the chassis board.
