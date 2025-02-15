# Requirements and Design Choices for Gemini

## Brief

This document states the core requirements and end offering that Gemini will provide.

## Board connections

| Name       | Pin  | A/D | Direction | Function |
|------------|------|-----|-----------|----------|
| **Connector 1 on Chassis to Mezzanine** |  |  |  |  |
| 5v        | Con1 | 0   | Pwr Out   | +5v regulated supply provided by chassis |
| Gnd       | Con1 | 1   | Pwr Out   | Gnd provided by chassis |
| LMTR1     | Con1 | 2 D | In        | Left motor input 1 to bridge |
| LMTR2     | Con1 | 3 D | In        | Left motor input 2 to bridge |
| RMTR1     | Con1 | 4 D | In        | Right motor input 1 to bridge |
| RMTR2     | Con1 | 5 D | In        | Right motor input 2 to bridge |
| **Connector 2 on Chassis to Mezzanine** |  |  |  |  |
| LQA       | Con2 | 0 D | Out       | Left quadrature encoder channel A |
| LQB       | Con2 | 1 D | Out       | Left quadrature encoder channel B |
| RQA       | Con2 | 2 D | Out       | Right quadrature encoder channel A |
| RQB       | Con2 | 3 D | Out       | Right quadrature encoder channel B |
| 3v3       | Con2 | 4 A | Pwr In    | 3V3 regulated supply provided by mezzanine |
| Vbatt     | Con2 | 5 A | Pwr Out   | Battery voltage supply after polarity protection |
| **Connector 3 on Mezzanine to Sensor Board** |  |  |  |  |
| Vemit     | Con3 | 0   | Pwr Out   | Power supply for sensor emitters, designers choice from 5v, Vbatt or 3v3 |
| Gnd       | Con3 | 1   | Pwr Out   | Gnd routed from chassis |
| 3v3       | Con3 | 4   | Pwr Out   | 3V3 regulated supply provided by mezzanine |
| SEN0DIO   | Con3 | 5 D | In/Out    | Digital I/O line 0 for sensors |
| SEN1DIO   | Con3 | 6 D | In/Out    | Digital I/O line 1 for sensors |
| SEN2DIO   | Con3 | 7 D | In/Out    | Digital I/O line 2 for sensors |
| SEN3DIO   | Con3 | 8 D | In/Out    | Digital I/O line 3 for sensors |
| SEN4DIO   | Con3 | 9 D | In/Out    | Digital I/O line 4 for sensors |
| SEN0AIP   | Con3 | 10 A | In       | Analogue input for sensor 0 |
| SEN1AIP   | Con3 | 11 A | In       | Analogue input for sensor 1 |
| SEN2AIP   | Con3 | 12 A | In       | Analogue input for sensor 2 |
| SEN3AIP   | Con3 | 13 A | In       | Analogue input for sensor 3 |
| SEN4AIP   | Con3 | 14 A | In       | Analogue input for sensor 4 |
| SEN5AIP   | Con3 | 15 A | In       | Analogue input for sensor 5 |
| SDA       | Con3 | 14 D | In/Out   | I2C Serial Data |
| SCL       | Con3 | 15 D | In/Out   | I2C Serial Clock |


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

## Wheels and Tyres
Standard 32mm Pololu wheels and tyres or equivalents will be used https://thepihut.com/products/pololu-wheel-32x7mm-pair-black


## Batteries

The design will accommodate a PP3 battery form factor. Builders may use any PP3 battery they prefer, or any other battery that will fit withing the form factor of a PP3 battery and presents a voltage no greater than 9v.
The battery connection to the chassis board will be 0.1inch x2. A polarised connector can be used but is not mandatory so a polarity protection diode is required on the chassis board.
