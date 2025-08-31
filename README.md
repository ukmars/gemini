
# Overview

Here is the UKMARS repository for our Gemini Micromouse. A robot capable of competing in our half size line following, Full Size Wall following and Full Size Maze Solving competitions. 

The Gemini robot is a second revision of the clubs first robot UKMARSBOT https://github.com/ukmars/ukmarsbot
Gemini has been designed so that the core microprocessor can be changed without having to change the chassis or sensor configuration. This has been done with what we call a mezzanine board. This "Mez" board connects the microprocessor to the chassis and the sensors.

At present the robot has been designed around the Raspberry Pi Pico (w)  to facilitate the use of MicroPython as a control language for the robot. This is taught is schools and allows schools to use the robot as part of their teaching aid. The Pico allows for other languages as well with the first alternative being C.

It is possible to build a Gemini robot using all commercially available parts.

In the future we hope to offer different Mez boards to allow a range of microcontrollers to be used. We are always looking for help from the community. So if you would like to design a mez board for a different microcontroller please get in touch.


This repo will contain all you need in order to build your own Gemini robot and get ready to compete in our competitions 

The groups website can be found https://ukmars.org/

A step by step build guide can currently be found https://ukmars.org/projects/gemini/

This "main" repo contains details relevant to the very latest PCB's that have been produced and distributed to schools and group members.  If you know you have a different version board than the latest (unlikely) you can find relevant KiCad documents in a release relating to your board. You will see a link to this on the right of this message

If you have found your way here without knowing about the UKMARS group, welcome. Becoming a member of UKMARS automatically gets you a set of PCB to build your own Gemini robot. All you have to do is source the parts and build.

# Technology details

All of the boards have been designed using the latest version 8 release of KiCad. They may get upgraded to version 9 but this is not a priority at the moment and the automations we have in place are not  compatible with v9.

You do not need KiCad installed in order to view the boards or the schematics

# Build details

The docs folder contains schematics, PCB renders and BOM lists. These are built directly from the KiCad and the component choices listed in the schematic represents the groups default choice of component. They are not necessarily the only option and builders are free to change the spec of components if they feel they have suitable technical knowledge to do so.
In addition to the components listed in the \docs\all_projects_bom.csv you will also need the following components to build a mouse.


# Supplier details

## Components

The following is a list of recommended UK supplier for the passive components of the board. Due to shipping costs we do recommend trying to source all passive components from one supplier if at all possible.

RSonline - https://uk.rs-online.com/web/

Rapid electronics - https://www.rapidonline.com/

Farnell - https://uk.farnell.com/

## Robot bits, Microprocessors and other

The following sites can be used to find the components not available from the above suppliers

https://thepihut.com/

https://shop.pimoroni.com/ - Motors and Encoders

https://www.amazon.co.uk/ - good for batteries and the DC motor controller

## Mechanical Parts not in the BOM

| Item                        | Description                                                                                                                                                                                                                                                                                                                                                                                          | part numbers and supplier options                                                                                                |
| --------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| Encoders and encoder cables | These can be ordered along side the motor so you might have already looked at these<br>You will need two 6 pin 1mm pitch JST-SH cables to connect the motor encoder to the main board. An adaptor is used to convert the JST-SH cable to 2.54 pitch to connect it to J1 and J2                                                                                                                       |                                                                                                                                  |
| Encoder adapter board       | This board converts JST-SH to 2.54 pins.<br>This does make soldering of the robot easier.<br>You can omit this part if you are comfortable soldering wires from a JST-SH cable direct to the chassis board                                                                                                                                                                                           | https://thepihut.com/products/breakout-for-6-pin-jst-sh-style-connector-side-entry                                               |
| JST_SH cables               | These cables connect the motor encoder to the chassis.<br><br>If you are comfortable soldering small cables direct to the chassis then you can use the second option. You DO NOT need the JST-SH adaptor board if you solder the wires direct to the chassis. This option is for experienced builders only as it increases the risk of problems with motors and/or encoders reading being reversed.  | https://thepihut.com/products/jst-gh-1-25mm-pitch-6-pin-cable-100mm-long<br><br><br>https://amzn.eu/d/hsau12j                    |
| Wheels and Tyres            | Pololu Wheel 32×7mm Pair available in black or white or you could 3D print some and use O rings for tyres                                                                                                                                                                                                                                                                                            | Pi Hut SKU: POL-1087 or SKU: POL-1088<br><br>https://thepihut.com/products/pololu-wheel-32x7mm-pair-black                        |
| Motor Mounts                | You need 2 motor mounts. They can be 3D printed if you have the capability or you can buy the Polulo ones.<br><br>3D print models can be found in the \mechanical folder. GeminiN20MotorMount.STL - Print upside down                                                                                                                                                                                | Pi Hut code SKU: POL-989<br><br>https://thepihut.com/products/pololu-micro-metal-gearmotor-bracket-pair-black                    |
| Battery and connector       | The battery should be a PP3 9Volt block Lithium Ion rechargeable one with micro USB charging socket and ideally charge level LEDs on the side.<br>Ideally 1200mAH capacity or greater                                                                                                                                                                                                                | https://amzn.eu/d/i8hoP5S<br><br>https://amzn.eu/d/cmckSHt<br><br>Rapid Online Order Code: 18-0092                               |
| Nuts and bolts              | If you are using 3D printed mounts which have 3mm holes in them you need 4 M3 x 30mm flat head polyamide bolts. Pack of M3 polyamide nuts 8 required<br><br>M3 x 10mm plastic spacers or 10mm piece of plastic tubing<br><br>Ebay is a good place to order small quantities of M3 hardware                                                                                                           | Rapid Online Order Code: 51-3852<br><br>Rapid-Online Order Code: 51-3437<br><br>https://www.ebay.co.uk/str/kaysfasteners<br><br> |
| Stand Offs                  | Used for mounting connecting the sensor and mez board to the chassis<br><br>Lots of ways to do this but a pack of standoff's makes this simple.<br><br>A nylon screw and nut can be used as a skid                                                                                                                                                                                                   | https://amzn.eu/d/eMlVNY3<br><br>                                                                                                |
|Screen|Not required, but a screen can be helpful. The important thing with a screen is the pin order which needs to be Gnd, Vcc, SCL, SDA left to right. Other screens can be used but will require custom connectors <br> *Warning* The screens available from Pimoroni can be used but their GND and Vcc pins are reversed when compared to the mez board connector. EXTREME care needs to be taken if wiring these|https://a.aliexpress.com/_EJEPBEO <br> https://amzn.eu/d/ffaTN75 

> All of the options here are what UKMARS believe to be the best starting options for schools and new builders. It does not represent the only options available. If you have suitable experience and knowledge feel free to swap components as you see fit. If you do deviate from the build guide and/or suggested components please make sure to mention this if you encounter problems.


