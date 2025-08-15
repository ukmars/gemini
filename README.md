
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