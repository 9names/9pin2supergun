# 9pin2supergun
Code for a Pi Pico to read a CD32 gamepad and output each control to a separate pin for use with Neo Geos and/or Superguns.

I am personally using this code with a [Monster Joysticks CD32 joystick](https://monsterjoysticks.com/deluxe-cd32-retro-joystick-kit-classic) and a [Minigun Supergun](https://www.arcade-projects.com/threads/minigun-supergun-an-open-source-supergun.9408/). I don't know if it works with anything else.

Aspirationally, this will also read a Mega Drive/Genesis controller (fairly simple), and possibly USB controllers (a lot more complex, and projects to do it already exist). I also intend to create a PCB that uses a bare RP2040 chip to make things a bit neater and easier to hook up.

## Requirements

* A Pi Pico or compatible device that exposes the GPIO pins
* (Optional, but recommended) a level shifter for shifting the three signal pins going to the CD32 pad
* (Perhaps needed, depending on what you're plugging it into) a level shifter to shift up the output to the DA15 port
* A DE9 and DA15[^1] connector (ideally plug for DE9, socket for DA15)

## Setup

Wire everything up as per the following (quite bad) diagram:

![Schematic showing the wiring of DE9 to Pico to DA15. Sorry for not listing it out in detail in text, a proper schematic will be available later.](schematic.svg)

Add a bidirectional level shifter for 3.3V to 5V between the Pico and pins 5, 6, and 9 of the DE9 to be on the safe side.

Build and install the firmware and you should now have a Neo Geo/Supergun-compatible output from your CD32 controller. If the buttons are wrong or not to your taste, either move them on the DA15 side - each button has a direct mapping to a single pin there - or modify the code to use different GPIOs for the output pins.

This code theoretically also copies signals from up/down/left/right through the GPIO system, but for just the CD32 support it's not needed so I wouldn't bother.

## Building

This is based on the [rp2040 project template](https://github.com/rp-rs/rp2040-project-template), have a look there for details. You need a `rust` build environment and the project will build with Cargo.

## Anything else

This polls at 125Hz - I don't know whether a real CD32 pad will be happy with this, but I don't see why it wouldn't. The timing for each polling burst is based on what I observed with my PAL Amiga 500+, so that should be fine, but it was only polling at 50Hz. Given how little time is spent polling compared to waiting even at 125Hz, though, I can't imagine what issues might arise.

## Credits

Thanks to [Mathew Carr's PSCD32 Development Diary](https://www.mrdictionary.net/PSCD32/diary/2019_08_09.htm) for documenting how the CD32 protocol actually works.

[^1]: Literally everyone seems to call this a DB15 connector, but it is actually DA15.
