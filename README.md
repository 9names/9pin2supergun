# 9pin2supergun
Code for a Pi Pico to read a CD32, Mega Drive/Genesis, or generic 9-pin gamepad and output each control to a separate pin for use with Neo Geos and/or Superguns.

I am personally using this code with a [Monster Joysticks CD32 joystick](https://monsterjoysticks.com/deluxe-cd32-retro-joystick-kit-classic) and a [Minigun Supergun](https://www.arcade-projects.com/threads/minigun-supergun-an-open-source-supergun.9408/).

There is a chance that there will be USB host support in the future, and this will be accommodated by the PCB I'm designing.

## Requirements

* A Pi Pico or compatible device that exposes the GPIO pins (in the future there will be a PCB design available that directly uses an RP2040, saving some space)
* Level shifters and pull-up resistors if you want to not risk the potential of having 5V in to the RP2040 (the schematic specifies ones that work, albeit in a surface mount package)
* Some kind of level shifter or power switch if you want to be able to use this with both CD32 and Mega Drive pads[^2]. Again, the schematic specifies ones that work, so go with that if you're comfortable with surface mount soldering or find them in a through-hole package
* Quite possibly level shifting for signals going out to the controller, depending on whether your controller is happy with 3.3V logic levels
* A DE9 and DA15[^1] connector (ideally plug for DE9, socket for DA15)

I went through quite a few headaches to get a level shifting setup that works in as generic a way as possible, so following what I've done will be the path of least resistance, just with some fairly fine pitched soldering. However, if you're not so worried about doing it Properly, there's probably little harm in playing around with those generic level shifter modules you get, and which I used during initial development of the CD32 side.

## Setup

Wire this up as per the schematic:

![Schematic showing how to wire up the setup, a KiCad project will come later](/9pin2supergun.svg)

The switch SW1 is used to select between CD32 and Mega Drive/Genesis mode; when tied to ground, it's Mega Drive mode, when floating, it's CD32 mode.

The button S1 and the LED D1 are currently unused, they will be set up once I have my PCB ready.

The level shifter U6 handles those inputs that are only ever inputs (pulled up to 5V). U7 handles both of the two outputs that can also be the +5V supply, and also the one remaining pin that's either an input or an output depending on whether it's in CD32 or Mega Drive mode.

Build and install the firmware and you should now have a Neo Geo/Supergun-compatible output from your CD32, Mega Drive, or generic 9-pin controller. By default, pressing both buttons on a two-button controller will send START too, to disable this behaviour, change MAP_GENERIC_AB_TO_START to false.

## Building

This is based on the [rp2040 project template](https://github.com/rp-rs/rp2040-project-template), have a look there for details. You need a `rust` build environment and the project will build with Cargo.

## Anything else

This polls at 125Hz for the CD32 - I don't know whether a real CD32 pad will be happy with this, but I don't see why it wouldn't. The timing for each polling burst is based on what I observed with my PAL Amiga 500+, so that should be fine, but it was only polling at 50Hz. Given how little time is spent polling compared to waiting even at 125Hz, though, I can't imagine what issues might arise.

The Mega Drive controller support polls at around 140Hz.

Current draw seems fairly low, hovering at around 20-25mA on the controllers I've tested it with. If you're worried, power it up via USB (not whilst connected to a 15-pin port!) and use a USB power meter to see how much it's drawing.

## Credits

Thanks to [Mathew Carr's PSCD32 Development Diary](https://www.mrdictionary.net/PSCD32/diary/2019_08_09.htm) for documenting how the CD32 protocol actually works, and to [the RetroSix Wiki's Mega Drive controller page](https://www.retrosix.wiki/controller-interface-sega-mega-drive) for documenting the Mega Drive bits.

[^1]: Literally everyone seems to call this a DB15 connector, but it is actually DA15.
[^2]: Annoyingly, they use different pinouts, with +5V being on a different pin
