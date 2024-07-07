import rp2
import machine
import time
from machine import Pin

@rp2.asm_pio(out_init=(rp2.PIO.OUT_LOW, rp2.PIO.OUT_LOW, rp2.PIO.OUT_LOW, rp2.PIO.OUT_LOW, rp2.PIO.OUT_LOW, rp2.PIO.OUT_LOW, rp2.PIO.OUT_LOW), sideset_init=rp2.PIO.OUT_LOW, set_init=rp2.PIO.OUT_LOW)
def read_cd32():
    # run at 125kHz
    label("begin")
    set(pins, 0).side(0).delay(2)  # Prepare to read
    in_(pins, 1).side(0).delay(1)  # And read the first bit
    # This could probably be looped, but I don't know if it gives us anything.
    # This reading bit takes 5 * 7 = 35 clock cycles 
    nop().side(1)  # Give us our first clock pulse
    in_(pins, 1).side(0).delay(3)  # And read the first bit
    nop().side(1)
    in_(pins, 1).side(0).delay(3)
    nop().side(1)
    in_(pins, 1).side(0).delay(3)
    nop().side(1)
    in_(pins, 1).side(0).delay(3)
    nop().side(1)
    in_(pins, 1).side(0).delay(3)
    nop().side(1)
    in_(pins, 1).side(0).delay(3)

    # We've read the bits we care about, there are two more bits but we don't care
    mov(pins, isr) # Get the bits into the output
    mov(isr, null) # Clear out the ISR for next time (the above mov will deal with the OSR)
    nop()

    set(x, 31)
    # And now we wait a load. There are 18 + 21 = 39 cycles above, and 5 cycles at the end, so 44 for our actual work.
    # To make things nice, let's add some extra delays to bring us to 72 so each busy loop can be 29 cycles.
    # Given this is 125Hz I don't think it actually *matters*, but still, it makes my numbers happier
    nop().delay(7)
    nop().delay(7)
    nop().delay(7)
    nop().delay(3)
    set(pins, 1).side(1).delay(3)          # We're keeping JOYMODE low most of the time, so give it a few cycles to reset
    label("wait_loop")
    nop().delay(7)
    nop().delay(7)
    nop().delay(7)
    nop().delay(3)
    jmp(x_dec, "wait_loop")

    jmp("begin")

sm = rp2.StateMachine(0, read_cd32, freq=125000, set_base=machine.Pin(18, mode=Pin.OUT, pull=Pin.PULL_DOWN), sideset_base=machine.Pin(16, mode=Pin.OUT, pull=Pin.PULL_DOWN), in_base=machine.Pin(17, mode=Pin.IN, pull=Pin.PULL_UP), out_base=machine.Pin(9, mode=Pin.OUT, pull=Pin.PULL_UP))
sm.active(1)
