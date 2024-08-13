//! Reads a CD32 gamepad and breaks it out into 7 individual active-low GPIO pins
#![no_std]
#![no_main]

use cortex_m::singleton;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use rp2040_hal as hal;

use hal::{
    clocks::{init_clocks_and_plls, Clock},
    dma::{double_buffer, DMAExt},
    gpio::{FunctionPio0, InOutPin, Pin, PinState, PullNone},
    pac,
    pio::PIOExt,
    sio::Sio,
    watchdog::Watchdog,
};

use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

// Turn a positive bitmask result into PinState::High and anything else into PinState::Low
fn get_pin_state(result: u32) -> PinState {
    PinState::from(result > 0)
}

// If using a generic two-button controller, map A+B to be start
const MAP_GENERIC_AB_TO_START: bool = true;

#[rp2040_hal::entry]
fn main() -> ! {
    info!("Program start");
    let mut pac: pac::Peripherals = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Connect GPIO2 to GND for Mega Drive/generic 9-pin. high or floating for CD32.
    let mut selector = pins.gpio2.into_pull_up_input();
    let use_cd32 = selector.is_high().unwrap();

    let mut pin_six_direction = pins.gpio22.into_push_pull_output();
    let mut shifter_oe = pins.gpio23.into_push_pull_output_in_state(PinState::High);

    let mut output_coin = InOutPin::new(pins.gpio12);
    let mut output_start = InOutPin::new(pins.gpio11);
    let mut output_b1 = InOutPin::new(pins.gpio7);
    let mut output_b2 = InOutPin::new(pins.gpio8);
    let mut output_b3 = InOutPin::new(pins.gpio9);
    let mut output_b4 = InOutPin::new(pins.gpio10);
    let mut output_b5 = InOutPin::new(pins.gpio13);
    let mut output_b6 = InOutPin::new(pins.gpio14);
    let mut output_up = InOutPin::new(pins.gpio3);
    let mut output_down = InOutPin::new(pins.gpio4);
    let mut output_left = InOutPin::new(pins.gpio5);
    let mut output_right = InOutPin::new(pins.gpio6);

    // CD32 code
    // Ideally this would be in its own function, but I am too new to Rust to figure out the semantics there, mostly in terms of the "pac" variable.
    if use_cd32 {
        info!("CD32 mode");
        pin_six_direction.set_high().unwrap();
        // Don't have coin on CD32 pad
        output_coin.set_high().unwrap();
        // We want to run at 125kHz
        let pio_multiplier: u16 = (clocks.system_clock.freq().to_Hz() / 125000)
            .try_into()
            .unwrap();

        info!("PIO multiplier is {}", pio_multiplier);
        // Set: latch
        let set_base: Pin<_, FunctionPio0, _> = pins.gpio20.into_function();
        let set_base_id = set_base.id().num;
        // Side set: clock/fire 1
        let side_set_base: Pin<_, FunctionPio0, _> = pins.gpio24.into_function();
        let side_set_base_id = side_set_base.id().num;
        // In: data/fire 2
        let in_base: Pin<_, FunctionPio0, PullNone> = pins.gpio29.into_function().into_pull_type();
        let in_base_id = in_base.id().num;
        // Single-purpose inputs
        let mut input_up = pins.gpio25.into_floating_input();
        let mut input_down = pins.gpio26.into_floating_input();
        let mut input_left = pins.gpio27.into_floating_input();
        let mut input_right = pins.gpio28.into_floating_input();
        // Power output
        let mut plus_five_volts = pins.gpio21.into_push_pull_output();
        plus_five_volts.set_high().unwrap();

        let read_cd32 = pio_proc::pio_asm!(
            ".side_set 1 opt",
            "begin:",
            "    set pins, 0    side 0 [2]",
            "    in  pins, 1    side 0 [1]", // Blue button (5)
            "    nop            side 1",
            "    in  pins, 1    side 0 [3]", // Red (4)
            "    nop            side 1",
            "    in  pins, 1    side 0 [3]", // Yellow (2)
            "    nop            side 1",
            "    in  pins, 1    side 0 [3]", // Green (1)
            "    nop            side 1",
            "    in  pins, 1    side 0 [3]", // Right front (6)
            "    nop            side 1",
            "    in  pins, 1    side 0 [3]", // Left front (3)
            "    nop            side 1",
            "    in  pins, 1    side 0 [3]", // Pause (Start)
            "    push noblock          [2]", // Move to the RX FIFO so the main code can deal with it
            "    set x, 31",
            "    nop                   [7]",
            "    nop                   [7]",
            "    nop                   [7]",
            "    nop                   [3]",
            "    set pins, 1    side 1 [3]",
            "wait_loop:",
            "    nop                   [7]",
            "    nop                   [7]",
            "    nop                   [7]",
            "    nop                   [3]",
            "    jmp x-- wait_loop",
            "    jmp begin",
        );

        let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
        let installed = pio.install(&read_cd32.program).unwrap();
        let (mut sm, rx, _) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
            .side_set_pin_base(side_set_base_id)
            .set_pins(set_base_id, 1)
            .in_pin_base(in_base_id)
            .clock_divisor_fixed_point(pio_multiplier, 0)
            .build(sm0);

        sm.set_pindirs([
            (set_base_id, hal::pio::PinDir::Output),
            (side_set_base_id, hal::pio::PinDir::Output),
            (in_base_id, hal::pio::PinDir::Input),
        ]);

        // Enable the output on the level shifter
        shifter_oe.set_low().unwrap();
        sm.start();

        let dma = pac.DMA.split(&mut pac.RESETS);
        let rx_buf = singleton!(: u32 = 0).unwrap();
        let rx_buf2 = singleton!(: u32 = 0).unwrap();

        let rx_transfer = double_buffer::Config::new((dma.ch0, dma.ch1), rx, rx_buf).start();
        let mut rx_transfer = rx_transfer.write_next(rx_buf2);

        loop {
            if rx_transfer.is_done() {
                let (rx_buf, next_rx_transfer) = rx_transfer.wait();
                // We only care about 7 bits of the 32 bits, make it a bit easier to deal with
                let our_data = *rx_buf >> 25;
                output_start
                    .set_state(get_pin_state(our_data & 0b1000000))
                    .unwrap();
                output_b1
                    .set_state(get_pin_state(our_data & 0b0001000))
                    .unwrap();
                output_b2
                    .set_state(get_pin_state(our_data & 0b0000100))
                    .unwrap();
                output_b3
                    .set_state(get_pin_state(our_data & 0b0100000))
                    .unwrap();
                output_b4
                    .set_state(get_pin_state(our_data & 0b0000010))
                    .unwrap();
                output_b5
                    .set_state(get_pin_state(our_data & 0b0000001))
                    .unwrap();
                output_b6
                    .set_state(get_pin_state(our_data & 0b0010000))
                    .unwrap();
                if our_data != 127 {
                    debug!("Got bits: {:#09b}", our_data);
                    debug!("            S361245");
                    // FIXME: Figure out how to do this properly for each pin with InOutPin
                    //debug!(
                    //    "Buttons: 1 {} 2 {} 3 {} 4 {} 5 {} 6 {} start {}",
                    //    output_b1.is_high().unwrap(),
                    //    output_b2.is_high().unwrap(),
                    //    output_b3.is_high().unwrap(),
                    //    output_b4.is_high().unwrap(),
                    //    output_b5.is_high().unwrap(),
                    //    output_b6.is_high().unwrap(),
                    //    output_start.is_high().unwrap(),
                    //);
                }
                rx_transfer = next_rx_transfer.write_next(rx_buf);

                output_up
                    .set_state(PinState::from(input_up.is_high().unwrap()))
                    .unwrap();
                output_down
                    .set_state(PinState::from(input_down.is_high().unwrap()))
                    .unwrap();
                output_left
                    .set_state(PinState::from(input_left.is_high().unwrap()))
                    .unwrap();
                output_right
                    .set_state(PinState::from(input_right.is_high().unwrap()))
                    .unwrap();

                debug!(
                    // FIXME: Base this on the output?
                    "Up: {}, Down: {}, Left: {}, Right: {}",
                    input_up.is_high().unwrap(),
                    input_down.is_high().unwrap(),
                    input_left.is_high().unwrap(),
                    input_right.is_high().unwrap()
                );
            }
        }
    } else {
        info!("MD mode");
        pin_six_direction.set_low().unwrap();
        // We want to run at 140kHz
        let pio_multiplier: u16 = (clocks.system_clock.freq().to_Hz() / 140000)
            .try_into()
            .unwrap();

        info!("PIO multiplier is {}", pio_multiplier);

        //let set_base: Pin<_, FunctionPio0, _> = pins.gpio15.into_function();
        //let set_base_id = set_base.id().num;
        // Side set: select
        let side_set_base: Pin<_, FunctionPio0, _> = pins.gpio21.into_function();
        let side_set_base_id = side_set_base.id().num;
        // In base:
        let in_base: Pin<_, FunctionPio0, _> = pins.gpio24.into_function();
        let in_base_id = in_base.id().num;
        let _in_plus_one: Pin<_, FunctionPio0, _> = pins.gpio25.into_function();
        let _in_plus_two: Pin<_, FunctionPio0, _> = pins.gpio26.into_function();
        let _in_plus_three: Pin<_, FunctionPio0, _> = pins.gpio27.into_function();
        let _in_plus_four: Pin<_, FunctionPio0, _> = pins.gpio28.into_function();
        let _in_plus_five: Pin<_, FunctionPio0, _> = pins.gpio29.into_function();

        let mut plus_five_volts = pins.gpio20.into_push_pull_output();
        plus_five_volts.set_high().unwrap();

        let read_md = pio_proc::pio_asm!(
            ".side_set 1",
            ".wrap_target",
            "    in  pins, 6        side 0", // Start/A/GND/GND
            "    in  pins, 6        side 1", // Normal
            "    nop                side 0",
            "    nop                side 1", // Same as first IN
            "    nop                side 0", // Same as second IN
            "    in  pins, 6        side 1", // All directions GND if 6-button
            "    in  pins, 6        side 0", // Mode/X/Y/Z
            "    push noblock       side 1", // Push off to FIFO
            "    set x, 31          side 1", // These delays get it down to around 140Hz polling
            "    nop           [6]  side 1", // This has been verified via oscilloscope rather than maths
            "    nop           [6]  side 1",
            "    nop           [6]  side 1",
            "    nop           [6]  side 1",
            "    nop           [6]  side 1",
            "    nop           [2]  side 1",
            "wait_loop:"                     // 30 cycles each iteration of this loop.
            "    nop           [6]  side 1",
            "    nop           [6]  side 1",
            "    nop           [6]  side 1",
            "    nop           [6]  side 1",
            "    nop                side 1",
            "    jmp x-- wait_loop  side 1",
            ".wrap",
        );

        let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
        let installed = pio.install(&read_md.program).unwrap();
        let (mut sm, rx, _) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
            .side_set_pin_base(side_set_base_id)
            .in_pin_base(in_base_id)
            .clock_divisor_fixed_point(pio_multiplier, 0)
            .build(sm0);

        sm.set_pindirs([
            (side_set_base_id, hal::pio::PinDir::Output),
            (in_base_id, hal::pio::PinDir::Input),
            (in_base_id + 1, hal::pio::PinDir::Input),
            (in_base_id + 2, hal::pio::PinDir::Input),
            (in_base_id + 3, hal::pio::PinDir::Input),
            (in_base_id + 4, hal::pio::PinDir::Input),
            (in_base_id + 5, hal::pio::PinDir::Input),
        ]);

        // Enable the output on the level shifter
        shifter_oe.set_low().unwrap();
        sm.start();

        let dma = pac.DMA.split(&mut pac.RESETS);
        let rx_buf = singleton!(: u32 = 0).unwrap();
        let rx_buf2 = singleton!(: u32 = 0).unwrap();

        let rx_transfer = double_buffer::Config::new((dma.ch0, dma.ch1), rx, rx_buf).start();
        let mut rx_transfer = rx_transfer.write_next(rx_buf2);

        loop {
            if rx_transfer.is_done() {
                let (rx_buf, next_rx_transfer) = rx_transfer.wait();
                // We only care about 24 bits of the 32 bits, make it a bit easier to deal with
                let our_data = *rx_buf >> 8;

                if our_data & 0b000000011000011000000000 == 0 {
                    // Mega Drive controller
                    // Three button Mega Drive controller buttons:
                    // Data: 0bxxxxxxxxxxxxSGGDUACRLDUA
                    output_start
                        .set_state(get_pin_state(our_data & 0b100000000000))
                        .unwrap();
                    output_down
                        .set_state(get_pin_state(our_data & 0b000100000000))
                        .unwrap();
                    output_up
                        .set_state(get_pin_state(our_data & 0b000010000000))
                        .unwrap();
                    output_b1
                        .set_state(get_pin_state(our_data & 0b000001000000))
                        .unwrap();
                    output_b3
                        .set_state(get_pin_state(our_data & 0b000000100000))
                        .unwrap();
                    output_right
                        .set_state(get_pin_state(our_data & 0b000000010000))
                        .unwrap();
                    output_left
                        .set_state(get_pin_state(our_data & 0b000000001000))
                        .unwrap();
                    output_b2
                        .set_state(get_pin_state(our_data & 0b000000000001))
                        .unwrap();

                    if our_data & 0b000000011110000000000000 == 0 {
                        // 6 button controller
                        // Extra buttons: 0bxMXYZx
                        let six_button_data = our_data >> 18;
                        debug!("Got a 6 button");
                        output_b4
                            .set_state(get_pin_state(six_button_data & 0b001000))
                            .unwrap();
                        output_b5
                            .set_state(get_pin_state(six_button_data & 0b000100))
                            .unwrap();
                        output_b6
                            .set_state(get_pin_state(six_button_data & 0b000010))
                            .unwrap();
                        output_coin
                            .set_state(get_pin_state(six_button_data & 0b010000))
                            .unwrap();
                    } else {
                        // Tidy up buttons we don't have
                        output_b4.set_high().unwrap();
                        output_b5.set_high().unwrap();
                        output_b6.set_high().unwrap();
                        output_coin.set_high().unwrap();
                    }
                } else {
                    // Generic controller
                    // 0b2RLDU1
                    if MAP_GENERIC_AB_TO_START {
                        output_start
                            .set_state(get_pin_state(our_data & 0b100001))
                            .unwrap();
                    } else {
                        output_start.set_high().unwrap();
                    }

                    output_b1
                        .set_state(get_pin_state(our_data & 0b000001))
                        .unwrap();

                    output_right
                        .set_state(get_pin_state(our_data & 0b010000))
                        .unwrap();
                    output_left
                        .set_state(get_pin_state(our_data & 0b001000))
                        .unwrap();
                    output_down
                        .set_state(get_pin_state(our_data & 0b000100))
                        .unwrap();
                    output_up
                        .set_state(get_pin_state(our_data & 0b000010))
                        .unwrap();

                    // Tidy up missing buttons
                    output_b3.set_high().unwrap();
                    output_b4.set_high().unwrap();
                    output_b5.set_high().unwrap();
                    output_b6.set_high().unwrap();
                    output_coin.set_high().unwrap();
                }

                debug!("Got bits: {:#026b}", our_data);
                rx_transfer = next_rx_transfer.write_next(rx_buf);
            }
        }
    }
}

// End of file
