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
    gpio::{FunctionPio0, Pin, PinState},
    pac,
    pio::PIOExt,
    sio::Sio,
    watchdog::Watchdog,
};

use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;
use embedded_hal::digital::StatefulOutputPin;

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

#[rp2040_hal::entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
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

    // We want to run at 125kHz
    let pio_multiplier: u16 = (clocks.system_clock.freq().to_Hz() / 125000)
        .try_into()
        .unwrap();

    info!("PIO multiplier is {}", pio_multiplier);

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let set_base: Pin<_, FunctionPio0, _> = pins.gpio18.into_function();
    let set_base_id = set_base.id().num;
    let side_set_base: Pin<_, FunctionPio0, _> = pins.gpio16.into_function();
    let side_set_base_id = side_set_base.id().num;
    let in_base: Pin<_, FunctionPio0, _> = pins.gpio17.into_function();
    let in_base_id = in_base.id().num;
    let mut input_up = pins.gpio22.into_pull_up_input();
    let mut input_down = pins.gpio21.into_pull_up_input();
    let mut input_left = pins.gpio20.into_pull_up_input();
    let mut input_right = pins.gpio19.into_pull_up_input();

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
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let mut output_coin = pins.gpio8.into_push_pull_output();
    // Don't have coin on CD32 pad
    output_coin.set_high().unwrap();
    let mut output_start = pins.gpio9.into_push_pull_output();
    let mut output_b1 = pins.gpio12.into_push_pull_output();
    let mut output_b2 = pins.gpio13.into_push_pull_output();
    let mut output_b3 = pins.gpio11.into_push_pull_output();
    let mut output_b4 = pins.gpio14.into_push_pull_output();
    let mut output_b5 = pins.gpio15.into_push_pull_output();
    let mut output_b6 = pins.gpio10.into_push_pull_output();
    let mut output_up = pins.gpio7.into_push_pull_output();
    let mut output_down = pins.gpio6.into_push_pull_output();
    let mut output_left = pins.gpio5.into_push_pull_output();
    let mut output_right = pins.gpio4.into_push_pull_output();

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
                debug!(
                    "Buttons: 1 {} 2 {} 3 {} 4 {} 5 {} 6 {} start {}",
                    output_b1.is_set_high().unwrap(),
                    output_b2.is_set_high().unwrap(),
                    output_b3.is_set_high().unwrap(),
                    output_b4.is_set_high().unwrap(),
                    output_b5.is_set_high().unwrap(),
                    output_b6.is_set_high().unwrap(),
                    output_start.is_set_high().unwrap(),
                );
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
        }
    }
}

// End of file
