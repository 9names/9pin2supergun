//! Reads a CD32 gamepad and breaks it out into 7 individual active-low GPIO pins
#![no_std]
#![no_main]

use cortex_m::singleton;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use rp2040_hal::{
    self as hal,
    gpio::{DynInOutPin, DynPinId, PullNone},
};

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

type Pio0 = rp2040_hal::pio::PIO<pac::PIO0>;
type Sm0 = rp2040_hal::pio::UninitStateMachine<(pac::PIO0, rp2040_hal::pio::SM0)>;
type DmaChannel0 = rp2040_hal::dma::Channel<rp2040_hal::dma::CH0>;
type DmaChannel1 = rp2040_hal::dma::Channel<rp2040_hal::dma::CH1>;

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

    // We want to run at 140kHz
    let pio_multiplier: u16 = (clocks.system_clock.freq().to_Hz() / 140000)
        .try_into()
        .unwrap();

    info!("PIO multiplier is {}", pio_multiplier);

    let (pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let dma = pac.DMA.split(&mut pac.RESETS);

    do_controller_things(pins, pio_multiplier, pio, sm0, dma.ch0, dma.ch1);
}

#[allow(dead_code)]
struct PioPins {
    sideset_base_id: u8,
    sideset_base: Pin<DynPinId, FunctionPio0, PullNone>,
    base_pin_id: u8,
    base: Pin<DynPinId, FunctionPio0, PullNone>,
    base_1: Pin<DynPinId, FunctionPio0, PullNone>,
    base_2: Pin<DynPinId, FunctionPio0, PullNone>,
    base_3: Pin<DynPinId, FunctionPio0, PullNone>,
    base_4: Pin<DynPinId, FunctionPio0, PullNone>,
    base_5: Pin<DynPinId, FunctionPio0, PullNone>,
}

struct OutPins {
    coin: DynInOutPin,
    start: DynInOutPin,
    b1: DynInOutPin,
    b2: DynInOutPin,
    b3: DynInOutPin,
    b4: DynInOutPin,
    b5: DynInOutPin,
    b6: DynInOutPin,
    up: DynInOutPin,
    down: DynInOutPin,
    left: DynInOutPin,
    right: DynInOutPin,
}

fn do_controller_things(
    pins: rp2040_hal::gpio::Pins,
    pio_multiplier: u16,
    mut pio: Pio0,
    sm: Sm0,
    dma_0: DmaChannel0,
    dma_1: DmaChannel1,
) -> ! {
    // Connect GPIO2 to GND for Mega Drive/generic 9-pin. high or floating for CD32.
    let mut selector = pins.gpio2.into_pull_up_input();
    let _use_cd32 = selector.is_high().unwrap();

    let mut pin_six_direction = pins.gpio22.into_push_pull_output();
    let mut shifter_oe = pins.gpio23.into_push_pull_output_in_state(PinState::High);

    let mut out = OutPins {
        coin: DynInOutPin::new(pins.gpio12.into_pull_type().into_function().into_dyn_pin()),
        start: DynInOutPin::new(pins.gpio11.into_pull_type().into_function().into_dyn_pin()),
        b1: DynInOutPin::new(pins.gpio7.into_pull_type().into_function().into_dyn_pin()),
        b2: DynInOutPin::new(pins.gpio8.into_pull_type().into_function().into_dyn_pin()),
        b3: DynInOutPin::new(pins.gpio9.into_pull_type().into_function().into_dyn_pin()),
        b4: DynInOutPin::new(pins.gpio10.into_pull_type().into_function().into_dyn_pin()),
        b5: DynInOutPin::new(pins.gpio13.into_pull_type().into_function().into_dyn_pin()),
        b6: DynInOutPin::new(pins.gpio14.into_pull_type().into_function().into_dyn_pin()),
        up: DynInOutPin::new(pins.gpio3.into_pull_type().into_function().into_dyn_pin()),
        down: DynInOutPin::new(pins.gpio4.into_pull_type().into_function().into_dyn_pin()),
        left: DynInOutPin::new(pins.gpio5.into_pull_type().into_function().into_dyn_pin()),
        right: DynInOutPin::new(pins.gpio6.into_pull_type().into_function().into_dyn_pin()),
    };

    info!("MD mode");
    pin_six_direction.set_low().unwrap();

    let piopins = PioPins {
        sideset_base_id: pins.gpio21.id().num,
        sideset_base: pins
            .gpio21
            .into_floating_input()
            .into_function()
            .into_dyn_pin(),
        base_pin_id: pins.gpio24.id().num,
        base: pins
            .gpio24
            .into_floating_input()
            .into_function()
            .into_dyn_pin(),
        base_1: pins
            .gpio25
            .into_floating_input()
            .into_function()
            .into_dyn_pin(),
        base_2: pins
            .gpio26
            .into_floating_input()
            .into_function()
            .into_dyn_pin(),
        base_3: pins
            .gpio27
            .into_floating_input()
            .into_function()
            .into_dyn_pin(),
        base_4: pins
            .gpio28
            .into_floating_input()
            .into_function()
            .into_dyn_pin(),
        base_5: pins
            .gpio29
            .into_floating_input()
            .into_function()
            .into_dyn_pin(),
    };

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

    let installed = pio.install(&read_md.program).unwrap();
    let (mut sm, rx, _) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
        .side_set_pin_base(piopins.sideset_base_id)
        .in_pin_base(piopins.base_pin_id)
        .clock_divisor_fixed_point(pio_multiplier, 0)
        .build(sm);

    sm.set_pindirs([
        (piopins.sideset_base_id, hal::pio::PinDir::Output),
        (piopins.base_pin_id, hal::pio::PinDir::Input),
        (piopins.base_pin_id + 1, hal::pio::PinDir::Input),
        (piopins.base_pin_id + 2, hal::pio::PinDir::Input),
        (piopins.base_pin_id + 3, hal::pio::PinDir::Input),
        (piopins.base_pin_id + 4, hal::pio::PinDir::Input),
        (piopins.base_pin_id + 5, hal::pio::PinDir::Input),
    ]);

    // Enable the output on the level shifter
    shifter_oe.set_low().unwrap();
    sm.start();

    let rx_buf = singleton!(: u32 = 0).unwrap();
    let rx_buf2 = singleton!(: u32 = 0).unwrap();

    let rx_transfer = double_buffer::Config::new((dma_0, dma_1), rx, rx_buf).start();
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
                out.start
                    .set_state(get_pin_state(our_data & 0b100000000000))
                    .unwrap();
                out.down
                    .set_state(get_pin_state(our_data & 0b000100000000))
                    .unwrap();
                out.up
                    .set_state(get_pin_state(our_data & 0b000010000000))
                    .unwrap();
                out.b1
                    .set_state(get_pin_state(our_data & 0b000001000000))
                    .unwrap();
                out.b3
                    .set_state(get_pin_state(our_data & 0b000000100000))
                    .unwrap();
                out.right
                    .set_state(get_pin_state(our_data & 0b000000010000))
                    .unwrap();
                out.left
                    .set_state(get_pin_state(our_data & 0b000000001000))
                    .unwrap();
                out.b2
                    .set_state(get_pin_state(our_data & 0b000000000001))
                    .unwrap();

                if our_data & 0b000000011110000000000000 == 0 {
                    // 6 button controller
                    // Extra buttons: 0bxMXYZx
                    let six_button_data = our_data >> 18;
                    debug!("Got a 6 button");
                    out.b4
                        .set_state(get_pin_state(six_button_data & 0b001000))
                        .unwrap();
                    out.b5
                        .set_state(get_pin_state(six_button_data & 0b000100))
                        .unwrap();
                    out.b6
                        .set_state(get_pin_state(six_button_data & 0b000010))
                        .unwrap();
                    out.coin
                        .set_state(get_pin_state(six_button_data & 0b010000))
                        .unwrap();
                } else {
                    // Tidy up buttons we don't have
                    out.b4.set_high().unwrap();
                    out.b5.set_high().unwrap();
                    out.b6.set_high().unwrap();
                    out.coin.set_high().unwrap();
                }
            } else {
                // Generic controller
                // 0b2RLDU1
                if MAP_GENERIC_AB_TO_START {
                    out.start
                        .set_state(get_pin_state(our_data & 0b100001))
                        .unwrap();
                } else {
                    out.start.set_high().unwrap();
                }

                out.b1
                    .set_state(get_pin_state(our_data & 0b000001))
                    .unwrap();

                out.right
                    .set_state(get_pin_state(our_data & 0b010000))
                    .unwrap();
                out.left
                    .set_state(get_pin_state(our_data & 0b001000))
                    .unwrap();
                out.down
                    .set_state(get_pin_state(our_data & 0b000100))
                    .unwrap();
                out.up
                    .set_state(get_pin_state(our_data & 0b000010))
                    .unwrap();

                // Tidy up missing buttons
                out.b3.set_high().unwrap();
                out.b4.set_high().unwrap();
                out.b5.set_high().unwrap();
                out.b6.set_high().unwrap();
                out.coin.set_high().unwrap();
            }

            debug!("Got bits: {:#026b}", our_data);
            rx_transfer = next_rx_transfer.write_next(rx_buf);
        }
    }
}

// End of file
