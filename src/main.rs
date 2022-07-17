//! A GameBoy emulator for the Raspberry Pi Pico...

#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

// ==========================================
// CONFIGURATION SECTION
// ==========================================

/// Path to your game data, relative to the `src/` directory.
const GAME_DATA: &'static [u8] = include_bytes!("../pokemon.gb");

/// Overclocking switch - disabling this will only use stock clocks for talking to
/// peripherals.
///
/// While this should be safe (this doesn't tweak voltage, just the core PLL), I obviously
/// am **not liable** for breaking your Pico.
const DO_OVERCLOCK: bool = true;

/// Check that only one display driver is active
#[cfg(any(
    all(
        feature = "st7789_display_driver",
        any(feature = "ssd1306_display_driver")
    ),
    all(
        feature = "ssd1306_display_driver",
        any(feature = "st7789_display_driver")
    )
))]
compile_error!("Only one display driver must be enabled!");

/// Throw an error if no display drivers are enabled.
#[cfg(not(any(feature = "st7789_display_driver", feature = "ssd1306_display_driver")))]
compile_error!("One display driver must be enabled!");

/// Check that only one frontend is active
#[cfg(any(
    all(feature = "pico_display", any(feature = "thumby")),
    all(feature = "thumby", any(feature = "pico_display"))
))]
compile_error!("Only one device configuration must be enabled!");

/// Throw an error if no frontends are enabled.
#[cfg(not(any(feature = "pico_display", feature = "thumby")))]
compile_error!("One device configuration must be enabled!");

// ==========================================
// RUNTIME SECTION
// ==========================================

use alloc::vec::Vec;
use app::{ColorPalette, DisplayProperties};

use cortex_m_rt::{exception, ExceptionFrame};
use defmt::*;
use defmt_rtt as _;
use oxidgb_core::{
    gpu::{GPU, PITCH},
    input::{build_input, GameboyButton},
};
use panic_probe as _;

#[macro_use]
extern crate alloc;

use alloc_cortex_m::CortexMHeap;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
pub use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{self, pac, sio::Sio, watchdog::Watchdog};
use bsp::hal::{rom_data::flash_range_program, Clock};

use embedded_hal::digital::v2::OutputPin;

use bsp::entry;

use embedded_time::rate::units::Extensions;

use core::{cmp::min, mem::MaybeUninit};

#[cfg(feature = "ssd1306_display_driver")]
use embedded_graphics_core::pixelcolor::BinaryColor;

#[cfg(feature = "ssd1306_display_driver")]
use ssd1306::mode::DisplayConfig;

#[cfg(feature = "st7789_display_driver")]
use embedded_graphics_core::{pixelcolor::Rgb565, prelude::RgbColor};

use embedded_hal::digital::v2::InputPin;

mod app;
mod clock;

pub struct BlitBuffer {
    #[cfg(feature = "st7789_display_driver")]
    // properties.display_width * properties.display_height, RGB565
    buffer: Vec<u16>,
}

/// Provides an interface to directly blit pixels to the screen.
pub trait AcceleratedBlit {
    /// Stolen from ST7789:
    ///
    /// Sets pixel colors in given rectangle bounds.
    ///
    /// # Arguments
    ///
    /// * `gpu` - core GPU
    /// * `buffer` - pixel buffer for rendering
    ///
    fn set_pixels(&mut self, gpu: &GPU, properties: &DisplayProperties, buffer: &mut BlitBuffer);
}

#[cfg(feature = "st7789_display_driver")]
impl<DI, OUT> AcceleratedBlit for st7789::ST7789<DI, OUT>
where
    DI: display_interface::WriteOnlyDataCommand,
    OUT: OutputPin,
    OUT::Error: core::fmt::Debug,
{
    fn set_pixels(&mut self, gpu: &GPU, properties: &DisplayProperties, buffer: &mut BlitBuffer) {
        let x_max = min(properties.display_width, 160);
        let y_max = min(properties.display_height, 144);

        let y_offset = (properties.display_height - y_max) / 2;
        let x_offset = (properties.display_width - x_max) / 2;

        for y in 0..y_max {
            for x in 0..x_max {
                let offset = (y * 160 + x) * PITCH;

                let packed_rgb = ((gpu.pixel_data[offset] as u16 & 0b11111000) << 8)
                    | ((gpu.pixel_data[offset + 1] as u16 & 0b11111100) << 3)
                    | (gpu.pixel_data[offset + 2] as u16 >> 3);

                let buffer_ptr = (y + y_offset) * properties.display_width + (x + x_offset);
                buffer.buffer[buffer_ptr] = packed_rgb;
            }
        }

        // TODO: This isn't particularly efficient
        self.set_pixels(
            properties.x_offset as u16,
            properties.y_offset as u16,
            properties.display_width as u16 + properties.x_offset as u16 - 1,
            properties.display_height as u16 + properties.y_offset as u16 - 1,
            buffer.buffer.iter().map(|x| *x),
        )
        .unwrap();
    }
}

#[cfg(feature = "ssd1306_display_driver")]
impl<DI, SIZE> AcceleratedBlit
    for ssd1306::Ssd1306<DI, SIZE, ssd1306::mode::BufferedGraphicsMode<SIZE>>
where
    DI: display_interface::WriteOnlyDataCommand,
    SIZE: ssd1306::size::DisplaySize,
{
    fn set_pixels(&mut self, gpu: &GPU, properties: &DisplayProperties, buffer: &mut BlitBuffer) {
        let x_max = min(properties.display_width, 160);
        let y_max = min(properties.display_height, 144);

        // Pack data for this screen
        for y in 0..y_max {
            for x in 0..x_max {
                let on = if properties.display_width < 160 {
                    // Attempt to scale the screen
                    let mut on_fields = 0;

                    for y_add in 0..3 {
                        let modded_y = y * 3 + y_add;
                        if modded_y >= 144 {
                            break;
                        }

                        for x_add in 0..2 {
                            let modded_x = x * 2 + x_add;
                            if modded_x >= 160 {
                                break;
                            }

                            let offset = (modded_y * 160 + modded_x) * PITCH;

                            if gpu.pixel_data[offset] > 128 {
                                on_fields += 1;
                            }
                        }
                    }

                    on_fields > 2
                } else {
                    let offset = (y * 160 + x) * PITCH;

                    gpu.pixel_data[offset] > 128
                };

                let i = y * properties.display_width + x;
                /*if on {
                    buffer.buffer[i / 8] |= 1u8 << (i % 8);
                } else {
                    buffer.buffer[i / 8] &= !(1u8 << (i % 8));
                }*/
                self.set_pixel(x as _, y as _, on);
            }
        }

        /*self.set_draw_area(
            (0, 0),
            (
                properties.display_width as u8,
                properties.display_height as u8,
            ),
        )
        .unwrap();
        self.bounded_draw(
            &buffer.buffer,
            properties.display_width,
            (0, 0),
            (
                properties.display_width as u8,
                properties.display_height as u8,
            ),
        )
        .unwrap();*/
        self.flush().unwrap();
    }
}

pub trait Flushable {
    fn flush(&mut self);
}

#[cfg(feature = "st7789_display_driver")]
impl<DI, OUT> Flushable for st7789::ST7789<DI, OUT>
where
    DI: display_interface::WriteOnlyDataCommand,
    OUT: OutputPin,
    OUT::Error: core::fmt::Debug,
{
    fn flush(&mut self) {
        // Not needed.
    }
}

#[cfg(feature = "ssd1306_display_driver")]
impl<DI, SIZE> Flushable for ssd1306::Ssd1306<DI, SIZE, ssd1306::mode::BufferedGraphicsMode<SIZE>>
where
    DI: display_interface::WriteOnlyDataCommand,
    SIZE: ssd1306::size::DisplaySize,
{
    fn flush(&mut self) {
        self.flush().unwrap();
    }
}

static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
const HEAP_SIZE: usize = 245 * 1024;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

const SAVE_DATA_SIZE: usize = 128 * 1024;
#[link_section = ".savegame"]
static SAVE_DATA: [MaybeUninit<u8>; SAVE_DATA_SIZE] = [MaybeUninit::uninit(); SAVE_DATA_SIZE];

/// Crash indicator
fn crash_loop() -> ! {
    // Try to configure a blinking LED - steal all interfaces and assume that
    // core 1 is dead at this point.
    let (mut pac, cp) = unsafe { (pac::Peripherals::steal(), pac::CorePeripherals::steal()) };

    let sio = Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    let mut delay = cortex_m::delay::Delay::new(cp.SYST, 125_000_000);

    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

/// An out of memory handler
#[alloc_error_handler]
fn oom(layout: core::alloc::Layout) -> ! {
    error!("Out of memory!");
    error!(
        "Used: {} bytes, free: {} bytes",
        ALLOCATOR.used(),
        ALLOCATOR.free()
    );
    error!("Attempted to allocate {} bytes", layout.size());

    crash_loop()
}

/// Hard fault error handler
#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    error!("Hard fault!");
    error!(
        "r0={:x}, r1={:x}, r2={:x}, r3={:x}, r12={:x}, lr={:x}, pc={:x}, xpsr={:x}",
        ef.r0(),
        ef.r1(),
        ef.r2(),
        ef.r3(),
        ef.r12(),
        ef.lr(),
        ef.pc(),
        ef.xpsr()
    );

    crash_loop()
}

#[entry]
fn main() -> ! {
    // -- RP2040 INIT

    // Setup the allocator with a conservative heap.
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }

    let mut pac = pac::Peripherals::take().unwrap();
    let cp = pac::CorePeripherals::take().unwrap();

    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = if DO_OVERCLOCK {
        clock::configure_overclock(
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
    } else {
        clock::configure_default_clocks(
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
    };

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Reset the LED light (used for error detection)
    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_low().unwrap();

    let mut delay = cortex_m::delay::Delay::new(cp.SYST, 125_000_000);

    // -- DISPLAY CONFIGURATIONS

    #[cfg(feature = "pico_display")]
    let (properties, display_interface, bl, input) = {
        // Configure pins
        // These are implicitly used by the spi driver if they are in the correct mode
        let _spi_sclk = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>();
        let _spi_mosi = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>();
        // We're using what would normally be the miso pin for data/command
        let dc = pins.gpio16.into_push_pull_output();
        // Chip select
        let cs = pins.gpio17.into_push_pull_output();
        // Backlight
        let bl = pins.gpio20.into_push_pull_output();

        // Setup and init the SPI device
        let spi = bsp::hal::Spi::<_, _, 8>::new(pac.SPI0);

        let spi = spi.init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(),
            //16_000_000u32.Hz(),
            32_000_000u32.Hz(),
            &embedded_hal::spi::MODE_3,
        );

        // Set properties of the screen we are using
        let properties = DisplayProperties {
            display_width: 240,
            display_height: 135,
            x_offset: 40,
            y_offset: 53,
        };

        // Add input handling
        let button_a = pins.gpio12.into_pull_up_input();
        let button_b = pins.gpio13.into_pull_up_input();
        let button_x = pins.gpio14.into_pull_up_input();
        let button_y = pins.gpio15.into_pull_up_input();

        let input_handler = move || {
            let mut input: heapless::Vec<GameboyButton, 4> = heapless::Vec::new();

            if button_a.is_low().unwrap() {
                input.push(GameboyButton::A).unwrap();
            }
            if button_b.is_low().unwrap() {
                input.push(GameboyButton::B).unwrap();
            }
            if button_x.is_low().unwrap() {
                input.push(GameboyButton::START).unwrap();
            }
            if button_y.is_low().unwrap() {
                input.push(GameboyButton::SELECT).unwrap();
            }

            build_input(&input)
        };

        (
            properties,
            display_interface_spi::SPIInterface::new(spi, dc, cs),
            bl,
            input_handler,
        )
    };

    #[cfg(feature = "thumby")]
    let (properties, display_interface, display_size, display_rotation, input) = {
        // https://github.com/TinyCircuits/TinyCircuits-Thumby-Lib/blob/master/src/Thumby.h
        /*
        #define THUMBY_CS_PIN 16
        #define THUMBY_SCK_PIN 18
        #define THUMBY_SDA_PIN 19
        #define THUMBY_DC_PIN 17
        #define THUMBY_RESET_PIN 20

        #define THUMBY_BTN_LDPAD_PIN 3
        #define THUMBY_BTN_RDPAD_PIN 5
        #define THUMBY_BTN_UDPAD_PIN 4
        #define THUMBY_BTN_DDPAD_PIN 6
        #define THUMBY_BTN_B_PIN 24
        #define THUMBY_BTN_A_PIN 27

        #define THUMBY_AUDIO_PIN 28

        #define THUMBY_SCREEN_RESET_PIN 20

        #define THUMBY_SCREEN_WIDTH 72
        #define THUMBY_SCREEN_HEIGHT 40
        */

        // These are implicitly used by the spi driver if they are in the correct mode
        let _spi_sclk = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>();
        let _spi_mosi = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>();
        // We're using what would normally be the miso pin for data/command
        let dc = pins.gpio17.into_push_pull_output();
        // Chip select
        let cs = pins.gpio16.into_push_pull_output();
        // Backlight
        //let bl = pins.gpio20.into_push_pull_output();

        // Setup and init the SPI device
        let spi = bsp::hal::Spi::<_, _, 8>::new(pac.SPI0);

        let spi = spi.init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(),
            16_000_000u32.Hz(),
            &embedded_hal::spi::MODE_3,
        );

        let mut rst = pins.gpio20.into_push_pull_output();
        rst.set_low().unwrap();
        delay.delay_ms(10);
        rst.set_high().unwrap();
        delay.delay_ms(10);

        let interface = ssd1306::prelude::SPIInterface::new(spi, dc, cs);

        // Set properties of the screen we are using
        let properties = DisplayProperties {
            display_width: 72,
            display_height: 40,
            x_offset: 0,
            y_offset: 0,
        };

        // Add input handling
        let button_l = pins.gpio3.into_pull_up_input();
        let button_u = pins.gpio4.into_pull_up_input();
        let button_r = pins.gpio5.into_pull_up_input();
        let button_d = pins.gpio6.into_pull_up_input();

        // TODO: Not supported under rp-pico
        //let button_b = pins.gpio24.into_pull_up_input();
        let button_a = pins.gpio27.into_pull_up_input();

        let input_handler = move || {
            let mut input: heapless::Vec<GameboyButton, 6> = heapless::Vec::new();

            if button_a.is_low().unwrap() {
                input.push(GameboyButton::A).unwrap();
            }
            /*if button_b.is_low().unwrap() {
                input.push(GameboyButton::B).unwrap();
            }*/

            if button_l.is_low().unwrap() {
                input.push(GameboyButton::LEFT).unwrap();
            }
            if button_u.is_low().unwrap() {
                input.push(GameboyButton::UP).unwrap();
            }
            if button_r.is_low().unwrap() {
                input.push(GameboyButton::RIGHT).unwrap();
            }
            if button_d.is_low().unwrap() {
                input.push(GameboyButton::DOWN).unwrap();
            }

            build_input(&input)
        };

        (
            properties,
            interface,
            ssd1306::size::DisplaySize72x40,
            ssd1306::rotation::DisplayRotation::Rotate0,
            input_handler,
        )
    };

    // -- DISPLAY DRIVERS
    #[cfg(feature = "st7789_display_driver")]
    let (display, palette) = {
        let mut display = st7789::ST7789::new(
            display_interface,
            None,
            Some(bl),
            properties.display_width as _,
            properties.display_height as _,
        );

        // initialize
        display.init(&mut delay).unwrap();
        // set default orientation
        display
            .set_orientation(st7789::Orientation::Landscape)
            .unwrap();
        display
            .set_tearing_effect(st7789::TearingEffect::HorizontalAndVertical)
            .unwrap();

        // Set the color values used by this screen
        let palette = ColorPalette {
            splash_bg: Rgb565::new(10, 10, 250),
            text_color: Rgb565::WHITE,
            clear_color: Rgb565::BLACK,
        };

        (display, palette)
    };

    #[cfg(feature = "ssd1306_display_driver")]
    let (display, palette) = {
        // Create a driver instance and initialize:
        let mut display = ssd1306::Ssd1306::new(display_interface, display_size, display_rotation)
            .into_buffered_graphics_mode();
        display.init().unwrap();
        display.set_display_on(true).unwrap();
        display
            .set_brightness(ssd1306::prelude::Brightness::NORMAL)
            .unwrap();

        let palette = ColorPalette {
            splash_bg: BinaryColor::Off,
            text_color: BinaryColor::On,
            clear_color: BinaryColor::Off,
        };

        (display, palette)
    };

    info!("Display initialized.");

    // -- MAIN APP START
    #[cfg(feature = "st7789_display_driver")]
    let blit_buffer = BlitBuffer {
        buffer: vec![0u16; properties.display_width * properties.display_height],
    };

    #[cfg(feature = "ssd1306_display_driver")]
    let blit_buffer = BlitBuffer {};

    info!("Framebuffer allocated.");

    app::run(
        display,
        palette,
        properties,
        blit_buffer,
        || delay.delay_ms(2000),
        input,
        |data| {
            if data.len() > 0 {
                unsafe {
                    flash_range_program(SAVE_DATA.as_ptr() as _, data.as_ptr(), data.len());
                }
            }
        },
    )
}
