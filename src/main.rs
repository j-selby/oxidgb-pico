//! A GameBoy emulator for the Raspberry Pi Pico...

#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

// ==========================================
// CONFIGURATION SECTION
// ==========================================

/// Path to your game data, relative to the `src/` directory.
const GAME_DATA: &'static [u8] = include_bytes!("../kirby.gb");

// ==========================================
// RUNTIME SECTION
// ==========================================

use app::{ColorPalette, DisplayProperties};

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

#[macro_use]
extern crate alloc;

use alloc_cortex_m::CortexMHeap;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    self,
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use embedded_hal::digital::v2::OutputPin;

use bsp::entry;

use embedded_time::rate::units::Extensions;

use core::fmt::Debug;
use core::mem::MaybeUninit;

use embedded_graphics_core::{pixelcolor::Rgb565, prelude::RgbColor};

mod app;

/// Provides an interface to directly blit pixels to the screen.
pub trait AcceleratedBlit {
    /// Stolen from ST7789:
    ///
    /// Sets pixel colors in given rectangle bounds.
    ///
    /// # Arguments
    ///
    /// * `sx` - x coordinate start
    /// * `sy` - y coordinate start
    /// * `ex` - x coordinate end
    /// * `ey` - y coordinate end
    /// * `colors` - anything that can provide `IntoIterator<Item = u16>` to iterate over pixel data
    ///
    fn set_pixels<T>(&mut self, sx: u16, sy: u16, ex: u16, ey: u16, colors: T)
    where
        T: IntoIterator<Item = u16>;
}

#[cfg(feature = "st7789_display_driver")]
impl<DI, OUT> AcceleratedBlit for st7789::ST7789<DI, OUT>
where
    DI: display_interface::WriteOnlyDataCommand,
    OUT: OutputPin,
    OUT::Error: Debug,
{
    fn set_pixels<T>(&mut self, sx: u16, sy: u16, ex: u16, ey: u16, colors: T)
    where
        T: IntoIterator<Item = u16>,
    {
        self.set_pixels(sx, sy, ex, ey, colors).unwrap();
    }
}

static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
const HEAP_SIZE: usize = 245 * 1024;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

/// An out of memory handler
#[alloc_error_handler]
fn oom(layout: core::alloc::Layout) -> ! {
    error!("Out of memory!");
    error!("Stats: {}, {}", ALLOCATOR.used(), ALLOCATOR.free());
    error!("Attempted to allocate: {}", layout.size());

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

#[entry]
fn main() -> ! {
    // -- RP2040 INIT

    // Setup the allocator with a conservative heap.
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, HEAP_SIZE) }

    let mut pac = pac::Peripherals::take().unwrap();
    let cp = pac::CorePeripherals::take().unwrap();

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
    let (properties, display_interface, bl) = {
        // Configure pins
        // These are implicitly used by the spi driver if they are in the correct mode
        let _spi_sclk = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>();
        let _spi_mosi = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>();
        // We're using what would normally be the miso pin for data/command
        let dc = pins.gpio16.into_push_pull_output();
        // Chip select
        let cs = pins.gpio17.into_push_pull_output();
        // Backlight
        let bl = pins.gpio21.into_push_pull_output();

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

        (
            properties,
            display_interface_spi::SPIInterface::new(spi, dc, cs),
            bl,
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
            splash_bg: Rgb565::new(50, 50, 180),
            text_color: Rgb565::WHITE,
            clear_color: Rgb565::BLACK,
        };

        (display, palette)
    };

    // -- MAIN APP START
    app::run(display, palette, properties, || delay.delay_ms(2000))
}
