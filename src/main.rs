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
use st7789::{Orientation, TearingEffect};

use core::cmp::min;
use core::mem::MaybeUninit;

use embedded_graphics_core::{draw_target::DrawTarget, pixelcolor::Rgb565, prelude::RgbColor};

use embedded_graphics::primitives::Rectangle;
use embedded_graphics::primitives::*;
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    prelude::*,
    text::{Baseline, Text, TextStyle},
};

use oxidgb_core::cpu::CPU;
use oxidgb_core::gpu::PITCH;
use oxidgb_core::mem::GBMemory;
use oxidgb_core::rom::GameROM;

static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
const HEAP_SIZE: usize = 245 * 1024;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[alloc_error_handler]
fn oom(layout: core::alloc::Layout) -> ! {
    error!("Out of memory!");
    error!("Stats: {}, {}", ALLOCATOR.used(), ALLOCATOR.free());
    error!("Attempted to allocate: {}", layout.size());

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

    let mut led_pin = pins.led.into_push_pull_output();
    led_pin.set_low().unwrap();

    let mut delay = cortex_m::delay::Delay::new(cp.SYST, 125_000_000);

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>();
    // We're using what would normally be the miso pin for data/command
    let dc = pins.gpio16.into_push_pull_output();
    // Chip select
    let cs = pins.gpio17.into_push_pull_output();
    // Backlight
    let bl = pins.gpio21.into_push_pull_output();

    let spi = bsp::hal::Spi::<_, _, 8>::new(pac.SPI0);

    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        //16_000_000u32.Hz(),
        32_000_000u32.Hz(),
        &embedded_hal::spi::MODE_3,
    );

    let display_interface = display_interface_spi::SPIInterface::new(spi, dc, cs);

    const DISPLAY_WIDTH: usize = 240;
    const DISPLAY_HEIGHT: usize = 135;

    const DISPLAY_X_OFFSET: usize = 40;
    const DISPLAY_Y_OFFSET: usize = 53;

    let mut display = st7789::ST7789::new(
        display_interface,
        None,
        Some(bl),
        DISPLAY_WIDTH as _,
        DISPLAY_HEIGHT as _,
    );

    // initialize
    display.init(&mut delay).unwrap();
    // set default orientation
    display.set_orientation(Orientation::Landscape).unwrap();
    display
        .set_tearing_effect(TearingEffect::HorizontalAndVertical)
        .unwrap();

    //display.set_display_backlight(&pac.PWM, 100);

    let rom = GameROM::build(GAME_DATA);

    info!("{}", rom.get_cart_name());

    // Draw a splash screen:
    let fill = PrimitiveStyle::with_fill(Rgb565::new(50, 50, 180));

    let text_style = MonoTextStyleBuilder::new()
        .font(&profont::PROFONT_24_POINT)
        .text_color(Rgb565::WHITE)
        .build();

    let small_text_style = MonoTextStyleBuilder::new()
        .font(&profont::PROFONT_10_POINT)
        .text_color(Rgb565::WHITE)
        .build();

    let text_render_style = TextStyle::with_baseline(Baseline::Top);

    let y_offset = DISPLAY_HEIGHT as i32 / 2;

    display.clear(Rgb565::BLACK).unwrap();

    // Text background:
    Rectangle::new(
        Point::new(
            0 + DISPLAY_X_OFFSET as i32,
            y_offset - 18 + DISPLAY_Y_OFFSET as i32,
        ),
        Size::new(DISPLAY_WIDTH as _, (y_offset + 18) as u32),
    )
    .into_styled(fill)
    .draw(&mut display)
    .unwrap();

    let text = "Oxidgb";
    let width = text.len() as i32 * 16;
    Text::with_text_style(
        text,
        Point::new(
            DISPLAY_WIDTH as i32 / 2 - width / 2 + DISPLAY_X_OFFSET as i32,
            DISPLAY_HEIGHT as i32 / 2 - 18 + DISPLAY_Y_OFFSET as i32,
        ),
        text_style,
        text_render_style,
    )
    .draw(&mut display)
    .unwrap();

    let text = rom.get_cart_name();
    let width = text.len() as i32 * 7;
    Text::with_text_style(
        &text,
        Point::new(
            DISPLAY_WIDTH as i32 / 2 - width / 2 + DISPLAY_X_OFFSET as i32,
            DISPLAY_HEIGHT as i32 / 2 + 50 + DISPLAY_Y_OFFSET as i32,
        ),
        small_text_style,
        text_render_style,
    )
    .draw(&mut display)
    .unwrap();

    // Build memory
    let memory = GBMemory::build(rom);

    // Build CPU
    let mut cpu = CPU::build(memory);

    // Update the screen now everything is ready:
    let text = format!("Used RAM: {}, free: {}", ALLOCATOR.used(), ALLOCATOR.free());
    let width = text.len() as i32 * 7;
    Text::with_text_style(
        &text,
        Point::new(
            DISPLAY_WIDTH as i32 / 2 - width / 2 + DISPLAY_X_OFFSET as i32,
            DISPLAY_HEIGHT as i32 / 2 - 65 + DISPLAY_Y_OFFSET as i32,
        ),
        small_text_style,
        text_render_style,
    )
    .draw(&mut display)
    .unwrap();

    let text = format!(
        "Used flash: {}",
        (GAME_DATA.as_ptr() as usize) + GAME_DATA.len() - 0x10000000
    );
    let width = text.len() as i32 * 7;
    Text::with_text_style(
        &text,
        Point::new(
            DISPLAY_WIDTH as i32 / 2 - width / 2 + DISPLAY_X_OFFSET as i32,
            DISPLAY_HEIGHT as i32 / 2 - 55 + DISPLAY_Y_OFFSET as i32,
        ),
        small_text_style,
        text_render_style,
    )
    .draw(&mut display)
    .unwrap();

    // We use the same buffer for the gameboy screen - clear it now:
    delay.delay_ms(2000);

    info!("Reached main loop!");

    let mut framebuffer = vec![0u16; DISPLAY_WIDTH * DISPLAY_HEIGHT/*  * 2*/];

    info!("Framebuffer allocated.");

    display.clear(Rgb565::BLACK).unwrap();

    loop {
        cpu.run();

        let x_max = min(DISPLAY_WIDTH, 160);
        let y_max = min(DISPLAY_HEIGHT, 144);

        let y_offset = (DISPLAY_HEIGHT - y_max) / 2;
        let x_offset = (DISPLAY_WIDTH - x_max) / 2;

        // TODO: This isn't particularly efficient - change how the runtime works?
        //       (generic framebuffer interface?)
        for y in 0..y_max {
            for x in 0..x_max {
                let offset = (y * 160 + x) * PITCH;

                let packed_rgb = ((cpu.mem.gpu.pixel_data[offset] as u16 & 0b11111000) << 8)
                    | ((cpu.mem.gpu.pixel_data[offset + 1] as u16 & 0b11111100) << 3)
                    | (cpu.mem.gpu.pixel_data[offset + 2] as u16 >> 3);

                let buffer_ptr = (y + y_offset) * DISPLAY_WIDTH + (x + x_offset);
                framebuffer[buffer_ptr] = packed_rgb;
            }
        }

        display
            .set_pixels(
                40,
                53,
                DISPLAY_WIDTH as u16 + DISPLAY_X_OFFSET as u16 - 1,
                DISPLAY_HEIGHT as u16 + DISPLAY_Y_OFFSET as u16 - 1,
                framebuffer.iter().map(|x| *x),
            )
            .unwrap();
    }
}
