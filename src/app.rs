//! The application itself, abstract from startup traits.

use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::MonoTextStyleBuilder,
    prelude::{PixelColor, Point, Primitive, Size},
    primitives::{PrimitiveStyle, Rectangle},
    text::{Baseline, Text, TextStyle},
    Drawable,
};

use core::fmt::Debug;

use defmt::*;

use crate::{AcceleratedBlit, BlitBuffer, Flushable, GAME_DATA};

use oxidgb_core::cpu::CPU;
use oxidgb_core::rom::GameROM;
use oxidgb_core::{input::GameboyInput, mem::GBMemory};

/// Colors to use with the screen.
pub struct ColorPalette<C: PixelColor> {
    pub splash_bg: C,
    pub text_color: C,
    pub clear_color: C,
}

/// Sizing properties of the display to draw onto.
pub struct DisplayProperties {
    pub display_width: usize,
    pub display_height: usize,
    pub x_offset: usize,
    pub y_offset: usize,
}

/// Runs the main application loop
///
/// * display: The display to render onto
/// * palette: Colors to use when rendering the splash screen
/// * properties: Display properties
/// * blit_buffer: Buffer used for converting core graphics
/// * delay: A timing source for sleeping before the loop starts
pub fn run<
    D: DrawTarget + AcceleratedBlit + Flushable,
    SleepFunc: FnOnce(),
    InputFunc: Fn() -> GameboyInput,
>(
    mut display: D,
    palette: ColorPalette<D::Color>,
    properties: DisplayProperties,
    mut blit_buffer: BlitBuffer,
    delay: SleepFunc,
    input: InputFunc,
) -> !
where
    D::Error: Debug,
{
    let rom = GameROM::build(GAME_DATA);

    info!("Found game: {}", rom.get_cart_name());

    // Draw a splash screen:
    let fill = PrimitiveStyle::with_fill(palette.splash_bg);

    let text_style = MonoTextStyleBuilder::new()
        .font(&profont::PROFONT_24_POINT)
        .text_color(palette.text_color)
        .build();

    let small_text_style = MonoTextStyleBuilder::new()
        .font(&profont::PROFONT_10_POINT)
        .text_color(palette.text_color)
        .build();

    let text_render_style = TextStyle::with_baseline(Baseline::Top);

    let y_offset = properties.display_height as i32 / 2;

    display.clear(palette.clear_color).unwrap();

    // Text background:
    Rectangle::new(
        Point::new(
            0 + properties.x_offset as i32,
            y_offset - 18 + properties.y_offset as i32,
        ),
        Size::new(properties.display_width as _, (y_offset + 18) as u32),
    )
    .into_styled(fill)
    .draw(&mut display)
    .unwrap();

    let text = "Oxidgb";

    let width = text.len() as i32
        * if properties.display_width < 100 {
            7
        } else {
            16
        };
    Text::with_text_style(
        text,
        Point::new(
            properties.display_width as i32 / 2 - width / 2 + properties.x_offset as i32,
            properties.display_height as i32 / 2 - 18 + properties.y_offset as i32,
        ),
        if properties.display_width < 100 {
            small_text_style
        } else {
            text_style
        },
        text_render_style,
    )
    .draw(&mut display)
    .unwrap();

    let text = rom.get_cart_name();
    let width = text.len() as i32 * 7;
    Text::with_text_style(
        &text,
        Point::new(
            properties.display_width as i32 / 2 - width / 2 + properties.x_offset as i32,
            properties.display_height as i32 / 2 + 50 + properties.y_offset as i32,
        ),
        small_text_style,
        text_render_style,
    )
    .draw(&mut display)
    .unwrap();

    display.flush();

    // Build memory
    let memory = GBMemory::build(rom);

    // Build CPU
    let mut cpu = CPU::build(memory);

    // We use the same buffer for the gameboy screen - clear it now:
    delay();

    info!("Reached main loop!");

    display.clear(palette.clear_color).unwrap();

    loop {
        cpu.mem.buttons = input();
        cpu.run();

        display.set_pixels(&cpu.mem.gpu, &properties, &mut blit_buffer);
        display.flush();
    }
}
