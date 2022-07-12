#[cfg(feature = "st7789_display_driver")]
use crate::bsp::hal::Clock;
use crate::bsp::{
    hal::{
        clocks::{init_clocks_and_plls, ClockSource, ClocksManager},
        pll::{common_configs::PLL_USB_48MHZ, setup_pll_blocking, PLLConfig},
        watchdog::Watchdog,
        xosc::setup_xosc_blocking,
    },
    pac::{CLOCKS, PLL_SYS, PLL_USB, RESETS, XOSC},
    XOSC_CRYSTAL_FREQ,
};
use embedded_time::rate::{Extensions, Megahertz};

/// Overclocked PLL_SYS configuration
/// 2x overclock
pub const PLL_SYS_FAST: PLLConfig<Megahertz> = PLLConfig {
    vco_freq: Megahertz(1500),
    refdiv: 1,
    post_div1: 3, // orig: 6
    post_div2: 2,
};

/// Configures the Pico's clocks with an overclocked PLL_SYS
pub fn configure_overclock(
    xosc_dev: XOSC,
    clocks_dev: CLOCKS,
    pll_sys_dev: PLL_SYS,
    pll_usb_dev: PLL_USB,
    resets: &mut RESETS,
    watchdog: &mut Watchdog,
) -> ClocksManager {
    watchdog.enable_tick_generation(XOSC_CRYSTAL_FREQ as u8);

    let mut clocks = ClocksManager::new(clocks_dev);
    // Enable the xosc
    let xosc = setup_xosc_blocking(xosc_dev, XOSC_CRYSTAL_FREQ.Hz())
        .map_err(|_e| "XOSC fail")
        .unwrap();

    // Configure PLLs
    //                   REF     FBDIV VCO            POSTDIV
    // PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHZ / 6 / 2 = 125MHz
    // PLL USB: 12 / 1 = 12MHz * 40  = 480 MHz / 5 / 2 =  48MHz
    let pll_sys = setup_pll_blocking(
        pll_sys_dev,
        xosc.operating_frequency().into(),
        PLL_SYS_FAST,
        &mut clocks,
        resets,
    )
    .map_err(|_e| "SYS fail")
    .unwrap();

    let pll_usb = setup_pll_blocking(
        pll_usb_dev,
        xosc.operating_frequency().into(),
        PLL_USB_48MHZ,
        &mut clocks,
        resets,
    )
    .map_err(|_e| "USB fail")
    .unwrap();

    // Configure clocks
    // CLK_REF = XOSC (12MHz) / 1 = 12MHz
    clocks
        .reference_clock
        .configure_clock(&xosc, xosc.get_freq())
        .map_err(|_e| "XOSC config fail")
        .unwrap();

    // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
    clocks
        .system_clock
        .configure_clock(&pll_sys, pll_sys.get_freq())
        .map_err(|_e| "SYS config fail")
        .unwrap();

    // CLK USB = PLL USB (48MHz) / 1 = 48MHz
    clocks
        .usb_clock
        .configure_clock(&pll_usb, pll_usb.get_freq())
        .map_err(|_e| "USB config fail")
        .unwrap();

    // CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
    clocks
        .adc_clock
        .configure_clock(&pll_usb, pll_usb.get_freq())
        .map_err(|_e| "ADC clock fail")
        .unwrap();

    // CLK RTC = PLL USB (48MHz) / 1024 = 46875Hz
    clocks
        .rtc_clock
        .configure_clock(&pll_usb, 46875u32.Hz())
        .map_err(|_e| "RTC clock fail")
        .unwrap();

    // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
    // Normally choose clk_sys or clk_usb
    clocks
        .peripheral_clock
        .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
        .map_err(|_e| "Peripheral clock fail")
        .unwrap();

    clocks
}

/// Configures default Pico clocks.
pub fn configure_default_clocks(
    xosc_dev: XOSC,
    clocks_dev: CLOCKS,
    pll_sys_dev: PLL_SYS,
    pll_usb_dev: PLL_USB,
    resets: &mut RESETS,
    watchdog: &mut Watchdog,
) -> ClocksManager {
    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        xosc_dev,
        clocks_dev,
        pll_sys_dev,
        pll_usb_dev,
        resets,
        watchdog,
    )
    .ok()
    .unwrap();

    clocks
}
