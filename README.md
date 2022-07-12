OxidGB Pico Port
================

This is a simple, display-only (*for now*) port of [OxidGB](https://github.com/j-selby/oxidgb) to the 
[Raspberry Pi Pico](https://www.raspberrypi.org/documentation/rp2040/getting-started/).

Prerequisites
-------------

You will need:

- [elf2uf2](https://github.com/raspberrypi/pico-sdk/tree/master/tools/elf2uf2), 
  part of the Raspberry Pi Pico SDK (if you have the SDK available, you already
  have this!).
    - Note that the Pico SDK is not required for this - you only need elf2uf2, so
      you don't need the main ARM toolchain/etc, just this binary.
- A **nightly** Rust compiler with the `thumbv6m-none-eabi` target installed. For
  example:

```
rustup default nightly
rustup target add thumbv6m-none-eabi
```

- Pico specific tools:

```
cargo install flip-link
cargo install probe-run
```

Building
--------

First, take your ROM of choice and put it in the root of the directory.

In `main.rs`, change your settings as required, including pins if required.

In your prompt of choice:

### Pico Display

```bash
cargo build --target thumbv6m-none-eabi --release
elf2uf2 -v target/thumbv6m-none-eabi/release/oxidgb-pico output.uf2
```

Pins:

| Label          | Pin # | Function  | GPIO # |
|----------------|-------|-----------|--------|
| SPI0 SCK       | 24    | SPI bus   | GPIO18 |
| SPI0 TX (MOSI) | 25    | SPI bus   | GPIO19 |
| SPI0 RX (DC)   | 21    | SPI bus   | GPIO16 |
| SPI0 CS        | 22    | SPI bus   | GPIO17 |
| GPIO20         | 26    | Backlight | GPIO20 |
| GPIO12         | 16    | A Button  | GPIO12 |
| GPIO13         | 17    | B Button  | GPIO13 |
| GPIO14         | 19    | START     | GPIO14 |
| GPIO15         | 20    | SELECT    | GPIO15 |

### Thumby (Untested!)

Note that for the Thumby you will need to load the output.uf2 file provided as
custom firmware. See the [Thumby docs](https://thumby.us/FAQ/) for this.

```bash
cargo build --target thumbv6m-none-eabi --release --no-default-features --features thumby
elf2uf2 -v target/thumbv6m-none-eabi/release/oxidgb-pico output.uf2
```

Running
-------

Flash `output.uf2` to your device as normal, and cross your fingers!

Development
-----------

If you have a [suitable debug probe](https://github.com/rp-rs/rp2040-project-template/blob/main/debug_probes.md)
attached, you should be able to directly run the port:

```bash
cargo run --target thumbv6m-none-eabi
```

This will flash the code to your device. You can also alternatively develop
straight from VSCode - see <https://github.com/rp-rs/rp2040-project-template#alternative-runners>
for instructions.

License
-------

Oxidgb is licensed under the MIT license. This can be found [here](LICENSE).
