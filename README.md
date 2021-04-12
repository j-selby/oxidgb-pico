OxidGB Pico Port
================

This is a simple, display-only (*for now*) port of [OxidGB](https://github.com/j-selby/oxidgb) to the 
[Raspberry Pi Pico](https://www.raspberrypi.org/documentation/rp2040/getting-started/).

Note that this by default **overclocks your device** - you have been warned! 

Prerequisites
-------------

You will need:

- [elf2uf2](https://github.com/raspberrypi/pico-sdk/tree/master/tools/elf2uf2), 
  part of the Raspberry Pi Pico SDK (if you have the SDK available, you already
  have this!).
    - Note that the Pico SDK is not required for this - you only need elf2uf2, so
      you don't need the main ARM toolchain/etc, just this binary.
- A **nightly** Rust compiler with the `thumbv6m-none-eabi` installed target. For
  example:

```
rustup default nightly
rustup target add thumbv6m-none-eabi
```


Building
--------

First, take your ROM of choice and put it in the root of the directory.

In `main.rs`, change your settings as required (including **overclocking** values).

In your prompt of choice:

```bash
cargo build --target thumbv6m-none-eabi --release
elf2uf2 -v target/thumbv6m-none-eabi/release/oxidgb-pico output.uf2
```

Running
-------

Flash `output.uf2` to your device as normal, and cross your fingers!

License
-------

Oxidgb is licensed under the MIT license. This can be found [here](LICENSE).
