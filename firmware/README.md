[![Travis](https://travis-ci.com/richardeoin/alakol.svg?branch=master)](https://travis-ci.com/richardeoin/alakol)

_Alakol_ firmware
======

## Getting Started

You'll need to [install Rust](https://www.rust-lang.org/tools/install). If
you use Rustup, it makes the next steps easier.

Secondly, you will need a linker. If you're not really sure about that, on
Debian/Ubuntu/Rasperry Pi try:

```
sudo apt-get update && sudo apt-get install build-essential
```

Then in this directory, you can issue the rustup incantations:

```
cd ./alakol/firmware
rustup toolchain install nightly
rustup target add thumbv7em-none-eabihf
rustup target add thumbv7em-none-eabihf --toolchain nightly
rustup run nightly cargo build -Z features=build_dep
```

The last line will take some minutes, there are borrowers working. The
build output will be in `target/thumbv7em-none-eabihf/debug`

If your programmer requires a `*.bin` file, then you can use
[cargo-binutils](https://github.com/rust-embedded/cargo-binutils)

```
# Won't work whilst we need -Z flags, see below
rustup run nightly cargo objcopy --bin alakol -- -O binary target/thumbv7em-none-eabihf/debug/alakol.bin
```

## Building

Unfortunately we still need nightly flags, due to cargo [#7915](https://github.com/rust-lang/cargo/issues/7915)

```
cargo build -Z features=build_dep
```

The build output in release mode is much nicer

```
cargo build --release -Z features=build_dep
```

# License

[Apache v2.0](LICENSE.md)
