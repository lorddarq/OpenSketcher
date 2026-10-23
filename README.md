# OpenSketcher

A Rust desktop application aiming to offer approachable 3D modeling with functionality overlapping SketchUp. Independent project; no affiliation with SketchUp or Trimble.

## Current state

Native application scaffold with an editor header, model sidebar, and placeholder viewport. Geometry editing, a 3D renderer, file persistence, and import/export are not implemented yet.

The app uses egui/eframe with its wgpu backend. Keep the model independent of UI and GPU resources when modeling is introduced; split modules only when there is code to put in them.

## Run

Install Rust using [rustup](https://rustup.rs/), then:

```sh
cargo run
```

The toolchain file selects Rust 1.92.0. Windows requires the Visual Studio C++ build tools and Windows SDK; macOS requires Xcode command line tools. On Ubuntu, install the native development dependencies:

```sh
sudo apt-get install build-essential pkg-config libx11-dev libxi-dev libxcursor-dev libxrandr-dev libxinerama-dev libxkbcommon-dev libwayland-dev libudev-dev libegl1-mesa-dev
```

## Check and build

```sh
cargo fmt --check
cargo clippy --all-targets -- -D warnings
cargo test
cargo build --release
```

Commit Cargo.lock after the first successful Cargo build to pin transitive dependencies. It is intentionally not ignored.

## Desktop distribution

The desktop workflow checks and builds on macOS, Windows, and Linux. Downloadable artifacts contain a macOS `.app`, a Windows `.exe`, or a Linux executable. These are unsigned development packages, not installers. macOS signing/notarization, Windows signing, Linux desktop integration, application icons, and release publishing remain future work. Build each platform on its own runner; no cross-compilation is assumed.

## First modeling milestone

Build one complete interaction: orbit/pan/zoom a perspective view, draw a dimensioned rectangle with snapping, push/pull it into a solid, undo/redo, and save/reopen the document. Resolve the topology and document format around that workflow before adding more tools. SketchUp file compatibility is not assumed.

## License

The intended direction is attribution-based open source. A license has not yet been selected; choose one and add its full text before public distribution. MIT or Apache-2.0 are candidates if reuse with preserved notices is the intended requirement.
