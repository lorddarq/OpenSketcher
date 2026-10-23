# OpenSketcher

A Rust desktop application aiming to offer approachable 3D modeling with functionality overlapping SketchUp. Independent project; no affiliation with SketchUp or Trimble.

## Current state

Native editor shell with a working wgpu 3D viewport: depth-tested shaded reference cube, edges, ground grid, and colored world axes. Geometry editing, picking, project persistence, and import/export are not implemented yet. The cube is demonstration geometry, not an editable project body.

The app uses egui/eframe with its wgpu backend. Keep the model independent of UI and GPU resources when modeling is introduced; split modules only when there is code to put in them.

## Viewport controls

- Click **Isometric**, **Top**, or **Side** for an immediate orthographic view. Top looks down Z with X right and Y up; Side looks along X with Z up. Switching views preserves pan and zoom.
- Left-drag to orbit freely. The active view changes to **Free orbit** after orbiting.
- Shift-left-drag, middle-drag, or right-drag to pan; scroll over the viewport to zoom.
- Click **Fit reference** to restore the initial camera, center, and zoom.

The first viewport uses orthographic projection throughout; perspective is future work. Its offscreen color/depth attachments resize with the viewport and display through egui. No extra rendering dependency was added: it uses eframe's wgpu integration.

Camera basis and isometric foreshortening have a runnable regression check in `cargo test`. To review interactions, run the app, switch all three presets, orbit, pan, zoom, resize the window, and reset the camera. Native macOS rendering and preset/orbit/zoom interactions were checked during implementation; Windows and Linux runtime behavior still needs platform verification.

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

The initial goals are modeling, scene management, texture-only materials, and barebones viewport rendering. Start with basic shape creation/editing, push/pull, straight profile extrusion, and axis snapping. The [product scope](docs/product-scope.md) records delivery order, initial assumptions, and the acceptance workflow. Application icons will use [Tabler Icons](https://tabler.io/icons).

The [soft-selection specification](docs/specs/soft-selection.md) defines behavior for node falloff, deformation previews, heatmap feedback, and editable project persistence.

Resolve the topology and document format around the first modeling workflow before adding more tools. SketchUp file compatibility is not assumed.

Projects will use a custom native Save/Load format that retains editable shape parameters, modeling features, and dependencies. Render meshes alone are not the saved model. See the [persistence requirements](docs/product-scope.md#project-saveload-and-native-format).

Truck is the selected geometry kernel. The [parametric face modeling research](docs/research/parametric-modeling-kernels.md) records the comparison and implementation validation cases, including boring and separate emerging bodies. Kernel integration is not implemented yet.

## License

The intended direction is attribution-based open source. A license has not yet been selected; choose one and add its full text before public distribution. MIT or Apache-2.0 are candidates if reuse with preserved notices is the intended requirement.
