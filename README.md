# OpenSketcher

A Rust desktop application aiming to offer approachable 3D modeling with functionality overlapping SketchUp. Independent project; no affiliation with SketchUp or Trimble.

## Current state

Native editor with a depth-tested wgpu viewport, ground grid, world axes, and Truck-backed parametric geometry. Create rectangles, circles, lines, and semicircles using two picked points on the XY plane. Select objects in the viewport or model list, then edit dimensions, extrusion, position, rotation, and scale with automatic updates. Delete and undo/redo are supported.

Projects save and load as `.osk` files, preserving editable geometry parameters and scene properties. Import/export to other formats, solid boring, face-level push/pull, node editing, and transform gizmos are not implemented yet. There is no longer a demonstration cube.

The app uses egui/eframe with its wgpu backend. Keep the model independent of UI and GPU resources when modeling is introduced; split modules only when there is code to put in them.

## Viewport controls

- Click **Isometric**, **Top**, or **Side** for an immediate orthographic view. Top looks down Z with X right and Y up; Side looks along X with Z up. Switching views preserves pan and zoom.
- Choose **Orbit**, then left-drag to orbit freely. The active view changes to **Free orbit** after orbiting.
- Shift-left-drag, middle-drag, or right-drag to pan; scroll over the viewport to zoom.
- Click **Fit model** to frame all geometry while preserving the viewing direction.

The first viewport uses orthographic projection throughout; perspective is future work. Its offscreen color/depth attachments resize with the viewport and display through egui. No extra rendering dependency was added: it uses eframe's wgpu integration.

Camera basis and isometric foreshortening have a runnable regression check in `cargo test`. To review interactions, run the app, switch all three presets, orbit, pan, zoom, resize the window, and reset the camera. Native macOS rendering, all four two-point creation gestures, rectangle preview, line picking, automatic extrusion/movement/rotation/scale edits, and undo/redo were checked during implementation; Windows and Linux runtime behavior still needs platform verification.

## Creation and editing

1. Choose **Rectangle**, **Circle**, **Line**, or **Semicircle**, then click two points on the XY construction plane. Use Top or Isometric; Side views the plane edge-on.
2. Rectangles use opposite corners; circles use center then radius; lines use endpoints; semicircles use diameter endpoints. Semicircles create a filled half-disc to the left of the directed diameter.
3. The first point anchors a green outline preview. The second commits one creation action. Escape cancels the preview without adding geometry. Snapping is on by default: 0.5-unit grid plus X/Y alignment near the first point. This does not yet include existing-geometry endpoint snapping.
4. Choose **Select** and click an object, or use the model list. Selected geometry is highlighted. Dimension, extrusion, position, Euler rotation, and local-axis scale values regenerate geometry automatically—there is no Apply or Revert Fields action.
5. Extrusion follows the profile's local +Z. Filled profiles create solids; lines create surfaces. Rotation and scaling pivot at the profile center (the center of a semicircle's diameter).
6. Use **Undo / Redo** or Cmd/Ctrl+Z and Cmd/Ctrl+Shift+Z. A continuous numeric drag or text edit is grouped as one undo action. Delete selected geometry with the inspector button or Delete/Backspace outside a text field. History retains up to 100 actions per session.

Input validation rejects invalid dimensions or geometry and keeps the last valid object. Current supported dimensions are 0.01–100 units, extrusion zero or 0.01–100, and positive scale 0.01–100. Negative extrusion and mirrored scaling are not yet supported. Shapes are independent; overlapping objects are not automatically fused.

The document stores authored parameters independently of GPU data. One Truck evaluator serves creation, live edits, and previews. Tessellated faces and sampled boundary edges feed rendering and picking; editing does not modify triangles directly. The regression check exercises all four profiles, two-point dimensions, extrusion, transformations, grouped live-edit history, invalid-input rejection, and deletion recovery.

## Save and open projects

Use **Save** (Cmd/Ctrl+S), **Save As** (Cmd/Ctrl+Shift+S), **Open** (Cmd/Ctrl+O), and **New** (Cmd/Ctrl+N). Files use the `.osk` extension and preserve object IDs, shape dimensions, extrusion, position, rotation, scale, camera, selection, and snapping. Loaded objects remain editable; the viewport mesh is regenerated with Truck.

An asterisk marks unsaved scene changes. New/Open/Close prompt before discarding them. Canceled or failed operations preserve the active document; saves replace the destination atomically. Session undo history and unfinished previews are not saved. See the [versioned .osk format](docs/formats/osk.md) for the encoding, limits, and validation contract.

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

Truck is the selected geometry kernel. The [parametric face modeling research](docs/research/parametric-modeling-kernels.md) records the comparison and implementation validation cases, including boring and separate emerging bodies. Truck modeling and tessellation are integrated; boring and persistent face-reference validation remain pending.

## License

The intended direction is attribution-based open source. A license has not yet been selected; choose one and add its full text before public distribution. MIT or Apache-2.0 are candidates if reuse with preserved notices is the intended requirement.
