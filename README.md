# OpenSketcher

A Rust desktop application aiming to offer approachable 3D modeling with functionality overlapping SketchUp. Independent project; no affiliation with SketchUp or Trimble.

## Current state

Native editor with a depth-tested wgpu viewport, ground grid, world axes, and Truck-backed parametric geometry. Create rectangles, circles, lines, and semicircles using two picked points on the XY plane. Choose Volume, Face, Edge, or Point selection in the toolbar. Volume selection exposes automatic dimension, extrusion, position, rotation, and scale edits; component modes pick and highlight individual topology elements. Move and Rotate expose constrained gizmos in all four selection modes. Whole-object deletion and undo/redo are supported.

Projects save and load as `.osk` files, preserving editable geometry parameters and scene properties. Import/export to other formats, solid boring, face-level push/pull, node editing, and transform gizmos are not implemented yet. There is no longer a demonstration cube.

## Feature checklist

Checked items are implemented; platform runtime verification is tracked separately below. Unchecked items are pending, including behavior that currently exists only in a specification.

### Creation and modeling

- [x] Canvas creation on the XY plane: rectangle corners, circle center/radius, line endpoints, semicircle diameter endpoints.
- [x] Live creation preview and Escape cancellation.
- [x] Default-on 0.5-unit grid snapping and X/Y alignment to the creation anchor.
- [x] Automatic numeric dimension, position, Euler rotation, and local scale editing.
- [x] Straight local +Z extrusion: closed profiles to solids, lines to surfaces.
- [x] Coplanar closed-profile partitioning: holes, edge notches, disconnected remaining regions, and new cut walls.
- [x] Move and Rotate gizmos in Volume, Face, Edge, and Point modes: world-axis arrows, move-plane handles, rotation rings, configurable pivot, snapping, and cancelable previews.
- [x] Persisted component transforms with adjacent-boundary rebuilding and topology-reference validation.
- [x] Whole-object deletion and grouped session undo/redo.
- [ ] Regular polygons and additional arc/freehand/rotated-rectangle tools.
- [ ] Circle Radius field and radius preview label (canvas input is already center/radius; properties still display diameter).
- [ ] Face-first sketching on a selected support face, including elevated faces and contextual reference points.
- [ ] Open-line face partitioning and exact analytic curved intersections (split curves currently use polygon approximations).
- [ ] Dedicated face-normal push/pull and component scaling (component movement/rotation are implemented).
- [ ] Negative extrusion, mirrored scaling, and a canvas Scale gizmo.
- [ ] Explicit X/Y/Z axis locks and labeled axis-inference feedback during drawing/moving.
- [ ] Existing-geometry endpoint, midpoint, face, and intersection snapping.
- [ ] Borer-only and generator-plus-borer operations, including pockets, through-cuts, and separate emerging bodies.
- [ ] Persistent semantic face/edge references for dependent modeling features.
- [ ] Whole-face NURBS/deformation-preserving movement and control-node soft selection with falloff, radius, and heatmap.

### Selection, viewport, and workspace

- [x] Volume, Face, Edge, and Point picking/highlighting, with depth-aware component selection.
- [x] Barebones GPU viewport: shaded faces, boundary edges, depth testing, grid, and world axes.
- [x] Isometric, Top, Side, free orbit, pan, zoom, fit model, and viewport resizing.
- [x] Floating draggable Tools and Model/properties panels; scrollable properties.
- [ ] Perspective projection.
- [ ] Scene management beyond the current document/model list; saved-view versus separate-scene behavior needs definition.
- [ ] Texture material loading, assignment, mapping, rendering, and portable project assets.
- [ ] Tabler icon assets and accessible icon buttons (current tools use text labels).

### Projects and distribution

- [x] Editable `.osk` Save/Load, New, Save As, native dialogs, and keyboard shortcuts.
- [x] Parameter/camera/selection/snapping persistence and regenerated coplanar regions.
- [x] Version checks/migration, input validation, atomic saves, dirty tracking, and unsaved-change prompts.
- [x] Runnable geometry, camera, selection, transform/history, and persistence regression checks.
- [x] macOS/Windows/Linux build and development-package workflow (unsigned executables; macOS app bundle).
- [x] Native macOS review of core creation/editing, viewport, persistence, and split-extrusion workflows.
- [ ] Windows and Linux native runtime verification.
- [ ] Model import/export and SketchUp file compatibility.
- [ ] Signed/notarized releases, installers, Linux desktop integration, application icons, and release publishing.

## Editor layout

The canvas fills the workspace behind floating panels. Tools, snapping, selection modes, and view presets start in a left overlay; the model list and properties start in a right overlay. Drag their title bars to reposition them within the canvas. Properties scroll vertically when content exceeds the available panel height; scrolling over a panel does not zoom the viewport. Panel positions are transient workspace UI state, not scene geometry or `.osk` properties.

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

Input validation rejects invalid dimensions or geometry and keeps the last valid object. Current supported dimensions are 0.01–100 units, extrusion zero or 0.01–100, and positive scale 0.01–100. Negative extrusion and mirrored scaling are not yet supported. Objects are not automatically fused. Newer coplanar closed profiles partition older profiles before extrusion, as described below.

The document stores authored parameters independently of GPU data. One Truck evaluator serves creation, live edits, and previews. Tessellated faces and sampled boundary edges feed rendering and picking; editing does not modify triangles directly. The regression check exercises all four profiles, two-point dimensions, extrusion, transformations, grouped live-edit history, invalid-input rejection, and deletion recovery.

## Selection modes

- **Volume:** select the whole object, including standalone profiles and lines. Whole-object transforms and deletion are available here.
- **Face:** select one kernel face across all its display triangles.
- **Edge:** select one topology edge across its sampled curve segments. Internal triangulation diagonals are not selectable edges.
- **Point:** show and pick visible topology vertices. These are not tessellation samples or NURBS deformation controls.

Face, Edge, and Point support Move/Rotate gizmos for the selected component; adjacent boundaries rebuild to remain connected. Whole-object numeric properties and deletion remain in Volume mode. Changing mode clears the previous selection but keeps Move/Rotate active. Completed component transforms retain their selection, and undo/redo preserves it when its topology slot still exists. Empty-space clicks clear selection within the current mode. Occluded components are not selectable through opaque faces. Edge/point acquisition uses screen-space tolerances so zoom does not shrink the click target.

`.osk` schema/modeling version 3 also preserves component-transform matrices and guarded topology references. Source connectivity must match before replay; an upstream edit that changes it is rejected rather than targeting another component. Files migrated from modeling version 1 reset component selection to Volume because partitioning changes topology slots. This guard is not a general semantic naming system for arbitrary CAD feature histories.

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

Cargo.lock pins the dependency versions; keep it committed when updating dependencies.

## Desktop distribution

The desktop workflow checks and builds on macOS, Windows, and Linux. Downloadable artifacts contain a macOS `.app`, a Windows `.exe`, or a Linux executable. These are unsigned development packages, not installers. macOS signing/notarization, Windows signing, Linux desktop integration, application icons, and release publishing remain future work. Build each platform on its own runner; no cross-compilation is assumed.

## First modeling milestone

The initial goals are modeling, scene management, texture-only materials, and barebones viewport rendering. Start with basic shape creation/editing, push/pull, straight profile extrusion, and axis snapping. The [product scope](docs/product-scope.md) records delivery order, initial assumptions, and the acceptance workflow. Application icons will use [Tabler Icons](https://tabler.io/icons).

The [soft-selection specification](docs/specs/soft-selection.md) defines behavior for node falloff, deformation previews, heatmap feedback, and editable project persistence.

The planned [face-first sketching workflow](docs/specs/face-sketching.md) requires selecting a supporting face before drawing on it, exposing contextual references, and displaying circle radius consistently. The [Plasticity/SketchUp research](docs/research/face-sketching-workflows.md) separates documented product behavior from OpenSketcher's chosen rules. Current creation remains XY-only until this work is implemented.

Resolve the topology and document format around the first modeling workflow before adding more tools. SketchUp file compatibility is not assumed.

Projects use the custom native `.osk` format to retain currently implemented editable shape parameters and scene properties. Future dependent features must preserve their authored inputs and references too. Render meshes alone are not the saved model. See the [persistence requirements](docs/product-scope.md#project-saveload-and-native-format).

Truck is the selected geometry kernel. The [parametric face modeling research](docs/research/parametric-modeling-kernels.md) records the comparison and implementation validation cases, including boring and separate emerging bodies. Truck modeling and tessellation are integrated; boring and persistent face-reference validation remain pending.

## Dependencies and attribution

Versions below are resolved from Cargo.lock. The full direct/transitive inventory, upstream copyright notices, license texts, and embedded-font notices are preserved in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). Some locked packages are platform-specific, optional, or build-time dependencies and are not shipped in every executable.

| Direct dependency | Locked version | Used for | Upstream credit | Declared license |
| --- | --- | --- | --- | --- |
| [eframe](https://github.com/emilk/egui) | 0.34.3 | Native editor shell, egui UI, wgpu integration, bundled fonts | Emil Ernerfeldt and egui contributors | MIT OR Apache-2.0; embedded fonts have separate terms below |
| [rfd](https://github.com/PolyMeilex/rfd) | 0.17.2 | Native file and confirmation dialogs | Copyright © 2022 Bartłomiej Maryńczak | MIT |
| [serde](https://github.com/serde-rs/serde) | 1.0.229 | Scene serialization traits and derives | Erick Tryzelaar, David Tolnay, and Serde contributors | MIT OR Apache-2.0 |
| [serde_json](https://github.com/serde-rs/json) | 1.0.154 | `.osk` JSON encoding/decoding | serde_json contributors | MIT OR Apache-2.0 |
| [tempfile](https://github.com/Stebalien/tempfile) | 3.27.0 | Same-directory temporary files for atomic saves | Copyright © 2015 Steven Allen; tempfile contributors | MIT OR Apache-2.0 |
| [truck-modeling](https://github.com/ricosjp/truck) | 0.6.0 | Parametric profiles, topology, transforms, and extrusion | RICOS Co. Ltd. and Truck contributors | Apache-2.0 |
| [truck-meshalgo](https://github.com/ricosjp/truck) | 0.4.0 | Tessellation for rendering and picking | RICOS Co. Ltd. and Truck contributors | Apache-2.0 |

Key transitive dependencies include **egui 0.34.3** (UI; MIT OR Apache-2.0), **wgpu 29.0.4** (GPU rendering; MIT OR Apache-2.0), **winit 0.30.13** (native windows/input; Apache-2.0), **Spade 2.15.1** (Truck's constrained triangulation; MIT OR Apache-2.0), and Truck's geometry/topology/math crates (Apache-2.0). The complete inventory is in the notices file rather than duplicating it here.

The enabled `epaint_default_fonts` dependency embeds these font assets. Their own licenses continue to apply:

| Embedded font | Attribution | License |
| --- | --- | --- |
| Hack Regular | Copyright © 2018 Source Foundry Authors; Bitstream Vera Sans Mono copyright © 2003 Bitstream, Inc.; DejaVu contributions are public domain | MIT and Bitstream Vera License |
| Noto Emoji Regular | Copyright © 2013 Google Inc.; Monotype Imaging Inc. / Monotype Design Team; Noto is a trademark of Google Inc. | SIL Open Font License 1.1 |
| Ubuntu Light | Copyright © 2011 Canonical Ltd.; Dalton Maag Ltd.; Ubuntu and Canonical are registered trademarks of Canonical Ltd. | Ubuntu Font Licence 1.0 |
| emoji-icon-font | Copyright © 2014 John Slegers | MIT |

[Tabler Icons](https://github.com/tabler/tabler-icons) is the chosen future icon source, not a currently bundled dependency or asset. When icons are added, preserve their upstream MIT license and copyright notice in the distribution.

Desktop packages include `LICENSE`, `NOTICE`, and `THIRD_PARTY_NOTICES.md` (inside macOS app resources, beside executables on Windows/Linux). Preserve applicable upstream copyright, attribution, license, and notice text when redistributing; credits in this table do not replace full notices. See [Apache-2.0 Section 4](https://www.apache.org/licenses/LICENSE-2.0) and the [MIT license terms](https://opensource.org/license/mit).

After a dependency update, run:

```sh
cargo fetch --locked
python3 scripts/update-third-party-notices.py
python3 scripts/update-third-party-notices.py --check
```

The generator uses Python's standard library, the Cargo cache, and pinned upstream notice snapshots in `third-party/upstream-notices.json`. New dependencies that omit license files need their exact upstream notice sources added there. It also extracts copyright and license records from the actual embedded fonts. Review new license obligations when updating dependencies; generation is notice collection, not legal approval.

## License

OpenSketcher is licensed under the [Apache License 2.0](LICENSE). Redistributions must preserve applicable attribution and copyright notices, include the license, and retain the attribution in [NOTICE](NOTICE) as required by Section 4. Modified files must carry prominent change notices. No visible UI credit is required.

Dependencies and third-party assets retain their own licenses.

## Canvas Move and Rotate

Select a Volume, Face, Edge, or Point, then choose **Move** or **Rotate**. You can also select geometry while either tool is active. Drag a colored handle and release to commit one undoable action; Escape cancels the preview. Switching tools or views and losing window focus also cancels an unfinished drag.

Move arrows constrain motion to world X (red), Y (green), or Z (blue). Small plane handles constrain motion to XY, YZ, or ZX. Rotate rings constrain rotation to a world axis and compose correctly with an existing object rotation. The highlighted handle and live numeric readout show the constraint. Handles retain a usable screen size as you zoom. An axis viewed end-on uses vertical dragging; edge-on rotation rings use a screen-drag fallback.

The pivot starts at the selected geometry's bounding-box center (the selected vertex for Point mode). **Alt-click** the canvas to place an offset pivot on the view-aligned plane through the current pivot; **Reset pivot** restores the automatic center. A point rotated around itself stays still, so place an offset pivot to rotate it. Pivot placement is transient and does not change authored geometry.

Snap defaults on: move offsets use 0.5-unit steps and rotation offsets use 15-degree steps. Turn Snap off for continuous changes. Volume numeric properties remain available for exact values.

Component edits retain the source profile plus ordered local transform matrices in `.osk`; they do not save a baked mesh. Selected faces/edges move rigidly, while shared boundaries deform neighboring faces. Curved and nonplanar component results currently use a regenerated faceted boundary validated with Truck, not exact edited NURBS surfaces. This is separate from future control-node soft selection and its heatmap/radius controls. The Volume inspector can **Remove component edits** as an undoable action when a topology-changing source edit is needed.

Component edits on a profile that cuts another region are currently rejected; Volume transforms of that cutting profile remain available. Topology-changing edits and deletions that invalidate saved component references are rejected without replacing the last valid scene. General semantic CAD feature references and exact curved-surface reconstruction remain pending.

Closed shapes drawn over older coplanar profiles now split their regions. Extruding the surrounding profile excludes the newer regions and creates walls along their boundaries; those inner regions retain their own height. This supports holes, boundary notches, and cuts that divide a profile. Open-line splitting and sketches on elevated solid faces remain pending. See the [.osk modeling rules](docs/formats/osk.md#coplanar-region-evaluation-modeling-version-2), including curved-boundary approximation and older-file migration.
