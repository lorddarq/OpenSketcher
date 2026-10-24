# OpenSketcher

[![Desktop build](https://github.com/lorddarq/OpenSketcher/actions/workflows/desktop.yml/badge.svg?branch=main)](https://github.com/lorddarq/OpenSketcher/actions/workflows/desktop.yml)
[![Rust](https://img.shields.io/badge/language-Rust-orange?logo=rust)](https://www.rust-lang.org/)
[![License: Apache 2.0](https://img.shields.io/badge/license-Apache--2.0-blue)](LICENSE)
[![Platforms](https://img.shields.io/badge/platforms-macOS%20%7C%20Windows%20%7C%20Linux-lightgrey)](#desktop-distribution)
[![Latest tag](https://img.shields.io/github/v/tag/lorddarq/OpenSketcher?label=version)](https://github.com/lorddarq/OpenSketcher/tags)

A Rust desktop application aiming to offer approachable 3D modeling with functionality overlapping SketchUp. Independent project; no affiliation with SketchUp or Trimble.

Release notes: [Changelog](CHANGELOG.md). Unsigned development builds are available from the [Desktop workflow](https://github.com/lorddarq/OpenSketcher/actions/workflows/desktop.yml).

## Current state

Native editor with a depth-tested wgpu viewport, ground grid, world axes, and Truck-backed parametric geometry. Create rectangles, circles, lines, and semicircles using two picked points on a selected or visible planar face, with XY fallback. Choose Volume, Face, Edge, or Point selection in the toolbar. Volume selection exposes automatic dimension, extrusion, position, rotation, and scale edits; component modes pick and highlight individual topology elements. Move and Rotate expose constrained gizmos in all four selection modes. Whole-object deletion and undo/redo are supported.

Projects save and load as `.osk` files, preserving editable geometry parameters and scene properties. Attached planar profiles now support solid boring and separate emerging bodies. Model import, general face-region push/pull, node editing, and a Scale gizmo remain pending. Supported primitive faces have a signed numeric Offset face control. There is no longer a demonstration cube.

## Feature checklist

Checked items are implemented; platform runtime verification is tracked separately below. Unchecked items are pending, including behavior that currently exists only in a specification.

### Creation and modeling

- [x] Canvas creation on selected/visible planar faces or XY: rectangle corners, circle center/radius, line endpoints, semicircle diameter endpoints.
- [x] Live creation preview and Escape cancellation.
- [x] Default-on geometry snapping (visible endpoints, edge midpoints, edges, circle centers/rims, projected limits) plus 0.5-unit grid and anchor alignment on the captured plane.
- [x] Project drawing preferences; base-XY projection and automatic coplanar cutting are opt-in. Existing cuts retain their saved intent.
- [x] Automatic numeric dimension, position, Euler rotation, and local scale editing.
- [x] Straight local +Z extrusion: closed profiles to solids, lines to surfaces.
- [x] Coplanar closed-profile partitioning: holes, edge notches, disconnected remaining regions, and new cut walls.
- [x] Move and Rotate gizmos in Volume, Face, Edge, and Point modes: world-axis arrows, move-plane handles, rotation rings, configurable pivot, snapping, and cancelable previews.
- [x] Persisted component transforms with adjacent-boundary rebuilding and topology-reference validation.
- [x] Whole-object deletion and grouped session undo/redo.
- [ ] Regular polygons and additional arc/freehand/rotated-rectangle tools.
- [ ] Circle Radius field and radius preview label (canvas input is already center/radius; properties still display diameter).
- [ ] Persistent parent-linked sketches, expanded contextual point/center overlays, and an explicit support-plane indicator (fixed placement on selected/elevated faces works).
- [ ] Open-line face partitioning and exact analytic curved intersections (split curves currently use polygon approximations).
- [x] Signed numeric Offset face for original box faces and circular/semicircular solid caps; preserves the opposite plane, source parameters, circular inference, and existing treatments.
- [ ] Face-normal canvas handle, general face-region push/pull, and component scaling (component movement/rotation are implemented).
- [ ] Negative extrusion, mirrored scaling, and a canvas Scale gizmo.
- [ ] Explicit X/Y/Z axis locks and labeled axis-inference feedback during drawing/moving.
- [ ] Geometry-intersection snapping and additional arc/curve references (circle centers/rims, endpoints, midpoints, edges, and projected limits are implemented).
- [x] Attached-profile Borer-only / Generator + borer: pockets, through-cuts, rim notches, and separate emerging bodies with signed travel; original primitive support faces and planar exit baseline.
- [ ] General trimmed/deformed-face supports and uneven exit handling.
- [x] Editable/removable face Chamfer: all six rectangular-solid faces and top/bottom rims of cylinders and semicircular extrusions.
- [x] Evaluated inward/outward Shell for convex planar chamfered, Boolean and moved solids, with editable openings, closed cavities and exact retained walls.
- [x] Draft selected evaluated planar walls, including concave notches, holes and pockets, with editable absolute angle, neutral input face or live construction plane and exact retained geometry.
- [x] Convex planar face Move/Rotate can rebuild corner topology and consume redundant faces, with guarded history and a return to volume selection.
- [x] Editable/removable Bevel with user-selected rounded or flat style; circular rims can also be selected in Edge mode.
- [x] Exact local prism-corner chamfers and analytic rounded/flat bevels on evaluated geometry, including inner/outer notch and rectangular-hole corners.
- [x] Parametric modifier distances/radii, semantic source-face references, `.osk` persistence, and undo/redo for add/edit/remove.
- [ ] Modifiers on partitioned/concave or component-deformed solids; arbitrary curved-face treatment and exact noncircular blends.
- [ ] Persistent semantic face/edge references for dependent modeling features.
- [x] Explicit control-net creation, fixed-boundary NURBS node editing with radius/falloff/heatmap, and shape-preserving movement of supported control faces. General trim/periodic-seam editing remains unsupported.

### Selection, viewport, and workspace

- [x] Volume, Face, Edge, and Point picking/highlighting, with depth-aware component selection.
- [x] Shift-click toggles multiple selections in all four modes; group Move/Rotate, volume deletion, atomic undo/redo, and saved selections.
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
- [x] Export evaluated geometry to binary STL, GLB, STEP, and DXF; STEP preserves retained kernel CAD surfaces.
- [x] STEP export of retained analytic/NURBS source and Boolean boundaries, independent of display triangles.
- [ ] Analytic reconstruction of mesh-only component/blend outputs and named CAD assemblies.
- [ ] Additional major 3D interchange formats for printers, CAD tools, and Blender (3MF, OBJ, PLY, IGES).
- [ ] Model import and SketchUp file compatibility.
- [ ] Signed/notarized releases, installers, Linux desktop integration, application icons, and release publishing.

## Export

Use **Export…** in the header and choose STL, GLB, STEP, or DXF. Native dialogs choose the destination; exports replace files atomically. The whole committed model is exported with current transforms, profile cuts, and component edits. Finish previews and correct invalid fields first. Export does not change the document or its saved state.

For this first version, **1 scene unit = 1 millimeter**. STL carries numeric millimeter coordinates (STL has no unit metadata); DXF declares millimeters; Truck STEP declares millimeters; GLB converts to meters and maps scene Z-up to glTF Y-up.

| Format | Output and limits |
| --- | --- |
| STL | Binary triangle mesh for printing; requires closed solids and rejects standalone lines/sheets. Separate overlapping solids are not unioned; check the result in a slicer. |
| GLB | glTF 2.0 binary file, one named node per body, triangle faces and boundary lines, neutral double-sided material. Suitable for Blender import. |
| STEP | Retained Truck analytic/NURBS boundaries for source solids and Boolean outputs; mesh-only edits use faceted boundaries. Intersection trims are sampled into splines at kernel tolerance. Standalone lines are rejected; use DXF/GLB. |
| DXF | ASCII AutoCAD 2000 entities: triangular 3DFACEs and LINE boundaries on per-body layers. Curves are sampled, and faces are not CAD solids. |

All formats use the current viewport tessellation tolerance, not a configurable manufacturing tolerance. Textures, scene cameras, and parametric feature history are not included. Keep `.osk` as the editable source. Exact CAD geometry and further common interchange formats remain on the checklist. File structure and geometry checks run in the Rust suite. Native macOS review verified the export menu, save dialog, and a three-body GLB output without changing the document; acceptance in external slicers/CAD applications and Blender remains pending.

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

1. Choose **Rectangle**, **Circle**, **Line**, or **Semicircle**, then click two points on the drawing plane. A selected planar face pins the support; otherwise the first click captures a visible planar face, with XY as fallback. Side views support vertical faces too.
2. Rectangles use opposite corners; circles use center then radius; lines use endpoints; semicircles use diameter endpoints. Semicircles create a filled half-disc to the left of the directed diameter.
3. The first point anchors a green outline preview. The second commits one creation action. Escape cancels the preview without adding geometry. Snapping is on by default: visible geometry references plus a 0.5-unit grid and anchor alignment in the captured plane. Geometry targets take priority over grid rounding, so an exact non-grid corner stays exact. The plane remains fixed until commit/cancel, including when the pointer leaves the face or Snap is disabled.
4. Choose **Select** and click an object, or use the model list. Selected geometry is highlighted. Dimension, extrusion, position, Euler rotation, and local-axis scale values regenerate geometry automatically—there is no Apply or Revert Fields action.
5. Extrusion follows the profile's local +Z. Filled profiles create solids; lines create surfaces. Rotation and scaling pivot at the profile center (the center of a semicircle's diameter).
6. Use **Undo / Redo** or Cmd/Ctrl+Z and Cmd/Ctrl+Shift+Z. A continuous numeric drag or text edit is grouped as one undo action. Delete selected geometry with the inspector button or Delete/Backspace outside a text field. History retains up to 100 actions per session.

Input validation rejects invalid dimensions or geometry and keeps the last valid object. Current supported dimensions are 0.01–100 units, extrusion zero or 0.01–100, and positive scale 0.01–100. Negative extrusion and mirrored scaling are not yet supported. Objects are not automatically fused. New profiles are independent by default. Explicitly cutting coplanar profiles partition older profiles before extrusion, as described below.

The document stores authored parameters independently of GPU data. One Truck evaluator serves creation, live edits, and previews. Tessellated faces and sampled boundary edges feed rendering and picking; editing does not modify triangles directly. The regression check exercises all four profiles, two-point dimensions, extrusion, transformations, grouped live-edit history, invalid-input rejection, and deletion recovery.

## Drawing preferences and geometry snapping

Use **Preferences** to configure this project's drawing defaults. **Snap to geometry** and **Snap to grid and anchor axes** start enabled; the Tools **Snap** checkbox temporarily disables both. Turning Snap off does not change an already captured plane or force a hovered face back to XY.

A selected planar face always takes priority. Otherwise the first point uses the visible planar face under the pointer; nearby visible boundaries can acquire a silhouette just outside it. Endpoint, midpoint, and edge targets use screen-space tolerances and actual evaluated geometry, including extrusion and component edits. Visible boundary coordinates can also project alignment limits across the active plane, with a labeled marker and guide. Hidden base geometry and references off the captured plane are excluded. Curved/nonplanar support faces report an error rather than pretending to be XY.

**Project drawing to base XY plane** is off by default. Enable it explicitly for world-plane placement through/under visible geometry; a selected face still wins. Placement settings apply when a sketch starts, and its plane stays locked during the gesture.

**New profiles cut coplanar regions** is also off by default. It sets cutting intent on new closed profiles only; it does not change existing shapes. Each closed profile exposes **Cut older coplanar profiles** in its inspector for an undoable per-profile change. This is the existing coplanar-profile partition operation, not a general solid Boolean or an imprint on an arbitrary solid face.

The settings and individual `cuts_profiles` flags persist in `.osk` version 5 (introduced in version 4). Modeling versions 2–3 migrate with cutting flags enabled to preserve existing cuts; version 1 keeps its original independent profiles. Newly drawn profiles still default to independent geometry. New closed profiles on supported original solid faces retain an attachment and explicit operation controls. Legacy independent profiles retain fixed placement.

Selected flat profiles use a separate coplanar display pass to avoid flickering against the supporting face. This leaves saved geometry and picking at the actual surface depth. Coincident picking prefers the newer profile without selecting hidden geometry through the solid.

## Selection modes

- **Volume:** select the whole object, including standalone profiles and lines. Whole-object transforms and deletion are available here.
- **Face:** select one modeling surface across its display triangles. Connected patches of the same cylinder form one wall.
- **Edge:** select a shape boundary across its sampled curve segments. Circular rim arcs select as one edge; internal cylinder junctions and triangulation diagonals are not editable edges.
- **Point:** show and pick visible topology vertices. These are not tessellation samples or NURBS deformation controls. For control editing, select a face, Create control net, then Edit nodes.

Face, Edge, and Point support Move/Rotate gizmos for the selected component; adjacent boundaries rebuild to remain connected. Whole-object numeric properties and deletion remain in Volume mode. Changing mode clears the previous selection but keeps Move/Rotate active. Completed component transforms retain their selection, and undo/redo preserves it when its topology slot still exists. Empty-space clicks clear selection within the current mode. Occluded components are not selectable through opaque faces. Edge/point acquisition uses screen-space tolerances so zoom does not shrink the click target.

`.osk` schema/modeling version 5 preserves drawing preferences and cutting intent alongside component-transform matrices and guarded topology references. Source connectivity must match before replay; an upstream edit that changes it is rejected rather than targeting another component. Files migrated from modeling version 1 reset component selection to Volume because partitioning changes topology slots. This guard is not a general semantic naming system for arbitrary CAD feature histories.

**Shift-click** adds/removes items in the current selection mode; an ordinary click replaces the selection, and clicking empty space clears it. Shift-clicking empty space preserves it. Shift-drag still pans. The model list supports Shift-click for volumes. The last selected item is the primary selection; changing mode clears the previous selection. Native macOS review verified additive volume/face selection, toggle removal, saved sets, a constrained two-body Move gesture, and one-step Undo.

Move/Rotate use the whole selection and a shared pivot. Shared component vertices transform once. A failed group edit leaves all bodies unchanged, and each committed gesture or group deletion is one undo step. Multiple volumes expose a count and group actions; numeric source/modifier properties require one body. Treatments accept multiple supported faces/edges in one atomic action; box edges are local and circular rims use their full perimeter. Component matrices replay after edge treatments through the faceted editing path; exact general surface reconstruction remains pending. `.osk` version 5 retains the selection set and union component transforms.

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

The [face-sketching workflow](docs/specs/face-sketching.md) now supports fixed placement on selected or visible planar faces, including extruded top/side faces, with geometry snapping. Expanded contextual overlays, persistent parent attachment, and consistent Radius inspector/readout remain pending. The [Plasticity/SketchUp research](docs/research/face-sketching-workflows.md) separates reference behavior from OpenSketcher's chosen rules.

Resolve the topology and document format around the first modeling workflow before adding more tools. SketchUp file compatibility is not assumed.

Projects use the custom native `.osk` format to retain currently implemented editable shape parameters and scene properties. Future dependent features must preserve their authored inputs and references too. Render meshes alone are not the saved model. See the [persistence requirements](docs/product-scope.md#project-saveload-and-native-format).

Truck is the selected geometry kernel. The [parametric face modeling research](docs/research/parametric-modeling-kernels.md) records the comparison and implementation validation cases, including boring and separate emerging bodies. Truck modeling and tessellation are integrated; boring and persistent face-reference validation remain pending.

## Dependencies and attribution

Versions below are resolved from Cargo.lock. The full direct/transitive inventory, upstream copyright notices, license texts, and embedded-font notices are preserved in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md). Some locked packages are platform-specific, optional, or build-time dependencies and are not shipped in every executable.

| Direct dependency | Locked version | Used for | Upstream credit | Declared license |
| --- | --- | --- | --- | --- |
| [eframe](https://github.com/emilk/egui) | 0.34.3 | Native editor shell, egui UI, wgpu integration, bundled fonts | Emil Ernerfeldt and egui contributors | MIT OR Apache-2.0; embedded fonts have separate terms below |
| [rayon](https://github.com/rayon-rs/rayon) | 1.12.0 | Bounded background geometry thread pool | Rayon contributors | MIT OR Apache-2.0 |
| [truck-shapeops](https://github.com/ricosjp/truck) | 0.4.0 | Solid Boolean subtraction | RICOS Co. Ltd. and Truck contributors | Apache-2.0 |
| [rfd](https://github.com/PolyMeilex/rfd) | 0.17.2 | Native file and confirmation dialogs | Copyright © 2022 Bartłomiej Maryńczak | MIT |
| [serde](https://github.com/serde-rs/serde) | 1.0.229 | Scene serialization traits and derives | Erick Tryzelaar, David Tolnay, and Serde contributors | MIT OR Apache-2.0 |
| [serde_json](https://github.com/serde-rs/json) | 1.0.154 | `.osk` JSON encoding/decoding | serde_json contributors | MIT OR Apache-2.0 |
| [tempfile](https://github.com/Stebalien/tempfile) | 3.27.0 | Same-directory temporary files for atomic saves | Copyright © 2015 Steven Allen; tempfile contributors | MIT OR Apache-2.0 |
| [truck-stepio](https://github.com/ricosjp/truck) | 0.3.0 | STEP serialization of evaluated CAD boundaries | RICOS Co. Ltd. and Truck contributors | Apache-2.0 |
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

## Circle face snapping and cut intent

Drawing tools snap to **Circle center** and **Circle rim** on intact circular faces and cylinder rims, including elevated, rotated, nonuniformly scaled, and beveled cases. These use the actual parametric boundaries, not the nearest tessellation chord. Existing Snap / geometry preferences control them; hidden and off-plane references are excluded. Circles altered by component edits or profile partitioning do not expose a falsely complete circular rim.

Center/rim inference is implemented. Automatic rim-start solid cutting is not implemented yet; current numeric extrusion still generates geometry. The [face-operation behavior notes](docs/specs/circle-face-operations.md) distinguish placement, support, and cut intent and record the remaining interaction clarification.

## Chamfer and bevel

Select **Face** and pick an original face of an extruded rectangle, or the top/bottom cap of a cylinder or semicircular extrusion. Choose **Add Chamfer** or **Add Bevel** in Model & properties. For cylinders and semicircular extrusions, **Edge** mode also accepts rim edges and treats the entire corresponding perimeter.

The editor switches to the owning body so its modifier list is immediately editable. Change **Distance** for Chamfer/flat Bevel, or **Radius** for rounded Bevel. Bevel offers **Rounded** and **Flat / segmented** styles; flat is a straight cross-section, equivalent to equal-distance chamfering. Noncircular rounded treatments expose 2–16 approximation segments. Values update automatically; Remove restores geometry without that operation. Add, edit, style changes, and removal support undo/redo. Source dimensions and placement remain editable.

Cylinder treatments regenerate analytic circular profiles and revolved surfaces. Noncircular source modifiers use convex clipping with grouped transition strips; evaluated prism bevels retain analytic curved bands. Render subdivisions and kernel patch seams within a surface are hidden and are not selectable faces: the base face and bevel surface remain distinct. Distances/radii use source-local units; nonuniform scale scales the result. Only one modifier per source face is accepted, with conservative size limits that preserve the solid. Partitioned profiles, cutting-dependent profiles, and mixing modifiers with component edits are explicitly rejected for now.

`.osk` schema/modeling version 5 stores the modifier inputs and source-face roles, never viewport triangles. Versions 1–4 migrate without adding modifiers or changing their geometry. All export formats use the same evaluated modified shape; STEP preserves retained analytic surfaces; approximated noncircular blends remain faceted. See [modifier behavior](docs/specs/face-modifiers.md).

## Canvas Move and Rotate

Select a Volume, Face, Edge, or Point, then choose **Move** or **Rotate**. You can also select geometry while either tool is active. Drag a colored handle and release to commit one undoable action; Escape cancels the preview. Switching tools or views and losing window focus also cancels an unfinished drag.

Move arrows constrain motion to world X (red), Y (green), or Z (blue). Small plane handles constrain motion to XY, YZ, or ZX. Rotate rings constrain rotation to a world axis and compose correctly with an existing object rotation. The highlighted handle and live numeric readout show the constraint. Handles retain a usable screen size as you zoom. An axis viewed end-on uses vertical dragging; edge-on rotation rings use a screen-drag fallback.

The pivot starts at the selected geometry's bounding-box center (the selected vertex for Point mode). **Alt-click** the canvas to place an offset pivot on the view-aligned plane through the current pivot; **Reset pivot** restores the automatic center. A point rotated around itself stays still, so place an offset pivot to rotate it. Pivot placement is transient and does not change authored geometry.

Snap defaults on: move offsets use 0.5-unit steps and rotation offsets use 15-degree steps. Turn Snap off for continuous changes. Volume numeric properties remain available for exact values.

Component edits retain the source profile plus ordered local transform matrices in `.osk`; they do not save a baked mesh. Selected faces/edges move rigidly, while shared boundaries deform neighboring faces. Curved and nonplanar component results currently use a regenerated faceted boundary validated with Truck, not exact edited NURBS surfaces. Explicit control-net operations retain exact NURBS surfaces, editable node translations and heatmap/radius controls; see the soft-selection specification. The Volume inspector can **Remove component edits** as an undoable action when a topology-changing source edit is needed.

Component edits on a profile that cuts another region are currently rejected; Volume transforms of that cutting profile remain available. Topology-changing edits and deletions that invalidate saved component references are rejected without replacing the last valid scene. General semantic CAD feature references and exact curved-surface reconstruction remain pending.

Closed profiles with **Cut older coplanar profiles** enabled split older coplanar regions. New profiles leave this off by default. Extruding the surrounding profile excludes the newer regions and creates walls along their boundaries; those inner regions retain their own height. This supports holes, boundary notches, and cuts that divide a profile. Open-line splitting and persistent attached-face dependencies remain pending; fixed sketches on elevated planar faces work. See the [.osk modeling rules](docs/formats/osk.md#coplanar-region-evaluation-modeling-version-2), including curved-boundary approximation and older-file migration.

Geometric operator specifications and researched gaps: [face editing, offset, extrusion, and Boolean roadmap](docs/specs/geometric-operators.md).

Geometric edits and project reevaluation use a background worker capped at two geometry threads. Repeated input is coalesced; the last valid scene remains visible. Escape discards pending results, while an already executing kernel call finishes in the background. Use a release build for interactive modeling performance.

Ordered Boolean tools: Shift-select two volumes or coplanar faces, then choose **Union**, **Subtract**, or **Intersect** in Model & properties. Subtract removes the second selection from the first; **Swap target / tool** reverses it. Results retain editable source operands and support undo/redo, `.osk` persistence, and **Keep tools**. Face operations produce planar sheets; curved/crossing face trimming remains pending. See the [Boolean operator specification](docs/specs/boolean-operators.md).

Chamfer supports **Equal distance**, **Two distances**, and **Distance and angle**, plus **Reverse direction**, on supported box faces and circular/semicircular cap perimeters. Distances are source-local; cylinder caps use radial and axial setbacks. These controls update automatically and persist in `.osk` version 8.

Edge-local box chamfers/bevels: select **Edge**, Shift-select the desired source edges, then **Chamfer** or **Bevel**. Each box edge is treated independently; face selections treat their full perimeters. Supported multi-selection across bodies applies in one undo step. These semantic edge targets persist in `.osk` version 9. Chamfer also supports retained convex planar Boolean-result and transition edges, with editable First/Second distances, Reverse distances and Remove in the owner inspector (`.osk` v32). Full-depth inner/outer corners on nonconvex planar prisms also support local chamfers (`.osk` v33): inner corners add material; outer corners cut. Evaluated straight-prism edges also support analytic Rounded bevels or Flat style, editable radius/distance, and removal (`.osk` v34). Repeated prism treatments preserve unrelated rounded bands and circular through-holes; `.osk` v35 stabilizes downstream face/edge slot ordering. The selected edge still needs two planar neighbors. General nonprismatic/curved edges and advanced fillets remain pending. Edge-mode Tangent chain optionally expands clicks along smooth retained boundaries; Shift-click toggles the chain.

Rejected operator values remain visible with **Input not applied** while geometry and Save/Export retain the last accepted model. Errors identify the object, operation, and relevant source/target reference. Correct the value or press **Escape** after leaving the field to discard the rejected draft; no Apply/Revert controls are required.

Attached sweeps offer **Through all**, **Up to next**, and **Up to face**, alongside manual Distance. Through all cuts the attached target and explicitly checked scope bodies; unchecked bodies are preserved. Up to face uses the referenced infinite planar support, including angled planes. Up to next requires full-profile coverage by finite planar faces, groups coplanar regions, and rejects nearer partial/curved obstructions. Circular end boundaries remain exact ellipses on angled planes. Target edits regenerate the sweep. Extents retain the version 10 tagged representation; multiple-body scope requires `.osk` version 21. General curved and multi-plane termination remain pending.

Select a planar face in Face mode and use **Extrude face → Distance** to extend its current trimmed boundary, including caps after chamfer or bevel. The existing treatment stays in place. **Offset source** remains available separately to resize the original primitive. Outward planar face sweeps persist in `.osk` v11 and support undo/redo. `.osk` v13 adds negative travel for subtraction, including through-cuts after passing the opposite wall. Arbitrary curved/Boolean-result face sweeps remain pending.

Edge/face/point gizmo edits now operate on the evaluated geometry of bored and Boolean solids. Newly authored edits replay after those construction operations, with multiselection, undo/redo and `.osk` v12 persistence. Upstream topology changes reject stale references safely. Curved component deformation still uses the existing faceted boundary reconstruction; exact analytic retrimming remains pending.

**Extrude face → Distance** uses positive travel to add material and negative travel to subtract it. Passing through the far wall creates an opening with no emerging extrusion or tool switch. Treated cap cuts preserve surviving chamfer/bevel geometry. Consecutive reversal retracts the previous sweep. Authored depth, undo/redo and `.osk` persistence remain intact.

Select an original planar face and choose **Shell**, or use **Shell · open top** in Volume properties, to make a hollow source prism. Choose **Inward** to keep the exterior fixed or **Outward** to keep the cavity at the source dimensions and grow the exterior. The opening plane stays fixed. Enable **Closed cavity** to keep every exterior face around a sealed interior. Edit **Wall thickness** and **Opening** directly, or **Remove shell** to restore the solid. Boxes support any of six openings; cylinders and semicylinders support either cap. These exact CAD boundaries, logical faces and circular snap references persist in `.osk` v17 (inward Shell introduced in v14) with undo/redo and STL/GLB/STEP/DXF export. Thickness is source-local before object scaling. Shelling treated, Boolean or component-deformed solids and drafting arbitrary evaluated faces remain pending.

Use **Draft walls** in Volume properties to taper a source box, cylinder or semicylinder. Edit **Angle** and choose a neutral **Top/Bottom** cap; positive angles expand away from the neutral cap, negative angles contract. **Remove draft** restores the prism. Exact cone/plane surfaces and semicylinder parabolic corner edges retain logical face selection, circular references where applicable, undo/redo, `.osk` v15 persistence and all four exports. Angles apply before object scaling. General draft of treated/evaluated faces and arbitrary neutral planes remains pending.

Retracting a shell’s annular opening removes its surrounding walls and preserves the material beneath the cavity. Moving the filled back cap inward can remove that back and open the shell through. Both operations retain exact shared boundaries and logical curved faces.

Closed Shell exports preserve the cavity as a STEP void and inward-oriented printable mesh boundary. Turning off **Closed cavity** restores the selected opening; removing Shell restores the source solid.

Face **Move/Rotate** now retrims convex planar solids exactly, including boxes with flat chamfers: selected support planes move, neighboring planes remain fixed and shared boundaries rebuild. Collapsing/connectivity-changing moves reject without changing accepted geometry. `.osk` v18 retains this behavior; older component histories keep their original deformation. Exact curved/concave face retrimming remains pending.

**Extrude face** also works on evaluated planar boundaries after exact Move/Rotate, on planar chamfer bands and on Boolean results. Sweeps replay in order with prior edits and save in `.osk` v19. Positive travel adds material; full prismatic inward travel cuts/removes it through the far wall. Complex inward intersections use the solid kernel and may still reject; curved or mesh-only deformed faces remain unsupported.

Inward sweeps on convex polyhedral boundaries now trim angled exits and boundary crossings directly with exact planar faces. Overrun removes only material inside the swept footprint, and disconnected face regions remain separately selectable. Curved/concave cases retain the solid-kernel fallback and its current limits.

Ordered volume **Subtract**, attached profile bores and face sweeps share the same exact convex planar subtraction path. Disjoint/touching-only supported tools preserve original topology; enclosed cutters retain proper CAD voids. Curved and concave boundaries still use Truck’s general solver.

Use **Add construction plane** to place a drawing reference. Move/rotate it in Volume mode, edit its placement, or choose **XY/XZ/YZ**. **Draw on this plane** pins drawing to it; **Auto drawing plane** restores hover-based placement. Selected physical faces take priority. References are wireframes, excluded from exports/Boolean tools and saved in `.osk` v20. Drawings capture their placement independently, so moving a reference does not move existing drawings.

For **Through all**, check additional earlier solids under **Through-all scope** to cut them with the same attached profile. Unchecked bodies stay unchanged, even between targets. Travel follows the scope’s geometry; automatic Through all emits no separate extension. Use **Distance** for manual generator + borer overshoot. Scope IDs persist in `.osk` v21.

### Delete faces and close openings

In Face mode, select one or more faces and choose **Delete face · Open**, **Cap**, or **Heal**. Open leaves surviving surfaces as a sheet; Cap closes planar openings while preserving holes. Heal extends neighboring planes on supported convex planar solids, for example to remove chamfer bands and restore sharp corners. In Volume mode, **Cap openings** closes planar boundaries on an existing open sheet. Operations use undo/redo and persist in `.osk` v22. Open sheets export to GLB/STEP/DXF; STL requires closure. General curved/concave healing and nonplanar caps remain pending.

### Replace a face

Select a source face, Shift-select a reference face, then choose **Replace face**. Or select one source face and choose a construction plane from **Replace using plane**. The source solid extends/trims to the reference's infinite plane; the reference stays unchanged. Reference edits regenerate the replacement, and removing it restores the source. Convex planar solids and original cylinder/semicylinder caps are supported; general curved/concave replacement remains pending. The feature uses `.osk` v23 and existing undo/redo/export paths.

### Imprint a profile onto a face

In Face mode select the solid face first, Shift-select a coplanar profile, then choose **Imprint**. The result keeps the same material and splits independent face regions. Select a region to push/pull it. Contained circles retain analytic edges and extrude as one logical curved wall; straight boundary crossings also split adjacent edges. Existing holes remain intact. History and input references persist in `.osk` v24. New closed-profile crossings retain analytic B-spline/NURBS trim curves, including circle-to-circle intersections and shared neighbor edges. Profiles sharing circular/elliptical rim intervals keep the existing boundary once and split the face with their interior edges; the resulting regions remain independently extrudable. Coincident-only profiles create no new region and reject. Arbitrary open curved tools and general nonconic spline overlaps remain pending.

For profiles with holes or multiple disconnected regions, select the solid face and use **Imprint using profile**. All regions of the chosen planar sheet are used; the existing two-face command uses only the chosen tool face. Holes remain independent material islands, and existing voids are preserved. Ring through-cuts can leave a separate central body. Whole-profile references persist in `.osk` v25.

Finite straight-line Imprint is available from a selected solid face via **Imprint using curve**. It partitions planar faces, including circular trims and holed rims, without changing material or flattening analytic neighboring surfaces. The finite line must cross the required boundaries; incomplete cuts reject. Source/line edits regenerate the result, with Keep tools, undo and `.osk` v26 persistence. General curved imprint tools remain pending.

New Imprints persist analytic-crossing intent in `.osk` v27. Source/profile edits regenerate curved crossings with undo/redo and the existing Keep tools/removal behavior. Older histories keep their prior evaluation policy. Full retained circular loops preserve snap references; clipped arcs retain analytic geometry without exposing an invented complete snapping circle.

Circle extrusions treat connected patches of the same cylinder as one modeling wall, with one edge per circular rim. Internal arc-junction lines are excluded from rendering, edge picking, transform authoring and geometry snapping. Selecting or moving the wall affects every member patch. A transform owning the whole retained boundary preserves its analytic curves/surfaces. The kernel patch topology remains internal so existing saved feature guards and legacy replay stay compatible; genuine cap, chamfer, bevel and imprinted boundaries remain separate.

Complete planar end-cap and complete-rim translations now retain analytic geometry on two-section solids, including cylinders, semicylinders, through-pipe caps and concave Boolean caps. The cap keeps its shape, the opposite cap stays fixed, and lateral moves produce an oblique cylinder without creating editable seam edges. These edits persist in `.osk` v28; cap rotations and more general partial curved edits remain pending.

New coplanar face **Union**, **Subtract**, and **Intersect** operations retain analytic circular/B-spline/NURBS trim curves, holes and disconnected regions. Opposite input normals are supported; subtraction still removes the second pick from the first. These sheet operations persist as `.osk` v29 and keep existing solid inputs intact. Older operations retain their saved evaluator. Shared circular/elliptical trim intervals—including different arc starts and partial arcs—are merged analytically. General nonconic spline overlaps, intersection-generated input trims and noncoplanar/curved-surface trimming remain pending.

Imprinted cap regions sharing an existing rim can be extruded independently. Negative travel through an eligible straight prism now removes that region with exact analytic geometry, avoiding coincident-wall CSG failures. New face sweeps persist this behavior in `.osk` v30; existing histories retain their evaluator. General blind/shared-wall, tapered and curved-exit trimming remain pending.

Point overlays, picking and snapping use a shared spatial triangle index, built with geometry off the UI thread. The 25-cylinder release benchmark reduced point visibility from 238 ms to 1.08 ms; this is one rendering pass, not total frame time. See the [CPU and memory investigation](docs/performance/cpu-investigation-2026-10-24.md) for index build/memory costs and remaining bottlenecks.


Evaluated Shell: select a planar face on supported convex geometry and choose **Shell**. The owner inspector exposes wall thickness, direction, closure, the source opening face and removal. This appends after existing operations and persists in `.osk` v36. Geometry with 4–64 planar supports is supported; general concave/curved shelling remains pending.

Face-mode **Draft faces** tilts selected planar walls about their intersection with an editable neutral input face. Angle is absolute: zero makes walls perpendicular; positive widens on the negative side of the oriented neutral plane. Changes apply automatically and support Remove and undo. Convex v37 histories retain their original replay; new `.osk` v38 operations also preserve concave notches, rectangular through-holes and pocket loops. Touching or consuming edits reject without changing the committed model. Curved Draft and concave topology-changing support consumption remain pending.

Use **Draft using plane** or choose a construction plane in the Draft neutral selector to keep its position/orientation live (`.osk` v39). Planes may be created after the solid. Moving or rotating one regenerates its dependents; remove or rebind Draft before deleting its reference. Construction planes remain excluded from exports.

Convex planar face movement now rebuilds incompatible chamfer corners and redundant supports (`.osk` v40). Existing topology-preserving edits retain their path. If component topology changes, the editor returns to volume selection so old face/edge slots cannot silently select different geometry. General concave topology changes and curved-neighbor edits remain pending.

New convex Draft operations can remove redundant chamfer faces or caps while retaining valid closed material (`.osk` v41). Earlier Draft histories keep their original rejection rules; recreate the operation to use the new evaluator. Source treatments remain in history and restore on Undo or removal.

Interactive evaluated edits and bore travel/mode changes now reuse their unchanged source BRep/tessellation. Source parameters and resolved profile partitions still invalidate the cache, and late operators/reference guards always replay. A two-thread release diagnostic measured 118 ms warm versus 219 ms forced cold for 12 rounded-cylinder cap edits; this measures geometry evaluation, not total frame time. See the [CPU investigation](docs/performance/cpu-investigation-2026-10-24.md).


### Control-net editing

In Face mode select a planar parallelogram and choose **Create control net**, then **Edit nodes**. Click/Shift-click nodes or use the control-coordinate chooser. Set **Soft selection** and **Radius** in millimeters; boundary nodes stay locked to retain shared trims. Drag a world-axis/plane handle (or the center handle in the view plane), or enter relative world displacement and choose **Translate nodes**. The marker heatmap, percentage feedback and separate explicit/soft counts show influence. Radius zero and the disabled toggle give hard selection. Escape/focus loss cancels a gesture; an accepted gesture is one undo step.

Leave node mode before **Move face** to preserve the entire deformation without falloff. Supported solid caps rebuild planar neighbors while fixing the opposite section; single-face sheets retain their translated trims. Saved `.osk` v42 features expose radius, local displacement and removal in the Volume inspector and remain editable after reopening. Periodic/interior-trimmed patches and unsupported adjacent geometry reject explicitly. See [soft selection](docs/specs/soft-selection.md) for eligibility, replay semantics and validation limits.
