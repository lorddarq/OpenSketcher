# OpenSketcher project format (.osk)

Schema version: 3. Modeling semantics version: 3.

An `.osk` file is a UTF-8 JSON document identified by `format: "OpenSketcher"`. The extension is specific to OpenSketcher; the encoding is documented and does not encrypt the scene. The native project stores authored parameters, not only viewport triangles. Truck reevaluates the parameters to restore editable geometry.

## Serialized scene

| Field | Meaning |
| --- | --- |
| `format` | Required `OpenSketcher` identifier. |
| `schema_version` | File layout version; currently 3. Versions 1–2 migrate on load. |
| `modeling_version` | Evaluation semantics version; currently 3. |
| `bodies` | Ordered scene objects with stable nonzero IDs. |
| `camera` | Orthographic preset/free-view label, yaw, pitch, span, and target. Angles are radians; span is the visible vertical extent. |
| `snapping` | Whether creation snapping is enabled. |
| `selected` | Selected owner object's ID, or null. |
| `selection_mode` | `volume`, `face`, `edge`, or `point`. |
| `selected_element` | Kernel topology output slot for a component, or null for Volume/no selection. |

Each object retains `id`, `shape`, `width`, `depth`, `extrusion`, `position`, `rotation`, `scale`, and ordered `component_edits`. Shape values are `rectangle`, `circle`, `line`, and `semicircle`. Dimensions and positions use the current application's unitless world coordinates; rotation uses degrees in local X/Y/Z order. Scale is a positive factor per local axis. Extrusion follows local +Z before object rotation/scaling. Circle/semicircle width is diameter; line width is length. Rectangle depth is its second dimension. Even currently unused per-shape fields are preserved.

Camera data includes `view` (`isometric`, `top`, `side`, or `free`), `yaw`, `pitch`, `span`, and `target`. It restores the actual camera orientation and framing, not just the preset label.

All currently implemented scene properties are serialized. Materials, textures, independent saved scenes, NURBS cages, deformation features, and boring do not yet exist in the application model; this version cannot encode those future features. Their introduction requires an explicit schema/modeling version decision and migration support. Do not silently drop unsupported fields.

## Excluded transient state

GPU buffers, evaluated meshes, native file path, current tool, incomplete creation previews, text-entry drafts, hover state, error messages, and session undo/redo stacks are not scene data. Opening a project starts a fresh undo history while retaining parametric editability. The next created object receives an ID beyond existing objects.

## File operations

- New, Open, Save, and Save As use the editor header and standard Cmd/Ctrl+N/O/S and Cmd/Ctrl+Shift+S shortcuts.
- Save and Open use native dialogs filtered to `.osk`. A missing/different save extension receives an appended `.osk`; a changed target that already exists requires replacement confirmation.
- Save As changes the active path only after a successful write. Canceling a dialog leaves the scene and path intact.
- Dirty tracking compares the current serialized scene to the last successfully saved/loaded state. Geometry, camera, selection, and snapping changes therefore count as scene changes. Undoing back to that state clears the indicator. Idle tool changes do not mark the project dirty.
- New, Open, and window close offer Save, Discard, or Cancel when the scene has unsaved changes. A failed/canceled save does not proceed with the destructive action.
- Finish or cancel an active creation preview before saving. Invalid property drafts must be corrected; Save does not silently serialize different values from the displayed draft.

## Validation and safe writes

Before saving or accepting a loaded document, validate format/version, all numeric bounds, unique nonzero object IDs, selected-object references, camera values, and actual Truck geometry evaluation. Unknown properties are rejected so an older application cannot silently erase newer data on resave.

Current limits are 16 MiB per document, 1,000 objects, and one million evaluated triangles per saved/loaded scene. Object parameter limits match the modeling controls. JSON cannot encode non-finite values. Input bytes are bounded before parsing, and geometry is evaluated before replacing the active document.

Write a complete temporary file beside the destination, synchronize its contents, then atomically replace the target with `tempfile::NamedTempFile::persist`. On write/replacement failure, preserve the last valid destination and keep the project dirty. This avoids partial-file overwrites; it is not an autosave, backup, or guarantee against every filesystem/power-loss scenario.

Schema and modeling versions 1–3 are supported. Older documents migrate to schema/modeling version 3 in memory, defaulting missing `component_edits` to an empty list. Modeling version 1 also gains coplanar profile partitioning; overlapping profiles split older regions, and component selection resets to Volume because topology slots change. Version 2 component selections remain intact. The existing file is not rewritten until Save. New saves use schema/modeling version 3; nonempty component edits cannot be saved under earlier versions. Unsupported versions are rejected without changing the open scene. Keep the format identifier and both version fields when writing a project.

Component slots refer to faces, unique topology edges, or unique topology vertices in this model evaluation, never GPU triangles or sampled curve vertices. Validate slot bounds against regenerated geometry. They are selection state, not feature dependencies or a general solution to persistent topological naming. Component transforms retain slots while source connectivity is unchanged. Undo/redo keeps component selection when its slot remains valid; future dependent CAD face operations still require semantic references.

## Verification

`cargo test` includes a project round-trip/failure-path check covering all four shape types, nontrivial transforms, exact parameter restoration, subsequent editing and new IDs, overwrite, malformed data, invalid versions/references/camera, and preservation of an existing file after failed saves.

Native review should create and edit geometry, save, start a new project, reopen, inspect restored camera/snapping/selection and parameters, edit again, and save to the same file. Also check Save As, canceled dialogs, unsaved-change prompts, and a failed Open that leaves the scene untouched. Cross-platform build checks do not prove native dialog behavior on every platform.

Native macOS verification completed: Save → New → Open restored the edited solid, camera, selection, and disabled snapping; an additional extrusion edit saved back successfully. Canceling the unsaved-change prompt and Save As preserved the scene/path. A malformed `.osk` was rejected visibly while the current scene remained intact. Windows and Linux native-dialog runtime verification remains pending.

## Coplanar region evaluation (modeling version 2)

Later-created closed profiles (higher object IDs) exclude their overlapping area from older coplanar profiles. Evaluation splits intersected boundary edges, assembles outer and hole loops, and extrudes those Truck faces, including the new side walls. A smaller profile stays at its own extrusion height; it is not raised by editing the surrounding profile. A cutter reaching the outer boundary produces a notch; a cutter crossing the whole profile can produce disconnected regions. Overlapping cutters exclude their union. Fully covered profiles remain editable authored objects with empty evaluated geometry.

Relationships are derived from saved profile parameters and stable creation IDs, so moving, resizing, deleting, undoing, and reloading regenerate the partition. There are no baked display-only holes. This rule applies to coplanar base profiles, not arbitrary intersecting solids or elevated solid faces. Open lines do not partition regions yet. Curved profiles participating in a split use polygonal boundaries with local chord tolerance of approximately 0.005 units (16–512 full-circle segments); untouched profiles retain the existing analytic Truck curves. Exact analytic curved intersections remain future work.

Split evaluation limits a region arrangement to 8,192 input boundary segments and reports failure before committing an edit if that budget or boundary assembly fails. Native macOS review verified a boundary notch, live outer-extrusion changes, and undo restoration. Regression checks cover holes, boundary crossings, disconnected output, overlapping exclusions, circular boundaries, different planes, history, and `.osk` round-trip regeneration.

## Component transforms (schema/modeling version 3)

Each `component_edits` entry stores selection `mode` (`face`, `edge`, or `point`), `element` output slot, a `topology` connectivity signature (unique edge endpoint slots and per-face edge slots), and a column-major affine `matrix` expressed in the owner's local coordinates. Replay starts from freshly evaluated source geometry and applies edits in order. Whole-volume transforms continue using the existing position/rotation/scale properties. No viewport mesh, gizmo geometry, drag preview, or custom pivot is serialized.

A face selects its boundary vertices and curves; an edge selects its endpoints and full curve; a point selects one topology vertex. Shared boundaries receive one consistent transform. Adjacent surface samples interpolate displacement from their boundary influence, and the regenerated faceted shell is checked with Truck for geometric consistency and, for solids, closed manifold topology and positive signed volume. Independently tessellated boundary samples are reconciled within 1e-6 local units. Curved/nonplanar component output is an approximation; editable source curves and operation matrices remain in the document. General self-intersection detection and exact analytic surface reconstruction remain outside this implementation.

An edit's connectivity signature must match before replay. Source dimension/pose changes that retain connectivity can reevaluate; a topology change or invalid deletion is rejected transactionally. Component edits of profiles that exclude regions from other bodies are explicitly rejected until dependent cutting-profile reconstruction is supported. Up to 100 component edits per body are accepted, with bounded topology data, finite affine matrices, and the existing document/geometry budgets. Clearing component edits in the Volume inspector is undoable.
