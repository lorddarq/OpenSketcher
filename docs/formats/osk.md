# OpenSketcher project format (.osk)

Schema version: 1. Modeling semantics version: 1.

An `.osk` file is a UTF-8 JSON document identified by `format: "OpenSketcher"`. The extension is specific to OpenSketcher; the encoding is documented and does not encrypt the scene. The native project stores authored parameters, not only viewport triangles. Truck reevaluates the parameters to restore editable geometry.

## Serialized scene

| Field | Meaning |
| --- | --- |
| `format` | Required `OpenSketcher` identifier. |
| `schema_version` | File layout version; currently 1. |
| `modeling_version` | Evaluation semantics version; currently 1. |
| `bodies` | Ordered scene objects with stable nonzero IDs. |
| `camera` | Orthographic preset/free-view label, yaw, pitch, span, and target. Angles are radians; span is the visible vertical extent. |
| `snapping` | Whether creation snapping is enabled. |
| `selected` | Selected object's ID, or null. |

Each object retains `id`, `shape`, `width`, `depth`, `extrusion`, `position`, `rotation`, and `scale`. Shape values are `rectangle`, `circle`, `line`, and `semicircle`. Dimensions and positions use the current application's unitless world coordinates; rotation uses degrees in local X/Y/Z order. Scale is a positive factor per local axis. Extrusion follows local +Z before object rotation/scaling. Circle/semicircle width is diameter; line width is length. Rectangle depth is its second dimension. Even currently unused per-shape fields are preserved.

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

Only schema/modeling version 1 is currently supported. Unsupported versions are rejected without changing the open scene. There are no earlier public schema migrations yet. Keep the format identifier and both version fields when writing a project.

## Verification

`cargo test` includes a project round-trip/failure-path check covering all four shape types, nontrivial transforms, exact parameter restoration, subsequent editing and new IDs, overwrite, malformed data, invalid versions/references/camera, and preservation of an existing file after failed saves.

Native review should create and edit geometry, save, start a new project, reopen, inspect restored camera/snapping/selection and parameters, edit again, and save to the same file. Also check Save As, canceled dialogs, unsaved-change prompts, and a failed Open that leaves the scene untouched. Cross-platform build checks do not prove native dialog behavior on every platform.
