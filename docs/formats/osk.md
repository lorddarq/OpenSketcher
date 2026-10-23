# OpenSketcher project format (.osk)

Schema version: 13. Modeling semantics version: 13.

An `.osk` file is a UTF-8 JSON document identified by `format: "OpenSketcher"`. The extension is specific to OpenSketcher; the encoding is documented and does not encrypt the scene. The native project stores authored parameters, not only viewport triangles. Truck reevaluates the parameters to restore editable geometry.

## Serialized scene

| Field | Meaning |
| --- | --- |
| `format` | Required `OpenSketcher` identifier. |
| `schema_version` | File layout version; currently 13. Versions 1–12 migrate on load. |
| `modeling_version` | Evaluation semantics version; currently 13. |
| `bodies` | Ordered scene objects with stable nonzero IDs. |
| `camera` | Orthographic preset/free-view label, yaw, pitch, span, and target. Angles are radians; span is the visible vertical extent. |
| `snapping` | Master geometry/grid snapping switch. |
| `drawing` | Project preferences: `geometry`, `grid_axes`, `project_to_base_plane`, and `cut_on_creation`. |
| `selected` | Selected owner object's ID, or null. |
| `selection_mode` | `volume`, `face`, `edge`, or `point`. |
| `selected_element` | Kernel topology output slot for a component, or null for Volume/no selection. |

Each object retains `id`, `shape`, `width`, `depth`, `extrusion`, `cuts_profiles`, `position`, `rotation`, `scale`, and ordered `component_edits`. Shape values are `rectangle`, `circle`, `line`, and `semicircle`. Dimensions and positions use the current application's unitless world coordinates; rotation uses degrees in local X/Y/Z order. Scale is a positive factor per local axis. Extrusion follows local +Z before object rotation/scaling. Circle/semicircle width is diameter; line width is length. Rectangle depth is its second dimension. Even currently unused per-shape fields are preserved.

Camera data includes `view` (`isometric`, `top`, `side`, or `free`), `yaw`, `pitch`, `span`, and `target`. It restores the actual camera orientation and framing, not just the preset label.

All currently implemented scene properties are serialized. Materials, textures, independent saved scenes, NURBS cages, and deformation features do not yet exist in the application model; this version cannot encode those future features. Their introduction requires an explicit schema/modeling version decision and migration support. Do not silently drop unsupported fields.

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

Schema and modeling versions 1–13 are supported. Older documents migrate to schema/modeling version 13 in memory, defaulting missing `component_edits` to an empty list and drawing preferences to their defaults. Existing profiles from modeling versions 2–3 receive `cuts_profiles: true`, preserving their automatic partitioning. Version 1 retains independent geometry with cutting flags false. New profiles default to `cuts_profiles: false`, and `drawing.cut_on_creation` remains false until explicitly enabled. Modeling version 1 retains independent geometry; its component selection resets to Volume for compatibility. Version 2 component selections remain intact. The existing file is not rewritten until Save. New saves use schema/modeling version 13; nonempty component edits cannot be saved under versions earlier than 3. Unsupported versions are rejected without changing the open scene. Keep the format identifier and both version fields when writing a project.

Component slots refer to faces, unique topology edges, or unique topology vertices in this model evaluation, never GPU triangles or sampled curve vertices. Validate slot bounds against regenerated geometry. They are selection state, not feature dependencies or a general solution to persistent topological naming. Component transforms retain slots while source connectivity is unchanged. Undo/redo keeps component selection when its slot remains valid; future dependent CAD face operations still require semantic references.

## Verification

`cargo test` includes a project round-trip/failure-path check covering all four shape types, nontrivial transforms, exact parameter restoration, subsequent editing and new IDs, overwrite, malformed data, invalid versions/references/camera, and preservation of an existing file after failed saves.

Native review should create and edit geometry, save, start a new project, reopen, inspect restored camera/snapping/selection and parameters, edit again, and save to the same file. Also check Save As, canceled dialogs, unsaved-change prompts, and a failed Open that leaves the scene untouched. Cross-platform build checks do not prove native dialog behavior on every platform.

Native macOS verification completed: Save → New → Open restored the edited solid, camera, selection, and disabled snapping; an additional extrusion edit saved back successfully. Canceling the unsaved-change prompt and Save As preserved the scene/path. A malformed `.osk` was rejected visibly while the current scene remained intact. Windows and Linux native-dialog runtime verification remains pending.

## Coplanar region evaluation (modeling version 2)

Later-created closed profiles (higher object IDs) with `cuts_profiles: true` exclude their overlapping area from older coplanar profiles. From modeling version 4, this intent is explicit and false for newly created profiles by default. Evaluation splits intersected boundary edges, assembles outer and hole loops, and extrudes those Truck faces, including the new side walls. A smaller profile stays at its own extrusion height; it is not raised by editing the surrounding profile. A cutter reaching the outer boundary produces a notch; a cutter crossing the whole profile can produce disconnected regions. Overlapping cutters exclude their union. Fully covered profiles remain editable authored objects with empty evaluated geometry.

Relationships are derived from saved profile parameters and stable creation IDs, so moving, resizing, deleting, undoing, and reloading regenerate the partition. There are no baked display-only holes. This rule applies to coplanar base profiles, not arbitrary intersecting solids or elevated solid faces. Open lines do not partition regions yet. Curved profiles participating in a split use polygonal boundaries with local chord tolerance of approximately 0.005 units (16–512 full-circle segments); untouched profiles retain the existing analytic Truck curves. Exact analytic curved intersections remain future work.

Split evaluation limits a region arrangement to 8,192 input boundary segments and reports failure before committing an edit if that budget or boundary assembly fails. Native macOS review verified a boundary notch, live outer-extrusion changes, and undo restoration. Regression checks cover holes, boundary crossings, disconnected output, overlapping exclusions, circular boundaries, different planes, history, and `.osk` round-trip regeneration.

## Component transforms (schema/modeling version 3)

Each `component_edits` entry stores selection `mode` (`face`, `edge`, or `point`), `element` output slot, a `topology` connectivity signature (unique edge endpoint slots and per-face edge slots), and a column-major affine `matrix` expressed in the owner's local coordinates. Replay starts from freshly evaluated source geometry and applies edits in order. Whole-volume transforms continue using the existing position/rotation/scale properties. No viewport mesh, gizmo geometry, drag preview, or custom pivot is serialized.

A face selects its boundary vertices and curves; an edge selects its endpoints and full curve; a point selects one topology vertex. Shared boundaries receive one consistent transform. Adjacent surface samples interpolate displacement from their boundary influence, and the regenerated faceted shell is checked with Truck for geometric consistency and, for solids, closed manifold topology and positive signed volume. Independently tessellated boundary samples are reconciled within 1e-6 local units. Curved/nonplanar component output is an approximation; editable source curves and operation matrices remain in the document. General self-intersection detection and exact analytic surface reconstruction remain outside this implementation.

An edit's connectivity signature must match before replay. Source dimension/pose changes that retain connectivity can reevaluate; a topology change or invalid deletion is rejected transactionally. Component edits of profiles that exclude regions from other bodies are explicitly rejected until dependent cutting-profile reconstruction is supported. Up to 100 component edits per body are accepted, with bounded topology data, finite affine matrices, and the existing document/geometry budgets. Clearing component edits in the Volume inspector is undoable.

## Drawing placement and preferences (schema/modeling version 4)

`drawing` defaults to `{ "geometry": true, "grid_axes": true, "project_to_base_plane": false, "cut_on_creation": false }`. These project-scoped preferences are separate from the master `snapping` toggle and from individual bodies' saved `cuts_profiles` flags. Changing preferences does not rewrite existing geometry or cut flags.

Creation captures an orthonormal frame from the selected planar face, or the first visible planar face, falling back to XY. Explicit base-XY projection applies only without a selected support. With geometry snapping enabled, a nearby visible boundary can acquire a plane just outside a silhouette; standalone boundary references retain their actual Z. Ray intersections and grid/anchor alignment stay in this captured frame. Visible endpoints, sampled-arc-length edge midpoints, and boundary edges precede coordinate rounding; projected boundary limits infer coordinates across the plane. Off-plane and occluded base targets do not alter its height. Disabling snapping leaves support-face placement intact.

The frame is encoded by each resulting profile's existing position and Euler rotation, preserving its actual surface height and extrusion normal. Temporary support choice, anchor, target label, and guide are not serialized. This is fixed placement, not a saved parent/face dependency. Curved or nonplanar support and edge-on ray intersections are rejected explicitly. Arbitrary solid-face imprint/boring remains separate scope.

Regression checks cover elevated/vertical supports, visible geometry references, non-grid bounds, projected limits, disabled snapping, default independent geometry, opt-in cuts, version-3 cut preservation, preference round-trips, coincident profile picking, and coplanar render partitioning. Native macOS drawing and Save verified a line and rectangle at the top face's exact Z=2.3 with cutting off; the supporting solid remained intact.

## Face modifiers (schema/modeling version 5)

Each body has a `modifiers` list (absent in legacy documents, default empty). Entries contain `kind` (`chamfer` or `bevel`), `face` (`top`, `bottom`, `left`, `right`, `front`, `back`), positive finite `amount` in source-local units, `style` (`flat` or `rounded`), and `segments` (2–16). Chamfer requires flat style. Segment count controls noncircular rounded approximations; circular roundovers remain analytic and use viewport tessellation internally. The face role is a source reference, not an output face slot.

Version 5 allowed at most six perimeter entries, one per source face. Version 9 additionally supports selected source edges as described below. All six source faces are supported for rectangles; circular and semicircular solids support top/bottom perimeter treatments. Limits conservatively keep amount below half of the applicable source dimensions/extrusion. Missing/consumed source faces and invalid solids fail transactionally. Treatment/component replay requires compatible topology; cutting-dependent and partitioned source profiles remain unsupported.

Replay reconstructs the source solid and applies all stored modifiers; pose is applied afterwards. Circular rims are built by revolving an authored radial section with lines or quarter-circle arcs. Other convex profile treatments use clipping and grouped transition surfaces. Render subdivisions and revolution patch seams are excluded from selectable topology; the base face and transition band remain separate surfaces. Exports consume the same evaluated result; current STEP output is still faceted.

Schema/modeling versions earlier than 5 reject nonempty modifiers. Legacy files migrate to version 13 with an empty list, preserving existing geometry and cutting semantics. Add/edit/remove and style changes use the existing body-level undo history.

## Multiple selection (schema/modeling version 5)

`selection` is an ordered list of `[body_id, component_slot_or_null]` pairs, interpreted in `selection_mode`. Volume entries use null; other modes require a valid component slot. Duplicates, missing references, and more than 32,768 entries are rejected. The last pair must agree with `selected`/`selected_element`, preserving a primary item for support-plane placement and existing editor behavior. Missing/empty `selection` falls back to the legacy primary item. New/Open, mode changes, and deletion reset or filter the set; undo/redo retain entries whose topology slots remain valid.

Component edits may store additional `elements` alongside the legacy `element`. Both identify one union of selected components in the saved topology signature; shared boundary vertices transform once. Legacy edits default `elements` to empty. Nonempty additional elements and selections of more than one item require version 5. Group gestures and volume deletions validate and commit a whole candidate scene as one undoable transaction.

### Primitive source-face offset

The numeric Offset face control updates existing dimensions/placement and retains `edge_treatments` (`modifiers` is accepted as a legacy field alias). It does not store a mesh transform or a separate offset feature. The transient relative-distance field resets when selection changes; saved source parameters regenerate the result. General offsets and arbitrary output-reference tracking remain specified future operations; attached target-linked cutters are described under version 6.

## Attached sweeps and boring (schema/modeling version 6)

`Body.bore` is optional (default null). It contains `target` (earlier body ID), semantic planar `face`, orthonormal `reference_frame` (column-major 4×4 initial support frame), signed world-unit `travel`, and `mode`: `generate`, `borer`, or `generator_and_borer`. Positive travel follows the outward support normal; negative travel goes inward. Attached profiles retain their shape dimensions and reference placement, with source `extrusion` zero. The current support frame maps this placement onto the edited target face. Invalid frames, missing targets/faces, unsupported deformed supports, and out-of-range travel are rejected before replacing the document.

The target remains under its original ID; the attached profile ID owns any separately generated exit body. Pocket/exact-exit Borer outputs have no separate solid. At zero travel the profile remains visible. Evaluated kernel boundaries, solid classification, pending jobs, and meshes are derived and not serialized. Version declarations below 6 reject nonnull `bore` fields. Legacy independent profiles acquire no attachment during migration.

Scene edits and loading evaluate on a background worker. Save/export wait for pending edits to finish or be cancelled, so only committed validated parameters are written.

### Version 7: ordered Boolean results

Version 7 introduced ordered Boolean results. Each body may contain nullable `boolean`:
`kind` (`union`, `subtract`, `intersect`), `operands` (two ordered `[body_id, face_index_or_null]` pairs), `topology` (two guarded source topologies), and `keep_tools`.
The first operand is the target. Both operand IDs precede the result ID. Face indices must both be present or both null. Boolean result bodies do not accept primitive transforms, bore operations, or component edits; their source bodies remain editable. Consumed source geometry is suppressed only during evaluation, preserving parametric source data and undo. Versions 1–6 cannot contain Boolean features.

### Version 8: asymmetric chamfer definitions

Version 8 introduced asymmetric definitions. `edge_treatments` entries additionally retain `chamfer` and `reverse`. `chamfer` is a tagged object: `{ "mode": "equal" }`, `{ "mode": "two_distances", "second": 0.5 }`, or `{ "mode": "distance_angle", "degrees": 30.0 }`. `amount` remains the first setback. The second setback is `amount * tan(degrees)` for distance/angle, in source-local units. Reverse swaps the first/second face assignments. The angle reference follows the first-distance face. Missing `chamfer`/`reverse` default to equal/false; older version declarations reject non-equal or reversed data. Bevel does not accept asymmetric chamfer fields.

STEP interchange does not alter `.osk` data: nonuniformly scaled revolutions are converted to exact rational NURBS only in the compressed export copy. Native authored geometry and cached kernel boundaries remain unchanged.

### Version 9: selected source edges

Version 9 introduced selected source edges. An `edge_treatments` entry may contain nullable `edge`, a second semantic source-plane role. With `edge: null` (the default), the operation treats the `face` perimeter as before. A non-null neighbor restricts a rectangular source operation to the shared edge of the two perpendicular supporting planes. Both roles survive source pose/dimension changes; output edge indices are not stored. Versions earlier than 9 reject non-null edge targets. Duplicate unordered edge pairs and edge/perimeter overlaps reject; up to eighteen distinct supported operations may be stored. Multiple selected inputs use the existing model transaction/history rather than adding document-level grouping data.

Rejected property drafts and contextual evaluation errors are transient. `.osk` Save always serializes the accepted model, even while rejected values remain visible in properties; Escape discards that draft. This does not change the document schema.

### Version 10: sweep termination extents

Current writers use schema/modeling version 13. `bore.extent` is a tagged object: `{ "type": "distance" }` (default), `{ "type": "through_all" }`, `{ "type": "up_to_next", "target": 2 }`, or `{ "type": "up_to_face", "target": 2, "face": "bottom" }`. End-body references must precede the profile ID, preventing cycles. Through all remains scoped to `bore.target`; end targets in other modes are termination references, not additional cutting targets. Missing extents preserve Distance behavior; earlier version declarations reject automatic extents. `travel` retains manual distance and supplies direction in automatic modes. A zero manual value defaults outward for New body and inward for boring modes. Resolved distance is regenerated from dependencies and not serialized.

### Evaluated planar face extrusion (version 11)

`face_extrusions` defaults to an empty list. Each ordered operation stores a semantic `face` (top/bottom/left/right/front/back) and a positive source-local `distance` in millimetres, at most 100. Maximum 64 operations. Evaluation sweeps the surviving trimmed face after edge treatments, sharing its boundary with the existing solid. Cylinder sweep patches remain a single selectable wall; the moved cap remains separately selectable. Existing chamfer/bevel geometry stays in place. Repeated operations target the current cap, preserving regeneration, undo/redo and exact boundary export. Earlier version declarations reject nonempty face extrusions. Version 13 adds the limited inward operation described below. Transition/curved face sweeps, component-deformed and Boolean-result face extrusions remain unsupported and reject atomically.

### Evaluated component edits (version 12)

Each `component_edits` entry now includes `evaluated: true` for newly authored face/edge/point transforms. These edits replay on the generated geometry after all attached bores and face extrusions, and before explicit Boolean operations consume it. Boolean-result edits replay immediately after their own Boolean operation, before downstream Boolean operations. The existing topology fingerprint, shared-boundary reconstruction, validation and atomic history remain in force. Multiple selected edges on one body form one operation.

Missing `evaluated` defaults to false, preserving legacy source-edit evaluation before bores. Earlier schema/modeling declarations reject evaluated edits. Rebuilt upstream geometry that changes topology rejects the edit rather than silently moving a different component. Component movement still uses the existing boundary deformation and faceted CAD fallback; exact analytic retrimming of deformed curved surfaces remains pending.

### Signed face extrusion (version 13)

Face extrusion distances may be negative (nonzero, finite, within ±100 source-local millimetres). Positive travel adds material; negative travel subtracts material. Passing the far wall opens a through-cut and does not create an emerging body. Authored travel is retained even when it overruns the solid. Existing chamfer/bevel geometry stays in place wherever it is outside the removed volume.

Blind recesses behind fully treated perimeters and through-cuts contained by planar exit caps use exact shared boundaries. Matching entry/exit rims connect to the existing exit edges without zero-area caps. Other exit intersections use the existing solid subtractor. Untreated primitive face cuts reuse source offset while blind; removing an entire cross-section through its full depth yields an empty evaluated result. The authored body stays in history for regeneration and undo.

Consecutive cap sweeps that include negative travel combine geometrically before evaluation, so reversal retracts an extension instead of adding coincident walls. All-outward histories retain their existing seams/topology for compatibility. Authored operation entries remain unchanged for undo and serialization. A zero net sweep restores the original boundary. Schema/modeling versions earlier than 13 reject negative face-extrusion distances. Arbitrary curved/evaluated-result face extrusion remains a separate outstanding operator requirement.
