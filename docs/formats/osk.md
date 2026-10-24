# OpenSketcher project format (.osk)

Schema version: 45. Modeling semantics version: 45.

An `.osk` file is a UTF-8 JSON document identified by `format: "OpenSketcher"`. The extension is specific to OpenSketcher; the encoding is documented and does not encrypt the scene. The native project stores authored parameters, not only viewport triangles. Truck reevaluates the parameters to restore editable geometry.

## Serialized scene

| Field | Meaning |
| --- | --- |
| `format` | Required `OpenSketcher` identifier. |
| `schema_version` | File layout version; currently 45. Versions 1–44 migrate on load. |
| `modeling_version` | Evaluation semantics version; currently 45. |
| `bodies` | Ordered scene objects with stable nonzero IDs. |
| `camera` | Orthographic preset/free-view label, yaw, pitch, span, and target. Angles are radians; span is the visible vertical extent. |
| `snapping` | Master geometry/grid snapping switch. |
| `drawing` | Project preferences: `geometry`, `grid_axes`, `project_to_base_plane`, and `cut_on_creation`. |
| `selected` | Selected owner object's ID, or null. |
| `selection_mode` | `volume`, `face`, `edge`, or `point`. |
| `selected_element` | Kernel topology output slot for a component, or null for Volume/no selection. |

Each object retains `id`, `shape`, `width`, `depth`, `extrusion`, `cuts_profiles`, `position`, `rotation`, `scale`, and ordered `component_edits`. Shape values are `rectangle`, `circle`, `line`, and `semicircle`. Dimensions and positions use the current application's unitless world coordinates; rotation uses degrees in local X/Y/Z order. Scale is a positive factor per local axis. Extrusion follows local +Z before object rotation/scaling. Circle/semicircle width is diameter; line width is length. Rectangle depth is its second dimension. Even currently unused per-shape fields are preserved.

Camera data includes `view` (`isometric`, `top`, `side`, or `free`), `yaw`, `pitch`, `span`, and `target`. It restores the actual camera orientation and framing, not just the preset label.

All currently implemented scene properties are serialized. Materials, textures, independent saved scenes and arbitrary custom cage bindings do not yet exist in the application model; this version cannot encode those future features. Their introduction requires an explicit schema/modeling version decision and migration support. Do not silently drop unsupported fields.

## Excluded transient state

GPU buffers, evaluated meshes, transient spatial acceleration indices, derived cylinder-wall/rim continuity groups and internal-edge masks, native file path, current tool, incomplete creation previews, text-entry drafts, hover state, error messages, and session undo/redo stacks are not scene data. Opening a project starts a fresh undo history while retaining parametric editability. The next created object receives an ID beyond existing objects.

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

Schema and modeling versions 1–41 are supported. Older documents migrate to schema/modeling version 41 in memory, defaulting missing `component_edits` to an empty list and drawing preferences to their defaults. Existing profiles from modeling versions 2–3 receive `cuts_profiles: true`, preserving their automatic partitioning. Version 1 retains independent geometry with cutting flags false. New profiles default to `cuts_profiles: false`, and `drawing.cut_on_creation` remains false until explicitly enabled. Modeling version 1 retains independent geometry; its component selection resets to Volume for compatibility. Version 2 component selections remain intact. The existing file is not rewritten until Save. New saves use schema/modeling version 41; nonempty component edits cannot be saved under versions earlier than 3. Unsupported versions are rejected without changing the open scene. Keep the format identifier and both version fields when writing a project.

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

Schema/modeling versions earlier than 5 reject nonempty modifiers. Legacy files migrate to version 41 with an empty list, preserving existing geometry and cutting semantics. Add/edit/remove and style changes use the existing body-level undo history.

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

Current writers use schema/modeling version 41. `bore.extent` is a tagged object: `{ "type": "distance" }` (default), `{ "type": "through_all" }`, `{ "type": "up_to_next", "target": 2 }`, or `{ "type": "up_to_face", "target": 2, "face": "bottom" }`. End-body references must precede the profile ID, preventing cycles. Through all includes `bore.target` and optional version 21 scope IDs; end targets in other modes are termination references, not additional cutting targets. Missing extents preserve Distance behavior; earlier version declarations reject automatic extents. `travel` retains manual distance and supplies direction in automatic modes. A zero manual value defaults outward for New body and inward for boring modes. Resolved distance is regenerated from dependencies and not serialized.

### Evaluated planar face extrusion (version 11)

`face_extrusions` defaults to an empty list. Each ordered operation stores a semantic `face` (top/bottom/left/right/front/back) and a positive source-local `distance` in millimetres, at most 100. Maximum 64 operations. Evaluation sweeps the surviving trimmed face after edge treatments, sharing its boundary with the existing solid. Cylinder sweep patches remain a single selectable wall; the moved cap remains separately selectable. Existing chamfer/bevel geometry stays in place. Repeated operations target the current cap, preserving regeneration, undo/redo and exact boundary export. Earlier version declarations reject nonempty face extrusions. Version 13 adds the limited inward operation described below. Transition/curved face sweeps, component-deformed and Boolean-result face extrusions remain unsupported and reject atomically.

### Evaluated component edits (version 12)

Each `component_edits` entry now includes `evaluated: true` for newly authored face/edge/point transforms. These edits replay on the generated geometry after all attached bores and face extrusions, and before explicit Boolean operations consume it. Boolean-result edits replay immediately after their own Boolean operation, before downstream Boolean operations. The existing topology fingerprint, shared-boundary reconstruction, validation and atomic history remain in force. Multiple selected edges on one body form one operation.

Missing `evaluated` defaults to false, preserving legacy source-edit evaluation before bores. Earlier schema/modeling declarations reject evaluated edits. Rebuilt upstream geometry that changes topology rejects the edit rather than silently moving a different component. Component movement still uses the existing boundary deformation and faceted CAD fallback; exact analytic retrimming of deformed curved surfaces remains pending.

### Signed face extrusion (version 13)

Face extrusion distances may be negative (nonzero, finite, within ±100 source-local millimetres). Positive travel adds material; negative travel subtracts material. Passing the far wall opens a through-cut and does not create an emerging body. Authored travel is retained even when it overruns the solid. Existing chamfer/bevel geometry stays in place wherever it is outside the removed volume.

Blind recesses behind fully treated perimeters and through-cuts contained by planar exit caps use exact shared boundaries. Matching entry/exit rims connect to the existing exit edges without zero-area caps. Other exit intersections use the existing solid subtractor. Untreated primitive face cuts reuse source offset while blind; removing an entire cross-section through its full depth yields an empty evaluated result. The authored body stays in history for regeneration and undo.

Consecutive cap sweeps that include negative travel combine geometrically before evaluation, so reversal retracts an extension instead of adding coincident walls. All-outward histories retain their existing seams/topology for compatibility. Authored operation entries remain unchanged for undo and serialization. A zero net sweep restores the original boundary. Schema/modeling versions earlier than 13 reject negative face-extrusion distances. Arbitrary curved/evaluated-result face extrusion remains a separate outstanding operator requirement.

### Shell / thickness (version 14)

Each body optionally stores `hollow: { "opening": "top", "thickness": 0.2 }`; missing/null means no Shell operation. `opening` uses the six existing semantic source-face roles. Boxes support all six; cylinders/semicylinders support top/bottom caps. Thickness is finite, positive and source-local before placement/scaling. It must leave a nonempty inset cross-section and a positive back wall. Semicircle arc/chord offsets impose an additional inradius limit (one quarter of source width).

Shell evaluation precedes face extrusion and component editing, reusing exact shared pocket boundaries. Unsupported source treatment/Boolean/source-deformation/partition combinations reject without replacing committed data. Source body dimensions are retained rather than overwritten by cavity dimensions. Removing `hollow` restores the original solid. Schema/modeling versions earlier than 14 reject nonnull `hollow`; old documents migrate with null and unchanged geometry.

### Draft (version 15)

Each body optionally stores `taper: { "degrees": 3.0, "neutral": "bottom" }`. Missing/null preserves the original undrafted solid. Neutral uses the existing `top`/`bottom` semantic cap roles. A positive angle expands the opposite section; a negative angle contracts it. Angles are finite within ±80 degrees, applied before object scaling. Collapse/inversion rejects atomically, including the semicylinder's arc/diameter intersection limit. A zero angle retains the Draft feature with unchanged shape. Older version declarations reject nonnull `taper`.

Draft evaluates on the source box/cylinder/semicylinder before face extrusions and evaluated component edits. Combinations with shell, source edge treatments, source deformation, partitions or Boolean-result source bodies currently reject until general trimming supports them. Authored dimensions and placement remain unchanged; derived cap dimensions and exact shared conical/parabolic boundaries are regenerated. Removing `taper` restores the original source prism. Viewport tessellation is not the stored draft geometry.

### Outward shell direction (version 16)

`hollow.outward` is a boolean, defaulting to `false` when missing. `true` grows the outer walls and back outside the source while preserving the source cross-section as the cavity. The selected opening plane remains fixed. Both schema and modeling versions must be at least 16 for outward shelling; older declarations reject it. The feature was introduced in version 16; current writer/reader versions are listed above.

Thickness remains a positive source-local distance applied before object scaling. Inward shelling requires a nonempty inset and back wall; outward thickness is bounded to 100 local units and does not shrink the original cavity. Semicylinders offset the circular support and straight diameter independently, with shared exact arc/line corner intersections. Derived outer support planes, attached profile support frames and target thickness include the outward offset. Source dimensions and placement are retained unchanged.

Signed cap sweeps operate on the actual shell boundary. Retracting an annular opening beyond its cavity floor removes the surrounding walls but preserves the central back material beneath that cavity, including overrun; this is distinct from pushing a filled cap through a solid. Back-cap subtraction thins/removes the back and then shortens the remaining side walls. These cases use exact boundary construction with shared edges, rather than coincident-face Boolean intersection.

### Closed source cavity (version 17)

`hollow.closed` is a boolean defaulting to `false`. A true value keeps every exterior cap, constructing a separate inward-oriented cavity boundary. Inward Shell requires thickness below half the source extrusion as well as the existing cross-section inset limits. Outward Shell grows both exterior caps and all supported outer side surfaces; the original volume becomes the cavity. `opening` remains serialized as the construction-frame reference and the opening to restore when `closed` is turned off. Both schema/modeling versions must be at least 17 for closed Shell. The feature was introduced in version 17; current writer/reader versions are listed above.

Inner/outer BReps are derived and are not serialized. Exported STEP distinguishes void boundaries from disconnected material islands. STL retains inward cavity triangle orientation and validates closed topology for both boundaries.

### Exact planar component retrim (version 18)

`component_edits[].retrim` defaults false. True is valid only for Face selection and requires schema/modeling 18. Its existing matrix transforms selected support planes on convex planar solids; neighboring planes are retained and exact intersections rebuild corners and trims. Topology references/element lists retain their existing guarded semantics. Results must keep compatible connectivity, noncollapsed edges and a bounded convex solid. Unsupported curvature/concavity or lost connectivity rejects rather than replaying the same true flag as a different deformation. Old missing/false edits preserve the previous weighted component behavior.

### Evaluated planar sweeps (version 19)

Optional `component_edits[].sweep` is null/missing for existing transforms, or an owner-local `[x,y,z]` displacement for an ordered planar-face extrusion. Non-null requires evaluated Face mode, a single element, `retrim: false`, identity `matrix`, finite nonzero vector components within ±10,000 local units and schema/modeling 19. The UI accepts signed normal travel within ±100 world mm and converts it through the owner placement. Source scale/placement changes then regenerate from the local vector. Connectivity guards and last-valid-model error recovery apply at the operation's actual history position. The saved shape is still authored geometry/history; no evaluated triangle cache is serialized.

### Construction references (version 20)

`body.construction` is a boolean defaulting false. True marks a rectangular, zero-extrusion reference using existing width/depth/position/rotation/scale and stable ID fields. It cannot carry cutting intent, geometry features, component deformation, a bore or Boolean definition. Its derived scene boundary is shown as a wireframe, retained for plane inference and excluded from every export format.

`drawing.plane` is null/missing for automatic plane acquisition, or a construction object's ID for an explicitly active drawing plane. Selected physical support faces take priority. IDs referencing missing/physical objects reject document loading/saving. The UI clears an active ID when its reference disappears through deletion/history. Active-plane choice is a drawing preference like snapping; object placement/deletion uses model undo. Profiles created on a reference store their captured world placement independently, without a live feature dependency.

Both schema/modeling versions must be at least 20 for construction objects or an active reference ID. Legacy objects default to physical geometry and legacy drawing preferences remain automatic. Construction references were introduced in version 20; current versions are listed at the top.

### Through-all target scope (version 21)

`bore.through_targets` defaults to an empty array. Nonempty lists require schema/modeling 21 and `extent.type: "through_all"`. Up to 64 IDs must be distinct, nonzero, earlier than the owning profile and different from the primary attached `target`. Missing/unsupported target geometry rejects evaluation. The primary target is implicitly included. Extra targets provide the automatic inward travel range and receive subtraction in Borer modes; New body only generates a sweep. Automatic scoped Generator + borer emits no extra body beyond the derived range. Manual Distance mode retains its previous emergence contract.

End distance is regenerated from conservative retained-surface/control-hull bounds and is not authored into `travel`. Target changes, removal and history regenerate all scoped results atomically. The field is cleared by the inspector when switching to another extent type. Legacy empty/missing scopes keep single-target evaluation. Scoped Through all was introduced in version 21; current versions are listed at the top.

Angled planar termination uses the existing Up to face / Up to next tags without new authored fields. Up to face references an infinite source support plane; Up to next resolves a complete finite evaluated planar region and rejects nearer partial/curved obstructions. Derived stop planes, projected wires, and resolved depth ranges are not serialized.

### Version 22: evaluated face deletion and closure

`component_edits[].face_edit` is optional/null for legacy edits. New values are `open`, `cap`, `heal`, and `cap_openings`. The existing `element` plus `elements` reference logical output faces under the stored connectivity signature. `cap_openings` keeps face 0 as an owner/topology anchor and closes all free boundaries. Other values delete their selected face regions. Operations require `evaluated: true`, `mode: "face"`, an identity matrix, `retrim: false`, and `sweep: null`. Versions below 22 reject non-null face operations. Invalid combinations and unresolved/unsupported closure reject transactionally.

Only authored intent/references are serialized; retained shells, derived caps, solid/sheet classification and circular snapping references regenerate on load. Missing face-edit fields preserve existing source/evaluated transforms and face sweeps.

### Version 23: live planar Replace Face

`boolean.kind` additionally accepts `replace_face`, gated to schema/modeling 23. Both `operands` must reference faces under the existing `topology` signatures; both owner IDs must precede the feature ID. The first pick supplies the source solid and the second supplies the live reference plane. `keep_tools` must be false: source output is suppressed, while the reference remains unchanged. A construction plane is permitted only as the second operand. The reference's finite region is not a trimming boundary.

No replacement matrix, derived plane, or resolved mesh is serialized. The result regenerates after its inputs, in the same ordered feature stage as Boolean operations. Removing the feature restores source visibility; removing/changing a required reference rejects atomically. Older files retain their existing Boolean semantics.

### Version 24: Imprint and tangent-aware evaluated sweeps

`boolean.kind: "imprint"` references a solid face first and a coplanar profile face second using existing picks and topology signatures. Source output is replaced by the split solid; the profile is consumed unless `keep_tools` is true. No material is removed by Imprint itself. Face partitions and shared edges regenerate rather than being serialized as viewport triangles.

New evaluated face sweeps store `tangent_walls: true` beside `sweep`. It groups analytic tangent-continuous side patches and enables the proven direct convex-interior pocket/parallel-exit path. Missing/false preserves legacy sweep grouping/evaluation. True requires a sweep; other operations reject the combination. Either Imprint or true tangent walls requires schema/modeling 24.

### Version 25: whole-profile Imprint tools

For `boolean.kind: "imprint"`, the first operand still requires a solid face element. The second operand may now use a null element, meaning all planar face regions of that retained sheet body. This whole-profile form requires schema/modeling 25; a non-null tool element preserves the existing single-face scope. Both input topology signatures remain mandatory and dependency IDs must precede the feature ID.

No flattened profile, derived island or projected mesh is persisted. Holes, disconnected regions, material parts and snap references regenerate from the authored profile. `keep_tools`, undo/redo, reference failures and input restoration retain their existing behavior.

### Version 26: finite line Imprint

`boolean.kind: "imprint_curve"` references a solid planar face first and an entire unextruded straight-line body second (`[id, null]`). Both input topology guards are retained. The line's finite extent is authoritative; it is not extended or projected to the support. Covered material intervals become shared cut edges and independent face regions, while intervals outside the tool remain unchanged. Lines ending inside material, nonseparating cuts, offset lines and unresolved/tangent intersections reject atomically. Original B-spline/NURBS boundary curves are split analytically, with the same split edges installed on neighboring retained surfaces. Source material is unchanged. Source and line are suppressed; `keep_tools` retains the line. Removing the feature restores inputs. This kind requires both schema and modeling version 26; older Imprint kinds retain their existing behavior.

### Version 27: analytic closed-profile crossings

`boolean.analytic_crossings` is an optional boolean, valid only for `kind: "imprint"`. Newly authored Imprints set it to true; missing/false retains the previous evaluation path. True requires schema and modeling version 27. On a planar support, retained B-spline/NURBS profile and source trims are intersected and split at shared parameter roots. Adjacent source surfaces receive the same split boundary edges. Trimmed tool segments outside the selected face are discarded, including those inside existing holes. Source material remains unchanged; each partition remains an independent logical face. The original operands, topology guards, Keep tools, suppression, removal and subsequent evaluated face edits retain their existing contracts.

Intersection search uses conservative control hulls, bounded subdivision and numerical root refinement at kernel tolerance. Exact overlapping straight trims are reconciled; general nonconic coincident overlaps, unresolved near-tangencies and intersection-curve trims reject atomically. Full retained tool circles keep their analytic snap references; clipped arcs retain their BRep curves without inventing a complete snapping circle outside the result. All-linear Imprints continue through the established exact linear reconciliation path.

Cylinder continuity is derived evaluation metadata and adds no serialized fields. New whole-wall/rim operations explicitly store all member faces/edges in the existing component `element`/`elements` arrays. Kernel connectivity signatures remain unchanged, and earlier authored component operations replay their original explicit picks.

### Version 28: exact complete-section translation

`component_edits[].section_transform` is an optional boolean. New eligible complete-cap/rim translations set it to true; missing/false retains the previous component deformation path. True requires schema and modeling version 28, `evaluated: true`, Face/Edge mode and a translation matrix. It cannot combine with `retrim`, a `sweep` or `face_edit`. Replay retains the original topology guard and complete member selection, recomputes the opposite section and applies an exact affine map to the retained curves/surfaces. Cap consumption, degenerate maps or changed section references reject before replacing the current scene. No evaluated mesh or inferred cylinder axis is serialized.

### Version 29: analytic planar face Booleans

`boolean.analytic_faces` is an optional boolean, valid for Union/Subtract/Intersect with two face operands. New face Booleans set it to true. True requires schema and modeling version 29; missing/false retains the historical triangle-derived polygonal evaluator, including in existing volume-prism operations. No existing feature is silently opted in during migration.

The selected coplanar retained boundaries are intersected and split analytically using the shared Imprint solver. Material-side winding determines which oriented trim pieces bound the requested set. Planar regions retain B-spline/NURBS curves, holes, disconnected outputs and original target-normal orientation; a complete circular output loop retains its center/rim snapping reference. Opposite input normals are normalized to the target's plane. The result is a separate sheet, preserving the established solid-input and Keep tools rules. Topology signatures, live dependencies, ordering and transaction rollback remain unchanged.

Curved-surface/noncoplanar trimming, general nonconic spline overlap merging and intersection-generated input trims remain unsupported and reject atomically. No polygon approximation is substituted for an opted-in analytic operation. Derived meshes, classifications and arrangements are transient.

The v29 analytic face path also merges proven shared rational-conic intervals (including differently parameterized circular/elliptical partial arcs). It splits overlap endpoints, retains one copy of each active interval and preserves complementary arcs. This resolves configurations previously rejected by the v29 solver without opting in legacy operations or adding serialized state. Common-support detection uses homogeneous Bernstein coefficient identities rather than viewport samples. Analytic Imprint reuses shared conic interval handling: already-existing rim pieces are omitted from the new divider set, while interior profile edges form independent regions. A coincident-only profile still rejects because it adds no partition. Missing/false `analytic_crossings` keeps the legacy path.

### Version 30: exact prismatic face through-cuts

New evaluated face sweeps set `component_edits[].prismatic_cut: true` alongside `tangent_walls`. True requires a sweep and tangent-wall grouping, plus schema/modeling version 30. Missing/false retains the historical through-cut evaluator; loading an old file does not opt its existing operations in.

On negative travel reaching/passing the far cap, eligible straight prisms regenerate the remaining cap material as an exact extrusion instead of invoking coincident-surface CSG. Eligibility requires normal sweep travel, two cap levels, axis-parallel planar sides or rational/B-spline sides whose paired homogeneous control rows prove a constant translation between cap planes. The result retains exact curve trims, holes/disconnected regions and tangent wall grouping; consuming the complete cap removes the prism. Blind, tapered, intermediate-section, oblique or unproven geometry keeps the established paths. No derived control proof, cap mesh or rebuilt boundary is serialized.

## Version 31: analytic equal-depth prism Booleans

New volume Union/Subtract/Intersect operations persist `boolean.analytic_prisms: true`. Eligibility reuses the retained-surface straight-prism proof: paired homogeneous control rows must describe the same normal translation, with complete planar end sections. Tapered/twisted/intermediate-section solids and divided cap regions stay on the general path. When eligible equal-depth prisms use the cap-region shortcut, their cap set operation now uses the shared analytic wire arrangement. Circular/elliptical holes and curved trim edges survive as analytic boundaries instead of sampled polygons. The existing sweep builds the resulting solids from those retained regions.

Missing/false retains the legacy prism evaluator; loading/migrating an older file does not opt its existing operations into the new topology. True is valid only for two volume operands and a set operation, requires schema/modeling version 31, and is rejected under older version declarations. Other ineligible volume operations retain their existing general kernel path. `analytic_faces` remains the separate intent for face-set operations.


## Version 32: evaluated planar edge chamfers

`component_edits[].edge_chamfer` optionally stores `[first_distance, second_distance]`, the positive finite setbacks along the two adjacent evaluated support faces, in owner-local units. The first support is the lower face slot in the guarded input topology; reversing distances swaps their values. An equal-distance chamfer has equal entries. Entries require `evaluated: true`, Edge mode, identity `matrix`, and no `retrim`, `section_transform`, `sweep` or `face_edit`. `element` plus `elements` contain explicit selected edges; one entry applies them together against one input topology signature.

Evaluation recomputes exact cut planes from the retained convex planar boundary and reconstructs shared B-rep edges. It supports evaluated Boolean and prior treatment boundaries without writing a mesh snapshot. Unsupported curved/concave boundaries, missing/ambiguous adjacency, stale topology, invalid distances and consumed support faces reject transactionally. Subsequent face edits use the resulting real planar surfaces. Removing the entry replays the input; later entries retain their existing topology guards.

Non-null `edge_chamfer` requires schema and modeling version 32. Missing/null leaves historical operations unchanged. Versions 1–35 migrate without creating new chamfers. Writers use version 32; older version declarations containing this field's non-null value reject explicitly.


## Version 33: local prismatic corner chamfers

`component_edits[].prism_chamfer` is an optional boolean selecting local full-depth corner treatment. True requires `edge_chamfer`; the existing evaluated Edge-mode, identity-matrix, setback validation and topology guards apply. Missing/false retains the version-32 convex clipping path, so migration does not reinterpret accepted chamfers.

New chamfers on nonconvex planar prisms set this flag after proving complete parallel caps and a full-depth selected straight edge. The two adjacent oriented face boundaries determine inner versus outer corner. Inner corners union a triangular cap footprint; outer corners subtract it. Exact planar face Booleans preserve holes and remaining regions; the cap is swept through the proven prism depth. One retained face per connected planar region preserves subsequent rigid face editing. Independently split coplanar supports, incomplete/split cap references, setbacks leaving adjacent faces, overlap/consumption and nonprismatic geometry reject transactionally.

True requires schema and modeling version 33. Versions 1–35 migrate with the flag false. New writers use version 33; no evaluated mesh snapshot, inferred corner classification or cap slot is serialized.


## Version 34: analytic prism bevels

`component_edits[].prism_bevel` optionally stores `{ "amount": 0.2, "style": "rounded" }` or `{ "amount": 0.2, "style": "flat" }`. Rounded amount is the owner-local circular radius; Flat amount is the equal setback distance. Amount must be finite, greater than 0.00001 and at most 10000. The entry requires evaluated Edge mode, identity matrix, explicit `element`/`elements` and the existing topology guard. It cannot combine with `edge_chamfer`, `prism_chamfer`, transforms, sweeps or face operations.

Rounded evaluation constructs a tangent circular cap arc and retains its swept analytic/NURBS band. The requested radius determines both setbacks from the angle between the adjacent planes. Flat evaluation reuses the v33 equal-setback prism chamfer. Both styles use the existing cap Boolean and full-depth sweep path; they add material at inner corners and remove it at outer corners. Revalidate the prism direction between rounded multi-edge treatments so later edges cannot overwrite an earlier bevel. Unsupported or overlapping treatments reject without committing.

Non-null `prism_bevel` requires schema and modeling version 34. Versions 1–35 migrate with null; existing chamfers and source bevel modifiers retain their previous replay. No display segment count, tessellation or evaluated mesh is serialized. Current writers use version 34.


## Version 35: curve-preserving prism treatment replay

`component_edits[].prism_curves` is an optional boolean. True requires either `prism_chamfer: true` with setbacks or non-null `prism_bevel`; all existing evaluated Edge-mode/identity-matrix checks still apply. New local prism treatments set it true, including when the first input is entirely planar. Missing/false preserves the strict planar v33/v34 entry and its prior output ordering.

True permits unrelated curved side walls when the retained boundary proves a straight prism. The selected edge must still have two planar support faces, be straight, and span complete parallel caps. Reuse the existing homogeneous-control-row proof for ruled B-spline/NURBS side walls. The cap Boolean retains existing curves while adding the new flat or rounded footprint. No curved edge or planar/curved junction is silently treated as a planar corner.

The result retains curved faces, groups only connected coplanar patches, sorts independent inner loops while preserving the outer loop, and uses the shared solid canonicalizer to order faces and boundary starts. Thus later explicit edge/face topology guards replay against the same slots instead of depending on hash traversal. This is deterministic regeneration, not general semantic naming across topology changes.

True requires schema/modeling version 35. Versions 1–35 migrate with false; loading does not opt older treatments into new ordering. Writers use version 35.


## Version 36: evaluated convex planar Shell

`component_edits[].face_edit` additionally accepts `{ "shell": { "thickness": 0.1, "outward": false, "closed": false } }`. Existing Open/Cap/Heal unit variants retain their string representation. Shell requires evaluated Face mode, identity transform, the existing topology signature, and explicit opening `element` plus optional `elements`. Thickness must be finite, greater than 0.00001 and no greater than 10000 owner-local units. The payload rejects unknown properties.

Shell offsets the input's retained convex planar supports (4–64 planes) and subtracts the cavity with the existing exact planar solver. Inward keeps the exterior; Outward keeps the original cavity. Open keeps selected opening planes fixed; Closed keeps all exterior faces and produces an enclosed void. The selected source faces remain stored while Closed is enabled, allowing reopening without guessing another face.

Consolidate cavity triangles into retained planar regions before subtraction, then preserve one face per connected region and canonicalize the output solid. Subsequent topology-preserving planar face edits can address inner and outer faces. Concave/curved inputs, consumed cavities, missing openings and invalid boundaries reject transactionally. Source `hollow` definitions keep their existing evaluation; an evaluated Shell is appended after prior component edits rather than inserted before them.

Non-null Shell face edits require schema/modeling version 36. Versions 1–35 migrate without adding a shell. Current writers use version 41; older version declarations containing this variant reject explicitly.


## Version 37: evaluated planar Draft

Evaluated Draft uses an existing evaluated Face `component_edits` entry: `face_edit: {"draft": {"degrees": 3.0, "neutral": 0}}`. `element` and `elements` identify the walls; `neutral` is a distinct zero-based face slot in the guarded **pre-draft** input topology. The transform is identity. Store operator inputs, not evaluated mesh snapshots. Source `taper` remains unchanged and keeps its v15 semantics.

For retained convex straight-edged planar solids with 4–64 supports, rotate selected wall supports about their original intersection lines with the neutral support. Degrees is an **absolute** angle relative to perpendicular to that support, within ±80 degrees. Positive widens away from the neutral plane toward the solid's interior; negative narrows; zero makes selected walls perpendicular, even if they were already drafted. Distances and angles are evaluated in owner-local space before object scaling. Untouched support planes and the neutral trace remain fixed. Reconstruct exact retained planar faces and canonicalize output for later guarded edits.

Reject nonfinite/out-of-range angles, missing or selected neutral slots, parallel wall/neutral planes, consumed supports, invalid solid reconstruction and unsupported curved/concave geometry. A failed live edit retains last accepted geometry/history. New Draft payloads require both version fields ≥37; versions 1–36 migrate without adding operations. Current writers use version 41.


## Version 38: loop-preserving evaluated Draft

New evaluated Draft payloads also store `planar_loops: true`, for example `face_edit: {"draft": {"degrees": 5.0, "neutral": 0, "planar_loops": true}}`. This enables straight-edged planar input with 4–256 supports, including concave Boolean notches, through-holes and blind pockets. Missing/false retains the original v37 convex-only evaluator, with its existing plane reconstruction and output ordering. This flag requires both version fields ≥38; it is not inferred while migrating older files.

Convex inputs with at most 64 supports continue to use convex support reconstruction. Other enabled planar inputs reuse the existing ordered-loop retrimming solver: preserve shared vertices/edges and outer/inner boundary winding, solve intersections of the replacement support planes, and reject collapsed/reversed/crossing loops or incompatible multi-plane corners. If a logical face consists of multiple retained planar patches, consolidate those patches into one connected face before solving. The requested face and neutral slots still refer to the guarded original input. Canonicalize the final retained boundary for downstream guarded edits. The boundary-intersection safety budget remains two million edge/triangle pairs.

Angles remain owner-local and absolute relative to the oriented neutral plane. Positive expands material on the negative side of that plane; on the other side the same signed slope contracts it. This matters when choosing an internal pocket floor or a reference whose extension crosses a concave body. Untouched support planes remain fixed, but their trim loops rebuild. General curved Draft, support-consuming topology changes and construction-plane neutral references remain pending.


## Version 39: live construction-plane Draft neutral references

Evaluated Draft optionally stores `neutral_plane: 9` beside `degrees`, `neutral` and `planar_loops`. The ID identifies an authored construction plane; it is not a snapshot of a normal, origin or display mesh. Missing/null preserves the existing input-face neutral semantics. While non-null, the stored `neutral` face slot is inactive. An external reference may therefore be used even when that inactive slot is missing or belongs to the selected wall set. Switching back to a source face chooses an explicit valid input-face slot. Non-null plane IDs require schema/modeling version 39; zero IDs reject. Older files migrate without adding a reference.

Resolve the plane from the complete authored body list on every evaluated component replay, for source and Boolean owners. Transform its XY support into owner-local space using `inverse(owner) * reference`, derive its oriented normal from the transformed X/Y tangents and ignore its finite drawing dimensions. The neutral support is infinite. Plane translation/rotation and normal reversal remain live; positive in-plane sizing does not change the support. Preserve absolute-angle and signed-side semantics from v38.

Construction planes cannot own modelling operations, so they may precede or follow the referencing solid without creating dependency cycles. Reject a removed reference or an ID belonging to ordinary geometry rather than silently rebinding to a face. Scene validation blocks deletion or conversion of a referenced plane while keeping the committed scene/history intact. Remove the Draft operation or choose another neutral reference before deleting that plane. Evaluation/export/save use the complete scene; standalone evaluation of a body with an external reference reports that the plane is missing. Construction references continue to be excluded from exported geometry.


## Version 40: topology-changing convex face transforms

New Face Move/Rotate component edits on convex straight-edged planar input with at most 64 supports store `convex_rebuild: true`, alongside `evaluated: true`, `retrim: true`, the selected face slots, guarded input topology and owner-local affine matrix. Missing/false preserves the original corner-connectivity solver. True requires an evaluated Face retrim operation and schema/modeling versions ≥40; other combinations reject.

First attempt the existing topology-preserving retrim. If it fails on eligible convex input, transform the selected support planes and rebuild the intersection of the complete half-space set. This can split incompatible multi-plane corners, change edges/vertices and omit planes that become redundant. Consolidate actual retained planar support faces and canonicalize the output boundary for downstream guarded replay. Original source treatments remain in history; a later transform may temporarily consume their faces. Empty/invalid results reject atomically. Existing concave and curved movement paths remain unchanged; general semantic naming across changing topology is not implied.

Component slots refer to the guarded input. Preview selection highlighting is hidden when those slots no longer match the output topology. After a committed transform or history change alters a selected component owner's topology, return the selection to its volume; never reuse an old face/edge/point slot as a different component. The user can then explicitly select a surviving output face for another operation.


## Version 41: convex Draft support consumption

New evaluated Draft operations on convex straight-edged planar input with at most 64 supports reuse the component entry's existing `convex_rebuild: true` field. The `face_edit.draft` payload remains unchanged. In v41, this flag may accompany an evaluated Face Draft with identity matrix and `retrim: false`; its v40 Face Move/Rotate use remains unchanged. Flagged Draft requires schema/modeling versions ≥41, while flagged Move/Rotate still requires ≥40. Missing/false retains the earlier Draft face-count rejection. Recreate an older Draft operation to opt into this new evaluator rather than silently upgrading saved histories.

The convex half-space reconstruction already rebuilds corner incidence. The new intent additionally permits redundant supports to disappear from the retained solid: a shrinking wall can consume a chamfer band or even the original opposite cap while leaving valid closed material. Keep the authored source/treatment inputs and the input neutral reference in history. The output remains the intersection of the drafted and unchanged oriented support planes; it is not a deformation of the old display mesh. Verify retained material closure before committing. Concave/curved Draft and unsupported partitioned supports do not gain topology-changing reconstruction from this flag.

Draft actions select their owning volume, and the inspector uses guarded pre-Draft neutral slots. Later output-face edits must be explicitly selected and retain their own input topology guards; changing the Draft so those guards no longer match rejects atomically. Existing undo/redo/removal and live construction-plane neutral semantics remain intact.


## Authored control nets and soft selection (v42)

An evaluated Face `component_edits` entry may contain `control`, mutually exclusive with other component flags. It requires an identity matrix, exactly one guarded input face and version 42 in both file versions. Missing `control` retains earlier replay. Its strict object fields are `domain` (stable nonzero body-local ID), `binding` (`rows`, `columns`, `uknots`, `vknots`), `version: 1` and `action`. Node identity is the `[row, column]` coordinate in that authored parameter lattice; knot/dimension changes and missing creation references fail rather than rebind.

Actions are `{kind: "create", planar: boolean}`, `{kind: "translate", sources: [[row,column], ...], locked: [[row,column], ...], enabled: boolean, radius: number, delta: [x,y,z]}` and `{kind: "move_face", delta: [x,y,z]}`. `planar: true` creates a fixed 5×5 degree-four clamped planar NURBS definition from an exact parallelogram. False binds a supported existing full clamped NURBS patch and preserves its source weights, knots and degree. Subsequent operations refer to that creation's domain and binding. Boundary nodes are implicitly locked; `locked` captures additional interior locks. Radius is in world millimeters; `delta` is an owner-local displacement converted from the interactive world request. This coordinate choice carries deformation with owner rotation. V1 recomputes shortest world control-edge paths and nearest-source smoothstep from each operation's upstream state, including nonuniform placement scale. Enabled false and radius zero give hard selection.

Replay retains source definitions, exact NURBS surfaces and trim wires. It never serializes display meshes, heatmap colors, active node selection, idle radius, hover or an in-progress drag. Control features currently form a contiguous suffix of component history; other component operators after domain creation reject rather than discard it. Other scene dependents continue through transactional evaluation. The [behavior specification](../specs/soft-selection.md#delivered-control-net-editing-2026-10-24) records supported geometry, validation limits and native evidence.


## Calculated cap rotation (v43)

Newly authored evaluated Face Move/Rotate operations that do not use planar retrimming or the earlier exact section-translation path set `cap_rotation: true`. The flag requires v43 in both file versions. Missing/false retains the previously saved evaluator. The operation retains the existing local matrix, input topology guard and selected elements; its source dimensions remain editable. Complete-boundary rigid transforms still preserve the retained shell directly. Other eligible operations require one complete planar cap, a fixed planar opposite section and ruled neighboring surfaces. Unsupported curved-face combinations reject instead of modifying display triangles.

Rebuild section vertices and curves once through shared kernel entity mapping. Transform only the selected cap's curves; retain the opposite section and update straight connecting edges. Transform the selected control row of each ruled NURBS neighbor, preserving its knots and rational weights. Planar side walls may become exact bilinear ruled patches. A complete conic cap whose center and geometric quadratic form are unchanged by the transform leaves the calculated body unchanged; rotating a circular cap about its cylinder axis must not introduce a seam pinch. Known cylinder patch seams remain one logical wall when the neighbors become a general loft. A changed output topology triggers the existing explicit selection reset.

The display uses surface normals generated by kernel tessellation, transformed with the inverse transpose under nonuniform placement. These normals, overlap-validation indices and meshes are derived cache data and are never native feature inputs. Validation reconciles shared display samples and checks actual overlapping triangle bounds under a bounded workload; display density alone does not trigger a quadratic triangle-count rejection. Original analytic surfaces and curves remain authoritative for STEP and future edits. Older faceted rotations remain unchanged on load; remove/recreate such an operation to adopt the new evaluator.


### Circular and periodic cages (versions 44–45)

Version 44 adds `binding.disk: true` and control algorithm version 2 for a 5×5 circular-cap cage. Its affine reference and source conic regenerate from upstream geometry. The cage displacement is multiplied by `(1-x²-y²)²` in normalized disk coordinates and composed exactly into a degree-eight polynomial NURBS skin. The circular trim and first derivatives remain fixed.

Version 45 adds control algorithm version 3, `binding.periodic: true`, and strictly ordered `binding.patches` containing guarded raw kernel-shell patch indices. The authored cage has eight angular rows and five axial columns; its graph wraps the final angular row to the first. Source ruled rational patches regenerate from upstream geometry. A single Cartesian polynomial displacement field is composed with every source patch and shared longitudinal edge; cap rims and first derivatives remain fixed. The periodic wall must be authored before cap domains. Disk and periodic flags are mutually exclusive, missing flags preserve previous semantics, and older version declarations reject these operations.

Derived cage nets, reference surfaces and generated skin coefficients are evaluation caches, not separately serialized scene authority. Ordered creation/translation/face-move operations, lattice bindings, source geometry and captured settings regenerate them. Original rational weights, degrees and knots remain in upstream source definitions; generated skins have their own explicit calculated basis. Arbitrary trimmed or periodic NURBS inputs remain unsupported.
