# Initial product scope

OpenSketcher is a Rust desktop modeler targeting macOS, Windows, and Linux.

## Goals and delivery order

1. Modeling: create and edit basic shapes, push/pull faces, extrude profiles, and snap to axes.
2. Project Save/Load: a custom native format that preserves parametric editing, as specified below. Establish its document model alongside modeling so saving does not flatten editable operations.
3. Scene management: scope to be detailed after the initial modeling workflow works. Whether scenes mean saved views or separate model documents remains an open product decision.
4. Material management: texture-only materials initially. Assignment, image loading, and mapping behavior will be specified with this milestone.
5. Barebones viewport rendering: simple visible geometry, edges, and textures; advanced lighting and offline rendering are outside the initial scope.

Basic viewport rendering and camera navigation are dependencies of modeling and must ship with it, even though rendering is also a product goal of its own.

## Geometry kernel decision

Truck is the selected geometry kernel (user decision, 2026-10-23). Use its modeling, topology, shape-operation, and tessellation crates as needed. Keep editable feature definitions and stable semantic references in OpenSketcher's document model; evaluated Truck shapes and viewport meshes are derived data. Do not add a second kernel or a backend abstraction speculatively.

The first implementation check must exercise both Borer only and Generator + borer, including pocket, exact exit, and overshoot with a separate body. The selection is approved; numerical robustness, face tracking, and desktop packaging remain to be verified through implementation. See the [kernel research and validation cases](research/parametric-modeling-kernels.md).

## First milestone: modeling

The following are initial implementation assumptions, not a claim that the full reference toolbar is required.

### Shapes and editing

- Create lines, rectangles, circles, semicircles, and regular polygons on a construction plane. Initial implemented tools cover lines, rectangles, circles, and semicircles; regular polygons remain pending.
- Define creation by picked points: opposite rectangle corners, line endpoints, circle center/radius, and semicircle diameter endpoints. Show a preview after the first point; commit on the second. Snapping is enabled by default.
- Circle interaction uses a canvas-picked center and mouse-defined radius, with Radius as the displayed measurement. Drawing on existing geometry requires prior face selection, exposing that face's relevant points and boundary references. Capture its support plane rather than switching planes on hover. The [face-first sketching specification](specs/face-sketching.md) records the behavior, current code gaps, and implementation sequence; selected/visible planar support and geometry snapping are implemented; full attachment/reference overlays and Radius presentation remain pending.
- Numeric property changes apply automatically, with undo/redo instead of Apply or Revert Fields actions. Group one continuous field edit into one undo action.
- Distinguish Volume, Face, Edge, and Point selection. Volume selects an entire object; Face/Edge/Point select individual topology elements and highlight only that component. Point selection is distinct from deformation control-node editing.
- Whole-object movement, dimension editing, and deletion are available in Volume mode. Component modes support constrained Move/Rotate gizmos; they must never silently transform or delete the owner. Component scaling remains pending.
- Show a live preview while drawing or editing; Escape cancels without changing committed geometry.
- Support numeric dimensions as well as pointer input.
- Undo and redo each committed modeling operation as one action.

### Face types and deformation editing

The [soft-selection behavior specification](specs/soft-selection.md) details node selection, influence calculation, heatmap feedback, transform interactions, failures, and parametric persistence. It distinguishes confirmed requirements from proposed implementation defaults.

Distinguish planar faces from NURBS surfaces and faces with an authored deformation/control cage. Also distinguish **Move face** from **Edit / move node**; selecting a deformed face does not implicitly enter node editing.

- **Move face:** translate the entire face while preserving its current shape. For NURBS or custom deformation, move its surface/control representation and trimming boundaries together; retain relative control-point positions and existing surface parameters. "Freeze deformation" means preserve the shape during this move, not bake it into triangles or discard future parametric editing.
- **Edit / move node:** use soft selection. The selected node receives the full drag displacement; surrounding nodes receive progressively smaller displacements over an adjustable influence radius, reaching zero at its boundary. Show the influence as a heatmap centered on the selected node, with a radius outline and numeric radius control so color is not the only feedback.
- A practical initial falloff is smoothstep: for normalized distance `t = clamp(distance / radius, 0, 1)`, influence is `1 - 3*t*t + 2*t*t*t`. This is an implementation proposal, not a required algorithm. Calculate influence from the pre-drag state and apply the current drag displacement to that state, avoiding cumulative drift within one gesture. A smooth displacement field reduces localized spikes but does not by itself guarantee curvature continuity or prevent folds.
- Restrict influence to the selected face's editable control structure; nearby unrelated faces must not deform merely because they are spatially close. Distance should follow the surface/control-cage neighborhood where spatial proximity would incorrectly bridge folded or disconnected regions; the exact metric requires validation with the chosen representation.
- Soft-selection influence is distinct from a NURBS rational weight. Moving nodes changes their positions; do not alter NURBS weights, degree, or knot vectors as an accidental side effect.
- A face attached to a solid must retain valid shared boundaries and adjacency after either edit. Rebuild affected neighboring geometry where supported, or reject the edit with an explicit explanation while preserving the last valid model. Do not silently tear seams or move the entire body instead.
- Preview edits before committing; Escape restores the original geometry. One completed drag creates one undoable action. Validate resulting geometry before replacing the committed model.

This is confirmed modeling scope; implementation sequencing follows the initial planar modeling and boring validation. Curved-face boring/extrusion is not implied by adding curved-face movement or node editing.

### Push/pull and extrusion

- Push/pull an eligible planar face along its normal, with a preview and numeric distance.
- Extrude a closed planar profile along a straight direction to form a solid.
- Share geometry operations between these tools wherever their behavior overlaps.
- Reject degenerate input without corrupting the model or leaving partial geometry.
- Offer two explicit operation modes for a planar face/profile: **Borer only** removes intersected material without generating a body beyond the target; **Generator + borer** removes intersected material and turns travel beyond the target's exit into an extrusion as a separate body. Never fuse that emerging body back into the target. Ordinary generation without boring remains available through extrusion.
- Example: a profile driven 150 mm through a 100 mm-thick target produces only a through-cut in Borer only mode; Generator + borer produces the same cut plus a separate 50 mm extrusion beyond the far surface. Travel ending inside the target produces a pocket; ending exactly at the exit produces a through-cut without a zero-thickness output body.
- Retain the selected operation mode, source profile/support reference, target body reference, direction, and total travel as editable feature inputs. Keep the modified target and emerging body identifiable as distinct outputs of that feature, and recompute them together after parameter changes or reopening.
- Required subtraction and exit-region construction are part of the first milestone's kernel evaluation. Arbitrary path sweeps, general-purpose Boolean tools, and automatic merging with neighboring solids remain deferred. Multiple targets, intervening gaps, and uneven exit surfaces need explicit semantics before extending beyond the initial single-target, planar-exit case.

### Axis snapping

- Infer alignment to the world X, Y, and Z axes from the current operation's anchor.
- Show the inferred axis visually using red, green, and blue, accompanied by an axis label so color is not the only cue.
- Allow explicit axis locking during drawing and moving.
- Evaluate snapping in world coordinates; use a screen-space acquisition tolerance so it remains usable as the camera zooms.
- Endpoint, midpoint, face, and intersection snapping are separate future scope.

### Viewport needed for modeling

- Keep the canvas behind floating overlays: tool controls on the left and model/properties on the right. Panels are draggable within canvas bounds, properties scroll, and panel pointer/scroll events must not activate canvas tools or camera zoom.

- Orbit, pan, and zoom with quick Isometric, Top, and Side presets. The initial implementation uses orthographic projection including free orbit; perspective is a subsequent addition.
- World axes, a construction grid, basic shaded faces, and visible edges.
- Depth-correct drawing and picking, visible selection, and tool previews.
- No lighting editor, photorealism, or offline renderer in this milestone.

### Acceptance workflow

1. Create a rectangle with numeric dimensions and visible axis inference.
2. Push/pull it into a box with a numeric height.
3. Orbit the result and select a face correctly, including when other faces occlude geometry.
4. Create a circle or polygon and extrude its profile into a solid.
5. Move geometry with an explicit axis lock and edit its dimensions.
6. Undo and redo creation, editing, and extrusion; canceling a preview creates no undo entry.
7. Save and reopen a document with geometry and dimensions intact.
8. In Generator + borer mode, push a smaller profile into a thicker body: verify a pocket before exit, a through-cut at exit, and a separate emerging extrusion beyond exit. Change travel back below the exit and verify the emerging body disappears without leaving stale geometry.
9. Switch the same feature to Borer only: the cut remains and the emerging body disappears. Switch back to Generator + borer: the separate body returns. Show the selected mode in the preview and feature properties; mode changes are undoable.
10. Save and reopen each mode, then edit target thickness and total travel; verify the cut and any separate output recompute together. Undo/redo the operation as one action.
11. Move a deformed face as a whole: verify the shape and relative control-point positions remain unchanged, and that it remains parametrically editable after Save/Load.
12. In node mode, move a node with soft selection: verify full central displacement, decreasing neighboring influence, zero displacement outside the radius, heatmap feedback, and no changes to unrelated faces. Cancel and undo/redo without drift; reject invalid geometry without retaining partial changes.

Each geometry operation needs a small runnable regression check, including degenerate input. Desktop builds alone do not verify pointer interactions or rendered results; exercise the acceptance workflow in the running app.

## Project Save/Load and native format

Projects must save to an OpenSketcher-specific native format and reopen with their parametric editing abilities intact. Here, "proprietary format" means a custom application format; it does not imply closed-source code, encryption, or restrictions on documenting the format.

### Document contract

- Preserve authored shape parameters, dimensions, units, construction planes, transforms, and ordered modeling features with their dependencies. Preserve constraints when constraint tools are introduced.
- Preserve NURBS control points, rational weights, degree, knot vectors, trims, and any custom deformation cage and bindings. Store authored deformation operations and the influence settings needed to reproduce them, rather than saving only their tessellated result.
- Store editable push/pull and extrusion inputs, including profile references, direction, and distance. For boring features, also preserve the selected operation mode, target reference, and semantic identities of the modified target and separate emerging body. A saved triangulated mesh alone is insufficient.
- Use stable entity and feature identifiers. Downstream operations must reference stable semantic geometry rather than transient GPU buffers or triangle indices. Define how references survive upstream edits before implementing dependent face operations.
- Reopening a project must permit changing an earlier shape dimension or extrusion distance and recomputing dependent features. Report invalid dependencies explicitly; never silently discard them or flatten the model.
- Keep authoritative document data separate from derived render meshes. Optional caches must be disposable and cannot be required to recover the editable model.
- Save scene state, texture assignments, mapping parameters, and referenced texture image assets when those features exist. Projects must remain portable without the original texture file paths.
- Version the file schema and the modeling-operation semantics needed for repeatable evaluation. Support explicit migrations for older supported versions; reject unsupported newer versions without changing the current document or rewriting the file.
- Parametric feature history is document content. Session undo/redo history need not persist across reopening.

### User behavior and data protection

- Provide New, Open, Save, and Save As with platform file dialogs and standard keyboard shortcuts.
- Show the current project name and unsaved-change state. Prompt to save, discard, or cancel before New, Open, or Close would lose changes.
- Save As switches the active project path only after a successful save. Canceled dialogs leave document state unchanged.
- Write a complete temporary file beside the destination, then replace the destination using the appropriate platform-safe operation. A failed save must preserve the previous valid file and leave unsaved changes marked as such.
- Load into a temporary document, validate it, and evaluate its features before replacing the active document. Malformed files, missing references, non-finite dimensions, invalid geometry, and unsupported versions must produce an actionable error without destroying the open project.
- Treat project files as untrusted input: bound resource consumption and validate embedded asset paths and sizes. Opening a project must not execute scripts or extract assets outside controlled storage.

### Acceptance workflow

1. Create a dimensioned rectangle and push/pull it into a solid; save and close the project.
2. Reopen it, change the rectangle dimensions and push/pull distance, and verify that the resulting solid recomputes correctly.
3. Repeat with a circular or polygonal profile and a dependent extrusion; confirm that stable references and parameters survive a second save/load cycle.
4. Verify Save As, canceled dialogs, unsaved-change prompts, and a failed save that preserves the previous file.
5. Attempt to open corrupt, invalid-reference, and unsupported-version files; confirm that the current project remains intact.
6. When textures ship, move the project to another machine without the source images and verify that assigned textures still load.

Leave runnable round-trip and failure-path checks with the persistence implementation. The selected extension is `.osk`. The initial implementation uses versioned UTF-8 JSON; see the [file format](formats/osk.md) for its current schema, included properties, safe-write behavior, and limits.

## Reference and icons

The supplied SketchUp toolbar image is a functional reference. It does not require copying its artwork or implementing every displayed tool. Freehand, rotated rectangle, arc variants, pie, components, and painting are deferred from the first milestone.

Use [Tabler Icons](https://tabler.io/icons), outline style, for application icons. Retain the upstream MIT license and attribution when vendoring assets. Include only icons actually used; do not add an icon framework or the entire catalog speculatively. Icon buttons need tooltips and accessible names, and active tools need a visible selected state.

## Current implementation boundary

The repository contains an editor shell, desktop build workflow, and a depth-tested wgpu viewport with Truck-backed geometry. Implemented controls include two-point XY creation for rectangles/circles/lines/semicircles, grid/axis alignment snapping enabled by default, Volume/Face/Edge/Point picking and highlighting, automatic whole-object numeric dimension/extrusion/position/rotation/scale editing, deletion, and session undo/redo. The model starts empty. GPU geometry is derived from authored parameters. Versioned `.osk` Save/Load preserves all currently implemented scene properties and editable parameters, with native dialogs and unsaved-change protection. Move/Rotate gizmos now edit Volume/Face/Edge/Point selections with saved component transforms and guarded topology references. Regular polygons, dedicated push/pull, boring, node deformation, exact curved component reconstruction, and a Scale gizmo remain pending. Keep geometry/document data independent of egui and GPU resources; introduce modules as implementation requires them rather than scaffolding a speculative engine.

### Coplanar region splitting

Closed profiles explicitly marked to cut older coplanar profiles partition their regions before extrusion. New profiles are independent by default; versions 1–3 preserve their existing cutting relationships on load. Newer profile IDs take precedence in overlapping areas. Extruding an older surrounding region excludes those areas and generates cut walls; newer regions keep their own extrusion height. Edge-touching profiles create notches and crossing profiles can leave disconnected regions. Edits, deletion, undo/redo, and `.osk` reload regenerate the result. Open-line partitions, persistent attached-face sketches, and exact analytic curved intersections remain pending; fixed placement on elevated planar faces is implemented. Split curved profiles currently use polygonal boundaries; untouched Truck curves remain analytic. See the [modeling version 2 rules](formats/osk.md#coplanar-region-evaluation-modeling-version-2).

### Move/Rotate gizmos

- [x] World X/Y/Z translation arrows, XY/YZ/ZX move-plane handles, and world-axis rotation rings in every selection mode.
- [x] Screen-sized handles, hit tolerances, hover/active highlighting, axis labels, and a live constrained distance/angle readout.
- [x] Pointer drag/release previews, Escape/tool/view/focus-loss cancellation, default 0.5-unit / 15-degree snapping, and one undo step per completed gesture.
- [x] Automatic selection pivot and Alt-click offset pivot; a point rotated about itself is a no-op.
- [x] True world rotation composition for an already rotated/scaled volume; component transforms preserve the owning body's authored pose.
- [x] Regenerated component boundary edits, adjacency rebuilding, topology-signature validation, atomic edit/delete rejection, and schema/modeling version 4 persistence (component transforms were introduced in version 3).
- [ ] Exact analytic curved/nonplanar component surface reconstruction (current output uses a regenerated faceted boundary).
- [ ] Component edits on a profile cutting another region; use Volume transforms for that profile for now.
- [ ] General self-intersection detection and semantic references beyond guarded output connectivity.
- [ ] Scale gizmo, NURBS/control-cage editing, and soft-selection falloff controls.

Regression checks cover all selection modes for movement/rotation across all four existing profile types, constrained handle math in each view preset, a complete press/move/release batch in one frame, world rotation composition, single-point pivot behavior, component reference errors, collapsing geometry, undo/redo, and `.osk` regeneration. Native macOS review verified volume-axis movement, face movement/rotation with connected walls, edge movement, and the fast-drag fix; native point/pivot interaction and Windows/Linux runtime review remain pending.

### Geometry snapping and independent drawing defaults

- [x] Visible endpoints, sampled edge midpoints, boundary edges, and projected boundary coordinates, alongside grid/anchor snapping.
- [x] Selected-face priority, first-visible-face plane capture, elevated/vertical support, and fixed-plane drawing throughout a gesture.
- [x] Screen-space acquisition, target labels/guides, and exclusion of hidden base/off-plane references.
- [x] Project preferences for geometry/grid snapping, optional base-XY projection, and optional automatic cutting; base projection and new-profile cutting default off.
- [x] Per-profile cutting intent and schema/modeling version 4 persistence with legacy cut preservation.
- [x] Coplanar selected-profile display without model-space offsets; coincident picking favors the newer profile.
- [ ] Geometry-intersection/analytic-center snapping, full contextual reference overlays, persistent attached-face dependencies, and arbitrary solid-face imprint/Boolean operations.
