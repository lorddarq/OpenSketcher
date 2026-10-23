# Geometric operators: reference workflows, gaps, and implementation

Research date: 2026-10-23. Two research agents investigated CAD face editing and SketchUp/Plasticity direct modeling independently. This document consolidates those findings against the current source. Checkboxes describe OpenSketcher delivery, not capabilities inherited from Truck.

## Operator distinctions

Keep three user intentions separate:

1. **Move / Rotate face:** rigidly reposition the selected face and rebuild its neighbors. A curved face retains its deformation; moving a cylindrical wall does not mean changing its radius.
2. **Offset face:** move a supporting surface along its normal. A planar support translates; a cylindrical support changes radius while its axis remains fixed. Rebuild trims and adjoining surfaces where supported.
3. **Extrude / bore:** sweep a planar region and explicitly choose material generation or removal, target bodies, direction, and extent. Placement inference supplies a reference, not permission to subtract unrelated bodies.

The current component Move/Rotate path stores matrices against guarded topology and deforms sampled boundaries/triangles. It is useful component editing, but it is not an exact CAD surface offset. Do not relabel it as one. Logical surface selection must remain independent of tessellation subdivisions.

## Reference findings

| Reference capability | Consequence for OpenSketcher |
| --- | --- |
| Fusion distinguishes face movement from offset, including different cylindrical results. [Move faces](https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/How-to-move-faces-of-a-CAD-body-part-in-Fusion-360.html) | Separate Move/Rotate from Offset face. Preserve analytic surfaces when a source-parameter edit suffices. |
| Fusion Press/Pull can modify an existing feature or create a new offset operation. [Press/Pull modes](https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/Press-Pull-gives-unexpected-results-in-Fusion-360.html) | Start with explicit source-shape editing. Do not invent a second history representation for the same primitive dimension. General offsets will require their own operation references. |
| Fusion Offset Face supports signed normal distances and multiple faces. [Offset Face](https://help.autodesk.com/view/fusion360/ENU/?contextId=SLD-OFFSET-FACES) | A signed numeric distance is baseline behavior. Multiple faces and tangent propagation need explicit rules rather than incidental vertex unions. |
| Onshape Move Face separates Offset, Translate, and Rotate; supports constrained extents and reapplying fillets. [Move Face](https://cad.onshape.com/help/Content/PartStudio/move_face.htm) | Neighbor extension/retrim and treatment rebuilding are geometric operator responsibilities. Preserve authored treatment values when supported. |
| Onshape Extrude separates New/Add/Remove/Intersect from extent and target scope. [Extrude](https://cad.onshape.com/help/Content/PartStudio/extrude.htm) | Store operation, target, and depth/extent independently of snapping. |
| Onshape chamfer supports symmetric/asymmetric setbacks, distance/angle, cylindrical edges, and tangent propagation. [Chamfer](https://cad.onshape.com/help/Content/PartStudio/chamfer.htm) | Current whole-perimeter equal-distance treatment is a subset. General edge selection comes before advanced corner options. |
| Onshape fillet supports constant/variable radius, partial edges, and corner controls. [Fillet](https://cad.onshape.com/help/Content/PartStudio/fillet.htm) | First establish reliable constant-radius edge blends; variable-radius and corner systems are later work. |
| Onshape Delete Face distinguishes Heal, Cap, and Leave Open. [Delete Face](https://cad.onshape.com/help/Content/PartStudio/delete_face.htm) | Removing render triangles is not face deletion/healing. Surface-body results need explicit representation. |
| FreeCAD documents broken references after topology changes. [Topological naming](https://github.com/FreeCAD/FreeCAD-documentation/blob/main/wiki/Topological_naming_problem.md) | Use semantic source roles where possible. Missing or ambiguous references must fail visibly instead of silently targeting a different output face. |
| SketchUp inference distinguishes center, on-edge, and on-face placement. [Drawing concepts](https://help.sketchup.com/en/sketchup/introducing-drawing-basics-and-concepts) | Circle center/rim inference is placement. It must retain the visible support elevation. |
| SketchUp Push/Pull supports material removal and openings through geometry. [Push/Pull](https://help.sketchup.com/en/sketchup/pushing-and-pulling-shapes-3d) | Add real pockets, through-cuts, and rim notches; coplanar source-profile partitioning is insufficient. |
| Plasticity separates normal offset/Grow behavior from extrusion output modes and explicit Boolean targets/tools. [Offset Face](https://doc.plasticity.xyz/solid/offset-face), [Extrude](https://doc.plasticity.xyz/solid/extrude), [Boolean](https://doc.plasticity.xyz/solid/boolean) | Define collision/termination and target scope explicitly. Do not infer a destructive operation solely from snapping. |
| Plasticity imprint projects curves onto solids/sheets with direction and occlusion controls. [Imprint Curve Body](https://doc.plasticity.xyz/solid/imprint-curve-body) | Selected visible support faces are the default; imprinted face regions require actual topology, not coincident display profiles. |
| Plasticity edge treatments support edge selection and tangent propagation; CV editing is a separate operation. [Fillet Shell](https://doc.plasticity.xyz/solid/fillet-shell), [Slide Surface CV](https://doc.plasticity.xyz/solid/slide-surface-cv) | Build independent logical-edge treatment before advanced blends; preserve the distinction between rigid face movement and node falloff. |
| SketchUp distinguishes softened surface selection from smooth shading or simply hiding edges. [Softening and smoothing](https://help.sketchup.com/en/sketchup/softening-smoothing-and-hiding-geometry) | Keep grouping/picking semantics separate from render-edge visibility. |

## Initial implementation audit

- `components.rs` replays component matrices against topology signatures, validates a faceted shell, and drops analytic shell/circle metadata after deformation.
- `edge_treatments.rs::target` already resolves original support faces to Top/Bottom/Left/Right/Front/Back. Reuse these semantic roles.
- Chamfer/bevel support original box faces and circular/semicircular cap perimeters, with logical transition surfaces. Arbitrary edges and exact noncircular blends remain unsupported.
- `regions.rs` partitions coplanar profiles. It does not subtract an elevated swept volume from a solid.
- `model.rs` validates candidate scenes transactionally and caches the latest successful evaluation. New operators must use that flow for viewport, saving, and export.
- The confirmed circle-margin gesture is **new circle center snapped onto an existing rim**. It is not a diameter-based creation gesture.

## P0: primitive planar Offset face

Initial implementation scope is deliberately source-shape editing, corresponding to modifying an existing primitive feature. It does not create an independent offset history feature.

- [x] Expose **Offset face** for one selected original planar face: all six box faces; Circle/Semicircle top and bottom caps.
- [x] A signed distance is measured in world units along the selected outward normal: positive grows outward, negative shrinks inward.
- [x] Keep the opposite support plane fixed and preserve other extents. Bottom/side edits therefore update both source dimensions and placement as necessary.
- [x] Preserve exact circular geometry, logical faces, and center/rim inference. Never route this operation through weighted mesh deformation.
- [x] Rebuild existing chamfers/bevels using unchanged authored amounts/styles. Reject a result that consumes a face or violates treatment limits.
- [x] Apply changed numeric values automatically; one continuous edit is one undo step. Selecting a face starts the relative distance at zero.
- [x] Reject nonfinite distance, collapsed/inverted thickness, component-deformed bodies, partitioned output, transition bands, and curved walls with a precise explanation.
- [x] Save/reopen and export through the existing source-parameter representation. No new dependency or document version is needed for this slice.

Acceptance: test each supported face on translated/rotated/nonuniformly scaled bodies. Its world support moves by exactly the requested signed distance; the opposite plane stays fixed. Cap offsets preserve diameter. Test treatment regeneration, invalid-edit rollback/history, persistence, and continued circular snapping. Native review must distinguish property interaction from kernel-only test evidence.

Delivered 2026-10-23: `components::offset_source_face` reuses the existing source-face roles and changes primitive parameters. The single-face properties control measures signed millimeter distance from the position at selection; selecting another face or undo/redo resets that reference. Kernel checks cover each supported role under rotated/nonuniformly scaled placement, treatment preservation, closed output, analytic circle metadata, invalid input, undo grouping, and `.osk` regeneration. Native macOS review verified outward/inward edits on a beveled cylinder cap, disabled offset on its transition band, retained bevel radius, collapsed-thickness rejection, undo/redo, and saved source values. A dedicated normal canvas handle remains pending; the control is numeric.

Inward offset is limited to positive remaining thickness. Crossing the opposite face is **not** implemented by making a primitive negative or inverted; it belongs to the explicit borer contract below.

## P0: solid bore / generator + borer

- [x] Capture and persist source profile, support reference, target body, direction, travel, and operation mode.
- [x] Provide **Borer only** and **Generator + borer** alongside ordinary generation. A rim-centered circle may initially choose Cut, with the resolved target/intent visible and editable.
- [x] Perform solid subtraction: contained pocket, contained through-hole, and boundary-crossing notch. Do not substitute `cuts_profiles`.
- [x] Distinguish pocket, exact exit, and overshoot. Borer emits no material beyond the target. Generator + borer emits a separately selectable body beyond the exit, never fused into the target.
- [x] Recompute target and emerging output together when profile, target thickness, or travel changes. Removing the operation restores material.
- [x] Keep unrelated bodies untouched; make failed kernel operations recoverable without partial document changes.

Acceptance: a circle centered on a cylinder rim makes a notch. A contained circle makes a pocket/hole. Travel 150 through thickness 100 creates a 50-unit separate body only in Generator + borer. Exact exit creates no zero-thickness body. Undo/redo, removal, saved mode/target references, reopening, and exports must agree.

### Delivered baseline and limits (2026-10-23)

`operators.rs` stores an attached profile's target body ID, semantic source-face role, initial support frame, signed world-unit travel, and New body / Borer only / Generator + borer mode. Drawing closed profiles on supported original solid faces captures this dependency; circle-rim placement initially chooses Borer. Legacy independent profiles keep their behavior. The profile follows subsequent target placement/thickness changes. Removing its operation restores target material. `.osk` version 6 retains these inputs and rejects older version declarations containing them.

Contained blind pockets and through profiles on an untouched primitive cap use exact boundary construction, avoiding 3D intersection solving at intermediate drag/field values. Pocket floors, inner walls, and entry rings share kernel edges. Circular containment uses analytic bounds, so thin pipe walls do not fall back to CSG because of display-sized clearance margins. Truck solid subtraction remains the fallback for boundary-crossing notches and treated/previously cut targets. The 150-through-100 regression verifies a separate 50-unit output. Emergence currently uses the primitive's opposite support plane. Uneven exits, disconnected targets, arbitrary trimmed/deformed support faces, and general blends require the remaining operator/reference work; this baseline is not a general Boolean editor.

The profile's ID owns any emerging body, distinct from the target. STL/GLB/STEP/DXF use evaluated solid classification rather than the source profile's zero extrusion. Tests cover pocket/exact exit/overshoot, cylinder-rim notch, closed boundaries, mode changes, operation removal, missing references, undo/redo, schema migration rejection, save/reopen, source attachment, and exports. Native macOS review verifies notch travel, separate emerging output, view switching, and saved feature data. Direct construction/support capture and Windows/Linux interactions still need broader native review.

### Interactive evaluation and CPU behavior

- [x] Evaluate model edits, component previews, history, and project loading off the UI thread.
- [x] Use one long-lived evaluator with at most one active job; coalesce repeated pending input rather than starting a thread per value.
- [x] Bound the geometry Rayon pool to at most two threads, leaving processor capacity for the UI.
- [x] Keep the last valid scene visible, reject stale/cancelled results, show progress, and support Escape cancellation.
- [x] Preserve separate undo boundaries across completed edits and retain edits to different objects while work is pending.
- [x] Pass invisible cutter BReps directly to the kernel; do not generate their disposable viewport meshes.

Cancellation discards pending results immediately; the already executing kernel call may finish in the background. Geometry evaluation still takes time, especially in debug builds. Native debug review switched views while the progress indicator was active and cancelled a pending travel change, retaining the prior committed value. Regression checks verify coalescing, stale result rejection, cross-object edits, and undo boundaries. General high-complexity latency/mesh budgets and adaptive preview quality remain part of the P1 performance work below.

## P1: operator completeness and references

- [ ] Imprint supported curves/closed profiles onto selected solid faces; split independent regions including inner loops and boundary-crossing profiles without touching occluded faces.
- [x] Cylindrical-wall Offset face changes radius without moving the axis; equal X/Y scale required, independent Z scale allowed. An elliptic cylinder is rejected because its parallel offset is not an ellipse.
- [ ] Exact planar face Translate/Rotate with adjacent-surface extension/retrim; preserve the rigid shape of curved/deformed faces.
- [ ] Arbitrary selected-edge chamfer/fillet, including concave edges and already edited solids. Original box edges now have edge-local treatment; Boolean-result/concave edges and exact general fillets remain pending.
- [x] Multiple supported face/edge operation inputs across one or several source bodies, applied atomically in one undo step.
- [ ] Optional tangent-chain propagation beyond the existing complete circular rim.
- [x] Two-distance chamfer, distance + angle, and direction reversal on supported source-face perimeters, including cylindrical rims. Arbitrary selected edges remain the separate gap above.
- [x] Attached-target Through all and parallel-planar Up to face / Up to next with explicit end-body/source-plane references.
- [ ] Complete extent termination on angled/curved/varying-depth geometry and multiple-body Through all scope.
- [ ] Persistent semantic face/edge references beyond current guarded connectivity; no silent ambiguous rebinding. Source support-face roles are now retained for attached sweeps; arbitrary output/trim/edge naming remains pending.
- [x] Identify failed supported operation and reference, retain last valid geometry, and distinguish rejected properties from committed parameters.
- [ ] Separate analytic evaluation from disposable preview tessellation; retain the existing duplicate-evaluation cache and measure interactive latency.
- [ ] Complete analytic STEP coverage for every edited surface. Retained kernel boundaries now export their analytic/NURBS surfaces directly; mesh-only component edits/noncircular approximated blends remain faceted. Nonuniformly scaled revolutions now export as exact rational tensor-product NURBS, preserving surface geometry and orientation. Intersection trims use independently sampled spline curves at kernel tolerance to avoid a pinned STEP-writer intersection-entity ID defect; cached geometry is never mutated by export.

## P2: broader operators

- [ ] Replace Face and Delete Face with explicit Heal / Cap / Open behavior.
- [ ] Shell / thickness and draft.
- [ ] Partial/variable-radius fillets and advanced corner blends.
- [ ] Datum planes and independently placed construction references.
- [ ] NURBS/control-cage editing and soft-selection falloff as specified separately.

These researched capabilities inform the backlog; they do not authorize speculative frameworks or copying another product's implementation. Preserve Truck as the selected kernel, reuse existing model/selection/history paths, and add only the representation required by each delivered operator.

## Ordered Boolean tools

- [x] Union, ordered subtraction, and intersection for closed volumes and coplanar planar faces, retained source parameters, Keep tools, and undo/redo.
- [ ] Curved/noncoplanar sheet trimming, general slice/split, and analytic planar arc preservation.

See [research, behavior, and current limitations](boolean-operators.md).

Pipe-bore regression: concentric circular cutters at shallow, intermediate, nearly-through, exact-through, and overshoot depths; thin walls; bottom-face bores; rotated/nonuniformly scaled targets. Kernel closure, signed volume, and welded print-mesh closure are checked.

## Asymmetric chamfer behavior

Delivered source-face perimeter treatments support Equal distance, Two distances, and Distance and angle, following the distinctions in [Onshape Chamfer](https://cad.onshape.com/help/Content/PartStudio/chamfer.htm). The first distance is measured along the selected source face; the second is measured along its neighboring face. For a cylinder cap these are radial and axial setbacks. At a right-angle edge, distance-and-angle derives the second setback as `first * tan(angle)`; the angle is measured from the first-distance face. Reverse swaps the two setbacks and changes the angle/distance reference to the adjacent face. Values are source-local, consistent with existing treatment distances and object scale.

Both setbacks must remain finite, positive, and within the existing conservative source limits. Angles are strictly between 0 and 90 degrees; consuming geometry or overlapping treatments fail transactionally. Numeric edits update automatically and retain the existing edit/undo boundaries. Cylindrical chamfers remain analytic conical transitions with one logical band; exact circular cap/rim snapping follows both setbacks. Box and semicircle treatments reuse the existing clipping geometry rather than a separate evaluator. Bevel retains its existing equal-radius behavior.

`.osk` version 8 persists the chamfer definition and direction. Missing fields mean equal distance, preserving versions 1–7; older version declarations reject asymmetric data. Regression coverage checks all supported source faces under rotated/nonuniform scale, two-distance/angle equivalence, reversal, ring metadata, four export encoders, invalid input rollback, history, and save/load. Native property interaction is tracked separately from these kernel checks.

Native macOS release review verified a cylinder with 0.25/0.50 two-distance setbacks, direction reversal changing cap radius and band depth, switching to distance/angle, automatic regeneration at 30 degrees, and saving the version-8 definition with reversed direction. All remain logical cap/band/wall surfaces. External CAD import validation remains pending.

## Edge-local treatment and multi-selection

Box Edge selection now resolves the two original supporting planes and restricts the operation to their shared edge. All twelve original edges are supported, including surviving source edges after other edges have been treated. It does not expand a box-edge pick to an entire face perimeter. Face selection retains whole-perimeter behavior; circular/semicircular rims retain their existing full-perimeter semantics.

Shift-selected supported faces or edges across one/many bodies use one validated transaction and one undo step. All picks are resolved before submission. Circular rim kernel patches deduplicate to one logical operation. Missing, component-deformed, Boolean-result, transition, and duplicate treated edges reject visibly; arbitrary concave edges and tangent-chain propagation remain open gaps. The two source planes are stored as semantic roles, so source dimension/placement changes do not reinterpret an output edge index. Asymmetric setbacks and direction reversal reference the first stored plane; rounded box bevels retain the existing grouped approximation, not a new claim of analytic general filleting.

`.osk` version 9 stores an optional neighboring source plane on each treatment. Missing values preserve old face-perimeter behavior. The source limits allow at most six perimeter operations plus twelve selected-edge operations, with duplicates/overlapping perimeter-edge operations rejected. Kernel regression verifies single-edge removed wedge volume, seven logical faces for one chamfer, all twelve simultaneous edges, grouped history, save/load, format version gating, rotated/nonuniformly scaled references, transition-edge rejection, and STL/GLB/STEP/DXF encoding. Native review is separate from these checks.

Native macOS release review verified Shift-clicking two visible box edges, applying exactly two local chamfers, automatic owner-property selection, one-step undo of the grouped operation, and saving both semantic edge references in version 9.

## Failed operation and rejected-input behavior

Source validation and kernel building share contextual error boundaries across creation, standalone evaluation, scene replay, previews, save/load, and export. Errors identify the owner by the same label used in Model. Treatment validation adds operation number, kind, and source face/edge; component replay adds the operation number and component reference; bore failures identify the profile, mode, target body, and support face; Boolean failures identify the result and ordered target/tool body/face references. Source-profile partition failures identify both affected body IDs. Offset failures include the source body and selected support role. Missing selections do not silently select another element.

Rejected edits never replace the committed model, geometry, or history. Properties retain rejected draft values so they can be corrected and explicitly display **Input not applied**. Viewport geometry, Save, and Export continue to use the last accepted model. Escape discards rejected numeric/operation drafts even after the worker has finished; pending-result cancellation continues to work as before. Failed unrelated operations such as opening a file do not invalidate an existing offset edit merely because an error occurred. No extra Apply/Revert controls or serialized draft state are added.

A worker regression checks contextual treatment/component/Boolean/bore failures, stale/missing references, unchanged committed geometry and history, correction/recovery, and subsequent undo. Native property rejection/save/discard behavior is reviewed separately.

Native macOS release review rejected an 89-degree cylinder chamfer while retaining the accepted 30-degree geometry. Properties showed **Input not applied** and the error identified Circle 1 / Operation 1 / Chamfer / Top. Saving while the rejected value remained visible retained 30 degrees in the actual `.osk` file. Escape after computation restored the field to 30 degrees and cleared the rejection state.

## Sweep extents: delivered behavior and remaining generalization

[Onshape Extrude](https://cad.onshape.com/help/Content/PartStudio/extrude.htm) distinguishes complete next-face termination, an underlying face/plane limit, and through-all scope. OpenSketcher now retains extent independently from material mode:

- **Distance** retains signed authored travel and is the migration default.
- **Through all** derives the attached target's full source thickness along the inward support normal. Its scope is explicitly that target alone. It does not invent an arbitrary large travel or cut neighboring bodies.
- **Up to face** references an earlier end body and semantic original support-face role. For parallel planar supports it terminates at the referenced infinite plane. Coincident, behind-direction, missing/consumed, angled, and unsupported references fail visibly.
- **Up to next** references an earlier end body and searches its evaluated parallel planar faces in the chosen direction. Coplanar faces are considered together; the whole profile must terminate. A partial nearer hit rejects rather than silently passing through it to a farther plane. Completely disjoint planes are skipped. Pristine primitive caps reuse analytic/conservative profile containment; trimmed regions use the shared planar Boolean region classifier, rather than a centre-ray hit.

End references determine termination only; they never expand the destructive target scope. An end body remains unchanged. Generator + borer still emits a separate body only for travel beyond the attached target exit. Source thickness, end-body pose, and referenced-plane changes regenerate travel; resolved distances are derived, not authored back into the manual travel field. Direction is explicit. Automatic extents reject unresolved, zero/backward, nonfinite, cyclic/forward, and over-budget distances. Undo/redo and atomic failure recovery use existing transactions.

`.osk` version 10 persists the tagged extent and dependency IDs/roles; versions 1–9 default missing extents to Distance. The inspector exposes extent, direction, and applicable end body/face choices. Kernel checks cover rotated/nonuniformly scaled box/cylinder targets, all three automatic options, external termination, unchanged end bodies, emerging output, regeneration, missing/cyclic references, partial/no termination, opposite direction, four exports, history, and migration/save/load. Native control interaction remains separately verified.

This delivery does not complete general termination: angled/curved trims, different stopping depths across a profile, and multiple-body Through all scope remain required work. Up to next uses sampled boundaries for trimmed face coverage, while parallel termination distance and pristine cap containment retain analytic source geometry.

### Evaluated face extrusion baseline

- [x] Outward extrusion of surviving planar source faces after chamfer/bevel, including exact circular cap boundaries; repeated extrusions preserve the existing treatment, logical cap/wall selection, save/load, undo/redo and retained BRep export.
- [ ] Through-cutting from treated faces and extrusion of arbitrary transition, curved, component-deformed or Boolean-result faces. Blind inward sweeps of fully treated planar cap perimeters are now supported.

Face mode defaults to **Extrude face** for planar caps. **Offset source** remains separate: it changes the primitive and rebuilds its treatment. Extrude face sweeps the evaluated trimmed boundary without repositioning the existing treatment. Distance edits apply automatically; unsupported/invalid operations preserve the committed model.

### Component transforms on evaluated solids

- [x] Move/rotate selected edges, faces and points of bored/generated solids and explicit Boolean results through the shared component transform path. New edits replay after construction, rather than against the primitive's obsolete topology. Multiple selections remain atomic, editable source parameters remain serialized, and invalid/stale operations preserve the accepted model. Legacy source edits retain their original replay stage.
- [ ] Exact analytic retrimming for component-deformed curved faces; current deformation retains logical selection groups and uses the existing faceted boundary fallback for CAD export.

### Signed treated-cap sweep

- [x] Negative face extrusion forms a blind recess from a fully chamfered/beveled cap, retaining the existing treatment and an exact shared BRep. Signed displacement and removed volume are checked on box/cylinder top/bottom caps under rotation and nonuniform scale. Undo/redo, save/load and four export paths cover the signed history.
- [x] Consecutive cap reversal retracts existing sweep geometry; full retraction restores the original boundary. Outward-only legacy topology is preserved. Reject opposite-wall/treatment collision and overlapping opposite recesses atomically.
- [ ] General intersecting/multiple-axis inward face sweeps and through-exit operations on treated geometry. Current guard preserves the solid until general trimming can resolve these cases.
