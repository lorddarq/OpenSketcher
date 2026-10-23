# Geometric operators: reference workflows, gaps, and implementation

Research date: 2026-10-23. Two research agents investigated CAD face editing and SketchUp/Plasticity direct modeling independently. This document consolidates those findings against the current source. Checkboxes describe OpenSketcher delivery, not capabilities inherited from Truck.

## Operator distinctions

Keep three user intentions separate:

1. **Move / Rotate face:** rigidly reposition the selected face and rebuild its neighbors. A curved face retains its deformation; moving a cylindrical wall does not mean changing its radius.
2. **Offset face:** move a supporting surface along its normal. A planar support translates; a cylindrical support changes radius while its axis remains fixed. Rebuild trims and adjoining surfaces where supported.
3. **Extrude / bore:** sweep a planar region and explicitly choose material generation or removal, target bodies, direction, and extent. Placement inference supplies a reference, not permission to subtract unrelated bodies.

The component Move/Rotate path stores matrices against guarded topology. New planar-face edits on convex planar solids rebuild exact neighboring plane intersections; other component edits and legacy histories deform sampled boundaries/triangles. It is useful component editing, but it is not an exact CAD surface offset. Do not relabel it as one. Logical surface selection must remain independent of tessellation subdivisions.

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
- [x] Share exact convex planar subtraction between evaluated face sweeps, ordered volume Subtract and attached profile bores; preserve original references for disjoint/touching-only tools.

Cancellation discards pending results immediately; the already executing kernel call may finish in the background. Geometry evaluation still takes time, especially in debug builds. Native debug review switched views while the progress indicator was active and cancelled a pending travel change, retaining the prior committed value. Regression checks verify coalescing, stale result rejection, cross-object edits, and undo boundaries. General high-complexity latency/mesh budgets and adaptive preview quality remain part of the P1 performance work below.

## P1: operator completeness and references

- [x] Explicit planar closed-profile Imprint: exact contained curved wires, existing inner-loop preservation, independent regions, and straight-boundary crossings with adjacent-edge reconciliation.
- [x] Holed and disconnected planar profile tools, exact nested analytic loops, whole-profile references, and independent island regions.
- [x] Finite straight-line Imprint on planar solid faces, including analytic curved trims, holes and concave regions; retain shared neighbor boundaries and independent regions.
- [ ] Complete Imprint for arbitrary open curves and analytic curved closed-profile crossings.
- [x] Cylindrical-wall Offset face changes radius without moving the axis; equal X/Y scale required, independent Z scale allowed. An elliptic cylinder is rejected because its parallel offset is not an ellipse.
- [x] Exact planar face Translate/Rotate on convex planar solids with retained adjacent planes, exact shared edges/faces, guarded connectivity and analytic STEP.
- [ ] General face Translate/Rotate beyond convex planar solids, including concave/curved neighbors and rigid curved/deformed face preservation.
- [ ] Arbitrary selected-edge chamfer/fillet, including concave edges and already edited solids. Original box edges now have edge-local treatment; Boolean-result/concave edges and exact general fillets remain pending.
- [x] Multiple supported face/edge operation inputs across one or several source bodies, applied atomically in one undo step.
- [ ] Optional tangent-chain propagation beyond the existing complete circular rim.
- [x] Two-distance chamfer, distance + angle, and direction reversal on supported source-face perimeters, including cylindrical rims. Arbitrary selected edges remain the separate gap above.
- [x] Attached-target Through all and parallel-planar Up to face / Up to next with explicit end-body/source-plane references.
- [x] Explicit multiple-body Through all scope with analytic conservative depth bounds, atomic selected-body cuts, preserved unchecked bodies and editable/persisted target IDs.
- [x] Up to face terminates profiles on angled planar supports with exact projected caps, including contained blind pockets.
- [x] Angled planar Up to next with finite full-profile coverage, coplanar face groups and nearer-surface obstruction checks.
- [ ] Complete curved and multi-plane varying-depth termination on general edited geometry.
- [ ] Persistent semantic face/edge references beyond current guarded connectivity; no silent ambiguous rebinding. Source support-face roles are now retained for attached sweeps; arbitrary output/trim/edge naming remains pending.
- [x] Identify failed supported operation and reference, retain last valid geometry, and distinguish rejected properties from committed parameters.
- [ ] Separate analytic evaluation from disposable preview tessellation; retain the existing duplicate-evaluation cache and measure interactive latency.
- [ ] Complete analytic STEP coverage for every edited surface. Retained kernel boundaries now export their analytic/NURBS surfaces directly; mesh-only component edits/noncircular approximated blends remain faceted. Nonuniformly scaled revolutions now export as exact rational tensor-product NURBS, preserving surface geometry and orientation. Intersection trims use independently sampled spline curves at kernel tolerance to avoid a pinned STEP-writer intersection-entity ID defect; cached geometry is never mutated by export.

## P2: broader operators

- [x] Delete Face · Open on retained boundaries, planar Delete Face · Cap / Cap openings with nested loops, and convex-planar Delete Face · Heal with ordered history, rollback and persistence.
- [x] Live planar Replace Face on convex planar solids and original circular/semicircular prism caps, with ordered source/reference picks, construction-plane references and mixed Boolean/replacement chains.
- [ ] General curved/concave Replace Face and Delete Face · Heal; nonplanar opening closure.
- [x] Exact inward Shell/thickness for source boxes (any of six planar openings), cylinders and semicylinders (either cap). Thickness/opening remain editable/removable with undo, serialization and export.
- [x] Exact source-prism Draft for boxes, cylinders and semicylinders, with signed angle, top/bottom neutral cap, exact curved surfaces/corner trims, editable/removable history and export.
- [x] Exact outward Shell for source boxes, cylinders and semicylinders; fixed cavity/opening, editable direction, semantic outer supports, cap editing, history and `.osk` persistence.
- [x] Closed inward/outward cavities for source boxes, cylinders and semicylinders, with retained inner/outer boundaries, thickness validation, editable closure and CAD void export.
- [ ] General shelling of already treated/Boolean/deformed solids, and draft of arbitrary evaluated faces or neutral references.
- [ ] Partial/variable-radius fillets and advanced corner blends.
- [x] Independently placed construction planes with XY/XZ/YZ presets, numeric/gizmo placement, explicit drawing activation, reference-centered grid, undo and project persistence.
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
- **Through all** derives conservative analytic bounds across the attached target and explicitly checked scope bodies. Unchecked neighboring bodies are preserved.
- **Up to face** references an earlier end body and semantic original support-face role. It terminates at the referenced infinite planar support, including angled planes. The entire profile must reach the plane ahead in the chosen direction. Coincident, behind-direction, missing/consumed, parallel-to-sweep, and unsupported references fail visibly.
- **Up to next** references an earlier end body and searches its evaluated planar faces in the chosen direction. Coplanar faces are considered together; the whole profile must terminate. A partial nearer hit rejects rather than silently passing through it to a farther plane. Completely disjoint planes are skipped. Pristine primitive caps reuse analytic/conservative profile containment; trimmed regions use the shared planar Boolean region classifier, rather than a centre-ray hit.

End references determine termination only; they never expand the destructive target scope. An end body remains unchanged. Generator + borer still emits a separate body only for travel beyond the attached target exit. Source thickness, end-body pose, and referenced-plane changes regenerate travel; resolved distances are derived, not authored back into the manual travel field. Direction is explicit. Automatic extents reject unresolved, zero/backward, nonfinite, cyclic/forward, and over-budget distances. Undo/redo and atomic failure recovery use existing transactions.

`.osk` version 10 persists the tagged extent and dependency IDs/roles; versions 1–9 default missing extents to Distance. The inspector exposes extent, direction, and applicable end body/face choices. Kernel checks cover rotated/nonuniformly scaled box/cylinder targets, all three automatic options, external termination, unchanged end bodies, emerging output, regeneration, missing/cyclic references, partial/no termination, opposite direction, four exports, history, and migration/save/load. Native control interaction remains separately verified.

General curved/multi-plane termination and cross-stage Boolean-result scope references remain required work. Up to next uses sampled boundaries for trimmed face coverage, while parallel termination distance and pristine cap containment retain analytic source geometry.

### Evaluated face extrusion baseline

- [x] Outward extrusion of surviving planar source faces after chamfer/bevel, including exact circular cap boundaries; repeated extrusions preserve the existing treatment, logical cap/wall selection, save/load, undo/redo and retained BRep export.
- [x] Ordered sweeps of evaluated planar faces after exact Move/Rotate, planar transition bands and Boolean results; outward shared-boundary construction, inward prismatic overrun, guarded references, history and persistence.
- [x] Exact convex polyhedral inward sweeps through angled/varying-depth exits, including boundary crossings and disconnected planar regions.
- [ ] Curved/mesh-deformed face extrusion and complete robust inward trimming on concave/curved evaluated solids.

Face mode defaults to **Extrude face** for planar caps. **Offset source** remains separate: it changes the primitive and rebuilds its treatment. Extrude face sweeps the evaluated trimmed boundary without repositioning the existing treatment. Distance edits apply automatically; unsupported/invalid operations preserve the committed model.

### Component transforms on evaluated solids

- [x] Move/rotate selected edges, faces and points of bored/generated solids and explicit Boolean results through the shared component transform path. New edits replay after construction, rather than against the primitive's obsolete topology. Multiple selections remain atomic, editable source parameters remain serialized, and invalid/stale operations preserve the accepted model. Legacy source edits retain their original replay stage.
- [ ] Exact analytic retrimming for component-deformed curved faces; current deformation retains logical selection groups and uses the existing faceted boundary fallback for CAD export.

### Signed treated-cap sweep

- [x] Negative face extrusion forms a blind recess from a fully chamfered/beveled cap, retaining the existing treatment and an exact shared BRep. Signed displacement and removed volume are checked on box/cylinder top/bottom caps under rotation and nonuniform scale. Undo/redo, save/load and four export paths cover the signed history.
- [x] Consecutive cap reversal retracts existing sweep geometry; full retraction restores the original boundary. Outward-only legacy topology is preserved. Negative travel through the opposite wall produces a subtractive opening, with no emerging body. Opposing pockets may meet to form a through-hole.
- [x] Exact planar-exit through-cuts from treated caps, including coincident entry/exit rims; preserve surviving treatment surfaces and clamp removal to existing material while retaining authored overrun. Other intersections route through the solid subtractor.
- [ ] General arbitrary evaluated-face sweep references and robust complex curved-exit trimming remain part of the broader face-editing/Boolean requirements.

Negative face extrusion follows the push/pull mental model: one action subtracts material while inside the solid and opens the solid after passing through it. No tool switch or emerging generator body is required. This differs intentionally from the explicit Generator + borer profile mode, whose separate-body behavior remains configurable.

### Source-prism Shell / thickness

Select an original planar face and choose **Shell** to remove that cap and create an inward cavity. Volume properties also offer **Shell · open top**. Wall thickness and opening update automatically; **Remove shell** restores the source solid. Boxes support all six openings. Cylinders and semicylinders support top/bottom openings. Source dimensions, placement and scale remain editable; thickness is uniform in source-local coordinates before object scaling.

The implementation reuses the existing exact pocket boundary builder, retaining shared edges and analytic/NURBS surfaces. Semicylinder insets offset the original circular arc and its diameter line, rather than translating a smaller half-disc. Cylinders expose five logical surfaces (outer wall, inner wall, back, opening rim, cavity floor); viewport tessellation patches are not separate faces. Circular inner rims/floor boundaries retain snapping metadata. Extending an annular cap keeps the new inner and outer walls separate.

Validation rejects collapsed/nonfinite thickness, unsupported openings, profile partitions, and combinations with source edge treatments, source deformation or Boolean operators. Invalid edits preserve the accepted geometry/history. Exact general offset/retrim for those combinations remains pending; no mesh-only shell approximation is substituted. `.osk` v14 stores the operation and older projects default to an unshelled solid.

### Source-prism Draft

**Draft walls** in Volume properties adds an editable signed angle and a neutral Top/Bottom cap. The neutral cap keeps the source dimensions and its plane while the other cap expands (positive angle) or contracts (negative angle). All side walls use that angle in source-local coordinates before object scaling. **Remove draft** restores the original prism; angle edits, neutral changes and removal use existing undo/redo and automatic updates. Finite angles within ±80 degrees are accepted only while the opposite section remains nonempty. Source parameters are retained in `.osk` v15.

Boxes use exact planar ruled walls. Cylinders use exact rational conical/cylindrical surfaces with three logical faces and true circular cap references. Semicylinders draft both the circular wall and diameter plane: their intersection edges are exact shared parabolas, with exact circular cap arcs and four logical faces. A scaled half-disc is not substituted. Regression checks verify surface radius and signed normal angle to 1e-8, parabolic edge incidence on both surfaces, neutral anchoring, mesh-volume bounds at the viewport tolerance, transformed bodies, collapse/nonfinite rejection, history, legacy version handling and four export encoders.

New source Draft/Shell actions are disabled on incompatible treated/deformed geometry. Arbitrary face sets, other neutral planes and operation combinations needing general offset/extension/retrim remain pending. Extent detection excludes drafted/shelled/extruded results from the untouched-primitive shortcut and evaluates their actual planar footprint. Checks cover an expanded drafted cap that exceeds the original radius and a shell opening whose centre is empty.

Source-shell rim retraction also keeps its original cavity floor, and back-cap cuts preserve side-wall thickness while thinning/removing the back wall. These cap cases reuse exact pocket construction instead of coplanar subtraction. Circular references follow the surviving floor/rims; negative overrun through a complete back-cap cross-section consumes the remaining material. Other complex intersections retain the general subtractor and its atomic failure behavior.

### Outward Shell and annular retraction

Shell properties now offer Inward/Outward. Outward grows material away from the original cross-section and closed back, leaving the original cross-section as the cavity. Its opening remains on the original support plane. Positive wall thickness is measured before object scaling. Boxes support every source opening; circles/semicircles either cap. Semicylinders preserve a true circular outer support and parallel straight chord, rather than a scaled/translated half-disc. Source dimensions stay authored and editable; changing direction/removing Shell uses the existing live editing and undo path.

The `.osk` v16 boolean `hollow.outward` defaults false for legacy shells; earlier version declarations reject outward shell data. Outer planar face selection and attached profile support/Through-all thickness use actual expanded supports. Circular outer/inner rims retain distinct analytic snapping radii. No dependency or mesh-only offset approximation was added.

Regression checks cover every supported opening on all three source types under rotation and nonuniform scale: closed retained boundaries, independent cross-section/volume formulas, stationary cavity floor/opening, exact semicircular offset corners, logical cylindrical faces and circle references, both cap extrusion directions and overrun, direction retention, undo/redo, invalid-input rollback, legacy/version-gate persistence and four export encoders. General shells on treated/evaluated solids and closed shells remain pending.

Annular opening retraction now has exact construction after reaching the cavity floor too: it leaves a stepped back, then only the central back core when all surrounding walls have been removed. This corrects the prior general-subtractor failure on coincident annular sweeps, for inward and outward shells. A filled back-cap sweep can instead consume the complete body. The result follows the swept face area, not the source profile’s enclosing disc.

Native macOS release review verified Inward→Outward on a cylinder, automatic thickness 0.4→0.7, separate undo boundaries for direction/thickness, redo, and toolbar Save. The saved v16 document retained `outward: true`, thickness 0.7 and unchanged source diameter 4/extrusion 3. Other source shapes/openings are covered by kernel checks; Windows/Linux native review remains pending.

### Closed source shells and enclosed boundaries

**Closed cavity · keep all outer faces** retains every exterior face and creates an enclosed inner boundary. The opening picker is disabled while closed; its retained value determines the source frame and restores the previous opening when closure is switched off. Inward thickness reduces the inner cross-section and offsets both inner caps, requiring a nonempty cavity. Outward thickness preserves the original source volume as the cavity and expands all outer supports. The source geometry, direction, thickness and closed/open state remain editable/removable with the existing undo path.

The retained BRep contains an outward exterior and an oppositely oriented cavity. Logical cylindrical selection has six surfaces: two outer caps, two inner caps and the two complete walls; tessellation patches do not become separate faces. Geometry reconciliation accepts properly enclosed inward boundaries, rejects unmatched/inverted boundaries, and keeps disconnected material islands independent. STEP groups cavities under their containing exterior as `BREP_WITH_VOIDS`, rather than writing each cavity as a second filled solid. The shared classifier uses signed volumes and sampled winding containment; analytic boundary surfaces/curves are retained for encoding. External CAD import acceptance remains separate from encoder checks.

`.osk` v17 adds `hollow.closed` (missing means false). Earlier version declarations reject closed shells. Checks cover both directions on all three source shapes under rotation/nonuniform scale, independent material-volume formulas, closed inner/outer topology, logical circle selection, reconciled print meshes and inversion rejection, cavity/island grouping, all four export encoders (including STEP void entities), editing/undo, save/load and legacy/version gates. General shelling on treated/Boolean/deformed geometry remains pending.

Native macOS release review verified closing an open outward cylinder, disabled opening selection while closed, Undo restoring the opening, Redo restoring closure, and saving `closed: true` with outward thickness 0.7 in v17. Source diameter 4/extrusion 3 remained unchanged. Internal cavity geometry and other shapes/directions are covered by kernel/export checks, not inferred from the exterior screenshot.

### Exact convex planar face Move / Rotate

New Face-mode gizmo operations on a convex solid with planar logical faces and straight boundary edges transform the selected supporting plane(s). Adjacent unselected supporting planes remain fixed; their exact intersections rebuild shared corners, edges and face trims. This covers source boxes, surviving planar chamfer faces and compatible convex evaluated polyhedra. It does not apply weighted vertex movement to those faces. Movement tangent to an infinite planar support leaves that support unchanged; use normal displacement to grow/shrink it. Rotation changes its orientation and trims against neighboring supports.

The existing component history records `retrim: true` in `.osk` v18. Missing/false retains legacy component deformation. Curved or concave bodies continue through that explicit existing representation; exact general curved/concave retrimming remains pending. The original logical topology stays guarded, and transformations that collapse an edge, consume a face, lose a bounded corner or require different corner connectivity reject transactionally. At most 256 planar faces are accepted for this exact path. Source placement and scale are applied consistently through the existing local/world transform conversion.

Kernel checks cover translated/rotated faces of plain and chamfered boxes with nonuniform scaling, invariant neighboring support equations, exact retained planar surfaces and shared topology, collapse rejection/unchanged history, undo/redo, legacy/version-gate save/load and all four exports. STEP assertions distinguish one analytic plane per logical face from triangulated fallback. The general face editing requirement remains open; this entry identifies the exact supported geometry rather than relabeling all component deformation as CAD editing.

Native macOS release review selected a box top face, moved it +1 along Z with the gizmo, rotated it 15° with the axis ring, verified rotation Undo, and saved v18. The file contained two evaluated Face edits with `retrim: true`; source width 4/depth 3/extrusion 3 stayed unchanged. Native review demonstrates picking/gizmo/history/save interaction; exact plane and export guarantees come from the geometry regressions above.

### Ordered evaluated planar face sweeps

Planar faces with retained kernel boundaries now expose **Extrude face** after exact component Move/Rotate and on Boolean results or flat transition regions. A guarded component operation records a local displacement vector and replays at its position in the existing ordered evaluated history. It does not insert the sweep before an earlier face movement. Changing the numeric Distance regenerates from the selection snapshot, so continuous property edits replace the same pending operation and keep the existing undo grouping. Source-only Offset remains separate and is not offered on an arbitrary evaluated face.

Positive Distance sweeps the selected evaluated trimmed boundary and joins its cap/walls to the retained body through shared kernel edges. The cap remains selected by its resulting support plane. Inward full prismatic caps remove an exact slab; reaching/passing the far cap consumes the existing material with no inverted/generated body. General inward profiles use the existing solid subtractor and atomic failure recovery. That fallback still has complex/coincident intersection limits; it is not completion of robust general curved cutting. Curved faces and mesh-only deformations reject with a specific explanation rather than fabricating an analytic boundary.

`.osk` v19 adds optional `component_edits[].sweep`, a three-component owner-local displacement; missing/null retains transform behavior. Sweep operations require evaluated Face mode, one guarded reference, no retrim flag, and an identity matrix. Initial UI Distance is measured along the selected world normal; the stored vector follows subsequent source placement/scale changes. Full operation ordering is preserved through save/load and Boolean consumption.

Checks cover moved/rotated planar faces with nonuniform source scale, world-normal added volume, inward slab/overrun volume, a subsequent face move after extrusion, planar chamfer-band sweeps, Boolean-result sweeps without revealing operands, invalid-input rollback, undo/redo, version gates/save/load and all four exports. Exact retrimming also recognizes seam vertices in the middle of an edge after a sweep: two supporting planes determine that line, so such vertices project to the exact intersection rather than being rejected for lacking a third corner plane.

Native macOS release review loaded the previously moved/15°-rotated box face, exposed evaluated Extrude face, applied Distance 0.5 automatically, and kept the new cap selected. Undo removed only the sweep; Redo restored it. Toolbar Save wrote v19 with the two preceding retrim operations followed by one evaluated sweep of magnitude 0.5, retaining original source width 4/depth 3/extrusion 3. Boolean/transition and inward cases are covered by geometry regressions; their broader native review remains separate.

### Exact convex planar subtraction for inward face sweeps

Inward evaluated-face sweeps now have an exact linear-boundary path after the existing prismatic-cap shortcut. It clips original face polygons outside the convex swept tool, clips reversed tool faces inside the body, reconciles every shared edge intersection, and constructs retained planar BRep faces. The implementation reuses the established half-space polygon clipper used by source treatments. It accepts convex closed boundaries with actual line edges and planar polygon faces; curved, holed/nonconvex or unsupported boundaries keep the solid-kernel fallback.

The requested floor ends at the authored distance exactly. The small entry margin runs along the sweep direction, so it neither changes side-wall planes nor adds unrequested depth under nonuniform source scaling. Overrun removes only existing material inside the swept footprint. A tilted cap can therefore leave a side wedge whose projection lies outside that footprint; it does not become an inverted or emerging solid. Coincident outer/tool supports do not introduce zero-thickness walls. Separate remaining regions of an original face are assigned separate logical selections; only adjacent fragments of the same source region merge.

Linear construction is bounded to 512 body planes, 256 tool planes, 4,096 output face fragments, 8,192 unique vertices and four million edge/vertex reconciliation candidates. Construction/validation failures preserve the accepted document; the existing background worker/cancellation model is unchanged. This improves the supported planar operator without claiming general curved/concave Boolean robustness or analytic/preview separation.

Regression independently checks a 15° tilted cap on a nonuniformly scaled box: blind-cut removed volume includes the exact boundary wedge term, and overrun leaves the analytically predicted side wedge. It verifies the direct planar path rather than merely allowing a successful kernel fallback, retained planar surfaces, closed welded print boundaries, undo/redo, `.osk` regeneration and four export encoders. A crossing slab produces two closed material bodies and twelve independently selectable faces; curved cylinder boundaries explicitly decline this linear path.

Native macOS release review applied −0.4 Distance to the previously moved/15°-rotated box cap. The cut updated automatically, the resulting floor remained selected, and the boundary outside its swept footprint remained visible. Undo restored the prior body, Redo restored the cut, and Save retained the two preceding transforms plus the signed 0.4 sweep in v19 with unchanged source dimensions. General exit/volume guarantees are from the regression checks above.

### Shared subtraction dispatch

The exact convex planar path now runs in `operators::difference`, shared by ordered volume Subtract, profile boring and face sweeps. Callers no longer choose different geometry algorithms for the same supported boundary. Surface-role mapping lives on `Geometry` and retains the existing exact-face/legacy-triangle correspondence checks. Unsupported curved/nonconvex/holed boundaries retain the existing Truck fallback; budgets and atomic failure behavior remain unchanged.

For supported disjoint or touching-only polyhedra, subtraction returns the original retained geometry without adding coplanar partitions or changing topology references. Tests verify ordered target/tool reversal, Keep tools, analytically known rotated-slab removal, two closed disconnected results, enclosed voids with STEP `BREP_WITH_VOIDS`, original-reference preservation, full consumption, undo/redo, save/load and four exports. Boundary-crossing rectangular profiles cover blind/exact-exit/overrun depths in Borer and Generator + borer modes, checking both target removal and separate emerging volume. No project-format change is needed for this shared evaluator improvement.

These are geometry/serialization/export checks of existing workflows. They do not establish native interaction coverage on every platform or completion of general curved/concave Boolean robustness.

### Construction / datum planes

**Add construction plane** creates a reference at the origin and activates it for drawing. It appears as a distinct wireframe rather than an opaque physical sheet. Select it in the Model list or select its outline in Volume mode, then use Move/Rotate gizmos, placement fields, XY/XZ/YZ orientation presets or reference width/depth. Existing object/history paths handle creation, placement, deletion and undo; no separate scene graph or dependency is added.

**Draw on this plane** activates an existing reference. The active plane overrides hovered physical surfaces and the base-plane preference; an explicitly selected physical face still takes precedence. **Auto drawing plane** returns to existing selected/hovered-face/XY behavior. Changing the active plane cancels an incomplete two-point gesture. Construction-plane creation automatically selects Volume mode and activates the new reference. If the reference is deleted or removed by undo, its active drawing preference clears safely.

Grid and anchor coordinates use the reference origin and rotated orthonormal axes with world-unit spacing. Reference display scaling changes its visible size, not grid units. New rectangles/circles/lines/semicircles capture placement on this infinite plane through the existing mouse construction path. They remain independently authored geometry: moving/deleting the datum later does not move/delete drawings already created there. An active reference does not implicitly attach a destructive bore to a solid beneath the pointer; explicitly selected physical support faces retain their prior attachment behavior.

Construction references are not export geometry, profile cutters, extrusions, Boolean operands or component-deformation targets. They remain selectable reference objects and snapping guides; their wireframes do not occlude physical surfaces. Body validation rejects incompatible geometry operations, and export filters references before format-specific physical geometry validation. A reference-only scene has no exportable geometry.

`.osk` v20 stores `body.construction` (missing means false) and optional `drawing.plane` (missing/null means automatic). Active IDs must resolve to construction objects. Existing projects preserve physical-object behavior. Checks cover arbitrary and axis-aligned orientations, nonuniform display scale, reference-centered grid spacing, plane inference, circle placement/extrusion, export exclusion in every format, whole-reference movement and undo, independent drawing retention, Boolean/invalid-operation rollback, deletion, persistence, legacy defaults and version gates. Native creation/placement/drawing interaction is verified separately.

Native macOS release review created/activated a construction plane, chose YZ, placed it at X=1, drew a two-click circle on that plane and extruded the circle to 1. Moving the reference to X=2 preserved the solid at X=1. Reference movement Undo/Redo worked; deleting the active reference preserved the solid and cleared activation, Undo restored the reference, and explicit reactivation/Save retained both objects and `drawing.plane: 1` in v20. Export exclusion and arbitrary reference orientations are established by the automated checks above.

### Explicit multiple-body Through all

An attached profile using **Through all** now lists additional earlier solid bodies as checkboxes. The attached body is always included; only checked additional bodies are cut in Borer modes. Bodies between the targets remain unchanged unless explicitly included. Up to 64 additional, distinct earlier IDs are retained. Unchecking/changing the scope regenerates affected bodies, and removing the bore restores their material. Switching away from Through all clears the additional list. Missing, duplicate, forward/self/attached duplicates or unsupported scope references reject transactionally.

The end distance derives from the current retained boundaries of the selected scope, not an arbitrary large travel or viewport vertices. Planar boundary lines, spline/rational control hulls and transformed revolutions provide conservative support intervals along the profile axis. Positive rational weights are required. The endpoint may conservatively extend beyond curved material; removal is clipped to existing geometry. Scope moves and changes regenerate this distance; manual travel retains direction rather than receiving the derived value. The existing 10,000 mm derived-travel limit applies.

Borer and Generator + borer cut the attached and checked bodies. Automatic scoped Through all ends at the scope and emits no emerging extension or material between bodies; use Distance for the existing manual emerging-body behavior. New body uses the scope for its full sweep extent without cutting targets. Scope is inward from the attached face. Existing single-target Through all keeps its original behavior when the additional list is empty.

Parallel contained secondary targets reuse the exact pocket/through-hole constructor, avoiding repeated cylinder intersection solving. Other secondary cuts use the shared solid subtractor. Every target is marked modified for subsequent feature evaluation, preventing later pristine shortcuts from rebuilding already removed material. Circular pocket/through-hole construction now preserves original outer snap circles and adds the actual entry/floor/exit references, including secondary targets whose support lies below the profile's initial plane.

`.osk` v21 adds optional `bore.through_targets` (missing means empty). It is valid only for Through all. Additional targets must be earlier evaluated standalone solids in the current pre-Boolean construction stage; explicit Boolean-result scope references remain part of the broader cross-stage reference gap. Existing component topology guards still apply when later edits replay after boring. Curved general cuts retain the solver's robustness limits; conservative bounds do not claim to fix that solver or Up to face trimming.

Checks cover box/cylinder stacks, multiple scope targets, an unchecked body between targets, all output modes, independent travel/removed-volume/end-plane assertions, downstream target movement, operation removal, atomic history/failure/deletion behavior, invalid scopes, legacy/version-gate persistence and four exports. Bounds are checked against transformed curved geometry and independent analytic full-cylinder extrema; viewport chord approximation is separated from exact sweep end-plane checks.

Native macOS release review loaded an attached Through-all circle over three separate boxes, checked only Rectangle 2 while leaving Rectangle 3 unchecked, verified checkbox Undo/Redo and saved v21. The saved bore retained primary target 1, `through_targets: [2]`, Through all and its original manual direction value. Actual selected/unchecked volume behavior and automatic regeneration are established by the geometry regressions above.

### Angled planar end faces

Up to face projects the source wire along its sweep axis onto the referenced plane. Circular boundaries remain analytic ellipses on that plane. The target used as the end reference is unchanged. Generate creates the terminated body; Borer subtracts the swept region; Generator + Borer retains its explicit separate-body emergence behavior. Ordinary negative face extrusion remains subtraction, including travel beyond the opposite boundary.

Contained blind pockets on pristine source caps join exact shared boundaries without invoking a general Boolean solve. Partial crossings, edited supports, and curved intersections retain the Boolean solver's limitations. Up to next also accepts angled planar stops; curved and multi-plane stops remain pending. No new serialized fields are required.

### Angled planar Up to next

Up to next searches finite evaluated planar regions on its explicit end body. Coplanar logical faces are considered together. The entire profile must reach one support plane ahead in the selected direction and within the existing distance bound. Coverage uses the shared planar region classifier after projecting the profile along sweep rays; pristine parallel source caps retain the analytic containment shortcut. Trimmed/curved region coverage still uses tessellated boundaries at kernel tolerance.

A plane's depth range alone does not determine the first finite hit. Before accepting a complete planar stop, the evaluator clips the other evaluated surface triangles between the profile and that stop, projects their finite regions, and rejects any overlapping nearer obstruction. This prevents jumping across a partial face, hole rim, or curved surface to a farther planar face. A partial patch entirely behind the accepted stop does not block it. General curved and piecewise-planar end caps remain separate unfinished work.

The accepted angled plane reuses exact projected-wire termination and contained-pocket construction from Up to face. Mode, scope, separate-body emergence, transaction/undo, error recovery, and serialization use existing paths. Regression checks cover all closed profile types, material modes, known volumes, end-body preservation, finite/disjoint/partial coverage, competing finite surfaces, direction, regeneration, rollback, undo/redo, `.osk` round-trip and four exports.

Native release review loaded an angled circular blind bore using Up to next, changed the end-body rotation from 15° to 30°, verified Undo/Redo, and rejected a narrowed end body while retaining the accepted geometry. Escape discarded rejected input; Save retained end width 4, rotation 30° and the existing Up to next target ID in the `.osk` file. Curved/multi-plane completion is still unchecked above.

### Delete Face: Open / Cap / Heal

Face mode exposes three explicit delete behaviors, accepting multiple logical faces on one or several bodies atomically. Operations address evaluated logical faces and remove every retained raw patch belonging to each selected face; tessellation subdivisions are not separate inputs. Source parameters stay authored. Operations replay in component-history order after earlier sweeps/transforms/Boolean evaluation, with existing topology guards and error recovery.

- **Open** removes selected surfaces and leaves the surviving exact boundary as a sheet when openings remain. Exposed boundary edges remain selectable. Removing an entire disconnected boundary can leave a closed solid; deleting all surfaces leaves empty geometry.
- **Cap** removes the selected faces and closes the resulting planar openings. Original analytic wires are preserved, including circular/elliptic trims. Coplanar nested loops form outer boundaries and holes; disjoint loops form separate caps. A nonplanar loop rejects rather than being flattened.
- **Cap openings** is available in Volume properties for retained open sheets. It closes all planar open boundaries without deleting more faces. Closed material must pass solid-boundary checks; coincident zero-thickness caps are rejected.
- **Heal** removes selected constraints from a convex planar solid and extends/intersects the remaining supports to form closed material. Removing chamfer bands can restore sharp planar corners. This path requires 4–64 remaining supports and rejects unbounded, collapsed, curved, or nonconvex results. General analytic curved/concave healing remains pending.

Open sheets export to GLB, STEP and DXF; STL explicitly rejects them until they are closed. Complete retained circular snap references survive where the ring remains. New caps and healed faces receive fresh output slots; downstream edits retain strict topology guards. Undo/redo restores the authored operations; removing component edits restores the source result.

`.osk` v22 adds optional `component_edits[].face_edit`: `open`, `cap`, `heal`, or `cap_openings` (missing/null preserves legacy transforms/sweeps). These operations require evaluated Face history, identity matrix and no sweep/retrim combination. Cap openings uses the first face as a guarded owner anchor. Regression checks cover transformed/nonuniformly scaled boxes, cylinders and semicylinders, annular cap holes, exposed edge selection, solid/sheet export rules, source regeneration, multi-body atomic rejection, undo/redo, round-trip, version gates and convex chamfer healing. General curved/concave replacement, nonplanar caps, and general curved healing remain unchecked requirements.

Native release review selected a box top face, used Delete face · Open, confirmed Cap openings appeared, re-capped it, and verified Undo/Redo restored the opening/closure. Save retained ordered `open` / `cap_openings` edits in `.osk` v22. A second native fixture deleted/healed one chamfer band and saved its guarded `heal` operation successfully.

### Live Replace Face and convex planar intersection

Select the source face first and the replacement reference face second, then choose **Replace face**. For a single selected face, **Replace using plane** lists available construction planes. The referenced infinite planar support replaces the selected support; its finite footprint does not clip the source. Source-face orientation is preserved independently of the reference face's outward normal. Neighboring boundaries are extended/retrimmed to the new plane.

Replacement reuses ordered two-input feature history, guarded topology and increasing feature IDs. The original source body remains authored but is suppressed from rendered/exported output; the replacement owns the resulting whole solid. The reference body remains unchanged. Removing the replacement restores the source. Source/reference edits regenerate the result; missing references, cyclic/forward dependencies, changed topology, collapsed/unbounded or unsupported geometry reject transactionally. Reference snapshots are not substituted for live dependencies.

Convex planar solids reuse exact plane retrimming, including evaluated Boolean results. Original circular/semicircular prism caps reuse exact variable-height sweep termination from the opposite fixed cap, retaining analytic circular/elliptic boundaries and logical curved surfaces. Altered curved prisms are checked against original evaluated geometry before this source reconstruction path is allowed; a bored/treated/deformed prism cannot silently lose its prior changes. General curved/concave replacement remains pending.

Mixed Replace/Boolean chains share the existing dependency-ordered evaluator. Unequal-depth convex planar Intersect now clips exact retained linear boundaries against the other operand's supports, avoiding coincident-face solver failures. Contained/identical inputs preserve their original boundary/reference layout; disjoint or touching-only intersections produce empty material. Curved/nonconvex intersection continues through the kernel path.

`.osk` v23 adds `boolean.kind: "replace_face"` using the existing two face picks and connectivity signatures. Its reference is always retained and its source always replaced, so `keep_tools` must be false. Replace Face is not a face-region set operation. The existing component history can follow the replacement result. Tests cover top/bottom replacement under rotation/nonuniform scale, known volumes and exact cap planes, logical circular surfaces, live regeneration, rollback, restoration, history, save/load/version gates, four exports, mixed feature chains, and affine/coincident/contained/touching convex intersections.

Native release review selected a cylinder top cap and chose its angled construction plane through Replace using plane. Changing the plane from 15° to 30° regenerated the cylinder cap visibly; Undo/Redo restored both. Removing the replacement restored the original cylinder, and Undo restored the feature. Save retained source face 5, construction reference face 1, guarded input signatures and the live 30° plane pose in `.osk` v23.

### Planar face Imprint and smooth swept walls

Select a solid face first and a coplanar unextruded profile face second, then choose **Imprint**. The operation splits only that selected logical face and retains the solid's material. Profiles on another plane reject; no projection onto hidden faces occurs. The source and profile remain authored and are suppressed by default. Keep tools retains the profile; removing Imprint restores both inputs. Upstream edits regenerate the partition under the existing topology guards.

Contained rectangular/circular/semicircular profiles reuse their exact analytic wires as shared boundaries between the outer region and the new inner face. Existing support holes remain holes. Straight-boundary crossings use the existing planar region classifier and exact linear boundary reconciliation, adding shared vertices to adjacent faces while leaving opposite faces intact. Disconnected regions are independently selectable. A profile that does not divide the support into nonempty regions rejects. Curved boundary crossings and open-curve imprinting remain pending; supported contained curves are not replaced by polygonal approximations. Classification retains the current kernel tessellation/tolerance limits.

New evaluated sweeps group side patches by analytic tangent continuity around each boundary wire. Smooth circular/elliptic arc segments form one logical wall; corners remain real edges, and separate loops keep separate walls. Grouping follows the selected boundary rather than the source body's primitive label, so a circular region imprinted on a box extrudes as one curved wall.

Supported inward regions on convex planar bodies reuse exact shared-boundary construction for blind pockets. A parallel complete prism exit reuses the existing through-cut builder and clamps removal to existing material, preserving authored overrun without generating an inverted extension. Conservative retained-curve bounds prove the interior/side clearance; unsupported concave/curved intersections retain the existing solver path. This does not complete general curved cutting.

`.osk` v24 adds `boolean.kind: "imprint"` and optional `component_edits[].tangent_walls`. Imprint requires two face references. New evaluated sweeps set tangent walls; missing/false retains the legacy grouping and evaluation behavior, preserving previously saved histories. The flag requires a sweep and cannot be used for unrelated transforms.

Regression covers all basic closed profiles on boxes/cylinders/semicylinders under rotation/nonuniform scale, existing annular loops, independent outward regions, fast blind/through cuts, crossing/touching rectangular boundaries, adjacent-edge splits, unchanged opposite-face area, keep/remove tools, rollback, history, migration/version gates, round-trip and four exports. Open curves, analytic curved crossings and general inward curved trimming remain unchecked requirements.

Native release review selected the box face first and Shift-selected its coincident circle profile. Imprint exposed a separate circular face; entering −4 mm on the 3 mm body produced a through-hole promptly, with no emerging extension. Undo restored the imprinted region and Redo restored the cut. Save retained ordered input picks, `imprint`, the −4 mm sweep and `tangent_walls: true` in `.osk` v24. Circular snap metadata for new pocket floors/exits is separately covered by kernel/export regression.

### Whole profiles, holes and disconnected imprint regions

For a selected solid face, **Imprint using profile** lists visible retained sheet bodies and uses all their regions as one tool. The two-face command continues to use only its explicitly selected tool face. Every selected region must be coplanar with the support; missing, nonplanar, offset or changed-topology inputs reject atomically. Whole-profile tools can be authored through existing face Union/Subtract or retained open sheets.

Contained tools preserve exact analytic wires and use the existing nested-loop classifier to construct the outside regions, hole islands, and profile regions separately. A tool's hole remains material; it is never filled into the imprinted region. If a tool hole surrounds an existing source hole, that original void remains a hole in the correct new island. Disconnected regions receive independent logical face slots. Retained circular snap references are copied only for actual selected boundary loops.

Straight-edge crossings also support holed/disconnected partitions. Holed planar patches are triangulated only as an exact linear reconciliation input; shared edges and logical groups hide those internal seams. Curved trims are not flattened by this path. The broader volume Boolean dispatch retains its prior strict linear conversion contract.

Through-cuts of ring regions use the same nested-loop classifier at the exit. The central material island receives its own closed cap and can remain a disconnected body, while the outer material remains closed. Existing tangent-wall grouping preserves one smooth wall per circular loop. Blind/outward/through sweeps, undo/redo, input edits, rollback, restore-input behavior, serialization and four exporters use the existing paths. General curved boundary crossings and open-curve imprints remain pending.

`.osk` v25 allows an Imprint tool pick with a null element to mean the entire profile body; its source still requires a face element. Existing explicit tool-face picks preserve their scope. Guards retain both input connectivity signatures; the whole-profile form is rejected under earlier declared versions. Checks cover ring/island areas, disconnected tools, shared-edge crossings with a holed partition, live source/profile dimension edits, separate material parts after ring through-cuts, exact nested circular loops on an existing annulus, printable boundary reconciliation, an authored analytic ring sheet, history, version gates, round-trip and four exports.

Native release review loaded an authored rectangular ring sheet, selected the solid support and chose the entire sheet through Imprint using profile. The center remained independently selectable; selecting/extruding the surrounding ring by 1 mm kept its hole open. Undo/Redo restored the flat/raised ring. Save retained the source sheet subtraction, whole-profile null tool element, guarded input signatures and tangent-aware sweep in `.osk` v25.

### Finite straight-curve Imprint

Select one solid planar face and choose **Imprint using curve**, then a visible standalone line. The line must be on that face plane; its actual endpoints determine the scope. A fully covered material interval is split into independent logical faces. Multiple intervals across a holed rim are supported. On a concave face, a finite line can cross just one arm; intersections outside the segment do not insert vertices or split other arms. A line stopping inside material rejects rather than extrapolating. Tangent/coincident-only contact and cuts that cannot form independent closed regions reject without changing the model.

Curve intersections use native B-spline/rational control hulls and parameter roots, preserving analytic subcurves rather than flattening circular boundaries. Shared original edges are split on adjacent surfaces too; those surfaces retain their original geometry. Region classification samples retained curves only to classify trim loops. Whole-circle snap references survive across split rims. Each resulting logical region can be selected/extruded independently. Source and line remain live, guarded inputs, suppressed by default; Keep tools retains the line and removing the operation restores the inputs. This operation serializes as `imprint_curve` in `.osk` v26.

Runnable checks cover rectangle/circle/semicircle caps, holes, finite concave-arm scope, transformed elliptic rims, unchanged material, analytic edge retention, independent positive sweeps, rejected finite/offset/stale inputs, live line changes, undo/redo, removal, Keep tools, version gates, save/reopen and STL/GLB/STEP/DXF output. Arbitrary curved tools, projected/noncoplanar imprints, intersection-curve trims and analytic curved closed-profile crossings remain pending.
