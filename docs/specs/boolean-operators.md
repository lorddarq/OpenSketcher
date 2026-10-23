# Ordered Boolean operators

## Research and product decisions

[Onshape Boolean](https://cad.onshape.com/help/Content/PartStudio/boolean.htm) distinguishes union, subtraction, and intersection, explicit targets, and optional retained tools. Earlier selected entity attributes survive a merge. Its documented surface support does not imply arbitrary open sheets can be treated as closed volumes.

[Rhino CurveBoolean](https://docs.mcneel.com/rhino/8/help/en-us/commands/curveboolean.htm) classifies overlapping planar regions, permits region selection, and outputs curves or planar surfaces. Input deletion is configurable. This is a useful model for OpenSketcher's face-region operations; crossing or curved surfaces instead require trimming/intersection curves.

OpenSketcher uses selection order consistently: Shift-click A, then B. Subtract means A minus B. Union is A ∪ B; Intersect is A ∩ B. Clicking Swap reverses the operand order. The primary selection can remain the most recent click without changing the first operand's role.

## Implemented contract

- [x] Exactly two selected volumes or faces expose Union, Subtract, Intersect.
- [x] Target/tool order appears before application and can be swapped.
- [x] Boolean results retain both authored operands as dependencies, rather than destroying source parameters.
- [x] Result inspector supports swapping operands, Keep tools, and removing the operation to restore its inputs.
- [x] Volume operations require closed solids. Their result is a new body, possibly containing disconnected solid components; empty intersections/subtractions are valid empty results.
- [x] By default consumed volume operands are hidden. Keep tools preserves the second operand. Sources remain accessible in the Model list for editing; suppression is derived, never saved as destructive deletion.
- [x] Face operations combine the selected regions only. They require planar, coplanar faces, support holes/disconnected regions, and create a separate sheet. Selecting a solid's face does not consume its entire solid. Standalone input sheets are consumed unless retained as tools.
- [x] Face topology references are guarded; a changed topology asks for reselection rather than silently choosing a different face.
- [x] Evaluation runs through the existing bounded background worker, preserving UI responsiveness and the last valid scene on failure. Undo/redo and automatic parameter updates reuse existing transactions.
- [x] `.osk` schema/modeling version 7 serializes operation kind, ordered operand references, face topology guards, and tool retention. Versions 1–6 remain readable.
- [x] Existing export paths receive evaluated result geometry and omit suppressed operands.

## Geometry limitations and next gaps

- [ ] General noncoplanar sheet trimming, curved face Boolean regions, intersection curves, and split/slice tools.
- [ ] Persistent semantic references for arbitrary generated faces, beyond guarded topology slots.
- [ ] Kernel-independent robust handling of every coincident/tangent solid case. Truck failures currently reject atomically.
- [x] Retain analytic B-spline/NURBS trims in newly authored coplanar face Union/Subtract/Intersect, including crossings, holes, disconnected regions and opposite input normals.
- [x] Coincident rational-conic trim merging, including circular/elliptical partial arcs with different interval boundaries, parameterizations and orientations.
- [ ] General nonconic spline overlap merging and intersection-generated input trims. These remain rejected in the analytic face evaluator; historical face/volume-prism features retain their saved polygonal behavior.
- [ ] Arbitrarily oriented matching-depth prism fast path. Current fast path covers world-Z straight prisms with matching cap planes; other solids use Truck CSG.
- [ ] Applying bores or direct component transforms to Boolean result bodies; edit source operands for now.
- [x] Native volume subtraction and operand-swap review.
- [ ] Native face-operation review and external CAD acceptance of Boolean exports.

The coincident-cap prism path shares the existing planar arrangement classifier, sweeps once, and retains a closed Truck shell. No extra geometry dependency or UI-thread CSG is introduced. Feature IDs reference earlier bodies, preventing cycles; removing a referenced operand rejects the transaction until the dependent Boolean is removed.

## Verification

The runnable `booleans::tests` check covers ordered subtraction in both directions, union/intersection, consumed/retained tools, parametric regeneration, serialization validation, and undo/redo. Broader geometry cases should extend this check as kernel limitations are addressed.

## Analytic planar face sets (.osk v29)

New face set operations share analytic intersections and directed trim-loop assembly with Imprint. The first picked face supplies the result normal and subtraction target. Boundaries are split at shared curve parameters; winding on both sides of each piece determines whether to keep or reverse it. Output faces retain the source trim curves independently of viewport tessellation. Complete circular loops retain center/rim snapping metadata. No solid is removed when only one of its faces is used; the output is a separate sheet.

`analytic_faces: true` versions this behavior. Old files replay the polygonal evaluator when the field is absent/false. Ordered volume operations keep their existing evaluator. Offset planes, curved supports, stale guards, unresolved nonconic spline overlaps and intersection-generated trims reject without changing committed inputs.

Runnable checks `analytic_face_booleans_keep_arcs_order_holes_and_saved_behavior` and `mixed_analytic_face_booleans_and_solid_face_inputs_share_the_arrangement` cover the three set operations, both subtraction orders, all rectangle/circle/semicircle pairings, contained/crossing/disjoint circles, opposite normals, holes, affine placement, live edits, rollback, undo/redo, STEP and `.osk` migration gates.

Native macOS review created an Intersect operation from two canvas-selected circle faces (Shift-click), saved `analytic_faces: true`, and verified subtraction reversal via Swap operands plus undo and save. The review project is `/tmp/osk-analytic-face-boolean-review.osk`; the isolated app is `dist/OpenSketcher Analytic Face Boolean Review.app`. This is macOS runtime evidence, not Windows/Linux interaction coverage.

### Shared conic boundaries

The analytic face evaluator recognizes an entire common conic support from rational quadratic Bézier controls and verifies the implicit equation's complete Bernstein coefficients on both trims. It does not infer overlap from display samples or a few matching points. The test is independent of source arc starts, rational parameterization and nonuniform affine placement. Ill-conditioned or unproven supports continue through the bounded general solver and may reject safely.

For proven common supports, endpoint-on-curve roots split both boundaries into matching intervals. Duplicate intervals are consumed once; complementary arcs with matching endpoints remain distinct. Material winding still determines the requested Union/Subtract/Intersect set. Identical regions therefore union/intersect to one region and subtract to empty. Circle/semicircle and rotated semicircle overlaps retain exact partial arcs, including trims produced by earlier Boolean features. No new file field is required: the existing `analytic_faces` path resolves a previously rejected configuration; legacy polygonal operations and legacy Imprints retain their prior paths.

`coincident_conic_face_sets_keep_exact_partial_and_reparameterized_arcs` and `coincident_trim_chains_preserve_history_and_reject_false_supports` cover shared conic intervals, tilted ellipses, different arc starts, partial overlap, trimmed-output chains, near-support rejection, undo/redo, rollback, persistence and analytic STEP export. General coincident nonconic splines remain pending. Analytic Imprint also reuses proven shared conic intervals, retaining the existing rim once and adding only interior divider pieces; coincident-only profiles reject because they do not partition a face.

Native macOS review opened a circle minus a same-radius semicircle rotated 17° (different rational arc intervals), verified the remaining half-disc, swapped operands to an empty result, then undid and saved the restored geometry. The isolated app is `dist/OpenSketcher Conic Overlap Review.app`, with `/tmp/osk-conic-overlap-review.osk`. This does not establish Windows/Linux interaction coverage.
