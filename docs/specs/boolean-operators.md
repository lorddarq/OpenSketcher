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
- [ ] Preserve analytic arcs in planar arrangement output. Current planar regions use evaluated face boundary sampling; result boundary edges are polygonal, while each planar region remains a single selectable face, not selectable tessellation triangles.
- [ ] Arbitrarily oriented matching-depth prism fast path. Current fast path covers world-Z straight prisms with matching cap planes; other solids use Truck CSG.
- [ ] Applying bores or direct component transforms to Boolean result bodies; edit source operands for now.
- [x] Native volume subtraction and operand-swap review.
- [ ] Native face-operation review and external CAD acceptance of Boolean exports.

The coincident-cap prism path shares the existing planar arrangement classifier, sweeps once, and retains a closed Truck shell. No extra geometry dependency or UI-thread CSG is introduced. Feature IDs reference earlier bodies, preventing cycles; removing a referenced operand rejects the transaction until the dependent Boolean is removed.

## Verification

The runnable `booleans::tests` check covers ordered subtraction in both directions, union/intersection, consumed/retained tools, parametric regeneration, serialization validation, and undo/redo. Broader geometry cases should extend this check as kernel limitations are addressed.
