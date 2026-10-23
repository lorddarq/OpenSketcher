# Circle faces: snapping and extrusion operations

Status: center/rim snapping implemented; initial attached solid cut implementation.

## Reference behavior

- [SketchUp drawing/inference](https://help.sketchup.com/en/sketchup/introducing-drawing-basics-and-concepts) distinguishes center, on-edge, and on-face references. Drawing onto a face divides that face; inward Push/Pull removes material, while pulling outward generates geometry. A geometric inference alone does not determine a Boolean operation.
- [SketchUp Push/Pull](https://help.sketchup.com/en/sketchup/pushing-and-pulling-shapes-3d) supports pockets and through openings with opposite-face constraints.
- [Fusion Extrude](https://help.autodesk.com/view/fusion360/ENU/?guid=SLD-EXTRUDE-SOLID) separates profile/support selection, direction/extent, and the explicit Cut/Join/New Body operation with target bodies.
- [Fusion circle/hole tutorial](https://help.autodesk.com/cloudhelp/ENU/Fusion-Sketch/files/GUID-AEFB2074-65F7-46CD-B15E-EF12AC2EE1BC.htm) places circular profiles using geometric references and explicitly extrudes them as cuts to make holes.

## Implemented inference

- Intact circular source geometry supplies derived circular rim descriptors (center plus two transformed radius axes). Nonuniform scale is an affine ellipse; center/rim references follow that evaluated placement.
- Expose Circle center and Circle rim labels during drawing. Analytic rim snapping minimizes pointer distance to the projected parametric curve, rather than to display mesh segments.
- Cylinder top/bottom caps retain their actual elevations. Chamfer/bevel cap insets and wall-transition rings expose their actual radii and elevations.
- Filter references by the captured support plane and visibility. A hidden base center must not pull a new sketch below an elevated cap. Snap and geometry preferences continue to apply.
- Component-deformed and partitioned circles do not advertise an intact source rim that no longer exists.
- This metadata is regenerated, not serialized as baked project geometry.

## Confirmed construction intent

The confirmed margin gesture is the **new circle’s center snapped onto an existing circular rim**, making an edge notch when extruded inward as a cut. Center-based placement remains available. This does not change circle creation into a diameter-endpoint gesture. See [geometric operator behavior and gaps](geometric-operators.md).

## Cut contract

1. Capture the support face and target body when a rim-start profile is created; retain the original profile parameters and an explicit cut feature. Circle placement on a center does not forbid a later Cut operation.
2. Expose the resolved operation, target, direction, and depth in properties/preview. The requested margin workflow can choose Cut initially; users must be able to switch it to ordinary generation deliberately.
3. Cut only the swept intersection with the chosen target. Crossing the face boundary produces a notch; a contained circle produces a pocket or hole. Snapping to a visible rim must never cause projection to the hidden base plane or delete unrelated bodies.
4. A pocket, exact breakthrough, and overshoot must be distinct outcomes. Preserve the existing planned Borer-only / Generator-plus-borer choice, with any emerging generation remaining a separate body.
5. Recompute after radius/position/depth changes and save/reopen; removing the cut restores target material. Group the feature change in undo/redo and reject kernel failures without replacing committed geometry.
6. The existing `cuts_profiles` option only partitions coplanar profile regions; it is not a solid cut on an elevated face. Do not reuse it as a misleading implementation of this contract.

## Checks

- [x] Runnable snapping regression: rotated/nonuniformly scaled elevated cap, exact center, arbitrary analytic rim point, rounded rim inset, visibility/support filtering, and disabled geometry snapping.
- [x] Confirm margin construction: new circle center snapped onto an existing rim.
- [x] Saved target association, explicit operation controls, and real solid subtraction on supported original primitive faces.
- [x] Geometry/regeneration checks for pocket, notch, breakthrough, overshoot, undo/removal, failure preservation, and export encoders.
- [ ] Broader native construction review and external slicer/CAD acceptance.
