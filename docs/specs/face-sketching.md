# Face-first sketching and circle creation

Status: behavior requirements and proposed implementation sequence; not implemented.
Date: 2026-10-23.
Related: [workflow research](../research/face-sketching-workflows.md), [soft selection](soft-selection.md), [product scope](../product-scope.md).

## Confirmed behavior

- Circle creation is driven on the canvas: pick a center point, move the mouse to set radius, then click to finish. Radius, not diameter, is the user-facing creation measurement.
- Drawing on existing geometry requires the user to select the supporting face first. Hovering over another face must not silently change the support.
- Selecting that face exposes its relevant points and boundary references so the user can position the new shape accurately.
- Snapping starts enabled. The canvas preview and numeric controls must describe the same geometry.

The precise shortcut choices, point display policy, face-boundary handling, and support persistence described below are proposed defaults. They are not claims that Plasticity or SketchUp use exactly this workflow.

## Face selection establishes context

1. In Face mode, the user clicks a visible supporting face.
2. Highlight that face and expose its boundary vertices, boundary edges, and applicable snap references. Keep these visible when a creation tool is activated; entering Circle must not discard the selected support.
3. Display the support explicitly, for example `Sketch plane: Object 3 / selected face`, with a local-axis indicator.
4. The selected face establishes a plane origin, orthonormal U/V axes, and normal N. These are geometric plane axes, not necessarily the raw NURBS parameter directions, which need not represent equal physical distances.
5. Starting a sketch captures this support. Orbiting or hovering over other faces does not change it. Offer an explicit Align view action; selecting a face alone need not move the camera.
6. A world-plane sketch remains available without a selected face. Indicate the active plane explicitly; never present a ground-plane result as though it were attached to a hovered face.

The initial support implementation should accept analytically planar faces, including translated, rotated, and scaled planar faces. A curved NURBS face is not a plane. Do not substitute the plane of one display triangle or average its normals. Report curved support as unsupported initially. Tangent-plane sketching and projecting a curve onto a curved surface are separate future operations with different geometry semantics.

## Exposed points are contextual references

Show only the selected face's relevant topology vertices and boundary references, rather than every vertex in the scene. Add available edge midpoints and analytic centers as snap references with distinct labels. Computed midpoint/center references are guides, not new editable model nodes.

Do not expose tessellation triangle vertices as authored nodes. Do not label a face centroid as an analytic circle center or other exact construction point. Circle center must come from the authored circle or analytic curve definition where available.

NURBS control points and custom deformation nodes belong to explicit node-edit mode. Selecting a support face may reveal them as a separate, labeled overlay when that mode exists, but must not make normal point picking deform the support accidentally. Soft-selection falloff remains governed by its own spec.

Keep hidden references depth-aware by default. Do not snap through unrelated opaque geometry. Use screen-space acquisition tolerances for points and edges, while calculating the result in the support's local plane coordinates.

## Center-and-radius circle gesture

| State | Canvas behavior |
| --- | --- |
| Await center | Show the active support and relevant snap points. Cursor position is projected onto the support plane. |
| Center picked | Fix the center; show its marker and a radial guide. |
| Radius preview | Mouse movement updates an analytic circular outline and a readout such as `R 2.50`. |
| Numeric radius | Entering a positive radius updates the same preview; the center/support remain fixed. |
| Commit | A second click, or Enter for an exact radius, commits one circle creation action. |
| Cancel | Escape cancels the pending circle without creating geometry or an undo entry. Keep the face support available for retry. |

Calculate radius as Euclidean distance in an orthonormal support frame: `r = hypot(u - center_u, v - center_v)`. The center and radius must not depend on camera zoom or screen aspect ratio. Circle preview remains circular in its plane even when it appears elliptical in perspective or an oblique orthographic view.

Reject zero, non-finite, and unsupported radii; keep the gesture active so the user can correct the radius. The current evaluator's width limits imply a radius range of 0.005–50 world units before object scaling; validate these existing limits consistently rather than introducing contradictory UI ranges.

Inspector edits for an authored circle should expose Radius consistently. The existing serialized `width` stores diameter and can remain so for compatible files: convert `radius = width / 2` and `width = 2 * radius` at the UI boundary. Do not reinterpret old `.osk` values or halve existing circles. Semicircle diameter-endpoint creation is a separate gesture and remains unchanged unless explicitly revised.

## Snapping and plane discipline

Proposed acquisition order is explicit point references, boundary edges, local-axis inference, then local grid. Resolve competing candidates by screen distance and visible depth with clear target feedback. A snapped endpoint must be on the captured support plane within the documented geometry tolerance; do not move it off-plane to reach an unrelated reference.

Snap in support-local U/V coordinates. Reuse the existing snapping toggle; do not add a second unrelated enabled state for face sketches. Keep snapping disabled if the user explicitly turns it off, including after Save/Load.

When the camera ray is parallel to the support plane, show an actionable status and offer Align view. Do not fall back silently to XY. Keep the first center/anchor unchanged if no valid cursor-plane intersection can be found.

## Face plane versus modifying its topology

Drawing coplanar geometry and splitting the supporting face are different operations. A floating duplicate disc is not an imprinted face region and must not be presented as one.

For the initial bounded implementation, retain the circle as an editable, face-supported profile. Clearly identify it as a profile. Do not imply that it has already split the parent face or cut a hole. To use the resulting region for face-level push/pull, the application must implement and validate an explicit imprint/split or a profile-driven solid operation with target/support references.

Proposed initial boundary policy: require the center to lie on the selected trimmed face, but allow the radius preview to extend across its boundary. Show the complete circle; do not silently clip it into a non-circle. A future imprint operation resolves intersections and resulting regions explicitly. This policy needs visual review on faces with holes and profiles crossing boundaries before those cases are enabled.

Creating a supported profile is one undoable document operation. Selecting a support or displaying guides does not change geometry. Invalid split/Boolean results must preserve the previously valid body.

## Parametric support and Save/Load

A sketch attached to a face needs the parent object/feature reference, a semantic face role, its local frame, local circle center, and radius. Parent transforms should carry the sketch with the parent; an upstream edit that removes or invalidates the support must report a dependency error.

Current `.osk` component selection slots are explicitly selection state, not persistent feature references. Do not reuse `selected_element` or a GPU triangle index as a saved sketch dependency. Establish source-feature roles for supported generated faces before shipping attached profiles. Extend the project schema deliberately and migrate existing files while retaining their current geometry and units.

A fixed world-plane sketch and an attached face sketch must remain distinguishable after reopening. Save/Load cannot silently detach the profile, replace the support with XY, or recompute the plane from whichever triangle happens to be first.

## Current code gaps and minimum sequence

Verified by repository inspection:

- `Body::from_points(Circle, ...)` already computes twice the center-to-cursor distance as the stored diameter. The basic mouse gesture exists.
- Circle inspector and preview currently display width/diameter, rather than the requested radius.
- `creation_point` uses `Camera::ground`, so all points are projected onto XY regardless of selected face.
- Creation does not capture a support face/frame, and `Body::from_points` is XY-specific.
- Point markers are drawn in Point selection mode, not as contextual references for a selected support face.
- Evaluated geometry retains face/edge/vertex grouping, but not a reusable planar support frame or stable attached-sketch dependency.

Implement in this order:

1. Reuse the existing circle gesture and correct the Radius inspector/readout through unit conversion, with a compatibility check for old files.
2. Carry the selected planar face into the creation context; expose its references and derive its local frame from kernel geometry.
3. Replace the hard-coded ground intersection with support-plane intersection and local-coordinate creation, shared by compatible shape tools.
4. Add contextual snapping and support feedback, with cancellation and edge-on handling.
5. Add stable parent/support references and `.osk` migration before shipping attached-profile persistence.
6. Implement imprint/split or targeted profile operations separately; do not claim that plane placement alone enables a pocket or borer.

## Acceptance cases

- A circle created with a 3-unit center-to-pointer distance shows Radius 3 and evaluates to diameter 6. Changing Radius to 4 gives diameter 8 automatically.
- Existing saved circles retain their size when the UI changes from Diameter to Radius.
- Select a raised, rotated planar face, activate Circle, and observe only its relevant points/references. Both clicks and the entire preview remain on that face's plane.
- Hover across another face during radius definition: the support remains fixed.
- Snap to a selected face vertex/midpoint/analytic center with visible target feedback; no unintended off-plane displacement occurs.
- Edge-on or curved support produces an explicit explanation and preserves the current scene.
- Escape cancels the circle preview without losing the support or adding history; commit and undo/redo operate as one action.
- Reopen an attached profile, edit the parent's transform, and confirm the profile follows; invalidate its support and receive a dependency error rather than silent reassignment.
- Coplanar profile placement is visibly distinct from an imprinted region or solid cut.
