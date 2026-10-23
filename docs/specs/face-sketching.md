# Face-first sketching and circle creation

Status: fixed placement on selected/visible planar faces and geometry snapping implemented; attached dependencies, expanded reference overlays, and Radius presentation remain pending.
Date: 2026-10-23.
Related: [workflow research](../research/face-sketching-workflows.md), [soft selection](soft-selection.md), [product scope](../product-scope.md).

## Confirmed behavior

- Circle creation is driven on the canvas: pick a center point, move the mouse to set radius, then click to finish. Radius, not diameter, is the user-facing creation measurement.
- A selected supporting face pins the plane. Without one, the first click can capture a visible planar face (updated drawing behavior); subsequent hovering over other faces must not silently change the captured support.
- Selecting that face exposes its relevant points and boundary references so the user can position the new shape accurately.
- Snapping starts enabled. The canvas preview and numeric controls must describe the same geometry.

The precise shortcut choices, point display policy, face-boundary handling, and support persistence described below are proposed defaults. They are not claims that Plasticity or SketchUp use exactly this workflow.

## Face selection establishes context

1. In Face mode, the user clicks a visible supporting face.
2. Highlight that face and expose its boundary vertices, boundary edges, and applicable snap references. Keep these visible when a creation tool is activated; entering Circle must not discard the selected support.
3. Display the support explicitly, for example `Sketch plane: Object 3 / selected face`, with a local-axis indicator.
4. The selected face establishes a plane origin, orthonormal U/V axes, and normal N. These are geometric plane axes, not necessarily the raw NURBS parameter directions, which need not represent equal physical distances.
5. Starting a sketch captures this support. Orbiting or hovering over other faces does not change it. Offer an explicit Align view action; selecting a face alone need not move the camera.
6. A world-plane sketch remains available through the opt-in base-XY preference or when no visible support is acquired. Indicate the active plane explicitly; never present a ground-plane result as though it were attached to a hovered face.

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

## Current implementation and remaining work

- [x] Existing mouse circle gesture uses center plus radius internally; authored `width` remains diameter.
- [x] Shared selected/visible-face plane capture and plane-local creation for rectangle, circle, line, and semicircle.
- [x] Elevated/vertical planar faces, orthonormal support frames, fixed-plane preview/cancellation, and explicit edge-on/nonplanar errors.
- [x] Depth-aware endpoint/midpoint/edge and projected-limit snapping plus plane-local grid/anchor alignment, with target labels and guides.
- [x] Exact fixed pose persists through position/rotation in `.osk` version 4; preferences and cutting intent also persist.
- [x] New profiles are independent by default. Optional coplanar cuts remain explicit and do not imply arbitrary solid-face imprint.
- [ ] Radius inspector/readout conversion and numeric radius entry during creation.
- [ ] Dedicated selected-support reference-point/center overlay, explicit plane identity, and Align view.
- [ ] Stable parent/support dependencies that carry the sketch when its parent changes. Current face placement remains independent world-space geometry.
- [ ] Arbitrary face imprint, Boolean cutting, and exact curved-surface sketching.

The attachment and contextual-display sections above describe the remaining intended behavior, not shipped dependencies. With no explicit selected face, initial visible-face acquisition avoids accidental projection to an extruded object's base. Preferences expose base-XY projection and automatic coplanar cutting separately; both are off by default. Snap can be disabled without changing support-face placement or an already captured plane.
