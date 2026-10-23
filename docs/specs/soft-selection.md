# Soft selection and face deformation

Status: behavioral specification; not implemented.
Date: 2026-10-23.
Related: [product scope](../product-scope.md#face-types-and-deformation-editing), [kernel research](../research/parametric-modeling-kernels.md).

## 1. Purpose and decision boundary

Allow a user to reshape an editable face by moving selected nodes while progressively influencing neighboring nodes. The supplied SketchUp soft-selection image is a behavior and feedback reference: strongly affected nodes appear warm, weakly affected nodes appear cool, and the deformation spreads around the selection instead of moving one isolated point.

User-confirmed requirements:

- Whole-face movement preserves the existing deformation and remains parametrically editable.
- Node editing is a separate mode with smooth, local influence around selected nodes.
- Show influence as a heatmap and provide an adjustable influence radius.
- Keep neighboring influence progressively weaker rather than creating an isolated spike.
- Preserve the editable result and its inputs through project Save/Load.

The exact selection shortcuts, distance metric, falloff formula, and persistence representation below are proposed initial defaults. They make the feature implementable and testable; they are not claims about SketchUp internals or verified Truck capabilities. The supplied reference does not imply copying its toolbar, artwork, or every tool it shows.

## 2. Terminology

| Term | Meaning |
| --- | --- |
| Editable node | An authored NURBS control point or deformation-cage node. It is not a tessellated viewport vertex. |
| Explicit selection | Nodes directly selected by the user; each receives influence 1. |
| Soft-selected node | An unselected, eligible node with influence strictly between 0 and 1. |
| Influence domain | The active face's editable control structure and its permitted connectivity. |
| Influence | A scalar from 0 to 1 multiplying the requested node displacement. |
| Radius | Maximum influence distance in project length units. |
| Rest state | The evaluated, undeformed-by-this-operation node positions immediately before a transform starts, or its upstream feature state during reevaluation. |

A NURBS control point generally does not lie on the visible surface. Draw the control structure explicitly so selecting a node does not suggest that it is a surface sample. NURBS rational weights are separate from soft-selection influence.

## 3. Modes and eligibility

### Move face

Translate the entire selected face as a shape-preserving operation. Move the surface/control representation and trims consistently. Relative control-point positions, NURBS weights, degree, and knots remain unchanged. Do not bake the face into triangles or silently move the entire body.

No soft-selection heatmap or radius is active in this mode. Whole-face movement is not implemented by applying a falloff to all nodes.

### Edit nodes

Enter this mode explicitly on an eligible face. Display editable nodes and control edges; ordinary faces without an editable control structure cannot enter it until an explicit modeling operation creates one. Do not silently tessellate a planar face to invent editable nodes.

The initial tool translates selected nodes and their soft neighborhood. Weighted rotation, scaling, sculpt brushes, adaptive subdivision, control-point insertion, and curvature constraints are outside this first implementation.

### Shared and constrained geometry

Influence remains within the active face's editable domain. Nearby independent bodies and disconnected cages are unaffected. If a cage node is shared by several patches, all patches bound to that same node must update consistently; exclude or reject the edit if that ownership cannot be handled safely.

Locked nodes are not selectable as transform sources and do not move through influence. They remain connectivity paths in the initial metric; a lock fixes a position rather than cutting the cage. Other geometric constraints must be respected or cause an explicit rejection; this tool must not silently disable them.

Faces attached to a solid need valid common boundaries after deformation. Rebuild adjacent geometry where supported. If the representation cannot maintain a valid solid, reject the edit and explain why. Do not accept an attractive preview that tears a seam.

## 4. Selection behavior

- Clicking an eligible visible node replaces the explicit selection with that node.
- Shift-click toggles a node in the explicit selection.
- Clicking empty viewport space clears selection; Escape during a transform cancels the transform first.
- Clicking selects the nearest visible node within the picking tolerance. The tolerance is in screen pixels and independent of zoom. Resolve overlapping projected nodes by visible depth, not arbitrary storage order.
- The initial selection is depth-aware. Selecting through hidden geometry and box/lasso selection are deferred; the soft-selection reference does not require them in the first implementation.
- Keep explicit and soft-selected counts separate. Soft selection never promotes a node into the explicit selection.
- No explicit selection means no influence field and no active transform gizmo.

Multiple selected nodes form a source set. Influence is based on distance to the nearest selected node, not summed across selected nodes. Overlap must not move a node farther than the requested transform. Selected nodes remain at influence 1 even when surrounded by other selected nodes.

## 5. Influence radius and distance

Provide a numeric Radius field using the project's displayed length units. Reject negative, non-finite, and unrepresentable values without changing the previous valid setting. Radius 0 is valid and gives hard selection: only explicitly selected nodes move. A Soft selection toggle also allows hard selection while retaining the last radius.

For a face without a previously used radius, initialize it to twice the median positive control-edge length in world units. If no positive edge length exists, initialize to 0. This is a proposed convenience default, not a geometry tolerance. Show the actual value so it is predictable and editable.

Use shortest-path distance over authored control-net/cage edges as the initial metric. Each edge's length is measured from rest-state positions in world units, including the current object transform. This is a connected-neighborhood approximation, not an exact geodesic distance on a NURBS surface. Nonuniform node spacing can make the influence less circular than a surface-distance method; validate this visually before release.

Disconnected components are unreachable unless they contain their own selected source. Never bridge folded surfaces simply because nodes are close in 3D space. For periodic NURBS faces, include genuine wraparound connectivity; do not infer it from overlapping coordinates. Preserve an explicit binding between displayed seam aliases and their authoritative nodes.

The radius remains in world/project units while the camera zooms. Zooming changes the visual size of the affected region, not which nodes are affected.

## 6. Falloff and displacement contract

For an eligible unselected node with distance `d` to the nearest selected source and radius `r > 0`:

```text
t = clamp(d / r, 0, 1)
w = 1 - 3*t*t + 2*t*t*t
```

Explicitly selected nodes have `w = 1`; locked and unreachable nodes have `w = 0`. Nodes at or beyond the radius have `w = 0`. Radius 0 avoids division and assigns influence only to explicit selection. Initial implementation uses this single smooth falloff; a curve editor and additional falloff presets are deferred.

| Distance from source | Influence | Movement for a 10 mm drag |
| --- | --- | --- |
| 0 | 1 | 10 mm |
| 0.25 × radius | 0.84375 | 8.4375 mm |
| 0.5 × radius | 0.5 | 5 mm |
| 0.75 × radius | 0.15625 | 1.5625 mm |
| radius or farther | 0 | 0 mm |

For a requested world-space translation `delta`, evaluate each node as:

```text
preview_position = rest_position + influence * delta
```

Do not average the absolute positions of neighboring nodes: that would relax or shrink the existing shape even without the intended movement. The weighted quantity is displacement. Smooth falloff reduces abrupt local displacement; it does not guarantee curvature continuity, preserve volume, or prevent self-intersection.

Capture rest positions, connectivity, source selection, radius, and influences when a drag begins. Every preview uses those same inputs and the current total delta, never the previous frame's deformed positions. Moving out and back to zero must restore the starting geometry without accumulated drift. Radius and selection controls are unavailable during an active drag; cancel or finish first.

## 7. Transform interaction

Use a translation gizmo at the world-space centroid of the explicit selection. A single selection places it at that node. Axis handles constrain translation to world X, Y, or Z; dragging a plane handle constrains it to that world plane. For an initial free-move handle, use the camera plane captured when the drag begins. All influenced nodes share the same requested delta scaled by their own influence.

Numeric entry specifies displacement, not an absolute destination. For an axis handle, accept a signed distance in project units. For free or plane translation, expose the relevant displacement components. Snapping constrains the selected handle's total delta before falloff is applied; do not independently snap every affected node, which would destroy the gradient. Explicit axis locking follows the modeling tool's shared axis-lock convention.

Camera orbit, pan, zoom, and view presets may be used while idle without modifying geometry or influence. During a node transform, keep camera controls and preset switching unavailable to avoid changing the drag interpretation. Clearly retain the active transform until commit or cancel.

On pointer release, validate and commit a valid nonzero transform as one action. Escape, pointer cancellation, or losing window focus cancels and restores the pre-drag geometry. A zero-displacement action creates no document change and no undo entry. Tool/mode switches during a drag cancel the preview before switching.

## 8. Heatmap and viewport feedback

- Render influence at editable node markers, not by replacing assigned face textures or materials.
- Use a continuous blue → cyan → green → yellow → red scale, with blue at 0 and red at 1. Explicit nodes have a distinct outline/shape so they remain distinguishable from strongly influenced neighbors.
- Display unaffected nodes in a muted style, with a legend identifying 0–100% influence. Do not imply that blue necessarily means positive influence.
- Show an approximate radius guide around sources; for multiple selections, show the affected neighborhood around the group rather than a misleading single sphere around its centroid. The calculated heatmap is authoritative where graph distance differs from the guide.
- Show Radius, explicit count, and soft-selected count. On hover or keyboard focus, expose the node's numeric influence percentage.
- Retain a clear translation gizmo and active axis/plane indicator. Heatmap colors and axis colors must be distinguishable by shape, labels, and context.
- Use depth-tested node markers by default; nodes behind opaque geometry must not appear selectable through it.
- Provide accessible names, numeric fields, and text status. Color alone must not convey selected state, influence, or errors. Labels/tooltips accompany any Tabler icon controls.

Heatmap influence remains frozen during a drag, matching the transform's captured weights. After commit, recompute the idle field from the new evaluated geometry for a possible next operation. The saved operation keeps its own original inputs; this idle refresh must not change the just-committed deformation.

## 9. State transitions and failures

| State/event | Required result |
| --- | --- |
| Enter node mode | Display eligible control structure; retain no stale selection from another face. |
| Change selection or idle radius | Recompute heatmap and counts; geometry unchanged. |
| Begin transform | Capture rest state and influence; show displacement and gizmo feedback. |
| Valid preview | Display deformed geometry and unchanged captured heatmap. |
| Invalid preview | Mark preview invalid and explain why; keep committed geometry untouched. |
| Release invalid preview | Restore pre-drag geometry, keep selection, and report the failure; no history entry. |
| Cancel/focus loss | Restore geometry and selection from before the drag; no dirty flag or history entry. |
| Commit | Update affected geometry and dependent features atomically; one undo entry. |
| Leave node mode | Hide node overlays and radius guide; keep committed deformation. |

Validation includes finite positions, eligible node references, valid cage bindings, supported NURBS definitions, nondegenerate geometry, and required trim/solid consistency. Reject edits that violate supported topology or introduce detected self-intersection. Be explicit about what the selected kernel can actually validate; passing control-point checks alone is not proof of a valid solid.

If a downstream feature cannot regenerate, the initial policy rejects the whole proposed edit and preserves the prior valid document. Do not leave part of the feature chain updated or silently drop failed features.

## 10. Undo, parameters, and project persistence

Each committed translation is one editable deformation feature, using stable face/domain and node identifiers. Store:

- Upstream input reference and explicit source node IDs.
- Soft-selection enabled state, radius, falloff identifier/version, and distance-metric identifier/version.
- Requested translation vector and its coordinate-frame semantics.
- Any constraint/binding inputs needed to reproduce the operation.

On reevaluation, obtain rest positions from the upstream feature result, recompute influence using that feature's saved settings, and apply its saved translation. Changing an upstream shape can consequently change the affected neighborhood, but may not silently rebind missing source nodes by index or spatial proximity. Missing references produce an explicit feature error.

Preserve NURBS control points, rational weights, degrees, knots, periodic connectivity, trims, cage definitions, and bindings in the native project model. Mesh caches, heatmap colors, current hover, gizmo state, and active drag are derived or transient. Never save an in-progress preview as committed geometry; finish or cancel before saving.

Undo restores geometry and feature state before the action; redo reproduces its saved result without reusing the current idle radius. Editing an existing deformation's radius or translation reevaluates that feature and its dependents atomically, creating one undoable parameter edit. Idle radius/selection changes alone do not mark the project dirty. Session undo history need not survive reopening.

## 11. Acceptance cases

| ID | Scenario | Observable result |
| --- | --- | --- |
| SS-01 | Move a deformed face as a whole | Relative control positions and surface definition remain unchanged; no soft falloff applied. |
| SS-02 | Translate one node by 10 mm | Selected node moves 10 mm; known half-radius neighbor moves 5 mm; boundary/outside nodes remain fixed. |
| SS-03 | Disable soft selection or set radius 0 | Only explicit sources move; retained radius restores when reenabled. |
| SS-04 | Select two nearby nodes | Both move fully; overlapping influence never exceeds 1; counts do not duplicate nodes. |
| SS-05 | Change camera zoom or preset while idle | Same nodes retain the same weights; radius value is unchanged. |
| SS-06 | Folded cage near an unrelated surface | No influence jumps across missing connectivity or into the unrelated surface. |
| SS-07 | Drag repeatedly out and back to zero | Original geometry restored within documented numerical tolerance; no cumulative drift. |
| SS-08 | Cancel, lose focus, or release invalid edit | Original document remains intact; no undo entry or new dirty state. |
| SS-09 | Lock a neighboring node | Node remains fixed; surrounding eligible nodes still follow the field. |
| SS-10 | Use a world-axis handle and snapping | All displacement remains parallel to the requested axis; gradient is preserved. |
| SS-11 | Edit a NURBS control net | Positions change as intended; rational weights, degree, and knots remain unchanged. |
| SS-12 | Edit near a shared seam or periodic boundary | Shared definitions remain consistent, or the edit is explicitly rejected without damage. |
| SS-13 | Undo/redo after changing idle radius | Original action replays using its own saved radius and sources. |
| SS-14 | Save, reopen, and edit a deformation parameter | Same editable feature and references restored; changed radius/delta recomputes geometry. |
| SS-15 | Upstream edit removes a selected node | Explicit feature error; no automatic index-based replacement or partially updated document. |
| SS-16 | Inspect heatmap without relying on color | Selected state, radius, counts, and focused-node influence remain understandable. |

Leave one small runnable numerical regression check for falloff, multi-source overlap, radius-zero behavior, and noncumulative displacement. Add persistence and topology failure checks when those flows exist. Run the visual interaction cases in the native app; numerical success alone does not verify marker visibility, picking, feedback, or deformation quality.

## 12. Delivery sequence and remaining validation

1. Prove Truck control-net access, trim handling, and supported face validation with a bounded editable surface.
2. Implement node picking, explicit selection, numeric radius, and the connected falloff field without changing geometry.
3. Add translation preview, gizmo constraints, commit/cancel, and undo as a complete operation.
4. Preserve feature inputs through Save/Load and parameter reevaluation.
5. Validate shared boundaries, periodic surfaces, and representative nonuniform/folded control structures before claiming support for them.

Validate the proposed graph metric and default radius against real models. If their influence is visibly anisotropic or coarse, revise the documented metric/version rather than silently changing old project evaluation. Smooth surfaces, watertight solids, and responsive interaction are separate validation targets. No general-purpose geometry engine, custom falloff editor, or new dependency is required by this specification alone.
