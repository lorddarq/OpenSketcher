# Face chamfer and bevel modifiers

Status: implementation preparation, 2026-10-23. No modifier runtime code is implemented yet.

## Confirmed behavior

- Select a face, then apply **Chamfer** or **Bevel** to it.
- Bevel style is user-selectable per modifier: rounded or flat/segmented. The choice remains editable and persists in `.osk`.
- Retain an editable modifier rather than replacing the source geometry with a baked mesh.
- Numeric changes update automatically using the existing property-edit workflow.
- Removing a modifier recomputes the result without that operation and preserves the source geometry.
- Save parameters and source-face references in `.osk`; retain editing/removal after reopening.
- Adding, editing, and removing a modifier participate in undo/redo.

## Proposed first semantics

These defaults prepare implementation; they are not additional confirmed requirements.

- A selected solid face identifies the boundary edges to treat. Keep the face center and rebuild its boundary, the neighboring faces, and the connecting corner patches together. This is edge treatment around a selected face, not uniform scaling or an inset floating above the face.
- **Chamfer:** an equal-distance cutback measured along each adjoining face, producing a flat transition. On perpendicular faces this produces a 45-degree cut; equal-distance does not imply a fixed angle on arbitrary dihedral angles.
- **Bevel:** expose the confirmed rounded versus flat/segmented style choice. Proposed controls are radius for the rounded transition and width/profile plus segment count for the flat/segmented treatment; their exact parameterization still needs validation. Do not conflate radius with chamfer distance or silently switch styles. Describe any approximation explicitly.
- Only face selection can add either modifier initially. Existing modifiers remain editable in the owning body's properties even after selection changes.
- Start the geometry proof with convex rectangular extrusions, first the top cap and then side faces. Do not advertise unrestricted face support until curved, concave, holed, and component-deformed cases pass their own validation.
- Unsupported cases remain selectable but explain why the operation is unavailable. Never silently treat all edges of the body, skip failed edges, or substitute a different face.
- Reject nonpositive, nonfinite, excessive, or geometry-collapsing values. Determine admissibility from the complete resulting boundary, not a single width/depth clamp. Retain the last valid geometry and show the invalid input as an error.
- Keep multiple modifiers in authored order. If editing/removing an earlier modifier invalidates a later target, reject that change with a useful explanation rather than silently deleting or retargeting the later operation. Suppression/reordering UI is outside the first pass.

## Verified integration points

- `src/main.rs`: Face mode already provides the body ID and selected face slot; the scrollable Model & properties panel owns automatic numeric editing. Add modifier controls there rather than a new panel.
- `src/model.rs`: `Body` contains the editable source parameters and component transforms. `Model::store` evaluates a candidate scene before committing and already groups live edits for undo/redo. Reuse that transaction/history path.
- `src/model.rs`: both single-body and scene evaluation route through `build`; coplanar regions are resolved before building the shell. Keep modifiers in the shared evaluator so rendering, picking, saving, and every export see the same result.
- `src/components.rs`: `Topology`, boundary welding, and `boundary_shell` provide connectivity/reconstruction checks. These checks do not constitute general self-intersection detection. Reuse them where applicable; do not duplicate mesh welding or build separate export geometry.
- `src/project.rs`: schema/modeling version 4 validates all bodies and regenerates geometry. A modifier-bearing schema must deliberately advance the format/modeling version, default legacy bodies to no modifiers, bound modifier input, and reject unsupported versions.
- `src/export.rs`: exporters consume `evaluate_scene`, so modifier output should flow through it without format-specific bevel/chamfer code.

The pinned `truck-modeling` 0.6 builder provides sweeps, planar attachment, homotopies, and transforms, but no ready-made 3D chamfer/fillet API. Its bundled bottle example explicitly identifies fillets as unavailable. Current upstream drafting work includes 2D fillet helpers; this does not supply a solid edge-treatment operation and is not a reason to upgrade dependencies for this feature.

Sources inspected: [Truck modeling API](https://docs.rs/truck-modeling/0.6.0/truck_modeling/builder/index.html), [upstream 2D drafting implementation](https://github.com/ricosjp/truck/blob/master/truck-drafting/src/geom_impls.rs). The locked crate source is authoritative for the initial implementation.

## Geometry and persistence plan

1. Prove a closed-box chamfer and both bevel styles without changing the UI or project schema. Construct transition faces and shared corner patches with Truck builders; verify solid validity and size limits before selecting the general algorithm. Prefer rebuilding supported analytic source geometry over treating every viewport triangle as a modeling edge.
2. Establish source-face roles for supported base primitives (for example, top/bottom caps and identified profile sides). Resolve roles against regenerated geometry. A raw `selected_element`, GPU triangle index, or transient Truck entity ID must not become the saved dependency. Use explicit failure for unsupported/ambiguous roles.
3. Add the smallest typed, ordered modifier data to `Body`: modifier kind, bevel style, source reference, and the parameters actually implemented. Reuse serialization/validation conventions. No generic plugin framework or new kernel dependency is needed for preparation.
4. Define evaluation order against existing `component_edits`. The first implementation may reject mixing modifiers and component edits until references across topology changes are reliable; do not guess that replaying old component slots is safe. Source pose and dimension edits must still regenerate supported modifiers.
5. Integrate shared evaluation and format migration, then face-selected Add actions and editable/removable rows. Finish each discrete add/remove action separately; group a continuous numeric edit as one undo step. Failed changes must preserve scene, history, and existing files.
6. Validate the complete workflow in the native viewport, `.osk` save/reopen, and exported geometry. Extend supported face classes only after the first case holds.

Distance/radius should initially be in source-local units, consistent with current profile dimensions; nonuniform object scale then scales the result along local axes. Label this clearly instead of promising a constant world-space radius. The source-vs-world metric is a proposed implementation choice to confirm before UI delivery.

## Acceptance checklist

- [ ] A selected supported face produces only the intended perimeter treatment; neighboring faces remain joined.
- [ ] Box top and side tests prove chamfer distance, both bevel styles, radius/profile, corner joins, consistent orientation, and a closed manifold solid.
- [ ] Switching bevel style updates automatically, survives save/reopen, and supports undo/redo without discarding the modifier.
- [ ] At least one transformed/nonuniformly scaled case verifies the documented parameter metric.
- [ ] Oversized values and unsupported geometry fail without changing the committed model or undo stack.
- [ ] Add → edit → remove restores the unmodified source; undo/redo reproduces each state.
- [ ] Editing source dimensions reevaluates the same semantic target or fails explicitly when it disappears.
- [ ] Multiple operations have deterministic order; upstream removal never silently retargets a dependent modifier.
- [ ] Legacy version-4 projects retain identical geometry; new modifier projects round-trip with editable parameters.
- [ ] Rendering, picking, and STL/GLB/STEP/DXF exports agree on the evaluated result.
- [ ] Native review verifies face selection, automatic edits, removal, save/reopen, errors, and undo/redo.

Next implementation step: the boxed-solid geometry proof and one runnable regression check. General curved-surface blends and a full feature-history framework remain separate work.
