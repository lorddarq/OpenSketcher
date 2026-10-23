# Face chamfer and bevel modifiers

Status: initial implementation, 2026-10-23.

## Behavior

- Select a solid face and use **Chamfer** or **Bevel** in Model & properties. The editor selects the owning body to expose its modifier list.
- Rectangular solids support all six faces. Circular and semicircular extrusions support top/bottom cap perimeters. In Edge mode a circular/semicircular rim segment selects the entire corresponding perimeter.
- Chamfer cuts back equally along the selected and neighboring source faces, controlled by Distance.
- Bevel style is user-selectable: Rounded uses Radius; Flat / segmented uses a single flat segment, sharing chamfer geometry. Rounded noncircular strips expose 2–16 approximation segments; cylinders regenerate analytic rounded surfaces instead.
- Parameters update automatically. Remove regenerates the source without that modifier. Adding, editing, style switching, and removal use undo/redo. No source geometry is baked or discarded.
- The base surface and bevel band are distinct selectable faces. Tessellation subdivisions and kernel seams within the same authored surface are internal rendering details, not displayed or selectable model faces.
- Source dimensions, position, rotation, and scale remain editable. Distance/radius is measured in local source units; nonuniform scale scales the resulting treatment.
- One modifier per source face, up to six per body. Editing supported source dimensions preserves semantic face targets. Oversized/nonfinite/nonpositive parameters and invalid geometry are rejected without replacing the last valid committed shape.

## Geometry and integration

`src/edge_treatments.rs` resolves selected face/rim output to semantic source-plane roles, validates modifier inputs, and regenerates geometry. No output slot, GPU triangle index, or transient Truck ID is serialized as a modifier target.

Cylinders revolve an authored radial profile: straight edges for walls/chamfers and quarter-circle arcs for rounded rims. Truck creates circular boundaries and revolved surfaces. Revolution patches are merged into logical surfaces, with their internal seams removed from displayed/pickable edges. Rounded cylinder quality follows normal viewport tessellation rather than changing the authored surface into segment faces.

Other convex extrusions are clipped against chamfer planes or tangent-plane approximations for rounded transitions. Shared corners are reconstructed together. Transition facets belonging to the same strip are grouped into one logical surface; only boundaries between authored surfaces are displayed/pickable. Semicircular profiles use 32 arc subdivisions internally. Exact analytic noncircular blends remain pending.

`Body.edge_treatments` (legacy `modifiers` data is accepted) retains inputs. `model::build` evaluates modifiers through the same path used by scene rendering, picking, validation, and exports. Kernel tessellation is shared with unmodified geometry. Boundary welding and Truck shell/solid validation reuse `components.rs`. `Model::store` validates a whole candidate scene before committing, and the existing history groups live property changes.

`.osk` schema/modeling version 5 serializes modifier kind, face role, distance/radius, style, and segment count. Versions 1–4 migrate with no modifiers and retain existing geometry. Older-version declarations reject nonempty modifier data. Invalid edits block saving while the last committed shape remains visible. STL/GLB/STEP/DXF consume the same evaluated result; STEP export is currently faceted even for analytic cylinder sources.

## Explicit limits and pending work

- [ ] Partitioned, holed/concave, or cutting-dependent profiles.
- [x] Regenerate supported source-face offsets after treatment, preserving authored treatment values.
- [ ] Exact surface reconstruction for component edits after treatments; existing component matrices replay after treatments through the faceted deformation path. General dependency mapping across arbitrary topology changes remains pending.
- [ ] Arbitrary curved wall faces, non-rim edge treatment, and exact noncircular rounded blends.
- [ ] Multiple modifiers on the same face; modifier reordering/suppression UI.
- [ ] More permissive size limits and general self-intersection validation beyond the supported convex construction.

Unsupported operations report errors instead of skipping edges, retargeting a face, or replacing a committed shape. Conservative size checks keep distance/radius below half of the applicable profile dimensions and extrusion.

## Verification

The runnable `edge_treatments::tests::face_and_cylinder_rim_modifiers_are_editable_closed_and_persistent` check covers rectangular/circular/semicircular solids, chamfer and both bevel styles, two opposing rim treatments, all six box faces, transformed/scaled output, solid validity, reduced volume, exact circular cap inset, stable logical face counts, failed edits, removal, undo/redo, schema migration, saved parameter round-trips, and all four exports for modified cylinders.

Native macOS review verified face-selected chamfer, direct cylinder rim selection in Edge mode, editable rounded/flat bevel style, automatic radius changes, removal, undo/redo, save/reopen, whole-band selection, hidden patch/subdivision edges, and smooth shading within each logical surface. Modifier creation currently requires one selected face/rim.

See [geometric operators](geometric-operators.md) for the distinction between source-face offset, general face movement, and solid cutting.

## Asymmetric chamfer controls

Chamfer now offers equal distance, two distances, and distance plus angle. The first distance follows the selected face; Reverse direction exchanges its role with the adjacent faces. Cylindrical cap chamfers use radial/axial setbacks and retain an analytic conical band. Both setbacks obey the existing conservative geometry limits. Rounded bevel behavior is unchanged. These authored inputs require `.osk` version 8; legacy fields default to equal distance. See [operator behavior and validation](geometric-operators.md#asymmetric-chamfer-behavior).

## Selected source edges and multiple input

Selecting one original box edge now treats that edge alone. The operation retains two source-plane roles in `.osk` version 9. Supported Shift-selected faces/edges on one or many bodies apply together and undo together; circular rim patches deduplicate. Face selection still treats its perimeter. Already treated transition edges, concave/Boolean-result edges, and optional tangent-chain propagation remain unsupported. Numeric editing/removal is available by selecting the owner body in Model. See the [operator checklist](geometric-operators.md#edge-local-treatment-and-multi-selection).
