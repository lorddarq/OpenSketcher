# Changelog

## Unreleased

- Add signed cap extrusion: negative travel subtracts material, including an opening after passing the far wall. Preserve exact shared boundaries for contained planar exits, retract consecutive sweeps on reversal, and retain outward-only history topology. Persist authored overrun in `.osk` v13.

- Fix edge/face/point gizmo edits on bored and Boolean solids by replaying new component edits on the evaluated boundary. Preserve legacy replay, atomic multiselection/history and `.osk` v12 persistence.

- Add evaluated planar face extrusion after chamfer/bevel: keep the treatment in place, extend the trimmed cap, preserve logical face selection, undo/redo and `.osk` v11 persistence.

- Add attached-target Through all and parallel-planar Up to next/Up to face sweep extents with explicit dependencies, full-region termination checks, direction, and `.osk` version 10 persistence.

- Identify rejected operations and source/target references in errors; mark uncommitted property values, retain accepted geometry/save data, and allow Escape to discard rejected drafts after computation.

- Add edge-local box treatments, semantic source-edge references, and atomic multi-face/multi-edge application across bodies; persist edge targets in `.osk` version 9.

- Export nonuniformly scaled revolved surfaces as exact rational NURBS and report STEP formatting errors without panicking.

- Add two-distance and distance/angle chamfers with direction reversal, including analytic cylindrical rims; preserve exact rim snapping and persist definitions in `.osk` version 8.

- Avoid expensive 3D intersection solving while editing contained pipe bores: construct blind-pocket floors/walls directly as well as through-holes; use analytic circular containment for thin walls.

- Added ordered Union/Subtract/Intersect tools for closed volumes and coplanar face regions, editable operand order, retained tools, undo/redo, and `.osk` version 7 persistence. General curved sheet trimming remains pending.

- Move geometry edits/previews/history and project reevaluation to one background evaluator; coalesce pending values, preserve edit/undo boundaries, cap geometry work at two threads, and discard cancelled/stale results.
- Avoid tessellating invisible cutter tools; construct contained through profiles with exact inner-wire extrusion.
- Add attached-profile New body / Borer only / Generator + borer modes, signed travel, pockets/notches/through cuts, and separate emerging bodies; `.osk` version 6 preserves target/source-face references.
- Add cylindrical-wall radius offset with fixed axis and equal radial-scale validation.
- Export retained analytic/NURBS CAD boundaries to STEP; keep faceted fallback for mesh-only edits and avoid the upstream intersection-entity numbering defect.


- Add a sourced geometric-operator specification and prioritized CAD/direct-modeling gaps.
- Add signed numeric Offset face for primitive planar supports, preserving the opposite plane, circular geometry, and authored edge treatments.

- Reuse the last validated scene for viewport updates and selection changes, avoiding duplicate circle tessellation on automatic value updates.

- Analytic circle-center/rim snapping on elevated, transformed, and beveled cylinder geometry.

- Shift-click multiple selection in all modes, group Move/Rotate and deletion, and atomic group undo/redo.
- Saved selection sets and union component transformations; corrected Shift+Command+Z redo precedence.

- Editable/removable Chamfer and Bevel modifiers, with rounded/flat styles, automatic updates, and undo/redo.
- Box-face and cylinder/semicircular rim treatments; cylinder profiles retain analytic circular and rounded surfaces.
- Logical base/bevel surfaces hide render subdivisions and internal kernel patch seams.
- `.osk` version 5 preserves modifiers and semantic source-face roles; older projects migrate without geometry changes.

## v0.1.0 — 2026-10-23

Initial development release of OpenSketcher.

- Rust desktop editor with a 3D viewport, isometric/top/side views, and free orbit.
- Canvas creation of rectangles, circles, lines, and semicircles; extrusion and numeric transforms.
- Volume, face, edge, and point selection; constrained Move/Rotate gizmos and undo/redo.
- Geometry/grid snapping, drawing on selected or visible faces, and opt-in coplanar profile cutting.
- Editable `.osk` projects with validated loading, atomic saves, and unsaved-change prompts.
- STL, GLB, faceted STEP, and DXF export, with explicit millimeter units.
- Apache-2.0 licensing, bundled dependency notices, and macOS/Windows/Linux build workflows.
- Updated feature checklist, including planned editable/removable face-based chamfer and bevel modifiers.

Development packages are unsigned. Windows/Linux runtime acceptance and external export-application testing remain pending. STEP geometry is faceted; chamfer, bevel, soft node selection, and solid boring are not implemented.

[Source at v0.1.0](https://github.com/lorddarq/OpenSketcher/tree/v0.1.0)
