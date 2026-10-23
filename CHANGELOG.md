# Changelog

## Unreleased

- Accelerate point visibility, picking and snapping with a shared triangle index built on the geometry worker. Preserve occlusion/silhouette and coincident selection rules; reuse the index across camera/selection changes and clear it on New.

- Reconstruct remaining cap material for proven straight-prism through-cuts, including imprinted half-cylinder regions sharing outer walls. Preserve analytic curves, tangent wall grouping, four exports and legacy replay through `.osk` v30 sweep intent.

- Support analytic Imprint profiles sharing circular/elliptical support rims. Retain existing rim intervals once, split neighboring edges at interior divider endpoints and expose independent editable regions on cylinder/semicylinder caps.

- Resolve coincident circular/elliptical intervals in analytic planar face Booleans. Prove shared conic supports from homogeneous curve coefficients, split overlap endpoints and retain one active interval across different arc starts, partial trims and affine placement.

- Retain analytic trim curves for coplanar face Union/Subtract/Intersect through the shared Imprint intersection/arrangement solver. Preserve holes, disconnected regions, ordered picks, opposite normals, snapping references and legacy evaluation via `.osk` v29.

- Use exact cylinder support normals across arc patches and continued extrusions, eliminating shading seams left by uneven tessellation.

- Retain analytic curves/surfaces for complete planar-cap and rim translations on two-section solids, including cylinders, semicylinders, pipes and concave Boolean caps. Keep the opposite cap fixed, preserve oblique cylinder continuity and version exact section intent in `.osk` v28.

- Treat connected cylinder patches as one editable wall and circular rims as one edge each. Exclude artificial longitudinal joins from display, picking, transform authoring and snapping; expand wall/rim edits across members while preserving legacy guards and whole-boundary analytic transforms.

- Preserve analytic B-spline/NURBS trims in closed-profile Imprint crossings, including circular caps, holes, transformed inputs and disconnected regions. Reuse shared boundary splitting and independent-region editing; add versioned `.osk` v27 crossing intent while preserving legacy history behavior.

- Add finite straight-line Imprint for planar faces, including analytic circular trims, holes and finite concave-arm cuts. Retain shared neighboring surface trims, independent region editing, live inputs, Keep tools, undo, four exports and `.osk` v26 persistence.

- Keep extrusion numeric drags active when live geometry changes rebuild the inspector; stabilize scalar and face-distance control identities.

- Extend planar Imprint to holed and disconnected profile tools with whole-profile references, exact nested analytic loops, independent islands, reconciled linear crossings and closed ring-cut exits. Retain live history and four export paths in `.osk` v25.

- Add explicit planar Imprint with exact contained curved boundaries, independent regions, existing-hole preservation and reconciled straight crossings. Group new swept walls by tangent continuity and reuse direct blind/through-cut boundaries on supported convex planar bodies; persist intent in `.osk` v24 while retaining legacy sweep behavior.

- Add live planar Replace Face and construction-plane references, exact convex-planar/circular-prism cap reconstruction, mixed replacement/Boolean chains and `.osk` v23 persistence. Fix unequal-depth convex planar Intersect at coincident supports through shared exact clipping.

- Add ordered Delete Face Open/Cap/convex-planar Heal and Cap openings, preserving exposed sheet edges and analytic cap wires, with multi-body transactions, undo, solid/sheet export validation and `.osk` v22 persistence.

- Extend Up to next to angled planar stops, retaining finite profile coverage, coplanar face groups, nearer-surface obstruction checks, exact end caps and existing history/persistence/export paths.

- Support angled planar Up to face termination with exact projected caps and contained blind pockets.

- Add explicit multiple-body Through all scope, retained analytic depth bounds, atomic scoped cuts, preserved unchecked bodies and `.osk` v21 persistence. Reuse exact contained-hole construction for secondary targets and preserve circular entry/exit snapping metadata.

- Add independently placed construction planes with orientation presets, numeric/gizmo placement, explicit drawing activation, reference-centered snapping, undo and `.osk` v20 persistence. Exclude reference geometry from exports and Boolean tools.

- Reuse exact convex planar subtraction for ordered volume Subtract and attached profile bores as well as face sweeps. Preserve topology for disjoint/touching tools; cover ordered operands, enclosed CAD voids and boundary-crossing generator/borer output.

- Construct convex planar inward cuts directly through angled exits, preserving exact travel, shared BRep edges and independent disconnected face regions. Avoid kernel solving for supported polyhedral cuts; retain fallback for curved/concave geometry.

- Extrude evaluated planar faces after exact Move/Rotate and on Boolean results/planar transition bands, retaining operation order, shared boundaries, undo and `.osk` v19 history. Use exact slab removal for full prismatic inward caps, and handle seam vertices during subsequent planar edits.

- Retrim convex planar faces exactly during Move/Rotate, preserving neighboring support planes, logical connectivity, analytic STEP surfaces and legacy deformation histories. Store exact replay intent in `.osk` v18.

- Add closed source Shell cavities in both directions, editable closure and `.osk` v17 persistence. Preserve enclosed voids in STEP and accept properly oriented cavity boundaries during print-mesh reconciliation.

- Add outward source Shell with a fixed cavity/opening, exact box/cylinder/semicylinder offsets, editable direction, actual outer support references and `.osk` v16 persistence.
- Preserve the central back material when retracting an annular shell opening past its cavity floor; construct shell cap cuts directly instead of solving coincident Booleans.

- Add exact source Draft for boxes, cylinders and semicylinders, with signed angle, neutral cap, conical surfaces/shared parabolic trims, undo/removal, four exports and `.osk` v15 persistence.
- Resolve automatic extents against actual drafted/shelled footprints; preserve shell floors/side thickness during cap retraction and back-wall cuts without coplanar Boolean work.

- Add editable inward Shell/thickness for source boxes, cylinders and semicylinders, with configurable cap opening, exact shared boundaries, logical curved faces, snapping metadata, undo/removal, four export paths and `.osk` v14 persistence.

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
