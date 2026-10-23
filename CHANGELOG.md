# Changelog

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
