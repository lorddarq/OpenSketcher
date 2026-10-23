# Circle creation and face sketching workflows

Research date: 2026-10-23. Sources below are official product manuals, read on that date.
This is documentation research, not a hands-on evaluation of either application.

## OpenSketcher requirements confirmed by the user

- Circle creation is center plus radius, placed with the mouse in the viewport.
- Drawing on an existing face requires selecting that face first.
- The selected face exposes its points and related geometry for placement.
- Snapping is on by default.
- Numeric edits apply immediately; creation and edits support undo/redo.
- Existing volume, face, edge, and point selection modes remain distinct.

These requirements define OpenSketcher behavior; they are not claims that the other products require the same sequence.

## SketchUp: documented behavior

### Circle interaction

The Circle tool establishes size by moving outward from its center and clicking to finish.
The radius appears dynamically in Measurements and can also be entered numerically.
Circle entities consist of segments but behave as one selectable curve.
Segment endpoints and midpoints remain available to the inference engine.
See [Drawing Basic Shapes](https://help.sketchup.com/en/sketchup/drawing-basic-shapes).

### Drawing plane and snapping

SketchUp documents inference cues and keyboard locks for drawing planes and directions.
Arrow keys select axis-oriented planes; Shift locks the current inference.
Its documented workflow does not establish a mandatory face-preselection step before drawing.
See [Drawing Basics and Concepts](https://help.sketchup.com/en/sketchup/introducing-drawing-basics-and-concepts).

### Face topology

Drawing a line or curve on an existing face divides that face.
The resulting regions can be push/pulled independently.
This is a topology operation, beyond displaying another shape at the same position.
See [Drawing Basics and Concepts](https://help.sketchup.com/en/sketchup/introducing-drawing-basics-and-concepts).

## Plasticity: documented behavior

### Circle interaction

Center Circle uses a center click, cursor movement to size, then a second click.
Tab permits dimension entry; the command finishes through OK or right-click.
The manual describes the defining dimension as diameter, so it should not be cited as proof of radius-labelled input.
OpenSketcher should use radius as requested while borrowing the center-to-size gesture.
See [Center Circle](https://doc.plasticity.xyz/tool/center-circle).

### Explicit face-based construction plane

Plasticity creates curves on its active construction plane.
Select a face or region and press Space to establish that plane and align the view.
Shift+Space establishes it without changing the view.
Its 2D Snapping projects picked points onto the active plane.
This supports explicit face-derived planes, but does not mean all sketching requires face selection.
See [Construction Planes](https://doc.plasticity.xyz/plasticity-essentials/plasticity-interface/construction-plane).

### Snap targets and visible points

Object snaps include control vertices, curves, regions, edges, and faces.
Targets include endpoints, midpoints, centers, intersections, and tangent/perpendicular locations.
A visible target dot and tooltip identify the snap.
See [Snap](https://doc.plasticity.xyz/plasticity-essentials/plasticity-interface/snap).

Toggle Points reveals or hides curve vertices/control vertices and surface control vertices.
Selecting a solid face can scope the command to that face.
The manual describes an explicit command, not automatic revelation whenever a face is selected.
See [Toggle Points](https://doc.plasticity.xyz/sketch/toggle-points).

### Drawing versus imprinting

Knife mode cuts faces while drawing, provided points snap to the target faces.
See [Sketching Essentials](https://doc.plasticity.xyz/tool/sketching-essentials).
Imprint separately projects curves onto a solid or sheet to create edges.
It supports projection direction and completion options.
See [Imprint Curve Body](https://doc.plasticity.xyz/solid/imprint-curve-body).
Thus a curve lying on a face plane and a curve modifying body topology are distinct concerns.

## Recommended OpenSketcher behavior

1. Select a planar face in Face mode before activating an on-face creation tool.
2. Keep that support face highlighted and expose its boundary vertices and useful snap markers.
3. Capture its plane when creation starts; hovering another face must not silently switch support.
4. Click the circle center, move to preview the radius, and click to commit one undoable operation.
5. Show a radius segment and live radius readout; reject a zero-size circle.
6. Keep snapping on by default and show the active target, rather than only rounding invisible coordinates.
7. Preserve the plane during orbiting; do not force a camera change merely because a face was selected.
8. Escape cancels the unfinished preview without adding history or geometry.

These are recommendations informed by the evidence above and the user's explicit requirements.
Without a selected face, ordinary drawing can still use the active world construction plane.
An unsupported curved face should give a clear explanation rather than pretending a tangent-plane circle conforms to it.

## Implementation boundaries and follow-up

Topology vertices, analytic snap locations, surface control vertices, and tessellation vertices are different data.
Expose actual topology and useful analytic locations; do not label rendering samples as editable NURBS controls.
Surface deformation control points belong to the separate soft-selection specification.
An on-face profile must not be reported as a split face until Truck actually rebuilds the face topology.
Persist the circle's plane, center, and radius; retain support identity when a dependent-feature model exists.
Verification should include rotated/scaled faces, orbiting during radius preview, snapping, cancellation, undo/redo, and save/reopen.
