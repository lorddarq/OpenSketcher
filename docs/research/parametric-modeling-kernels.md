# Parametric face modeling: kernel comparison

Research date: 2026-10-23. This is a source review, not a compiled evaluation. No kernel dependency or implementation has been added.

## Decision and research recommendation

The user selected Truck on 2026-10-23. Proceed with Truck; the validation cases below now guide implementation rather than gate a pending product decision. No compiled kernel evaluation has been completed yet. The alternative kernels remain research context, not planned parallel implementations.

Evaluate Truck first for planar profiles, editable dimensions, and the newly confirmed boring behavior below. Keep the authored feature definitions authoritative and regenerate boundary representation (B-rep) and viewport triangles from them. Truck provides extrusion building blocks, but adopting it now also requires proving subtraction and exit-region construction; extrusion alone no longer meets the milestone. Writing another geometry kernel is unnecessary.

Open CASCADE through Rust bindings remains an alternative to revisit only if concrete failures show Truck cannot satisfy required behavior. Its underlying capabilities are wider, but neither the Rust wrapper nor packaging should be assumed complete.

## What “parametric” must mean here

A surface parameterization, such as a point evaluated from `(u, v)`, is different from an editable document. A serialized B-rep may preserve exact faces without remembering that a box came from a 2 m rectangle extruded by 3 m. OpenSketcher needs authored profiles, dimensions, directions, feature order, and references in its native document, regardless of kernel.

Likewise, an extrusion and arbitrary push/pull are different operations. Extruding a closed planar profile creates a new solid. Changing the depth of that extrusion is straightforward regeneration. Moving a selected face of an existing body can require extending neighboring surfaces, rebuilding trims, resolving intersections, and repairing topology. A prism API alone does not establish support for that second behavior.

## Candidates

| Approach | Verified capability | Application work and limitations | Fit |
| --- | --- | --- | --- |
| Truck | Rust B-rep kernel; topology and geometry are separate crates. `try_attach_plane` makes a planar face from boundary wires; `tsweep` sweeps a face into a solid. | Feature history, semantic references, UI tools, validation policy, and project schema remain ours. General face relocation is not established by the documented sweep API. | First candidate for the limited modeling scope. |
| OCCT via `bschwind/opencascade-rs` | Rust source exposes planar faces with holes, `Face::extrude`, subtractive extrusion, and extrusion up to a face. These wrap OCCT algorithms. | Wrapper calls itself work in progress. Need to check error propagation, access to operation history, API coverage, and native build/distribution. | Strong fallback when solid operations exceed Truck's proven behavior. |
| Custom polygon/half-edge model | A design option, not an evaluated dependency. Planar loops plus adjacency can support simple face extrusion. | We would own tolerances, intersection, topology repair, curved entities, naming, and robust edits. A raw triangle mesh loses authored intent unless accompanied by feature definitions. | Use triangles as viewport output; avoid building a custom general kernel now. |

Truck API evidence: [builder module](https://docs.rs/truck-modeling/latest/truck_modeling/builder/index.html), [`tsweep` example creating a cube](https://docs.rs/truck-modeling/latest/truck_modeling/builder/fn.tsweep.html), and [project/license](https://github.com/ricosjp/truck). The published docs inspected are `truck-modeling` 0.6.0. [Truck shape operations](https://docs.rs/truck-shapeops/latest/truck_shapeops/) also expose solid intersection/union; API presence is not a robustness benchmark.

OCCT wrapper evidence: [repository and WIP statement](https://github.com/bschwind/opencascade-rs), [face implementation](https://github.com/bschwind/opencascade-rs/blob/main/crates/opencascade/src/primitives/face.rs). The methods returning `Solid` or `Shape` in this source must not be treated as proof that every invalid geometry input returns a recoverable Rust error.

### Other Rust projects screened

- [Fornjot](https://www.fornjot.app/) explicitly states that the project has shut down. It remains a learning resource, not a recommended new dependency.
- [Monstertruck](https://docs.rs/monstertruck/latest/monstertruck/) is a Truck fork with renamed modeling APIs and modular geometry, meshing, and solid-operation crates. It is a comparison candidate during the spike, not a demonstrated drop-in replacement: this review has not tested its correctness, migration cost, or desktop builds.
- [next.BREP.io](https://next.brep.io/) publishes a Rust B-rep kernel and history-based application. Its site describes a custom Autodrop3d license requiring modifications to be contributed upstream. Do not assume the same licensing fit as Apache-2.0; inspect the exact crate and license before reuse. Its advertised capabilities were not benchmarked here.

## Stable face references are the central risk

Do not save a triangle index, face enumeration position, pointer-derived kernel ID, or nearest-centroid guess as a dependency. A resize or Boolean can reorder, split, merge, or remove faces. Truck documents a distinct identity for each newly constructed face, even when boundaries match; that does not provide stable identity after regeneration. [Truck face identity documentation](https://docs.rs/truck-topology/latest/truck_topology/struct.Face.html).

For initial generated solids, use application-owned feature IDs and semantic output roles: `extrusion/<id>/start`, `end`, and `side/<profile-edge-id>`. Preserve profile-edge IDs when resizing. This naming scheme is a recommendation and works only within supported construction rules; changing polygon edge count can invalidate a side reference and must be reported explicitly.

OCCT OCAF provides a more elaborate naming framework using operation history, registered topology evolution, and selection recomputation. It still depends on correct operation history and integration; selecting OCCT does not solve naming automatically. This review has not verified OCAF/TNaming coverage in the Rust wrapper. [Official OCAF naming guide](https://occt3d.com/dev/doc/overview/html/occt_user_guides__ocaf.html), [TNaming selector reference](https://www.occt3d.com/dev/doc/refman/html/class_t_naming___selector.html).

## Initial tool semantics

### Confirmed boring behavior (updated 2026-10-23)

The user confirmed two configurable modes: **Borer only** removes material with no generated body beyond the exit; **Generator + borer** also converts travel beyond the exit into an extrusion as a **separate body**. For a smaller profile driven 150 mm through a 100 mm target, Borer only produces a through-cut; Generator + borer produces that cut plus a separate 50 mm extrusion beyond the far face. At the exit exactly, there is no zero-thickness output body.

Represent this as one parametric feature with a target reference, profile/support reference, mode, direction, and total travel. Give the modified target and emerging body separate semantic output roles. Parameter changes must regenerate both outputs; references to an emerging body that no longer exists must fail explicitly. The initial validation case assumes one target and a planar exit. Do not generalize the exit to a centroid ray or simply use sweep-minus-body: that can create unintended material before entry or in internal gaps. Uneven exits and repeated entry/exit need additional product rules.

The kernel spike must cover both modes, switching between them, pocket, exact breakthrough, and overshoot; resizing the target; reversing an edit back to a pocket; output identities after Save/Load; and one-step undo/redo. This requires solid subtraction and construction of the portion beyond the exit, not just changing the source extrusion depth. The restrictions below describe ordinary extrusion editing and do not replace boring mode.

1. Build a valid planar profile from a local plane and closed outer loop, allowing holes only when supported and tested.
2. Produce an extrusion from that profile, direction, and signed distance using one shared evaluator.
3. Push/pull on its generated end face changes the extrusion distance. Explain or reject unsupported face edits rather than silently treating them as unrelated solids.
4. A side-face edit can change a source profile only where a unique parameter mapping exists; postpone general direct face movement until its reconstruction behavior is proven.
5. Generate viewport triangles and selection IDs from evaluated topology. Keep render caches disposable.

Keep circles as authored analytic circles, separately from explicit regular polygons; tessellation quality must not change the editable radius or polygon edge count. Use `f64` for document/kernel calculations and local `f32` coordinates for GPU submission if required by the renderer. Define units, supported scale, and geometry/tessellation tolerances explicitly. These are design recommendations, not verified implementation behavior.

Truck's tessellation API requires tolerance above its minimum and cautions that tessellation is not necessarily closed even for a solid. Some face outputs are optional meshes. Check every required face succeeds and retain face provenance before flattening triangles; a valid B-rep and a watertight render mesh are separate checks. [MeshableShape documentation](https://docs.rs/truck-meshalgo/latest/truck_meshalgo/tessellation/trait.MeshableShape.html).

These are proposed restrictions, not a replacement for the broader product requirement. The product must explicitly expose which faces can be edited in the first milestone.

### Subsequent face-editing decision

The user distinguished whole-face movement from node editing for NURBS and custom-deformed faces. Whole-face movement must preserve the current deformation without baking away editability. Node mode must use adjustable soft-selection falloff with a heatmap, rather than displacing only a single isolated node. The [product scope](../product-scope.md#face-types-and-deformation-editing) defines these behaviors and persistence requirements. This is an additional product requirement, not a verified Truck capability: evaluate control-net access, trim updates, shared-boundary reconstruction, and deformation validity before implementing it. NURBS rational weights and soft-selection influence must remain distinct.

## Licensing and desktop delivery

Truck declares Apache-2.0. The `opencascade-rs` repository declares LGPL-2.1; OCCT's own distribution terms and exception must also be checked for the exact version/linking arrangement chosen. These are different licensing footprints, so the application's eventual attribution-based license does not replace dependency obligations. [Truck license](https://github.com/ricosjp/truck/blob/master/LICENSE), [wrapper license](https://github.com/bschwind/opencascade-rs/blob/main/LICENSE), [OCCT license text](https://github.com/Open-Cascade-SAS/OCCT/blob/master/LICENSE_LGPL_21.txt).

Truck's Rust implementation avoids the OCCT C++ binding layer. OCCT wrapper build code links native toolkits and handles platform-specific linking; source support is not proof of a working distributable on all desktops. Neither option was compiled, packaged, launched, or benchmarked in this review. [Wrapper build script](https://github.com/bschwind/opencascade-rs/blob/main/crates/opencascade-sys/build.rs).

## Small spike gate before adoption

Use one headless executable with assertions, then wire only its output into the existing viewport. Do not scaffold multiple production engines.

- Create a rectangle and a circle, form planar faces, and extrude them; check expected dimensions, valid closed topology, and generated normals.
- Resize the profile and distance, regenerate, and confirm semantic end/side references survive supported edits.
- Verify the confirmed boring operation: pocket, exact exit, and overshoot with a separate output body; repeat after changing target thickness and after a save/load cycle.
- Exercise a concave profile and a profile with a hole if those are intended for the first release.
- Reject open, self-intersecting, zero-area, non-finite, and zero-distance inputs without retaining partial results.
- Save authored feature inputs, reload, reevaluate, and compare geometry and references within a documented tolerance.
- Verify tessellation and picking keep each triangle associated with its semantic face.
- Build and launch the same spike on macOS, Windows, and Linux; measure regeneration and tessellation time on representative profiles.

Use these checks to validate the selected Truck implementation. If a required case fails, investigate the cause and report the limitation before proposing a kernel change. General solid-face push/pull requires an additional adjacent-face/intersection check before being promised.
