# CPU and memory investigation — 2026-10-24

## Findings

The strongest measured CPU costs are full-scene geometry regeneration and point-visibility testing. The current small review scene does not show sustained idle CPU use. No large geometry memory leak has been established. The initial investigation added an ignored, runnable timing harness. The follow-up below implements spatial acceleration without changing authored modeling data or selection semantics.

Measurements are local macOS ARM64 release builds, with the final workload harness running in the same two-thread Rayon configuration as the native evaluator. They are wall times, not CPU percentages, and exclude most UI/GPU submission costs. These controlled scenes identify expensive paths; they do not prove which operation caused the user's unrecorded spike.

| Workload | Display triangles | Geometry regeneration | Normal calculation | Point overlay visibility |
| --- | ---: | ---: | ---: | ---: |
| 1 cylinder, diameter 20 | 6,146 | 15.7–22.7 ms | 1.7–2.6 ms | 0.54 ms / 6 points |
| 25 cylinders, diameter 20 | 153,650 | 277–289 ms | 30.7–31.4 ms | 164.5 ms / 150 points |
| 100 cylinders, diameter 20 | 614,600 | 1.12–1.24 s | 123–125 ms | Not run |
| Saved shared-rim Imprint/through-cut review | 644 | 16.7–17.2 ms | Not separately measured | Not separately measured |

Only the first cylinder's extrusion changes in each multi-cylinder rebuild. Whole-scene mesh cloning took approximately 0.6–1.4 ms at 25 cylinders and 2.1–4.4 ms at 100. It is a real allocation/retention cost, but substantially smaller than regeneration and normals in these scenes.

An earlier run outside the bounded native worker pool measured 0.74–0.78 s for 100 cylinders; use the final two-thread values above when comparing with the app.

## Confirmed costly paths

1. **Initial point-mode repaint visibility scaled with points × triangles.** `src/main.rs` draws every geometry point and calls `selection::visible` for each. That calls `surface_hit`, which scans all scene triangles and allocates a hit list. The measured 164.5 ms is visibility alone for a single overlay update, before actual point drawing. Edge picking and snapping also call visibility repeatedly for nearby candidates. This path can be expensive during orbiting/mouse movement without any geometry edit.
2. **One changed parameter invalidates a whole-scene cache.** `model::evaluate_scene` caches exactly one complete authored scene per thread. Any body difference enters `build_scene`, rebuilding all non-Boolean bodies, replaying operations and recomputing continuity groups. There is no per-body/dependency incremental evaluation. Repeated validation of an unchanged scene does benefit from the cache.
3. **Fixed world-space tessellation is relatively dense.** `model::tessellate` uses tolerance 0.01. A plain cylinder at diameter 100 generated 24,578 triangles in the existing timing check; diameter 4 generated 1,538. There is no screen-space preview quality/LOD policy. Triangle count grows with physical dimensions even if a model occupies few screen pixels.
4. **Selection refresh rebuilds CPU display data.** `OpenSketcher::refresh` clones committed geometry into the viewport. `Viewport::update_scene` recomputes normals and shading and creates a fresh GPU vertex buffer. Selection-only changes therefore repeat geometry-derived work. Normals alone cost 123–125 ms for the 100-cylinder scene.
5. **General CSG can monopolize the worker.** Unsupported exact fast paths fall back to Truck shape operations. The evaluator coalesces pending requests and limits its Rayon pool to two threads, but Escape only discards/cancels the result; an executing kernel call continues until it returns. Root-isolation budgets exist in the newer imprint solvers; there is no universal interrupt/deadline for Truck CSG.

## Hidden geometry and rendering

Boolean/Imprint inputs are evaluated before their outputs and suppressed afterward. Consumed inputs end as empty `Geometry` entries and are not submitted as hidden triangles to the viewport. Keep tools intentionally retains inputs. Invisible attached-bore cutters already use retained kernel shells without generating their disposable display mesh on the direct cutter path.

However, ordinary consumed Boolean/Imprint source bodies are still built and tessellated before suppression, and intermediate operations can tessellate again. Hidden dependency evaluation is therefore a CPU cost even when there are no hidden GPU draw calls. Analytic boundary evaluation and disposable display tessellation are not yet separate stages.

The viewport is rendered on every UI repaint, including the roughly 60 Hz polling repaint while the geometry worker runs. It does not currently reuse a rendered viewport texture when only a progress label changes. This is an additional CPU/GPU overhead candidate, not separately quantified here. Old viewport textures are explicitly unregistered on resize; replacement GPU buffers use owned wgpu handles.

## Memory and leak evidence

The running small native review process was observed at negligible idle CPU. A three-second `sample` showed the main thread waiting for macOS events, the geometry worker waiting on its channel, and Rayon workers parked. Physical footprint was 62.1 M, with a reported peak of 92.8 M.

`leaks` reported 416 allocations totaling 19,936 bytes in three NSXPC/AppIntents connection object graphs. It also warned that the process was not debuggable and inspection was restricted. This is a small framework-cycle report, not evidence of an expanding geometry leak, and the restricted single scan cannot rule out other leaks. Raw local evidence is in `/tmp/opensketcher-idle-cpu-sample.txt` and `/tmp/opensketcher-idle-leaks.txt`.

There are several bounded memory multipliers:

- Committed geometry, viewport geometry and the main thread's scene cache hold separate display arrays. Worker thread-local caches can retain additional old scenes.
- Undo history is capped at 100 authored snapshots. It is not an unbounded mesh history, but snapshots include component topology guards and can be large.
- An evaluator request clones the full `Model`, including history, into desired and in-flight work. Large histories can therefore create allocation pressure.
- The dirty-state comparison creates an authored project snapshot during UI drawing, cloning current body data and topology guards. This was not a major measured cost in the primitive-only workload, but merits profiling on long edit histories.

These explain retention/high-water marks without demonstrating a leak. A repeat-edit/load soak with allocation attribution is still needed to test sustained growth on the user's actual project.

## Prioritized next work

1. Implemented: shared world-space triangle acceleration for visibility, picking and snapping. See the follow-up measurements below.
2. Cache normals/shading per evaluated geometry and update selection coloring without rebuilding all derived display data.
3. Add dependency-aware per-body evaluation and defer tessellation of consumed/intermediate inputs. Preserve live reference validation.
4. Introduce a disposable preview tessellation budget independent of exact CAD geometry and export quality.
5. Reuse unchanged viewport renders during progress/UI-only repaints; reduce large scene/history cloning after measuring complex histories.
6. Instrument per-operation kernel timings and strengthen cancellation/work budgets where the kernel permits it. Continue exact fast paths for known coincident-boundary cases.

## Reproduction

```sh
cargo test --offline --release circle_update_timings -- --ignored --nocapture
cargo test --offline --release geometry_workload_timings -- --ignored --nocapture
OSK_PROFILE_PROJECT=/absolute/path/project.osk cargo test --offline --release geometry_workload_timings -- --ignored --nocapture
```

The optional project path replays the saved project without changing its parameters. Built-in workloads change only the first body's extrusion. Timings are diagnostic output, not brittle machine-specific pass/fail thresholds. Both timing commands and Clippy passed during this investigation. No dependencies were added.

## Follow-up: indexed visibility implemented

Application visibility, all selection modes and snapping now share a world-space triangle BVH. Its boxes include the existing barycentric silhouette tolerance; leaf intersections retain the previous triangle test. Coincident hits preserve the same latest owner/face and original triangle-order tie resolution. Camera changes and viewport resize use the existing index.

The geometry evaluator builds the index off the UI thread and publishes it with the matching scene. Committed and preview results carry an immutable `Arc` index; the viewport and committed state share it without duplicating its arrays. Selection-only refreshes reuse it. New clears both the scene and index (and the old main-thread scene cache), and loading/history/preview replacement installs a matching pair. No spatial data is serialized and no dependencies were added.

In the same release benchmark run with two geometry threads:

| Scene | Exhaustive point overlay | Indexed point overlay | Index construction | Index allocation capacity |
| --- | ---: | ---: | ---: | ---: |
| 1 cylinder / 6 points | 0.408 ms | 0.029 ms | 1.6–1.8 ms | 0.27 MiB |
| 25 cylinders / 150 points | 238.5 ms | 1.079 ms | 53.6 ms | 8.5 MiB |
| 100 cylinders | Not measured | Not measured | 244–282 ms | 34.0 MiB |

The approximately 220× improvement applies to the 25-cylinder point visibility pass, not total application frame time. Index construction is additional work and memory, intentionally moved to the worker. Full-scene regeneration and normal/color buffer rebuilding remain separate measured bottlenecks.

`indexed_queries_preserve_hits_visibility_and_prune_triangle_work` compares indexed and exhaustive hits, visibility and all four selection modes across isometric/top/side views, projected points, edge midpoints and a screen grid. It also covers coincident owners, empty scenes and pruning most triangle tests. The evaluator coalescing check verifies that its published index matches its scene.

Native macOS review loaded a 25-cylinder scene in Point mode, switched to Top (bottom vertices stayed hidden), selected and saved point 3 on object 21, used New and clicked the empty canvas, then reopened the saved scene with the selected point restored. The isolated app is `dist/OpenSketcher Point Visibility Review.app`; its project is `/tmp/osk-point-visibility-review.osk`. This is native macOS lifecycle/picking evidence; it is not a measured native frame-rate claim or Windows/Linux verification.

Full regression suite: 81 passed, 2 manual timing checks ignored. Clippy and the release build passed.

## Follow-up: selection refresh reuses shading

`SceneDisplay` now pairs the spatial index and per-vertex shading with one immutable evaluated scene. The evaluator computes both on its geometry worker; committed state and the viewport share the same `Arc`. Selection refreshes reuse shading instead of rebuilding the normal accumulation map and cylinder normals. Preview, cancellation, New, history and loading follow the existing scene publication lifecycle. The lighting formula and sharp logical face boundaries are unchanged; no file format changes or new dependencies are involved.

This removes the previously measured 33 ms (25 cylinders) / 130 ms (100 cylinders) normal calculation from selection refreshes, by inspection of the shared refresh path. It does not remove that work from geometry evaluation or establish a total frame-time improvement. Cached shades cost 12 bytes per triangle (approximately 1.8 MiB / 7.0 MiB for those scenes), shared rather than copied on refresh. Vertex coloring, buffer creation and scene cloning still occur on refresh. The shading regression check compares every cached sample against the original normal/light formula, including a cylinder and an empty scene. Existing cylinder seam checks continue to exercise the same normal helper.

After the shading cache change: 82 tests passed, 2 manual timing checks ignored; Clippy and the release build passed. Native UI review above predates this final cache change; the cache change is covered by code checks, not a new native frame-rate measurement.

## Follow-up: unchanged base geometry reuse

Scene evaluation retains the latest successful scene's base geometry, keyed by exact authored body parameters and the freshly resolved profile boundary loops. Changing one body can reuse unchanged bases instead of reconstructing and tessellating every primitive. All body validation, profile dependency resolution, evaluated edits, bores, Booleans and their reference checks still run; final Boolean outputs are not reused under a partial key. Failed evaluation does not publish replacement bases. Removed bodies disappear from the next successful cache; empty scenes clear it. This adds one retained base-geometry snapshot per evaluating thread, not an unbounded history, and does not yet provide incremental Boolean evaluation or deferred preview tessellation.

The runnable `base_cache_reuses_unchanged_bodies_and_invalidates_profile_dependencies` check verifies kernel face identity reuse for an unchanged host, replacement for the changed body, host invalidation when a cutting profile moves, exact triangle/topology equality against cold evaluation, invalid parameter rejection and empty cache cleanup. Full regression suite after this change: 83 passed, 2 manual timing checks ignored. Clippy passed.

An isolated release benchmark after checks/builds finished measured repeated single-cylinder edits at 15.3–16.1 ms for a 25-cylinder scene and 30.0–35.4 ms for 100 cylinders in `evaluate_scene`, versus the earlier 262–271 ms / 1.05–1.13 s full rebuild measurements. First evaluations in this run were 361 ms / 848 ms; the preceding workload can contribute reusable bases. These are evaluator timings, not end-to-end latency: rebuilding the whole spatial index still took 54–55 ms / 246–250 ms, and shading took 32–33 ms / 129–131 ms on geometry updates. The application release build passed. Native UI review has not been repeated for base-cache reuse.

## Follow-up: dimension/extrusion dragging

The user localized the CPU spikes to dragging dimensions or extrusion. Inspection confirmed the evaluator already coalesces live input into one in-flight request and one desired request. Previously, superseded in-flight work still prepared all display shading and the BVH before the UI discarded its result. A shared atomic revision now lets the worker reject stale requests before applying changes, between authored changes, before final scene evaluation and before display preparation. Rejected stale responses use the existing revision discard path; the latest parameters and undo grouping remain intact. An active kernel call is still not interruptible, and a newer edit arriving during display preparation cannot stop that preparation mid-call.

BVH construction now computes each tolerant triangle bounding box once, instead of repeating it at every tree level. The existing triangle ordering, split comparator and ray intersection tests are unchanged. Temporary bounds storage costs 48 bytes per triangle (about 28 MiB for 614600 triangles), released after construction; persistent index size is unchanged.

Release measurements: index construction for 25 cylinders fell from 54–55 ms to 28–33 ms; 100 cylinders fell from 246–250 ms to 132.8–133.0 ms. These measure index construction, not application latency. The exhaustive/indexed hit-equivalence regression passed; all three evaluator tests passed, including stale request rejection, rapid edit coalescing, cancellation and error recovery. No native drag-rate measurement was performed for these changes.
