//! Sweeps of evaluated planar faces, preserving the existing treated boundary.
use crate::{
    edge_treatments::FaceTarget,
    model::{Body, Geometry},
};
use serde::{Deserialize, Serialize};
use truck_modeling::{EuclideanSpace, InnerSpace, Solid, SquareMatrix, Transform, builder};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Extrusion {
    pub face: FaceTarget,
    pub distance: f64,
}
pub fn validate(b: &Body) -> Result<(), String> {
    if b.face_extrusions.len() > 64
        || b.face_extrusions
            .iter()
            .any(|e| !e.distance.is_finite() || e.distance == 0. || e.distance.abs() > 100.)
    {
        return Err(
            "Face extrusion requires a nonzero distance within ±100 mm (maximum 64 operations)."
                .into(),
        );
    }
    if !b.face_extrusions.is_empty()
        && (crate::components::has_source_edits(b) || b.bore.is_some() || b.boolean.is_some())
    {
        return Err(
            "Face extrusion of component-deformed or Boolean geometry is not supported yet.".into(),
        );
    }
    Ok(())
}
pub fn target(b: &Body, g: &Geometry, face: usize) -> Result<FaceTarget, String> {
    let inverse = crate::regions::transform(b)
        .invert()
        .ok_or("Invalid placement.")?;
    let points: Vec<_> = g
        .triangles
        .iter()
        .zip(&g.triangle_faces)
        .filter(|(_, f)| **f == face)
        .flat_map(|(t, _)| t.iter().map(|p| inverse.transform_point(*p)))
        .collect();
    for role in [
        FaceTarget::Top,
        FaceTarget::Bottom,
        FaceTarget::Left,
        FaceTarget::Right,
        FaceTarget::Front,
        FaceTarget::Back,
    ] {
        let (n, mut d) = role.evaluated_plane(b);
        d += b
            .face_extrusions
            .iter()
            .filter(|e| e.face == role)
            .map(|e| e.distance)
            .sum::<f64>();
        if b.extrusion > 0.
            && role.supported(b)
            && points
                .chunks_exact(3)
                .any(|t| (t[1] - t[0]).cross(t[2] - t[0]).dot(n) > 1e-12)
            && points.iter().all(|p| (p.to_vec().dot(n) - d).abs() < 1e-6)
        {
            return Ok(role);
        }
    }
    Err("Select a planar base face or its extruded cap.".into())
}
pub fn apply(b: &Body, g: &mut Geometry) -> Result<(), String> {
    let mut prior = b.clone();
    prior.face_extrusions.clear();
    // Reversing the same cap retracts its previous sweep; it must not leave
    // coincident inner walls from an outward sweep followed by an inward one.
    let mut operations: Vec<Extrusion> = Vec::new();
    for operation in &b.face_extrusions {
        let mut combined = operation.clone();
        if operation.distance < 0.
            || operations
                .last()
                .is_some_and(|e| e.face == operation.face && e.distance < 0.)
        {
            while operations.last().is_some_and(|e| e.face == operation.face) {
                combined.distance += operations.pop().unwrap().distance;
            }
        }
        // Preserve existing all-outward histories and their component topology.
        operations.push(combined);
    }
    for operation in operations.iter().filter(|e| e.distance.abs() > 1e-8) {
        let previous = prior
            .face_extrusions
            .iter()
            .filter(|e| e.face == operation.face)
            .map(|e| e.distance)
            .sum::<f64>();
        if previous * operation.distance < 0. {
            return Err("Reversing a cap after editing another face requires undoing the intervening face operation first.".into());
        }
        let shell = g
            .shell
            .as_ref()
            .ok_or("Face extrusion requires an exact boundary.")?;
        let raw = crate::model::tessellate(shell)?;
        let indices: Vec<_> = (0..raw.face_count)
            .filter(|f| target(&prior, &raw, *f) == Ok(operation.face))
            .collect();
        if indices.is_empty() {
            return Err("The extrusion's planar face no longer exists.".into());
        }
        let selected: truck_modeling::Shell = indices.iter().map(|i| shell[*i].clone()).collect();
        let face = if indices.len() == 1 {
            selected[0].clone()
        } else {
            builder::try_attach_plane(&selected.extract_boundaries())
                .map_err(|e| format!("Cannot join the selected planar face: {e}"))?
        };
        if operation.distance < 0. {
            let (normal, _) = operation.face.plane(b);
            let thickness = if normal.z.abs() > 0.5 {
                b.extrusion
            } else if normal.x.abs() > 0.5 {
                b.width
            } else {
                b.depth
            };
            let opposite = operation.face.opposite();
            let thickness = thickness + operation.face.evaluated_plane(b).1
                - operation.face.plane(b).1
                + opposite.evaluated_plane(b).1
                - opposite.plane(b).1;
            let exit = thickness
                + previous
                + prior
                    .face_extrusions
                    .iter()
                    .filter(|e| e.face == opposite)
                    .map(|e| e.distance)
                    .sum::<f64>();
            // Reuse source offset for a pristine cap or a shell opening whose
            // fixed cavity floor is untouched. A full primitive cut consumes it.
            if (b.hollow.is_none()
                || b.hollow.as_ref().is_some_and(|shell| {
                    !shell.closed
                        && shell.opening == operation.face
                        && -operation.distance < exit - shell.thickness - 1e-5
                }))
                && b.taper.is_none()
                && b.edge_treatments.is_empty()
                && prior.face_extrusions.is_empty()
            {
                if -operation.distance >= exit - 1e-7 {
                    *g = Geometry::default();
                } else {
                    let mut source = b.clone();
                    source.face_extrusions.clear();
                    source.component_edits.clear();
                    let distance = operation.distance
                        * crate::regions::transform(b)
                            .transform_vector(normal)
                            .magnitude();
                    let source =
                        crate::components::offset_source_face(&source, operation.face, distance)?;
                    *g = crate::model::build(&source, None)?;
                }
                prior.face_extrusions.push(operation.clone());
                continue;
            }
            if b.hollow.is_some() {
                if prior.face_extrusions.is_empty()
                    && let Some(result) =
                        crate::shelling::retract_back(b, operation.face, operation.distance)?.or(
                            crate::shelling::retract_opening(
                                b,
                                operation.face,
                                operation.distance,
                            )?,
                        )
                {
                    *g = result;
                    prior.face_extrusions.push(operation.clone());
                    continue;
                }
                *g = subtract_sweep(
                    b,
                    g,
                    &face,
                    normal,
                    operation.distance,
                    -operation.distance >= exit - 1e-7,
                )?;
                prior.face_extrusions.push(operation.clone());
                continue;
            }
            if -operation.distance >= exit - 1e-7 {
                if let Some(result) =
                    through_cut(&prior, g, &raw, &face, &indices, operation.face, exit)?
                {
                    *g = result;
                } else {
                    *g = subtract_sweep(b, g, &face, normal, operation.distance, true)?;
                }
                prior.face_extrusions.push(operation.clone());
                continue;
            }
            let opposite_setback = b
                .edge_treatments
                .iter()
                .map(|m| {
                    let (a, c) = m.setbacks();
                    if m.face == opposite {
                        c
                    } else if m.edge == Some(opposite) {
                        a
                    } else {
                        0.
                    }
                })
                .fold(0., f64::max);
            let perimeter = b
                .edge_treatments
                .iter()
                .any(|m| m.face == operation.face && m.edge.is_none());
            let different_axis = operations
                .iter()
                .any(|e| e.face != operation.face && e.face != opposite);
            if !perimeter || different_axis || -operation.distance >= exit - opposite_setback - 1e-7
            {
                *g = subtract_sweep(b, g, &face, normal, operation.distance, false)?;
                prior.face_extrusions.push(operation.clone());
                continue;
            }
        }
        let vector = crate::regions::transform(b)
            .transform_vector(operation.face.plane(b).0 * operation.distance);
        let mut result = extend_face(
            g,
            &face,
            &indices,
            vector,
            b.shape == crate::model::Shape::Circle,
            false,
        )?;
        // Retain snap circles on the existing boundary and add the moved cap rim.
        result.circles = g.circles.clone();
        let (normal, mut plane) = operation.face.evaluated_plane(&prior);
        plane += prior
            .face_extrusions
            .iter()
            .filter(|e| e.face == operation.face)
            .map(|e| e.distance)
            .sum::<f64>();
        let inverse = crate::regions::transform(b)
            .invert()
            .ok_or("Invalid placement.")?;
        for ring in &g.circles {
            if (inverse.transform_point(ring.center).to_vec().dot(normal) - plane).abs() < 1e-6 {
                let mut moved = ring.clone();
                moved.center += vector;
                result.circles.push(moved);
            }
        }
        *g = result;
        prior.face_extrusions.push(operation.clone());
    }
    Ok(())
}

// Match retained affine circle metadata to a complete authored boundary wire.
pub(crate) fn circle_on_wire(
    circle: &crate::model::CircularEdge,
    wire: &truck_modeling::Wire,
) -> bool {
    use truck_modeling::{BoundedCurve, ParametricCurve};
    let u = circle.u;
    let v = circle.v;
    let normal = u.cross(v);
    let determinant = u.magnitude2() * v.magnitude2() - u.dot(v).powi(2);
    if determinant < 1e-20 {
        return false;
    }
    wire.iter().all(|edge| {
        let curve = edge.oriented_curve();
        let (start, end) = curve.range_tuple();
        [start, (start + end) / 2., end].into_iter().all(|t| {
            let p = curve.subs(t) - circle.center;
            let a = (p.dot(u) * v.magnitude2() - p.dot(v) * u.dot(v)) / determinant;
            let b = (p.dot(v) * u.magnitude2() - p.dot(u) * u.dot(v)) / determinant;
            (a * a + b * b - 1.).abs() < 1e-6 && p.dot(normal).abs() < 1e-7 * normal.magnitude()
        })
    })
}

// Shared by semantic source-cap sweeps and ordered evaluated-face operations.
fn tangent_roles(
    face: &truck_modeling::Face,
    mut offset: usize,
) -> std::collections::HashMap<truck_modeling::EdgeID, usize> {
    let mut roles = std::collections::HashMap::new();
    for wire in face.boundaries() {
        for (edge, group) in wire.iter().zip(crate::model::tangent_groups(&wire)) {
            roles.insert(edge.id(), offset + group);
        }
        offset += wire.len();
    }
    roles
}

// A proven straight prism has no intermediate sections, and every side
// surface is invariant along the sweep axis. Rebuild its remaining cap region
// instead of intersecting coincident curved walls in the general CSG kernel.
pub(crate) fn prismatic_section(
    g: &Geometry,
    plane: crate::snapping::Plane,
) -> Result<Option<(f64, Vec<usize>)>, String> {
    use truck_modeling::{ParametricSurface, Surface, Vector4};
    let Some(shell) = &g.shell else {
        return Ok(None);
    };
    let heights = g
        .points
        .iter()
        .map(|p| (*p - plane.origin).dot(plane.normal))
        .collect::<Vec<_>>();
    let Some(depth) = heights.iter().find(|h| h.abs() > 1e-7).map(|h| -*h) else {
        return Ok(None);
    };
    if depth <= 1e-7
        || heights
            .iter()
            .any(|h| h.abs() > 1e-7 && (*h + depth).abs() > 1e-7)
    {
        return Ok(None);
    }
    let mut entry = Vec::new();
    for (index, face) in shell.iter().enumerate() {
        let surface = face.oriented_surface();
        let (points, degrees) = match surface {
            Surface::Plane(p) => {
                let n = p.normal().normalize();
                let h = (p.subs(0., 0.) - plane.origin).dot(plane.normal);
                if n.dot(plane.normal) > 1. - 1e-7 && h.abs() < 1e-7 {
                    entry.push(index);
                } else if !((n.dot(plane.normal) < -1. + 1e-7 && (h + depth).abs() < 1e-7)
                    || n.dot(plane.normal).abs() < 1e-7)
                {
                    return Ok(None);
                }
                continue;
            }
            Surface::BSplineSurface(p) => (
                p.control_points()
                    .iter()
                    .map(|row| row.iter().map(|p| p.to_vec().extend(1.)).collect())
                    .collect::<Vec<Vec<Vector4>>>(),
                p.degrees(),
            ),
            Surface::NurbsSurface(p) => (p.control_points().clone(), p.degrees()),
            _ => return Ok(None),
        };
        let translated = |pairs: Vec<(Vector4, Vector4)>| {
            let Some((a, b)) = pairs.first() else {
                return false;
            };
            if a.w.abs() < 1e-12 || b.w.abs() < 1e-12 {
                return false;
            }
            let start = (a.truncate() / a.w - plane.origin.to_vec()).dot(plane.normal);
            let end = (b.truncate() / b.w - plane.origin.to_vec()).dot(plane.normal);
            if !((start.abs() < 1e-7 && (end + depth).abs() < 1e-7)
                || (end.abs() < 1e-7 && (start + depth).abs() < 1e-7))
            {
                return false;
            }
            pairs.iter().all(|(a, b)| {
                let scale = a.w.abs().max(b.w.abs()).max(1.);
                (a.w - b.w).abs() < 1e-10 * scale
                    && (b.truncate() - a.truncate() - plane.normal * ((end - start) * a.w))
                        .magnitude()
                        < 1e-7 * scale
                    && (a.truncate() - plane.origin.to_vec() * a.w - plane.normal * (start * a.w))
                        .dot(plane.normal)
                        .abs()
                        < 1e-7 * scale
            })
        };
        let u = degrees.0 == 1
            && points.len() == 2
            && translated(
                points[0]
                    .iter()
                    .copied()
                    .zip(points[1].iter().copied())
                    .collect(),
            );
        let v = degrees.1 == 1
            && points.iter().all(|row| row.len() == 2)
            && translated(points.iter().map(|row| (row[0], row[1])).collect());
        if !u && !v {
            return Ok(None);
        }
    }
    Ok(Some((depth, entry)))
}

fn prismatic_through_remainder(
    g: &Geometry,
    indices: &[usize],
    plane: crate::snapping::Plane,
    distance: f64,
) -> Result<Option<Geometry>, String> {
    let Some((depth, caps)) = prismatic_section(g, plane)? else {
        return Ok(None);
    };
    if -distance < depth - 1e-7 {
        return Ok(None);
    }
    let shell = g.shell.as_ref().unwrap();
    let entry: truck_modeling::Shell = caps
        .into_iter()
        .filter(|i| !indices.contains(i))
        .map(|i| shell[i].clone())
        .collect();
    if entry.is_empty() {
        return Ok(Some(Geometry::default()));
    }
    let mut output = truck_modeling::Shell::new();
    let mut roles = Vec::new();
    let mut next = 0;
    for cap in crate::face_operations::planar_regions(entry.extract_boundaries())? {
        let cap = cap.inverse();
        let side_roles = tangent_roles(&cap, next + 2);
        let sweep: Solid = builder::tsweep(&cap, -plane.normal * depth);
        let boundary = &sweep.boundaries()[0];
        for face in boundary {
            roles.push(if face.id() == cap.id() {
                next
            } else if face.id() == boundary.last().unwrap().id() {
                next + 1
            } else {
                face.edge_iter()
                    .find_map(|e| side_roles.get(&e.id()).copied())
                    .ok_or("Prismatic cut wall lost its source curve.")?
            });
            output.push(face.clone());
        }
        next = roles.iter().copied().max().unwrap() + 1;
    }
    Solid::try_new(output.connected_components())
        .map_err(|e| format!("Invalid prismatic through-cut: {e}"))?;
    let mut result = crate::model::tessellate(&output)?;
    crate::edge_treatments::merge_surfaces(&mut result, &roles);
    result.is_solid = true;
    result.circles = g
        .circles
        .iter()
        .filter(|ring| {
            output
                .iter()
                .flat_map(|f| f.boundaries())
                .any(|w| circle_on_wire(ring, &w))
        })
        .cloned()
        .collect();
    Ok(Some(result))
}

fn extend_face(
    g: &Geometry,
    face: &truck_modeling::Face,
    indices: &[usize],
    vector: truck_modeling::Vector3,
    circular_walls: bool,
    tangent_walls: bool,
) -> Result<Geometry, String> {
    let sweep: Solid = builder::tsweep(face, vector);
    let shell = g
        .shell
        .as_ref()
        .ok_or("Face extrusion requires an exact boundary.")?;
    let mut joined = shell.clone();
    let mut roles = g.surface_mapping()?.to_vec();
    let cap_role = roles[indices[0]];
    for index in indices.iter().rev() {
        joined.remove(*index);
        roles.remove(*index);
    }
    let side_role = roles.iter().copied().max().unwrap_or(0).max(cap_role) + 1;
    let tangent_roles = if tangent_walls {
        tangent_roles(face, side_role)
    } else {
        std::collections::HashMap::new()
    };
    for patch in sweep.boundaries()[0].iter() {
        if patch.id() == face.id() {
            continue;
        }
        // The translated cap is last in Truck's translational face sweep.
        let cap = patch.id() == sweep.boundaries()[0].last().unwrap().id();
        joined.push(patch.clone());
        roles.push(if cap {
            cap_role
        } else if tangent_walls {
            patch
                .edge_iter()
                .find_map(|edge| tangent_roles.get(&edge.id()).copied())
                .ok_or("Swept wall lost its source curve.")?
        } else if circular_walls {
            let boundary = face
                .boundaries()
                .iter()
                .position(|wire| {
                    patch
                        .edge_iter()
                        .any(|edge| wire.iter().any(|source| source.id() == edge.id()))
                })
                .ok_or("Swept wall lost its source boundary.")?;
            side_role + boundary
        } else {
            side_role + roles.len()
        });
    }
    Solid::try_new(joined.connected_components())
        .map_err(|e| format!("Face extrusion did not produce a closed solid: {e}"))?;
    let mut result = crate::model::tessellate(&joined)?;
    crate::edge_treatments::merge_surfaces(&mut result, &roles);
    result.is_solid = true;
    result.circles = g.circles.clone();
    if tangent_walls {
        for ring in &g.circles {
            if face
                .boundaries()
                .iter()
                .any(|wire| circle_on_wire(ring, wire))
            {
                let mut moved = ring.clone();
                moved.center += vector;
                result.circles.push(moved);
            }
        }
    }
    Ok(result)
}

pub(crate) fn evaluated_sweep(
    g: &Geometry,
    element: usize,
    vector: truck_modeling::Vector3,
    circular_walls: bool,
    tangent_walls: bool,
    prismatic_cut: bool,
) -> Result<Geometry, String> {
    if !g.is_solid {
        return Err("Face extrusion requires a closed solid.".into());
    }
    let plane = crate::snapping::Plane::face(g, element)
        .ok_or("Select a planar face; curved face sweeps require a separate operation.")?;
    let shell = g
        .shell
        .as_ref()
        .ok_or("This component deformation does not retain an exact boundary for extrusion.")?;
    let indices = g
        .surface_mapping()?
        .iter()
        .enumerate()
        .filter(|(_, role)| **role == element)
        .map(|(i, _)| i)
        .collect::<Vec<_>>();
    if indices.is_empty() {
        return Err("The selected face no longer exists.".into());
    }
    let selected: truck_modeling::Shell = indices.iter().map(|i| shell[*i].clone()).collect();
    let face = if indices.len() == 1 {
        selected[0].clone()
    } else {
        builder::try_attach_plane(&selected.extract_boundaries()).map_err(|e| e.to_string())?
    };
    let distance = vector.dot(plane.normal);
    if distance.abs() < 1e-8 {
        return Err("Extrusion must leave the selected face plane.".into());
    }
    if distance < 0. {
        // A full prismatic cap removes exactly a slab. Reuse exact plane
        // retrimming, avoiding coincident Boolean faces and clamping overrun.
        if let Ok(supports) = crate::components::planar_supports(g) {
            let mut depth = None;
            let prismatic = supports
                .iter()
                .enumerate()
                .filter(|(i, _)| *i != element)
                .all(|(_, (n, d))| {
                    if (*n + plane.normal).magnitude() < 1e-7 {
                        depth = Some(*d + plane.normal.dot(plane.origin.to_vec()));
                        true
                    } else {
                        n.dot(vector).abs() < 1e-8
                    }
                });
            if prismatic && let Some(depth) = depth {
                return if -distance >= depth - 1e-7 {
                    Ok(Geometry::default())
                } else {
                    crate::components::retrim_faces(
                        g,
                        &[element],
                        truck_modeling::Matrix4::from_translation(vector),
                    )
                };
            }
        }
        if tangent_walls && let Ok(supports) = crate::components::convex_supports(g) {
            let profile = Geometry {
                shell: Some(std::iter::once(face.clone()).collect()),
                ..Default::default()
            };
            let strictly_inside = supports.iter().all(|(n, d)| {
                profile
                    .projection_range(*n)
                    .is_ok_and(|bounds| bounds[1] + n.dot(vector) < *d - 1e-7)
            });
            if strictly_inside {
                return extend_face(g, &face, &indices, vector, circular_walls, true);
            }
            // A complete planar prism exit can be removed without coincident
            // CSG faces. Other intersections continue through the solid solver.
            let axis = plane.normal;
            let sides_clear = supports
                .iter()
                .filter(|(n, _)| n.dot(axis).abs() < 1. - 1e-7)
                .all(|(n, d)| {
                    n.dot(axis).abs() < 1e-7
                        && profile
                            .projection_range(*n)
                            .is_ok_and(|bounds| bounds[1] < *d - 1e-7)
                });
            if sides_clear
                && let Some((exit_role, depth)) = supports
                    .iter()
                    .enumerate()
                    .filter(|(_, (n, _))| n.dot(axis) < -1. + 1e-7)
                    .map(|(i, (_, d))| (i, *d + plane.origin.to_vec().dot(axis)))
                    .filter(|(_, d)| *d > 1e-7)
                    .min_by(|a, b| a.1.total_cmp(&b.1))
                && -distance >= depth - 1e-7
            {
                let exits = g
                    .surface_mapping()?
                    .iter()
                    .enumerate()
                    .filter(|(_, role)| **role == exit_role)
                    .map(|(i, _)| i)
                    .collect::<Vec<_>>();
                let selected: truck_modeling::Shell =
                    exits.iter().map(|i| shell[*i].clone()).collect();
                let exit = if selected.len() == 1 {
                    selected[0].clone()
                } else {
                    builder::try_attach_plane(&selected.extract_boundaries())
                        .map_err(|e| e.to_string())?
                };
                let raw = crate::model::tessellate(shell)?;
                if let Some(result) = through_boundary(
                    g,
                    &raw,
                    (&face, &indices),
                    (&exit, &exits),
                    -axis * depth,
                    circular_walls,
                    true,
                )? {
                    return Ok(result);
                }
            }
        }
        if prismatic_cut
            && vector.cross(plane.normal).magnitude() < vector.magnitude() * 1e-7
            && let Some(result) = prismatic_through_remainder(g, &indices, plane, distance)?
        {
            return Ok(result);
        }
        let margin = -vector.normalize() * 1e-4;
        let start = builder::transformed(
            &face.inverse(),
            truck_modeling::Matrix4::from_translation(margin),
        );
        let tool: Solid = builder::tsweep(&start, vector - margin);
        return crate::operators::difference(g, &tool.boundaries()[0]);
    }
    let mut result = extend_face(g, &face, &indices, vector, circular_walls, tangent_walls)?;
    for ring in g.circles.iter().filter(|_| !tangent_walls) {
        if (ring.center - plane.origin).dot(plane.normal).abs() < 1e-6 {
            let mut moved = ring.clone();
            moved.center += vector;
            result.circles.push(moved);
        }
    }
    Ok(result)
}

/// Direct shared-boundary through-cut for a profile contained by the exit cap.
/// Complex intersections fall back to the existing solid subtractor.
fn through_cut(
    b: &Body,
    g: &Geometry,
    raw: &Geometry,
    entry: &truck_modeling::Face,
    entry_indices: &[usize],
    role: FaceTarget,
    distance: f64,
) -> Result<Option<Geometry>, String> {
    let shell = g.shell.as_ref().ok_or("Missing solid boundary.")?;
    let exit_indices: Vec<_> = (0..raw.face_count)
        .filter(|f| target(b, raw, *f) == Ok(role.opposite()))
        .collect();
    if exit_indices.is_empty() {
        return Ok(None);
    }
    let exit_shell: truck_modeling::Shell =
        exit_indices.iter().map(|i| shell[*i].clone()).collect();
    let exit = if exit_indices.len() == 1 {
        exit_shell[0].clone()
    } else {
        builder::try_attach_plane(&exit_shell.extract_boundaries()).map_err(|e| e.to_string())?
    };
    let vector = crate::regions::transform(b).transform_vector(role.plane(b).0 * -distance);
    through_boundary(
        g,
        raw,
        (entry, entry_indices),
        (&exit, &exit_indices),
        vector,
        b.shape == crate::model::Shape::Circle,
        false,
    )
}

fn through_boundary(
    g: &Geometry,
    raw: &Geometry,
    entry: (&truck_modeling::Face, &[usize]),
    exit: (&truck_modeling::Face, &[usize]),
    vector: truck_modeling::Vector3,
    circular_walls: bool,
    tangent_walls: bool,
) -> Result<Option<Geometry>, String> {
    let (entry, entry_indices) = entry;
    let (exit, exit_indices) = exit;
    let shell = g.shell.as_ref().ok_or("Missing solid boundary.")?;
    let mut profile = crate::model::tessellate(&std::iter::once(entry.clone()).collect())?;
    crate::components::map_geometry(
        &mut profile,
        truck_modeling::Matrix4::from_translation(vector),
    );
    // When entry and exit rims coincide, connect to the original exit edges
    // directly. This avoids zero-area caps and keeps exact circular boundaries.
    let entry_wires = entry.boundaries();
    let exit_wires = exit.boundaries();
    let mut matched = None;
    if entry_wires.len() == 1
        && exit_wires.len() == 1
        && entry_wires[0].len() == exit_wires[0].len()
    {
        let source = &entry_wires[0];
        let mut edges: Vec<_> = exit_wires[0].inverse().into_iter().collect();
        for _ in 0..edges.len() {
            if source.iter().zip(&edges).all(|(a, c)| {
                (a.front().point() + vector - c.front().point()).magnitude() < 1e-6
                    && (a.back().point() + vector - c.back().point()).magnitude() < 1e-6
            }) {
                matched = Some(edges.clone().into_iter().collect::<truck_modeling::Wire>());
                break;
            }
            edges.rotate_left(1);
        }
    }
    if matched.is_none() && !crate::booleans::face_coverage(&profile, raw, exit_indices)?.0 {
        return Ok(None);
    }
    let mut roles = if g.surface_roles.len() == shell.len() {
        g.surface_roles.clone()
    } else if g.triangle_faces.len() == shell.len() {
        g.triangle_faces.clone()
    } else {
        return Err("Missing evaluated surface mapping.".into());
    };
    let side_role = roles.iter().copied().max().unwrap_or(0) + 1;
    let exit_role = roles[exit_indices[0]];
    let mut joined = shell.clone();
    let mut removed = entry_indices.to_vec();
    removed.extend(exit_indices);
    removed.sort_unstable();
    removed.dedup();
    for i in removed.iter().rev() {
        joined.remove(*i);
        roles.remove(*i);
    }
    let walls: truck_modeling::Shell = if let Some(wire) = matched {
        builder::try_wire_homotopy(&entry_wires[0], &wire).map_err(|e| e.to_string())?
    } else {
        let sweep: Solid = builder::tsweep(entry, vector);
        let boundary = &sweep.boundaries()[0];
        boundary[1..boundary.len() - 1].iter().cloned().collect()
    };
    let tangent_roles = if tangent_walls {
        tangent_roles(entry, side_role)
    } else {
        std::collections::HashMap::new()
    };
    for wall in walls {
        joined.push(wall.clone());
        roles.push(if tangent_walls {
            wall.edge_iter()
                .find_map(|edge| tangent_roles.get(&edge.id()).copied())
                .ok_or("Cut wall lost its source curve.")?
        } else if circular_walls {
            side_role
        } else {
            side_role + roles.len()
        });
    }
    if !joined.extract_boundaries().is_empty() {
        let wires: Vec<_> = joined
            .extract_boundaries()
            .iter()
            .map(|w| w.inverse())
            .collect();
        if tangent_walls {
            let next = roles.iter().copied().max().unwrap_or(0) + 1;
            for (i, face) in crate::face_operations::planar_regions(wires)?
                .into_iter()
                .enumerate()
            {
                joined.push(face);
                roles.push(if i == 0 { exit_role } else { next + i });
            }
        } else {
            joined.push(
                builder::try_attach_plane(&wires)
                    .map_err(|e| format!("Cannot close the subtractor exit: {e}"))?,
            );
            roles.push(exit_role);
        }
    }
    Solid::try_new(joined.connected_components())
        .map_err(|e| format!("Invalid through-cut boundary: {e}"))?;
    let mut result = crate::model::tessellate(&joined)?;
    crate::edge_treatments::merge_surfaces(&mut result, &roles);
    result.is_solid = true;
    result.circles = g.circles.clone();
    let plane =
        crate::snapping::Plane::face(raw, entry_indices[0]).ok_or("Missing planar cut entry.")?;
    for ring in &g.circles {
        if (ring.center - plane.origin).dot(plane.normal).abs() < 1e-6
            && (!tangent_walls || entry_wires.iter().any(|wire| circle_on_wire(ring, wire)))
        {
            let mut moved = ring.clone();
            moved.center += vector;
            result.circles.push(moved);
        }
    }
    Ok(Some(result))
}
fn subtract_sweep(
    b: &Body,
    g: &Geometry,
    face: &truck_modeling::Face,
    normal: truck_modeling::Vector3,
    distance: f64,
    through: bool,
) -> Result<Geometry, String> {
    let axis = crate::regions::transform(b).transform_vector(normal);
    let margin = 1e-4;
    let start = builder::transformed(
        &face.inverse(),
        truck_modeling::Matrix4::from_translation(axis.normalize() * margin),
    );
    let sweep: Solid = builder::tsweep(
        &start,
        axis * distance - axis.normalize() * margin * (if through { 2. } else { 1. }),
    );
    crate::operators::difference(g, &sweep.boundaries()[0])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        edge_treatments::{Kind, Modifier},
        model::{Shape, evaluate},
    };
    #[test]
    fn prismatic_reconstruction_requires_a_complete_parallel_through_cut() {
        let mut body = crate::model::Body::new(crate::model::Shape::Circle);
        body.id = 1;
        body.width = 4.;
        body.extrusion = 3.;
        for tapered in [false, true] {
            body.taper = if tapered {
                Some(crate::drafting::Draft {
                    degrees: 9.,
                    neutral: FaceTarget::Bottom,
                })
            } else {
                None
            };
            let g = crate::model::evaluate(&body).unwrap();
            let cap = (0..g.face_count)
                .find(|f| crate::snapping::Plane::face(&g, *f).is_some_and(|p| p.normal.z > 0.99))
                .unwrap();
            let indices = g
                .surface_mapping()
                .unwrap()
                .iter()
                .enumerate()
                .filter(|(_, role)| **role == cap)
                .map(|(i, _)| i)
                .collect::<Vec<_>>();
            let plane = crate::snapping::Plane::face(&g, cap).unwrap();
            assert!(
                prismatic_through_remainder(&g, &indices, plane, -2.)
                    .unwrap()
                    .is_none()
            );
            let full = prismatic_through_remainder(&g, &indices, plane, -4.).unwrap();
            if tapered {
                assert!(full.is_none(), "Do not flatten a tapered solid to a prism");
            } else {
                assert!(full.unwrap().triangles.is_empty());
            }
        }
    }

    #[test]
    fn treated_cap_extrudes_without_moving_treatment() {
        for shape in [Shape::Circle, Shape::Rectangle] {
            for kind in [Kind::Chamfer, Kind::Bevel] {
                for face in [FaceTarget::Top, FaceTarget::Bottom] {
                    for distance in [1., -1.] {
                        let mut b = Body::new(shape);
                        b.id = 1;
                        b.width = 4.;
                        b.depth = 4.;
                        b.extrusion = 4.;
                        b.edge_treatments.push(Modifier::new(kind, face, &b));
                        b.position = [3., -2., 1.];
                        b.rotation = [15., 25., 35.];
                        b.scale = [1.5, 0.75, 2.];
                        let inverse = crate::regions::transform(&b).invert().unwrap();
                        let original = evaluate(&b).unwrap();
                        b.face_extrusions.push(Extrusion { face, distance });
                        let changed = evaluate(&b).unwrap();
                        assert!(changed.is_solid);
                        let volume = |g: &Geometry| {
                            g.triangles
                                .iter()
                                .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                                .sum::<f64>()
                        };
                        let cap_area = original
                            .triangles
                            .iter()
                            .zip(&original.triangle_faces)
                            .filter(|(_, f)| {
                                target(
                                    &Body {
                                        face_extrusions: vec![],
                                        ..b.clone()
                                    },
                                    &original,
                                    **f,
                                ) == Ok(face)
                            })
                            .map(|(t, _)| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
                            .sum::<f64>();
                        let expected = cap_area
                            * crate::regions::transform(&b)
                                .transform_vector(face.plane(&b).0 * distance)
                                .magnitude()
                            * distance.signum();
                        assert!(
                            (volume(&changed) - volume(&original) - expected).abs() < 0.1,
                            "The signed sweep must add/remove the cap's volume"
                        );
                        assert!(
                            (0..changed.face_count).any(|f| target(&b, &changed, f) == Ok(face)),
                            "Extruded cap must remain separately selectable"
                        );
                        assert!(changed.face_count <= original.face_count + 4);
                        assert!(changed.points.iter().any(|p| {
                            (inverse.transform_point(*p).z
                                - if face == FaceTarget::Top {
                                    4. + distance
                                } else {
                                    -distance
                                })
                            .abs()
                                < 1e-6
                        }));
                        for p in &original.points {
                            assert!(
                                changed.points.iter().any(|q| (*p - *q).magnitude() < 1e-6),
                                "Existing treatment moved"
                            );
                        }
                        b.face_extrusions.push(Extrusion {
                            face,
                            distance: 0.5,
                        });
                        evaluate(&b).unwrap();
                        let mut model = crate::model::Model::default();
                        model.commit(b.clone()).unwrap();
                        let mut next = model.bodies[0].clone();
                        next.face_extrusions.push(Extrusion {
                            face,
                            distance: 0.25,
                        });
                        model.edit(next).unwrap();
                        model.finish_edit();
                        model.undo();
                        assert_eq!(model.bodies[0].face_extrusions.len(), 2);
                        model.redo();
                        assert_eq!(model.bodies[0].face_extrusions.len(), 3);
                        let mut invalid = model.bodies[0].clone();
                        invalid.face_extrusions[0].distance = -101.;
                        assert!(model.edit(invalid).is_err());
                        assert_eq!(model.bodies[0].face_extrusions.len(), 3);
                        let saved = serde_json::to_vec(&b).unwrap();
                        assert_eq!(serde_json::from_slice::<Body>(&saved).unwrap(), b);
                        if shape == Shape::Circle
                            && kind == Kind::Chamfer
                            && face == FaceTarget::Top
                        {
                            let path = std::env::temp_dir()
                                .join(format!("osk-face-extrusion-{}.osk", std::process::id()));
                            crate::project::Project::new(
                                vec![b.clone()],
                                crate::viewport::Camera::default(),
                                true,
                                Some(1),
                            )
                            .save(&path)
                            .unwrap();
                            assert_eq!(
                                crate::project::Project::load(&path).unwrap().bodies,
                                vec![b.clone()]
                            );
                            std::fs::remove_file(&path).unwrap();
                            for format in [
                                crate::export::Format::Stl,
                                crate::export::Format::Glb,
                                crate::export::Format::Step,
                                crate::export::Format::Dxf,
                            ] {
                                let path = path.with_extension(format.extension());
                                crate::export::save(&[b.clone()], format, &path).unwrap();
                                assert!(std::fs::metadata(&path).unwrap().len() > 0);
                                std::fs::remove_file(path).unwrap();
                            }
                        }
                    }
                }
            }
        }
        let mut b = Body::new(Shape::Circle);
        b.id = 1;
        b.width = 4.;
        b.extrusion = 4.;
        for face in [FaceTarget::Top, FaceTarget::Bottom] {
            b.edge_treatments
                .push(Modifier::new(Kind::Chamfer, face, &b));
        }
        let base = evaluate(&b).unwrap();
        b.face_extrusions = vec![
            Extrusion {
                face: FaceTarget::Top,
                distance: 1.,
            },
            Extrusion {
                face: FaceTarget::Top,
                distance: 1.,
            },
        ];
        let extended = evaluate(&b).unwrap();
        assert_eq!(
            extended.face_count,
            base.face_count + 2,
            "Legacy outward sweeps retain their topology"
        );
        b.face_extrusions.push(Extrusion {
            face: FaceTarget::Top,
            distance: -2.,
        });
        assert_eq!(
            evaluate(&b).unwrap().topology,
            base.topology,
            "Full retraction restores the original boundary"
        );
        b.face_extrusions = vec![
            Extrusion {
                face: FaceTarget::Top,
                distance: -3.,
            },
            Extrusion {
                face: FaceTarget::Bottom,
                distance: -2.,
            },
        ];
        assert!(evaluate(&b).unwrap().is_solid);
        b.face_extrusions = vec![Extrusion {
            face: FaceTarget::Top,
            distance: -4.,
        }];
        assert!(evaluate(&b).unwrap().is_solid);
        b.edge_treatments.clear();
        b.face_extrusions[0].distance = -1.;
        assert!(evaluate(&b).unwrap().is_solid);
        b.face_extrusions[0].distance = -5.;
        assert!(evaluate(&b).unwrap().triangles.is_empty());
        for shape in [Shape::Circle, Shape::Rectangle] {
            for kind in [Kind::Chamfer, Kind::Bevel] {
                for face in [FaceTarget::Top, FaceTarget::Bottom] {
                    let mut b = Body::new(shape);
                    b.id = 1;
                    b.width = 4.;
                    b.depth = 4.;
                    b.extrusion = 4.;
                    b.position = [3., -2., 1.];
                    b.rotation = [15., 25., 35.];
                    b.scale = [1.5, 0.75, 2.];
                    b.edge_treatments.push(Modifier::new(kind, face, &b));
                    let original = evaluate(&b).unwrap();
                    let volume = |g: &Geometry| {
                        g.triangles
                            .iter()
                            .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                            .sum::<f64>()
                    };
                    let area = original
                        .triangles
                        .iter()
                        .zip(&original.triangle_faces)
                        .filter(|(_, f)| target(&b, &original, **f) == Ok(face))
                        .map(|(t, _)| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
                        .sum::<f64>();
                    for distance in [-4., -5., -10.] {
                        b.face_extrusions = vec![Extrusion { face, distance }];
                        let cut = evaluate(&b).unwrap();
                        assert!(cut.is_solid);
                        Solid::try_new(vec![cut.shell.clone().unwrap()]).unwrap();
                        assert!(
                            (volume(&original) - volume(&cut) - area * 8.).abs() < 0.1,
                            "Through-cut volume must stop at the solid, regardless of overrun"
                        );
                        assert!(
                            !(0..cut.face_count).any(|f| target(&b, &cut, f) == Ok(face)),
                            "A through-cut has no pocket floor or emerging extrusion cap"
                        );
                        if shape == Shape::Circle
                            && kind == Kind::Chamfer
                            && face == FaceTarget::Top
                            && distance == -5.
                        {
                            let dir = tempfile::tempdir().unwrap();
                            let path = dir.path().join("through-cut.osk");
                            let mut project = crate::project::Project::new(
                                vec![b.clone()],
                                crate::viewport::Camera::default(),
                                true,
                                None,
                            );
                            // The through-cut consumed its picked cap. Face mode
                            // saves with no selection, rather than a stale slot.
                            project.selection_mode = crate::selection::SelectionMode::Face;
                            project.save(&path).unwrap();
                            assert_eq!(
                                crate::project::Project::load(&path).unwrap().bodies,
                                vec![b.clone()]
                            );
                            for format in [
                                crate::export::Format::Stl,
                                crate::export::Format::Glb,
                                crate::export::Format::Step,
                                crate::export::Format::Dxf,
                            ] {
                                assert!(
                                    !crate::export::encode(&[b.clone()], format)
                                        .unwrap()
                                        .is_empty()
                                );
                            }
                        }
                    }
                }
            }
        }
    }
    #[test]
    fn convex_planar_inward_sweeps_trim_angled_exits_and_disconnected_results() {
        use crate::components;
        use truck_modeling::{Deg, Matrix4};
        let volume = |g: &Geometry| {
            g.triangles
                .iter()
                .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                .sum::<f64>()
        };
        let mut body = Body::new(Shape::Rectangle);
        body.id = 1;
        body.width = 4.;
        body.depth = 3.;
        body.extrusion = 5.;
        body.scale = [1.5, 0.75, 1.2];
        let source = evaluate(&body).unwrap();
        let face = (0..source.face_count)
            .find(|f| target(&body, &source, *f) == Ok(FaceTarget::Top))
            .unwrap();
        let center =
            components::pivot(&source, crate::selection::SelectionMode::Face, Some(face)).unwrap();
        let rotation = Matrix4::from_translation(center.to_vec())
            * Matrix4::from_angle_x(Deg(15.))
            * Matrix4::from_translation(-center.to_vec());
        body = components::transformed(
            &body,
            &source,
            crate::selection::SelectionMode::Face,
            Some(face),
            rotation,
        )
        .unwrap();
        let g = evaluate(&body).unwrap();
        let plane = crate::snapping::Plane::face(&g, face).unwrap();
        let angle = 15_f64.to_radians();
        let area = 13.5 / angle.cos();
        for distance in [-0.4, -12.] {
            let edited = components::sweep_face(&body, &g, face, distance).unwrap();
            let result = evaluate(&edited).unwrap();
            let margin = plane.normal * 1e-4;
            let entry = builder::transformed(
                &g.shell.as_ref().unwrap()[face].inverse(),
                Matrix4::from_translation(margin),
            );
            let tool: Solid = builder::tsweep(&entry, plane.normal * distance - margin);
            let direct = crate::edge_treatments::subtract_convex(
                &g,
                &tool.boundaries()[0],
                g.surface_mapping().unwrap(),
            )
            .unwrap()
            .expect("The exact planar path must handle this case");
            let expected = if distance > -1. {
                81. - area * (-distance) + 0.5 * 6. * angle.tan() * distance * distance
            } else {
                let left_height = 6. - 1.125 * angle.tan();
                0.5 * 6. * left_height * left_height * angle.tan()
            };
            assert!(
                (volume(&result) - expected).abs() < 1e-6,
                "{distance}: {} vs {expected}",
                volume(&result)
            );
            assert!((volume(&direct) - expected).abs() < 1e-6);
            components::solid_parts(result.shell.as_ref().unwrap()).unwrap();
            assert!(
                result
                    .shell
                    .as_ref()
                    .unwrap()
                    .iter()
                    .all(|f| matches!(f.oriented_surface(), truck_modeling::Surface::Plane(_)))
            );
            let mut mesh = result.clone();
            components::weld_geometry(&mut mesh);
            components::boundary_shell(&mesh, true).unwrap();
            let mut model = crate::model::Model::from_bodies(vec![body.clone()]);
            model.commit(edited.clone()).unwrap();
            model.undo();
            assert_eq!(model.bodies, vec![body.clone()]);
            model.redo();
            assert_eq!(model.bodies, vec![edited.clone()]);
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("angled-cut.osk");
            crate::project::Project::new(
                vec![edited.clone()],
                crate::viewport::Camera::default(),
                true,
                Some(1),
            )
            .save(&path)
            .unwrap();
            assert_eq!(
                crate::project::Project::load(&path).unwrap().bodies,
                vec![edited.clone()]
            );
            for format in crate::export::Format::ALL {
                assert!(
                    !crate::export::encode(std::slice::from_ref(&edited), format)
                        .unwrap()
                        .is_empty()
                );
            }
        }
        let mut base = Body::new(Shape::Rectangle);
        base.id = 1;
        base.width = 4.;
        base.depth = 3.;
        base.extrusion = 5.;
        let target = evaluate(&base).unwrap();
        let mut tool = Body::new(Shape::Rectangle);
        tool.width = 1.;
        tool.depth = 8.;
        tool.extrusion = 9.;
        tool.position = [0., 0., -2.];
        let tool = crate::model::primitive_shell(&tool).unwrap();
        let split = crate::edge_treatments::subtract_convex(
            &target,
            &tool,
            target.surface_mapping().unwrap(),
        )
        .unwrap()
        .unwrap();
        assert!((volume(&split) - 45.).abs() < 1e-6);
        assert_eq!(
            split.face_count, 12,
            "Disconnected planar regions must remain separately selectable"
        );
        assert_eq!(
            components::solid_parts(split.shell.as_ref().unwrap())
                .unwrap()
                .len(),
            2
        );
        let mut circle = Body::new(Shape::Circle);
        circle.extrusion = 5.;
        let curved = evaluate(&circle).unwrap();
        assert!(
            crate::edge_treatments::subtract_convex(
                &curved,
                &tool,
                curved.surface_mapping().unwrap()
            )
            .unwrap()
            .is_none(),
            "Curved BReps must not be reduced to sampled planar polygons"
        );
    }
}
