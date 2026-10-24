//! Exact complete-cap translation with a fixed opposite section.
use crate::{components, model::Geometry, selection::SelectionMode, snapping::Plane};
use truck_modeling::{EuclideanSpace, InnerSpace, Matrix4, SquareMatrix, Vector3, Vector4};

/// Return an exact affine section map only when all selected vertices form a
/// complete planar cap and every remaining vertex is on its opposite plane.
/// Intermediate sections and partial cap/rim selections use their existing paths.
pub(crate) fn translation(
    g: &Geometry,
    selected: &[usize],
    matrix: Matrix4,
) -> Result<Option<Matrix4>, String> {
    if !g.is_solid || g.shell.is_none() || selected.is_empty() || selected.len() == g.points.len() {
        return Ok(None);
    }
    if (matrix.x - Vector4::unit_x()).magnitude() > 1e-12
        || (matrix.y - Vector4::unit_y()).magnitude() > 1e-12
        || (matrix.z - Vector4::unit_z()).magnitude() > 1e-12
    {
        return Ok(None);
    }
    let plane = (0..g.face_count).find_map(|face| {
        let points = components::vertices(g, SelectionMode::Face, Some(face));
        if points.is_empty() || !points.iter().all(|v| selected.contains(v)) {
            return None;
        }
        Plane::face(g, face).filter(|plane| {
            selected
                .iter()
                .all(|i| (g.points[*i] - plane.origin).dot(plane.normal).abs() < 1e-7)
        })
    });
    let Some(plane) = plane else {
        return Ok(None);
    };
    translation_from_plane(g, selected, matrix, plane)
}

/// Shared cap reconstruction for a planar boundary with an independently deformed interior.
pub(crate) fn translation_from_plane(
    g: &Geometry,
    selected: &[usize],
    matrix: Matrix4,
    plane: Plane,
) -> Result<Option<Matrix4>, String> {
    let remaining: Vec<_> = g
        .points
        .iter()
        .enumerate()
        .filter(|(i, _)| !selected.contains(i))
        .map(|(_, p)| (*p - plane.origin).dot(plane.normal))
        .collect();
    let Some(&opposite) = remaining.first() else {
        return Ok(None);
    };
    if opposite.abs() < 1e-7 || remaining.iter().any(|d| (*d - opposite).abs() > 1e-7) {
        return Ok(None);
    }
    let height = -opposite;
    let delta = matrix.w.truncate();
    if (height + delta.dot(plane.normal)) / height <= 1e-7 {
        return Err("Moving this cap consumes or crosses its fixed opposite section; use inward extrusion to subtract material.".into());
    }
    let n = plane.normal / height;
    let d = plane.origin.to_vec().dot(plane.normal) + opposite;
    let result = Matrix4::from_cols(
        (Vector3::unit_x() + delta * n.x).extend(0.),
        (Vector3::unit_y() + delta * n.y).extend(0.),
        (Vector3::unit_z() + delta * n.z).extend(0.),
        (-delta * d / height).extend(1.),
    );
    if result.invert().is_none() {
        return Err("The cap movement creates a degenerate section.".into());
    }
    Ok(Some(result))
}

/// Rebuild a complete planar cap and ruled neighbors from their retained control curves.
/// Both sections keep their own curve parameterization; display samples are never inputs.
pub(crate) fn rotate_cap(
    g: &Geometry,
    selected: &[usize],
    matrix: Matrix4,
) -> Result<Option<Geometry>, String> {
    use truck_modeling::{
        BSplineSurface, BoundedCurve, Curve, Invertible, NurbsSurface, ParametricCurve,
        ParametricSurface, Point3, Surface, Transform, Transformed, builder,
    };
    let Some(shell) = &g.shell else {
        return Ok(None);
    };
    if !g.is_solid || selected.is_empty() || selected.len() == g.points.len() {
        return Ok(None);
    }
    let cap = (0..g.face_count).find_map(|face| {
        let vertices = components::vertices(g, SelectionMode::Face, Some(face));
        (vertices.len() == selected.len() && vertices.iter().all(|v| selected.contains(v)))
            .then(|| Plane::face(g, face).map(|plane| (face, plane)))
            .flatten()
    });
    let Some((cap, plane)) = cap else {
        return Ok(None);
    };
    let mapping = g.surface_mapping()?;
    let cap_indices = (0..shell.len())
        .filter(|&i| mapping[i] == cap)
        .collect::<Vec<_>>();
    if cap_indices.len() != 1 {
        return Ok(None);
    }
    let on_cap = |p: Point3| (p - plane.origin).dot(plane.normal).abs() < 1e-7;
    // A rotation about a complete circular/elliptic cap's symmetry leaves the geometry unchanged.
    let symmetric = shell[cap_indices[0]].boundaries().iter().all(|wire| {
        g.circles.iter().any(|ring| {
            if !crate::face_extrusions::circle_on_wire(ring, wire) {
                return false;
            }
            let u = matrix.transform_vector(ring.u);
            let v = matrix.transform_vector(ring.v);
            (matrix.transform_point(ring.center) - ring.center).magnitude() < 1e-8
                && (0..3).all(|i| {
                    (0..3).all(|j| {
                        (u[i] * u[j] + v[i] * v[j] - ring.u[i] * ring.u[j] - ring.v[i] * ring.v[j])
                            .abs()
                            < 1e-8
                    })
                })
        })
    });
    if symmetric {
        return Ok(Some(g.clone()));
    }
    let opposite = (0..g.face_count)
        .filter(|&face| face != cap)
        .find_map(|face| {
            let vertices = components::vertices(g, SelectionMode::Face, Some(face));
            if vertices.iter().any(|v| selected.contains(v)) {
                return None;
            }
            let p = Plane::face(g, face)?;
            g.points
                .iter()
                .enumerate()
                .filter(|(i, _)| !selected.contains(i))
                .all(|(_, q)| (*q - p.origin).dot(p.normal).abs() < 1e-7)
                .then_some(p)
        });
    let Some(opposite) = opposite else {
        return Ok(None);
    };
    let height = (plane.origin - opposite.origin).dot(opposite.normal);
    if height.abs() < 1e-7 {
        return Ok(None);
    }
    let on_fixed = |p: Point3| (p - opposite.origin).dot(opposite.normal).abs() < 1e-7;
    let map_point = |p: Point3| {
        if on_cap(p) {
            matrix.transform_point(p)
        } else {
            p
        }
    };
    // Every retained edge must belong to a section or be a straight section connector.
    for edge in shell.edge_iter() {
        let curve = edge.curve();
        let (a, b) = curve.range_tuple();
        let samples = [curve.subs(a), curve.subs((a + b) / 2.), curve.subs(b)];
        if samples.iter().all(|p| on_cap(*p)) {
            if samples.iter().any(|p| {
                (matrix.transform_point(*p) - opposite.origin).dot(opposite.normal) / height <= 1e-7
            }) {
                return Err("Rotating this cap crosses its fixed opposite section.".into());
            }
        } else if !samples.iter().all(|p| on_fixed(*p))
            && !(matches!(curve, Curve::Line(_))
                && ((on_cap(samples[0]) && on_fixed(samples[2]))
                    || (on_fixed(samples[0]) && on_cap(samples[2]))))
        {
            return Ok(None);
        }
    }
    let mut surfaces = Vec::new();
    for face in shell.iter() {
        let points = face
            .edge_iter()
            .flat_map(|e| [e.front().point(), e.back().point()])
            .collect::<Vec<_>>();
        if points.iter().all(|p| on_cap(*p)) {
            surfaces.push(face.surface().transformed(matrix));
            continue;
        }
        if points.iter().all(|p| on_fixed(*p)) {
            surfaces.push(face.surface());
            continue;
        }
        let mut net = match face.surface() {
            Surface::NurbsSurface(net) => net,
            Surface::BSplineSurface(net) => NurbsSurface::new(BSplineSurface::new(
                net.knot_vecs().clone(),
                net.control_points()
                    .iter()
                    .map(|row| row.iter().map(|p| p.to_vec().extend(1.)).collect())
                    .collect(),
            )),
            Surface::Plane(p) => {
                let bounds = face.absolute_boundaries();
                if bounds.len() != 1 || bounds[0].len() != 4 {
                    return Ok(None);
                }
                let a = bounds[0]
                    .iter()
                    .find(|e| on_cap(e.front().point()) && on_cap(e.back().point()));
                let b = bounds[0]
                    .iter()
                    .find(|e| on_fixed(e.front().point()) && on_fixed(e.back().point()));
                let (Some(a), Some(b)) = (a, b) else {
                    return Ok(None);
                };
                let Surface::NurbsSurface(mut net) = builder::homotopy(a, &b.inverse()).surface()
                else {
                    unreachable!()
                };
                if net.normal(0.5, 0.5).dot(p.normal()) < 0. {
                    net.invert();
                }
                net
            }
            _ => return Ok(None),
        };
        let p = net.control_points();
        let pair = |a: Vector4, b: Vector4| {
            if a.w <= 0. || b.w <= 0. || (a.w - b.w).abs() > 1e-10 * a.w.max(b.w).max(1.) {
                return false;
            }
            let a = Point3::from_vec(a.truncate() / a.w);
            let b = Point3::from_vec(b.truncate() / b.w);
            on_cap(a) && on_fixed(b) || on_fixed(a) && on_cap(b)
        };
        let ruled =
            net.udegree() == 1 && p.len() == 2 && p[0].iter().zip(&p[1]).all(|(a, b)| pair(*a, *b))
                || net.vdegree() == 1 && p.iter().all(|row| row.len() == 2 && pair(row[0], row[1]));
        if !ruled {
            return Ok(None);
        }
        for p in net.control_points_mut() {
            let point = Point3::from_vec(p.truncate() / p.w);
            if on_cap(point) {
                let moved = matrix.transform_point(point);
                if (moved - opposite.origin).dot(opposite.normal) / height <= 1e-7 {
                    return Err("Rotating this cap consumes the fixed opposite section.".into());
                }
                *p = moved.to_vec().extend(1.) * p.w;
            }
        }
        surfaces.push(Surface::NurbsSurface(net));
    }
    let rebuilt = shell.mapped(
        |p| map_point(*p),
        |curve| {
            let (a, b) = curve.range_tuple();
            if [a, (a + b) / 2., b].iter().all(|t| on_cap(curve.subs(*t))) {
                curve.transformed(matrix)
            } else if let Curve::Line(line) = curve {
                Curve::Line(truck_modeling::Line(map_point(line.0), map_point(line.1)))
            } else {
                curve.clone()
            }
        },
        Clone::clone,
    );
    for (face, surface) in rebuilt.iter().zip(surfaces) {
        face.set_surface(surface);
    }
    if !rebuilt.is_geometric_consistent() {
        return Err("Cap rotation could not retain consistent shared boundaries.".into());
    }
    components::solid_parts(&rebuilt)?;
    let mut result = crate::model::tessellate(&rebuilt)?;
    // Preserve the source's actual logical curved walls across artificial patch seams.
    let mut logical = g.clone();
    logical.group_cylinder_patches();
    let roles = mapping
        .iter()
        .map(|f| logical.face_group(*f))
        .collect::<Vec<_>>();
    crate::edge_treatments::merge_surfaces(&mut result, &roles);
    result.is_solid = true;
    let rings = g
        .circles
        .iter()
        .map(|ring| {
            let mut ring = ring.clone();
            if on_cap(ring.center) {
                ring.center = matrix.transform_point(ring.center);
                ring.u = matrix.transform_vector(ring.u);
                ring.v = matrix.transform_vector(ring.v);
            }
            ring
        })
        .collect::<Vec<_>>();
    result.retain_circles(&rings);
    crate::soft_selection::validate_intersections(&result)?;
    Ok(Some(result))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        edge_treatments::FaceTarget,
        model::{Body, Model, Shape, evaluate, evaluate_scene},
        project::Project,
        viewport::Camera,
    };
    use truck_modeling::Transform;
    #[test]
    fn rotated_cylinder_caps_retain_calculated_surfaces() {
        use truck_modeling::{Point3, Rad, Surface};
        let mut body = Body::new(Shape::Circle);
        body.id = 1;
        body.width = 4.;
        body.extrusion = 3.;
        let source = evaluate(&body).unwrap();
        let face = (0..source.face_count)
            .find(|&f| crate::edge_treatments::target(&body, &source, f) == Ok(FaceTarget::Top))
            .unwrap();
        let pivot = Point3::new(0., 0., 3.);
        for axis in [Vector3::unit_z(), Vector3::unit_x()] {
            let matrix = Matrix4::from_translation(pivot.to_vec())
                * Matrix4::from_axis_angle(axis, Rad(0.3))
                * Matrix4::from_translation(-pivot.to_vec());
            let changed =
                components::transformed(&body, &source, SelectionMode::Face, Some(face), matrix)
                    .unwrap();
            let output = evaluate(&changed).unwrap();
            let shell = output.shell.as_ref().expect(
                "Cap rotation must retain calculated surfaces, not deform display triangles",
            );
            assert!(shell.is_geometric_consistent());
            assert!(
                shell
                    .iter()
                    .any(|f| matches!(f.surface(), Surface::NurbsSurface(_)))
            );
            assert_eq!(output.triangle_normals.len(), output.triangles.len());
            if axis == Vector3::unit_z() {
                assert_eq!(
                    output.triangles, source.triangles,
                    "Axial cap rotation must not pinch a rotationally symmetric cylinder"
                );
            }
        }
    }
    #[test]
    fn translated_curved_caps_keep_exact_geometry_and_the_opposite_cap_fixed() {
        for shape in [Shape::Circle, Shape::Semicircle] {
            for cap in [FaceTarget::Top, FaceTarget::Bottom] {
                for affine in [false, true] {
                    let mut source = Body::new(shape);
                    source.id = 1;
                    source.width = 4.;
                    source.extrusion = 3.;
                    if affine {
                        source.position = [2., -3., 1.];
                        source.rotation = [15., 25., 35.];
                        source.scale = [1.4, 0.7, 1.2];
                    }
                    let g = evaluate(&source).unwrap();
                    let face = (0..g.face_count)
                        .find(|f| crate::edge_treatments::target(&source, &g, *f) == Ok(cap))
                        .unwrap();
                    let vertices = components::vertices(&g, SelectionMode::Face, Some(face));
                    let shift = Vector3::new(0.3, -0.2, 0.4);
                    let matrix = Matrix4::from_translation(shift);
                    let moved = components::transformed(
                        &source,
                        &g,
                        SelectionMode::Face,
                        Some(face),
                        matrix,
                    )
                    .unwrap();
                    assert!(moved.component_edits[0].section_transform);
                    let out = evaluate(&moved).unwrap();
                    assert!(out.shell.is_some());
                    assert_eq!(out.topology, g.topology);
                    assert!(out.shell.as_ref().unwrap().is_geometric_consistent());
                    for (i, (before, after)) in g.points.iter().zip(&out.points).enumerate() {
                        let expected = if vertices.contains(&i) {
                            matrix.transform_point(*before)
                        } else {
                            *before
                        };
                        assert!((*after - expected).magnitude() < 1e-7);
                    }
                    assert_eq!(out.circles.len(), g.circles.len());
                    if shape == Shape::Circle {
                        assert_eq!(out.face_elements(1), vec![1, 2, 3]);
                        assert_eq!(
                            out.internal_edges.len(),
                            3,
                            "Oblique cylinders retain one modeling wall"
                        );
                    }
                    let mut model = Model::from_bodies(vec![source.clone()]);
                    model.commit(moved.clone()).unwrap();
                    model.undo();
                    assert_eq!(model.bodies, vec![source]);
                    model.redo();
                    assert_eq!(model.bodies, vec![moved.clone()]);
                    let project =
                        Project::new(vec![moved.clone()], Camera::default(), true, Some(1));
                    let file = tempfile::NamedTempFile::new().unwrap().into_temp_path();
                    project.save(file.as_ref()).unwrap();
                    assert_eq!(
                        Project::load(file.as_ref()).unwrap().bodies,
                        vec![moved.clone()]
                    );
                    for format in crate::export::Format::ALL {
                        assert!(
                            !crate::export::encode(std::slice::from_ref(&moved), format)
                                .unwrap()
                                .is_empty()
                        );
                    }
                    let mut old = serde_json::to_value(&project).unwrap();
                    old["modeling_version"] = 27.into();
                    assert!(
                        serde_json::from_value::<Project>(old)
                            .unwrap()
                            .validate()
                            .unwrap_err()
                            .contains("28")
                    );
                }
            }
        }
    }
    #[test]
    fn through_pipe_and_concave_boolean_caps_remain_analytic() {
        use crate::booleans::{Boolean, Kind};
        for pipe in [false, true] {
            let mut a = Body::new(if pipe {
                Shape::Circle
            } else {
                Shape::Rectangle
            });
            a.id = 1;
            a.width = 6.;
            a.depth = 6.;
            a.extrusion = 3.;
            let mut b = Body::new(if pipe {
                Shape::Circle
            } else {
                Shape::Rectangle
            });
            b.id = 2;
            b.width = if pipe { 2. } else { 4. };
            b.depth = 2.;
            b.extrusion = 3.;
            b.position[0] = if pipe { 0. } else { 2. };
            let mut bodies = vec![a, b];
            let scene = evaluate_scene(&bodies).unwrap();
            let mut difference = Body::new(Shape::Rectangle);
            difference.id = 3;
            difference.boolean = Some(Boolean::new(
                Kind::Subtract,
                &[(1, None), (2, None)],
                SelectionMode::Volume,
                &scene,
            ));
            bodies.push(difference.clone());
            let scene = evaluate_scene(&bodies).unwrap();
            let g = &scene[2].1;
            let cap = (0..g.face_count)
                .find(|face| Plane::face(g, *face).is_some_and(|p| p.normal.z > 0.99))
                .unwrap();
            let moved = components::transformed(
                &difference,
                g,
                SelectionMode::Face,
                Some(cap),
                Matrix4::from_translation(Vector3::new(0.2, 0.1, 0.4)),
            )
            .unwrap();
            assert!(
                moved.component_edits.last().unwrap().section_transform,
                "pipe={pipe}"
            );
            bodies[2] = moved;
            let out = evaluate_scene(&bodies).unwrap();
            assert!(out[2].1.shell.is_some());
            assert!(out[2].1.shell.as_ref().unwrap().is_geometric_consistent());
            assert_eq!(out[2].1.topology, g.topology);
            for format in crate::export::Format::ALL {
                assert!(!crate::export::encode(&bodies, format).unwrap().is_empty());
            }
        }
    }

    #[test]
    fn complete_rims_use_the_cap_map_and_consuming_moves_roll_back() {
        let mut source = Body::new(Shape::Circle);
        source.id = 1;
        source.width = 4.;
        source.extrusion = 3.;
        let g = evaluate(&source).unwrap();
        let edge = (0..g.edge_count)
            .find(|edge| {
                !g.internal_edges.contains(edge)
                    && components::vertices(&g, SelectionMode::Edge, Some(*edge))
                        .iter()
                        .all(|v| (g.points[*v].z - 3.).abs() < 1e-7)
            })
            .unwrap();
        let moved = components::transformed(
            &source,
            &g,
            SelectionMode::Edge,
            Some(edge),
            Matrix4::from_translation(Vector3::new(0.4, 0., 0.5)),
        )
        .unwrap();
        assert!(moved.component_edits[0].section_transform);
        assert!(evaluate(&moved).unwrap().shell.is_some());
        let mut model = Model::from_bodies(vec![source.clone()]);
        assert!(
            components::transformed(
                &source,
                &g,
                SelectionMode::Edge,
                Some(edge),
                Matrix4::from_translation(-Vector3::unit_z() * 4.)
            )
            .unwrap_err()
            .contains("opposite section")
        );
        assert_eq!(model.bodies, vec![source.clone()]);
        model.commit(moved.clone()).unwrap();
        let mut stale = moved;
        stale.component_edits[0].topology.edges.pop();
        assert!(model.edit(stale).is_err());
        let mut legacy = model.bodies[0].clone();
        legacy.component_edits[0].section_transform = false;
        assert!(
            evaluate(&legacy).unwrap().shell.is_none(),
            "Missing/false keeps legacy component deformation"
        );
    }
}
