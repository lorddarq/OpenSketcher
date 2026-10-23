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
                    let file = tempfile::NamedTempFile::new().unwrap();
                    project.save(file.path()).unwrap();
                    assert_eq!(
                        Project::load(file.path()).unwrap().bodies,
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
