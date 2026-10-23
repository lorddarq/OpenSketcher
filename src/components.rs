//! Authored component transforms. Base profiles remain editable; output is regenerated.
use crate::{
    model::{Body, Geometry, Shape},
    selection::SelectionMode,
};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use truck_modeling::{
    Edge, EuclideanSpace, InnerSpace, Matrix3, Matrix4, Point3, Shell, Solid, SquareMatrix,
    Transform, Vertex, Wire, builder,
};

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Topology {
    pub edges: Vec<[usize; 2]>,
    pub faces: Vec<Vec<usize>>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentEdit {
    pub mode: SelectionMode,
    pub element: usize,
    pub topology: Topology,
    pub matrix: [[f64; 4]; 4],
}

pub fn vertices(g: &Geometry, mode: SelectionMode, element: Option<usize>) -> Vec<usize> {
    match (mode, element) {
        (SelectionMode::Volume, _) => (0..g.points.len()).collect(),
        (SelectionMode::Point, Some(i)) if i < g.points.len() => vec![i],
        (SelectionMode::Edge, Some(i)) => g
            .topology
            .edges
            .get(i)
            .map(|v| v.to_vec())
            .unwrap_or_default(),
        (SelectionMode::Face, Some(i)) => {
            let mut vertices: Vec<_> = g
                .topology
                .faces
                .get(i)
                .into_iter()
                .flatten()
                .flat_map(|edge| g.topology.edges[*edge])
                .collect();
            vertices.sort_unstable();
            vertices.dedup();
            vertices
        }
        _ => Vec::new(),
    }
}

pub fn pivot(g: &Geometry, mode: SelectionMode, element: Option<usize>) -> Option<Point3> {
    let points: Vec<_> = match mode {
        SelectionMode::Face => g
            .triangles
            .iter()
            .zip(&g.triangle_faces)
            .filter(|(_, face)| Some(**face) == element)
            .flat_map(|(t, _)| t.iter().copied())
            .collect(),
        SelectionMode::Edge => g
            .edges
            .iter()
            .zip(&g.edge_groups)
            .filter(|(_, edge)| Some(**edge) == element)
            .flat_map(|(e, _)| e.iter().copied())
            .collect(),
        _ => vertices(g, mode, element)
            .iter()
            .map(|i| g.points[*i])
            .collect(),
    };
    if points.is_empty() {
        return None;
    }
    Some(Point3::from(std::array::from_fn::<_, 3, _>(|axis| {
        let low = points.iter().map(|p| p[axis]).fold(f64::INFINITY, f64::min);
        let high = points
            .iter()
            .map(|p| p[axis])
            .fold(f64::NEG_INFINITY, f64::max);
        (low + high) / 2.
    })))
}

pub fn transformed(
    body: &Body,
    g: &Geometry,
    mode: SelectionMode,
    element: Option<usize>,
    matrix: Matrix4,
) -> Result<Body, String> {
    let mut result = body.clone();
    if matrix == Matrix4::identity() {
        return Ok(result);
    }
    if mode == SelectionMode::Volume {
        result.position = matrix.transform_point(Point3::from(body.position)).into();
        let delta = Matrix3::from_cols(
            matrix.x.truncate(),
            matrix.y.truncate(),
            matrix.z.truncate(),
        );
        if delta != Matrix3::identity() {
            let rotation = Matrix3::from_cols(
                matrix.x.truncate(),
                matrix.y.truncate(),
                matrix.z.truncate(),
            ) * Matrix3::from_angle_z(truck_modeling::Deg(body.rotation[2]))
                * Matrix3::from_angle_y(truck_modeling::Deg(body.rotation[1]))
                * Matrix3::from_angle_x(truck_modeling::Deg(body.rotation[0]));
            result.rotation = crate::regions::rotation_angles(rotation);
        }
    } else {
        let element = element.ok_or("Select a component first.")?;
        if vertices(g, mode, Some(element)).is_empty() {
            return Err("Selected component no longer exists.".into());
        }
        if mode == SelectionMode::Point
            && (matrix.transform_point(g.points[element]) - g.points[element]).magnitude2() < 1e-18
        {
            return Ok(result);
        }
        let placement = crate::regions::transform(body);
        result.component_edits.push(ComponentEdit {
            mode,
            element,
            topology: g.topology.clone(),
            matrix: (placement.invert().ok_or("Invalid object transform.")? * matrix * placement)
                .into(),
        });
    }
    result.validate()?;
    Ok(result)
}

pub fn validate(edits: &[ComponentEdit]) -> Result<(), String> {
    if edits.len() > 100 {
        return Err("A body supports up to 100 component edits.".into());
    }
    for edit in edits {
        let m = Matrix4::from(edit.matrix);
        if edit.mode == SelectionMode::Volume
            || edit.topology.edges.len() > 8192
            || edit.topology.faces.len() > 8192
            || edit.topology.faces.iter().map(Vec::len).sum::<usize>() > 32768
            || edit
                .matrix
                .iter()
                .flatten()
                .any(|v| !v.is_finite() || v.abs() > 1e6)
            || [m.x.w, m.y.w, m.z.w].iter().any(|v| v.abs() > 1e-10)
            || (m.w.w - 1.).abs() > 1e-10
            || m.invert().is_none()
        {
            return Err("Invalid component transform or topology reference.".into());
        }
    }
    Ok(())
}

pub fn apply(body: &Body, g: &mut Geometry) -> Result<(), String> {
    if body.component_edits.is_empty() {
        return Ok(());
    }
    let placement = crate::regions::transform(body);
    map_geometry(g, placement.invert().ok_or("Invalid object transform.")?);
    weld_geometry(g);
    for edit in &body.component_edits {
        if edit.topology != g.topology {
            return Err("An upstream edit changed component topology. Undo it or remove component edits before changing topology.".into());
        }
        let selected = vertices(g, edit.mode, Some(edit.element));
        if selected.is_empty() {
            return Err("A component transform references a missing element.".into());
        }
        let matrix = Matrix4::from(edit.matrix);
        let old = g.clone();
        let weight = |i: usize| if selected.contains(&i) { 1. } else { 0. };
        let mut lengths = vec![0.; g.edge_count];
        for (edge, group) in old.edges.iter().zip(&old.edge_groups) {
            lengths[*group] += (edge[1] - edge[0]).magnitude();
        }
        let mut traveled = vec![0.; g.edge_count];
        let mut edge_weights = Vec::new();
        for (edge, group) in old.edges.iter().zip(&old.edge_groups) {
            let [a, b] = old.topology.edges[*group];
            let length = (edge[1] - edge[0]).magnitude();
            let fraction = |distance: f64| {
                if lengths[*group] > 1e-12 {
                    distance / lengths[*group]
                } else {
                    0.
                }
            };
            let blend = |t: f64| weight(a) * (1. - t) + weight(b) * t;
            edge_weights.push([
                blend(fraction(traveled[*group])),
                blend(fraction(traveled[*group] + length)),
            ]);
            traveled[*group] += length;
        }
        let moved = |p: Point3, w: f64| p + (matrix.transform_point(p) - p) * w;
        for (i, p) in g.points.iter_mut().enumerate() {
            *p = moved(*p, weight(i));
        }
        for (edge, weights) in g.edges.iter_mut().zip(&edge_weights) {
            for i in 0..2 {
                edge[i] = moved(edge[i], weights[i]);
            }
        }
        for (triangle, face) in g.triangles.iter_mut().zip(&old.triangle_faces) {
            for p in triangle {
                let mut sum = 0.;
                let mut influence = 0.;
                let mut boundary = None;
                for ((edge, group), weights) in
                    old.edges.iter().zip(&old.edge_groups).zip(&edge_weights)
                {
                    if !old.topology.faces[*face].contains(group) {
                        continue;
                    }
                    let ab = edge[1] - edge[0];
                    let t = ((*p - edge[0]).dot(ab) / ab.magnitude2().max(1e-20)).clamp(0., 1.);
                    let distance = (*p - (edge[0] + ab * t)).magnitude2();
                    let w = weights[0] * (1. - t) + weights[1] * t;
                    if distance < 1e-16 {
                        boundary = Some(w);
                        break;
                    }
                    sum += 1. / distance;
                    influence += w / distance;
                }
                *p = moved(
                    *p,
                    boundary.unwrap_or_else(|| if sum > 0. { influence / sum } else { 0. }),
                );
            }
        }
        weld_geometry(g);
        validate_boundary(g, body.extrusion > 0. && body.shape != Shape::Line)?;
    }
    map_geometry(g, placement);
    Ok(())
}
fn map_geometry(g: &mut Geometry, matrix: Matrix4) {
    for p in g
        .points
        .iter_mut()
        .chain(g.edges.iter_mut().flatten())
        .chain(g.triangles.iter_mut().flatten())
    {
        *p = matrix.transform_point(*p);
    }
}

// Truck tessellates neighboring analytic surfaces independently. Reconcile tiny local-space
// projection differences against shared boundary samples before and after deformation.
pub fn weld_geometry(g: &mut Geometry) {
    let mut buckets: HashMap<[i64; 3], Vec<Point3>> = HashMap::new();
    for p in g
        .points
        .iter_mut()
        .chain(g.edges.iter_mut().flatten())
        .chain(g.triangles.iter_mut().flatten())
    {
        let key = [p.x, p.y, p.z].map(|v| (v / 1e-6).floor() as i64);
        let mut matched = None;
        'search: for x in -1..=1 {
            for y in -1..=1 {
                for z in -1..=1 {
                    if let Some(points) = buckets.get(&[key[0] + x, key[1] + y, key[2] + z]) {
                        for candidate in points {
                            if (*p - *candidate).magnitude2() < 1e-12 {
                                matched = Some(*candidate);
                                break 'search;
                            }
                        }
                    }
                }
            }
        }
        if let Some(point) = matched {
            *p = point;
        } else {
            buckets.entry(key).or_default().push(*p);
        }
    }
}

fn validate_boundary(g: &Geometry, closed: bool) -> Result<(), String> {
    boundary_shell(g, closed).map(|_| ())
}

pub fn boundary_shell(g: &Geometry, closed: bool) -> Result<Shell, String> {
    if g.points
        .iter()
        .chain(g.edges.iter().flatten())
        .chain(g.triangles.iter().flatten())
        .any(|p| {
            [p.x, p.y, p.z]
                .iter()
                .any(|v| !v.is_finite() || v.abs() > 1e6)
        })
    {
        return Err("Component transform exceeds supported coordinates.".into());
    }
    if g.edges.iter().any(|e| (e[1] - e[0]).magnitude2() < 1e-16) {
        return Err("Component transform collapses an edge.".into());
    }
    let mut vertices: HashMap<[i64; 3], Vertex> = HashMap::new();
    let mut edges: HashMap<([i64; 3], [i64; 3]), Edge> = HashMap::new();
    let mut shell = Shell::default();
    for t in &g.triangles {
        if (t[1] - t[0]).cross(t[2] - t[0]).magnitude2() < 1e-20 {
            return Err("Component transform collapses a face.".into());
        }
        let keys = t.map(|p| [p.x, p.y, p.z].map(|v| (v * 1e7).round() as i64));
        for i in 0..3 {
            vertices
                .entry(keys[i])
                .or_insert_with(|| builder::vertex(t[i]));
        }
        let wire: Wire = (0..3)
            .map(|i| {
                let a = keys[i];
                let b = keys[(i + 1) % 3];
                let key = if a < b { (a, b) } else { (b, a) };
                let edge = edges
                    .entry(key)
                    .or_insert_with(|| builder::line(&vertices[&key.0], &vertices[&key.1]));
                if a < b { edge.clone() } else { edge.inverse() }
            })
            .collect();
        shell.push(
            builder::try_attach_plane(&[wire])
                .map_err(|e| format!("Cannot rebuild adjacent faces: {e}"))?,
        );
    }
    if !shell.is_geometric_consistent() {
        return Err("Component transform creates inconsistent adjacent faces.".into());
    }
    if closed && !shell.is_empty() {
        for part in shell.connected_components() {
            let volume: f64 = part
                .face_iter()
                .map(|face| {
                    let wire = &face.boundaries()[0];
                    let a = wire[0].front().point().to_vec();
                    let b = wire[1].front().point().to_vec();
                    let c = wire[2].front().point().to_vec();
                    a.dot(b.cross(c)) / 6.
                })
                .sum();
            if volume <= 1e-10 {
                return Err("Component transform collapses or inverts a solid region.".into());
            }
            Solid::try_new(vec![part])
                .map_err(|e| format!("Component transform would open or tear the body: {e}"))?;
        }
    }
    Ok(shell)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        model::{Model, evaluate, evaluate_scene},
        project::Project,
        viewport::Camera,
    };
    use truck_modeling::{Deg, Vector3};
    #[test]
    fn all_selection_modes_transform_with_history_persistence_and_shared_boundaries() {
        for shape in [
            Shape::Rectangle,
            Shape::Circle,
            Shape::Semicircle,
            Shape::Line,
        ] {
            let mut body = Body::new(shape);
            body.id = 1;
            body.extrusion = 2.;
            let g = evaluate(&body).unwrap();
            for mode in [
                SelectionMode::Volume,
                SelectionMode::Face,
                SelectionMode::Edge,
                SelectionMode::Point,
            ] {
                let element = if mode == SelectionMode::Volume {
                    None
                } else {
                    Some(0)
                };
                for rotate in [false, true] {
                    let center = pivot(&g, mode, element).unwrap() + Vector3::new(0.3, 0.2, 0.1);
                    let matrix = if rotate {
                        Matrix4::from_translation(center.to_vec())
                            * Matrix4::from_angle_z(Deg(8.))
                            * Matrix4::from_translation(-center.to_vec())
                    } else {
                        Matrix4::from_translation(Vector3::new(0.15, 0.1, 0.2))
                    };
                    let edited = transformed(&body, &g, mode, element, matrix).unwrap();
                    let mut model = Model::from_bodies(vec![body.clone()]);
                    model
                        .commit(edited.clone())
                        .unwrap_or_else(|e| panic!("{shape:?} {mode:?} rotate={rotate}: {e}"));
                    let output = evaluate_scene(&model.bodies).unwrap();
                    let chosen = vertices(&g, mode, element);
                    for i in 0..g.points.len() {
                        let expected = if chosen.contains(&i) {
                            matrix.transform_point(g.points[i])
                        } else {
                            g.points[i]
                        };
                        assert!(
                            (output[0].1.points[i] - expected).magnitude() < 1e-6,
                            "{shape:?} {mode:?}"
                        );
                    }
                    assert_eq!(output[0].1.topology, g.topology);
                    let directory = tempfile::tempdir().unwrap();
                    let path = directory.path().join("gizmo.osk");
                    Project::new(model.bodies.clone(), Camera::default(), true, Some(1))
                        .save(&path)
                        .unwrap();
                    let loaded = Project::load(&path).unwrap();
                    assert_eq!(loaded.bodies, model.bodies);
                    let regenerated = evaluate_scene(&loaded.bodies).unwrap();
                    assert_eq!(regenerated[0].1.points, output[0].1.points);
                    model.undo();
                    assert_eq!(model.bodies[0], body);
                    model.redo();
                    assert_eq!(model.bodies[0], edited);
                }
            }
        }
        let mut body = Body::new(Shape::Rectangle);
        body.id = 1;
        body.extrusion = 2.;
        let geometry = evaluate(&body).unwrap();
        let mut invalid = transformed(
            &body,
            &geometry,
            SelectionMode::Point,
            Some(0),
            Matrix4::from_translation(Vector3::unit_x() * 0.1),
        )
        .unwrap();
        invalid.component_edits[0].topology.edges.clear();
        let mut model = Model::from_bodies(vec![body.clone()]);
        assert!(model.commit(invalid).is_err());
        assert_eq!(model.bodies[0], body);
        assert!(!model.can_undo());
        let neighbor = geometry
            .topology
            .edges
            .iter()
            .find(|edge| edge.contains(&0))
            .unwrap();
        let other = if neighbor[0] == 0 {
            neighbor[1]
        } else {
            neighbor[0]
        };
        let collapsed = transformed(
            &body,
            &geometry,
            SelectionMode::Point,
            Some(0),
            Matrix4::from_translation(geometry.points[other] - geometry.points[0]),
        )
        .unwrap();
        assert!(model.commit(collapsed).is_err());
        assert_eq!(model.bodies[0], body);
        assert!(!model.can_undo());
        // World-axis rotation composes with an existing, nonuniformly scaled pose.
        let mut posed = body.clone();
        posed.position = [2., 3., 4.];
        posed.rotation = [20., 35., 70.];
        posed.scale = [2., 0.5, 1.5];
        let source = evaluate(&posed).unwrap();
        let center = Vector3::new(1., -2., 0.);
        let matrix = Matrix4::from_translation(center)
            * Matrix4::from_angle_x(Deg(30.))
            * Matrix4::from_translation(-center);
        let rotated = transformed(&posed, &source, SelectionMode::Volume, None, matrix).unwrap();
        let result = evaluate(&rotated).unwrap();
        for (a, b) in source.points.iter().zip(&result.points) {
            assert!((matrix.transform_point(*a) - *b).magnitude() < 1e-6);
        }
        // An outer region with a notch also supports face editing, while deleting
        // its cutting profile must not invalidate saved component references.
        let mut outer = Body::new(Shape::Rectangle);
        outer.id = 1;
        outer.width = 6.;
        outer.depth = 4.;
        outer.extrusion = 2.;
        let mut inner = Body::new(Shape::Rectangle);
        inner.id = 2;
        inner.cuts_profiles = true;
        inner.width = 1.;
        inner.position = [-2.5, 0., 0.];
        let mut split = Model::from_bodies(vec![outer.clone(), inner]);
        let scene = evaluate_scene(&split.bodies).unwrap();
        let source = &scene[0].1;
        let top = source
            .triangles
            .iter()
            .zip(&source.triangle_faces)
            .find(|(t, _)| t.iter().all(|p| (p.z - 2.).abs() < 1e-7))
            .map(|(_, face)| *face)
            .unwrap();
        let edited = transformed(
            &outer,
            source,
            SelectionMode::Face,
            Some(top),
            Matrix4::from_translation(Vector3::unit_z() * 0.5),
        )
        .unwrap();
        split.commit(edited).unwrap();
        let before = split.bodies.clone();
        assert!(split.delete(2).is_err());
        assert_eq!(split.bodies, before);
        // A single point rotated about itself is a no-op, not an owner-body rotation.
        let p = geometry.points[0];
        let matrix = Matrix4::from_translation(p.to_vec())
            * Matrix4::from_angle_z(Deg(30.))
            * Matrix4::from_translation(-p.to_vec());
        assert_eq!(
            transformed(&body, &geometry, SelectionMode::Point, Some(0), matrix).unwrap(),
            body
        );
    }
}
