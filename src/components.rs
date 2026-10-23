//! Authored component transforms. Base profiles remain editable; output is regenerated.
use crate::{
    model::{Body, Geometry, Shape},
    selection::SelectionMode,
};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use truck_modeling::{
    Edge, EuclideanSpace, InnerSpace, Matrix3, Matrix4, Point3, Shell, Solid, SquareMatrix,
    Transform, Vector3, Vertex, Wire, builder,
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
    /// False preserves legacy edits on the source; true edits the evaluated result.
    #[serde(default)]
    pub evaluated: bool,
    pub mode: SelectionMode,
    pub element: usize,
    #[serde(default)]
    pub elements: Vec<usize>,
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
    transformed_many(
        body,
        g,
        mode,
        &element.into_iter().collect::<Vec<_>>(),
        matrix,
    )
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OffsetTarget {
    Plane(crate::edge_treatments::FaceTarget),
    CylinderWall,
}
impl OffsetTarget {
    pub fn label(self) -> &'static str {
        match self {
            Self::Plane(face) => face.label(),
            Self::CylinderWall => "Cylinder wall",
        }
    }
}
pub fn offset_target(body: &Body, g: &Geometry, face: usize) -> Result<OffsetTarget, String> {
    if let Ok(role) = crate::edge_treatments::target(body, g, face) {
        return Ok(OffsetTarget::Plane(role));
    }
    if body.shape == Shape::Circle
        && body.extrusion > 0.
        && body.component_edits.is_empty()
        && body.bore.is_none()
    {
        let inverse = crate::regions::transform(body)
            .invert()
            .ok_or("Invalid source placement.")?;
        let triangles: Vec<_> = g
            .triangles
            .iter()
            .zip(&g.triangle_faces)
            .filter(|(_, f)| **f == face)
            .map(|(t, _)| t.map(|p| inverse.transform_point(p)))
            .collect();
        if !triangles.is_empty()
            && triangles.iter().all(|t| {
                let n = (t[1] - t[0]).cross(t[2] - t[0]);
                n.z.abs() <= n.magnitude() * 1e-6
                    && t.iter()
                        .all(|p| (p.x.hypot(p.y) - body.width / 2.).abs() < 1e-5)
            })
        {
            return Ok(OffsetTarget::CylinderWall);
        }
    }
    Err("Select an original planar source face or cylindrical wall; transition/deformed faces require general surface retrimming.".into())
}
pub fn offset_source(body: &Body, target: OffsetTarget, distance: f64) -> Result<Body, String> {
    match target {
        OffsetTarget::Plane(face) => offset_source_face(body, face, distance),
        OffsetTarget::CylinderWall => {
            (|| {
            body.validate()?;
            if body.shape != Shape::Circle
                || body.extrusion <= 0.
                || body.bore.is_some()
                || !body.component_edits.is_empty()
                || !distance.is_finite()
            {
                return Err("Cylinder wall offset requires an undeformed circular solid and finite distance.".into());
            }
            if (body.scale[0] - body.scale[1]).abs() > 1e-9 * body.scale[0].max(body.scale[1]) {
                return Err("Cylinder wall offset requires equal X/Y scale. An ellipse's parallel offset is not another ellipse.".into());
            }
            let mut changed = body.clone();
            changed.width += 2. * distance / body.scale[0];
            changed.validate()?;
            Ok(changed)
            })().map_err(|reason:String|body.error(format!("Offset cylinder wall: {reason}")))
        }
    }
}

/// Edit an existing primitive's support plane without deforming its render mesh.
/// Distance is signed along the outward world normal; the opposite plane stays fixed.
pub fn offset_source_face(
    body: &Body,
    face: crate::edge_treatments::FaceTarget,
    distance: f64,
) -> Result<Body, String> {
    (|| {
    use crate::edge_treatments::FaceTarget;
    body.validate()?;
    if !distance.is_finite() {
        return Err("Face offset must be a finite distance.".into());
    }
    if !body.component_edits.is_empty() {
        return Err("Offset face currently requires an undeformed source solid; existing component edits cannot be retrimmed analytically yet.".into());
    }
    if body.shape == Shape::Line || body.extrusion <= 0. || !face.supported(body) {
        return Err(
            "Offset face supports original box faces and circular/semicircular solid caps.".into(),
        );
    }
    let placement = crate::regions::transform(body);
    let (normal, _) = face.plane(body);
    // Primitive placement is rotation * diagonal scale: this axis remains normal
    // to its support plane, even with nonuniform scale.
    let delta = distance / placement.transform_vector(normal).magnitude();
    let mut result = body.clone();
    let shift = match face {
        FaceTarget::Top | FaceTarget::Bottom => {
            result.extrusion += delta;
            if result.extrusion < 0.01 {
                return Err("Face offset would collapse or invert the solid. Leave at least 0.01 local units of thickness; passing through requires a Borer operation.".into());
            }
            if face == FaceTarget::Bottom {
                normal * delta
            } else {
                Vector3::new(0., 0., 0.)
            }
        }
        FaceTarget::Left | FaceTarget::Right => {
            result.width += delta;
            normal * (delta / 2.)
        }
        FaceTarget::Front | FaceTarget::Back => {
            result.depth += delta;
            normal * (delta / 2.)
        }
    };
    result.position = (Point3::from(body.position) + placement.transform_vector(shift)).into();
    result.validate()?;
    Ok(result)
    })().map_err(|reason:String|body.error(format!("Offset {}: {reason}",face.label())))
}
fn transformed_many(
    body: &Body,
    g: &Geometry,
    mode: SelectionMode,
    elements: &[usize],
    matrix: Matrix4,
) -> Result<Body, String> {
    let element = elements.first().copied();
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
            && elements.len() == 1
            && (matrix.transform_point(g.points[element]) - g.points[element]).magnitude2() < 1e-18
        {
            return Ok(result);
        }
        let placement = crate::regions::transform(body);
        result.component_edits.push(ComponentEdit {
            evaluated: true,
            mode,
            element,
            elements: elements.iter().skip(1).copied().collect(),
            topology: g.topology.clone(),
            matrix: (placement.invert().ok_or("Invalid object transform.")? * matrix * placement)
                .into(),
        });
    }
    result.validate()?;
    Ok(result)
}

pub fn transform_selection(
    bodies: &[Body],
    scene: &[(u64, Geometry)],
    mode: SelectionMode,
    picks: &[crate::selection::Pick],
    matrix: Matrix4,
) -> Result<Vec<Body>, String> {
    if let Some((id, element)) = picks
        .iter()
        .find(|p| !crate::selection::valid(**p, scene, mode))
    {
        return Err(format!(
            "Transform · body {id} · {} {} no longer exists.",
            mode.label(),
            element.map_or(String::new(), |e| e.saturating_add(1).to_string())
        ));
    }
    let mut result = bodies.to_vec();
    for body in &mut result {
        let selected: Vec<_> = picks.iter().filter(|p| p.0 == body.id).collect();
        if selected.is_empty() {
            continue;
        }
        if mode == SelectionMode::Volume
            && let Some(bore) = &body.bore
        {
            // The attached profile follows its selected parent once, rather than
            // receiving the same group movement a second time through its frame.
            if picks.iter().any(|p| p.0 == bore.target) {
                continue;
            }
            let target = bodies
                .iter()
                .find(|b| b.id == bore.target)
                .ok_or("Attached profile target no longer exists.")?;
            let resolved = crate::operators::resolve_profile(body, target, bore)?;
            let moved = transformed(&resolved, &Geometry::default(), mode, None, matrix)?;
            let delta = crate::operators::attachment_delta(target, bore)?;
            let mut source = transformed(
                &moved,
                &Geometry::default(),
                mode,
                None,
                delta.invert().ok_or("Invalid support placement.")?,
            )?;
            source.bore = body.bore.clone();
            *body = source;
            continue;
        }
        let geometry = &scene
            .iter()
            .find(|(id, _)| *id == body.id)
            .ok_or("Selected body no longer exists.")?
            .1;
        let elements: Vec<_> = selected.iter().filter_map(|p| p.1).collect();
        *body = if selected.len() == 1 {
            transformed(body, geometry, mode, selected[0].1, matrix)?
        } else {
            transformed_many(body, geometry, mode, &elements, matrix)?
        };
    }
    Ok(result)
}

pub fn validate(edits: &[ComponentEdit]) -> Result<(), String> {
    if edits.len() > 100 {
        return Err("A body supports up to 100 component edits.".into());
    }
    for (index, edit) in edits.iter().enumerate() {
        let m = Matrix4::from(edit.matrix);
        if edit.mode == SelectionMode::Volume
            || edit.elements.len() > 8192
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
            return Err(format!(
                "Component operation {} · {} {}: invalid transform or topology reference.",
                index + 1,
                edit.mode.label(),
                edit.element.saturating_add(1)
            ));
        }
    }
    Ok(())
}

pub fn has_source_edits(body: &Body) -> bool {
    body.component_edits.iter().any(|edit| !edit.evaluated)
}
pub fn apply(body: &Body, g: &mut Geometry) -> Result<(), String> {
    apply_stage(body, g, false)
}
pub fn apply_evaluated(body: &Body, g: &mut Geometry) -> Result<(), String> {
    apply_stage(body, g, true)
}
fn apply_stage(body: &Body, g: &mut Geometry, evaluated: bool) -> Result<(), String> {
    if !body
        .component_edits
        .iter()
        .any(|e| e.evaluated == evaluated)
    {
        return Ok(());
    }
    let placement = crate::regions::transform(body);
    map_geometry(g, placement.invert().ok_or("Invalid object transform.")?);
    weld_geometry(g);
    for (index, edit) in body
        .component_edits
        .iter()
        .enumerate()
        .filter(|(_, e)| e.evaluated == evaluated)
    {
        (|| {
        if edit.topology != g.topology {
            return Err("An upstream edit changed component topology. Undo it or remove component edits before changing topology.".into());
        }
        let mut selected = Vec::new();
        for element in std::iter::once(&edit.element).chain(&edit.elements) {
            let vertices = vertices(g, edit.mode, Some(*element));
            if vertices.is_empty() {
                return Err(format!("Missing {} {}.",edit.mode.label(),element.saturating_add(1)));
            }
            selected.extend(vertices);
        }
        selected.sort_unstable();
        selected.dedup();
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
        validate_boundary(g, if evaluated {old.is_solid} else {body.extrusion > 0. && body.shape != Shape::Line})?;
        Ok(())
        })().map_err(|reason:String|format!("Component operation {} · {} {}: {reason}",index+1,edit.mode.label(),edit.element.saturating_add(1)))?;
    }
    g.shell = None;
    g.circles.clear();
    map_geometry(g, placement);
    Ok(())
}
pub fn map_geometry(g: &mut Geometry, matrix: Matrix4) {
    if let Some(shell) = &mut g.shell {
        *shell = builder::transformed(shell, matrix);
    }
    for circle in &mut g.circles {
        circle.center = matrix.transform_point(circle.center);
        circle.u = matrix.transform_vector(circle.u);
        circle.v = matrix.transform_vector(circle.v);
    }

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
    // Two independently projected surfaces may each differ from their shared
    // kernel boundary by TOLERANCE, so their mutual separation can be twice that.
    let tolerance = 2. * truck_modeling::TOLERANCE;
    let mut buckets: HashMap<[i64; 3], Vec<Point3>> = HashMap::new();
    for p in g
        .points
        .iter_mut()
        .chain(g.edges.iter_mut().flatten())
        .chain(g.triangles.iter_mut().flatten())
    {
        let key = [p.x, p.y, p.z].map(|v| (v / tolerance).floor() as i64);
        let mut matched = None;
        'search: for x in -1..=1 {
            for y in -1..=1 {
                for z in -1..=1 {
                    if let Some(points) = buckets.get(&[key[0] + x, key[1] + y, key[2] + z]) {
                        for candidate in points {
                            if (*p - *candidate).magnitude2() < tolerance * tolerance {
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
    fn evaluated_pipe_edges_move_after_boring() {
        use crate::{
            edge_treatments::FaceTarget,
            model::evaluate_scene,
            operators::{Bore, BoreMode},
        };
        let mut base = Body::new(Shape::Circle);
        base.id = 1;
        base.width = 4.;
        base.extrusion = 2.;
        let mut cutter = Body::new(Shape::Circle);
        cutter.id = 2;
        cutter.width = 2.;
        cutter.position = [0., 0., 2.];
        let mut bore = Bore::new(&base, FaceTarget::Top).unwrap();
        bore.mode = BoreMode::Borer;
        bore.travel = -2.;
        cutter.bore = Some(bore);
        let bodies = vec![base, cutter];
        let scene = evaluate_scene(&bodies).unwrap();
        let g = &scene[0].1;
        let edges: Vec<_> = g
            .topology
            .edges
            .iter()
            .enumerate()
            .filter(|(_, e)| {
                let [a, b] = e.map(|i| g.points[i]);
                (a.x.hypot(a.y) - 1.).abs() < 1e-6
                    && (b.x.hypot(b.y) - 1.).abs() < 1e-6
                    && (a.z - b.z).abs() > 1.9
            })
            .take(2)
            .map(|(i, _)| (1, Some(i)))
            .collect();
        assert_eq!(edges.len(), 2);
        let matrix = Matrix4::from_translation(Vector3::new(0.2, 0., 0.));
        let changed =
            transform_selection(&bodies, &scene, SelectionMode::Edge, &edges, matrix).unwrap();
        let moved =
            evaluate_scene(&changed).expect("Selected edges of the evaluated pipe must move");
        let selected: Vec<_> = edges
            .iter()
            .flat_map(|p| vertices(g, SelectionMode::Edge, p.1))
            .collect();
        for i in selected {
            assert!(
                (moved[0].1.points[i] - matrix.transform_point(g.points[i])).magnitude() < 1e-6
            );
        }
        validate_boundary(&moved[0].1, true).unwrap();
        let mut model = Model::from_bodies(bodies.clone());
        model.commit_changes(&changed).unwrap();
        model.undo();
        assert_eq!(model.bodies, bodies);
        model.redo();
        assert_eq!(model.bodies, changed);
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("moved-pipe.osk");
        Project::new(changed.clone(), Camera::default(), true, Some(1))
            .save(&path)
            .unwrap();
        assert_eq!(Project::load(&path).unwrap().bodies, changed);
        let mut stale = changed[0].clone();
        stale.component_edits[0].topology.edges.pop();
        assert!(model.edit(stale).is_err());
        assert_eq!(model.bodies, changed);

        // The same shared edit path must run after an explicit Boolean result.
        let mut a = Body::new(Shape::Rectangle);
        a.id = 1;
        a.extrusion = 2.;
        let mut b = a.clone();
        b.id = 2;
        b.position[0] = 1.;
        let mut sources = vec![a, b];
        let scene = evaluate_scene(&sources).unwrap();
        let mut result = Body::new(Shape::Rectangle);
        result.id = 3;
        result.boolean = Some(crate::booleans::Boolean::new(
            crate::booleans::Kind::Subtract,
            &[(1, None), (2, None)],
            SelectionMode::Volume,
            &scene,
        ));
        sources.push(result);
        let scene = evaluate_scene(&sources).unwrap();
        let changed = transform_selection(
            &sources,
            &scene,
            SelectionMode::Edge,
            &[(3, Some(0))],
            matrix,
        )
        .unwrap();
        let out = evaluate_scene(&changed).unwrap();
        for i in vertices(&scene[2].1, SelectionMode::Edge, Some(0)) {
            assert!(
                (out[2].1.points[i] - matrix.transform_point(scene[2].1.points[i])).magnitude()
                    < 1e-6
            );
        }
        validate_boundary(&out[2].1, true).unwrap();
        // Old .osk edits omit the new stage flag and keep their original replay order.
        let mut legacy = changed[2].component_edits[0].clone();
        legacy.evaluated = false;
        let mut json = serde_json::to_value(&legacy).unwrap();
        json.as_object_mut().unwrap().remove("evaluated");
        assert_eq!(
            serde_json::from_value::<ComponentEdit>(json).unwrap(),
            legacy
        );
    }
    #[test]
    fn cylindrical_offset_changes_radius_and_preserves_axis() {
        let mut body = Body::new(Shape::Circle);
        body.id = 1;
        body.width = 4.;
        body.extrusion = 3.;
        body.position = [2., 3., 4.];
        body.rotation = [15., 25., 35.];
        body.scale = [2., 2., 0.5];
        body.edge_treatments
            .push(crate::edge_treatments::Modifier::new(
                crate::edge_treatments::Kind::Bevel,
                crate::edge_treatments::FaceTarget::Top,
                &body,
            ));
        let g = evaluate(&body).unwrap();
        assert!(
            (0..g.face_count)
                .any(|f| offset_target(&body, &g, f) == Ok(OffsetTarget::CylinderWall))
        );
        for distance in [-0.25, 0.25] {
            let changed = offset_source(&body, OffsetTarget::CylinderWall, distance).unwrap();
            assert_eq!(changed.position, body.position);
            assert_eq!(changed.rotation, body.rotation);
            assert_eq!(changed.extrusion, body.extrusion);
            assert_eq!(changed.edge_treatments, body.edge_treatments);
            let out = evaluate(&changed).unwrap();
            assert!(
                (out.circles[0].u.magnitude() - g.circles[0].u.magnitude() - distance).abs() < 1e-7
            );
            assert!(out.shell.is_some());
        }
        assert!(offset_source(&body, OffsetTarget::CylinderWall, -5.).is_err());
        body.scale[1] = 1.;
        assert!(offset_source(&body, OffsetTarget::CylinderWall, 0.25).is_err());
    }
    #[test]
    fn source_face_offsets_preserve_opposite_planes_and_analytic_geometry() {
        use crate::edge_treatments::{FaceTarget, Kind, Modifier};
        for shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            let mut body = Body::new(shape);
            body.id = 1;
            body.width = 4.;
            body.depth = 3.;
            body.extrusion = 2.;
            body.position = [3., -2., 4.];
            body.rotation = [17., 31., 43.];
            body.scale = [1.5, 0.75, 2.];
            body.edge_treatments
                .push(Modifier::new(Kind::Bevel, FaceTarget::Top, &body));
            let source = evaluate(&body).unwrap();
            for face in [
                FaceTarget::Top,
                FaceTarget::Bottom,
                FaceTarget::Left,
                FaceTarget::Right,
                FaceTarget::Front,
                FaceTarget::Back,
            ] {
                if !face.supported(&body) {
                    continue;
                }
                let (normal, _) = face.plane(&body);
                let normal = crate::regions::transform(&body)
                    .transform_vector(normal)
                    .normalize();
                let limits = |g: &Geometry| {
                    g.triangles
                        .iter()
                        .flatten()
                        .map(|p| p.to_vec().dot(normal))
                        .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), d| {
                            (min.min(d), max.max(d))
                        })
                };
                let original = limits(&source);
                for distance in [-0.3, 0.3] {
                    let changed = offset_source_face(&body, face, distance).unwrap();
                    assert!(changed.component_edits.is_empty());
                    assert_eq!(changed.edge_treatments, body.edge_treatments);
                    let g = evaluate(&changed).unwrap();
                    assert!(g.shell.is_some());
                    let mut closed = g.clone();
                    weld_geometry(&mut closed);
                    boundary_shell(&closed, true)
                        .unwrap_or_else(|e| panic!("{shape:?} {face:?} {distance}: {e}"));
                    let bounds = limits(&g);
                    assert!(
                        (bounds.0 - original.0).abs() < 1e-6,
                        "{shape:?} {face:?}: opposite support moved"
                    );
                    assert!(
                        (bounds.1 - original.1 - distance).abs() < 1e-6,
                        "{shape:?} {face:?}: incorrect world distance"
                    );
                    assert!(
                        (0..g.face_count)
                            .any(|f| crate::edge_treatments::target(&changed, &g, f) == Ok(face))
                    );
                    if shape == Shape::Circle {
                        assert_eq!(changed.width, body.width);
                        assert_eq!(g.circles.len(), source.circles.len());
                        for (a, b) in g.circles.iter().zip(&source.circles) {
                            assert!((a.u - b.u).magnitude() < 1e-8);
                            assert!((a.v - b.v).magnitude() < 1e-8);
                        }
                    }
                }
            }
            let mut model = Model::from_bodies(vec![body.clone()]);
            for d in [0.1, 0.2] {
                model
                    .edit(offset_source_face(&body, FaceTarget::Top, d).unwrap())
                    .unwrap();
            }
            model.finish_edit();
            let edited = model.bodies.clone();
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("offset.osk");
            Project::new(edited.clone(), Camera::default(), true, Some(1))
                .save(&path)
                .unwrap();
            assert_eq!(Project::load(&path).unwrap().bodies, edited);
            model.undo();
            assert_eq!(model.bodies, vec![body.clone()]);
            assert!(!model.can_undo(), "one continuous edit is one undo step");
            model.redo();
            assert_eq!(model.bodies, edited);
            for distance in [f64::NAN, f64::INFINITY, -4., -5.] {
                assert!(offset_source_face(&body, FaceTarget::Top, distance).is_err());
                assert_eq!(model.bodies, edited);
            }
            // A valid primitive resize can still invalidate a treatment; commit
            // must reject it without consuming history or replacing geometry.
            let mut oversized = body.clone();
            oversized.edge_treatments[0].amount = 0.9;
            assert!(offset_source_face(&oversized, FaceTarget::Top, -1.).is_err());
            let deformed = transformed(
                &body,
                &source,
                SelectionMode::Point,
                Some(0),
                Matrix4::from_translation(Vector3::unit_z() * 0.1),
            )
            .unwrap();
            assert!(offset_source_face(&deformed, FaceTarget::Top, 0.1).is_err());
        }
    }
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
