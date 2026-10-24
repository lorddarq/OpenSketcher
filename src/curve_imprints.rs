//! Finite straight-curve imprints. Trim curves and neighboring surfaces stay analytic.
use crate::{booleans::Boolean, model::Geometry};
use std::collections::{HashMap, HashSet};
use truck_modeling::{
    BoundedCurve, Curve, Cut, Edge, EdgeID, EuclideanSpace, Face, InnerSpace, ParametricCurve,
    Point3, Shell, Vector3, Vertex, VertexID, Wire, builder,
};

const EPS: f64 = 1e-7;

// Isolate roots using the spline control hull, not the display tessellation.
// Homogeneous numerators remain valid even for zero-weight control points.
pub(crate) fn roots(
    curve: Curve,
    origin: Point3,
    normal: Vector3,
    budget: &mut usize,
) -> Result<Vec<f64>, String> {
    if *budget == 0 {
        return Err("Curve intersection exceeded its subdivision budget.".into());
    }
    *budget -= 1;
    let (lo, hi) = curve.range_tuple();
    let distance = |p: Point3| (p - origin).dot(normal);
    if matches!(curve, Curve::Line(_)) {
        let first = distance(curve.subs(lo));
        let last = distance(curve.subs(hi));
        if first.abs() < 1e-11 && last.abs() < 1e-11 {
            return Ok(vec![lo, hi]);
        }
        if first * last <= 0. || first.abs() < 1e-11 || last.abs() < 1e-11 {
            let t = lo + (hi - lo) * first / (first - last);
            return Ok(vec![t.clamp(lo, hi)]);
        }
        return Ok(vec![]);
    }
    let (coefficients, scale, knots) = match &curve {
        Curve::Line(line) => (
            vec![distance(line.subs(lo)), distance(line.subs(hi))],
            1.,
            vec![],
        ),
        Curve::BSplineCurve(spline) => (
            spline
                .control_points()
                .iter()
                .map(|p| distance(*p))
                .collect(),
            1.,
            spline.knot_vec().iter().copied().collect(),
        ),
        Curve::NurbsCurve(spline) => (
            spline
                .control_points()
                .iter()
                .map(|p| {
                    normal.dot(Vector3::new(p.x, p.y, p.z)) - origin.to_vec().dot(normal) * p.w
                })
                .collect(),
            spline
                .control_points()
                .iter()
                .map(|p| p.w.abs())
                .fold(0., f64::max),
            spline.knot_vec().iter().copied().collect(),
        ),
        Curve::IntersectionCurve(_) => {
            return Err("Imprint on intersection-curve trims is not supported yet.".into());
        }
    };
    let tolerance = 1e-11 * scale.max(1e-30);
    let mut hits = Vec::new();
    if distance(curve.subs(lo)).abs() < 1e-10 {
        hits.push(lo);
    }
    if distance(curve.subs(hi)).abs() < 1e-10 {
        hits.push(hi);
    }
    let min = coefficients.iter().copied().fold(f64::INFINITY, f64::min);
    let max = coefficients
        .iter()
        .copied()
        .fold(f64::NEG_INFINITY, f64::max);
    if min >= -tolerance || max <= tolerance {
        return Ok(hits);
    }
    let internal_knot = knots.into_iter().find(|t| *t > lo + 1e-8 && *t < hi - 1e-8);
    let signs: Vec<_> = coefficients
        .iter()
        .filter(|c| c.abs() > tolerance)
        .map(|c| c.is_sign_positive())
        .collect();
    let variations = signs.windows(2).filter(|p| p[0] != p[1]).count();
    if internal_knot.is_none()
        && variations == 1
        && distance(curve.subs(lo)) * distance(curve.subs(hi)) < 0.
    {
        let mut lower = lo;
        let mut upper = hi;
        let sign = distance(curve.subs(lo)).is_sign_positive();
        for _ in 0..60 {
            let mid = (lower + upper) / 2.;
            if distance(curve.subs(mid)).is_sign_positive() == sign {
                lower = mid;
            } else {
                upper = mid;
            }
        }
        hits.push((lower + upper) / 2.);
        return Ok(hits);
    }
    if hi - lo < 1e-5 {
        return Err("Cannot resolve near-tangent trim intersections at kernel tolerance.".into());
    }
    let split = internal_knot.unwrap_or((lo + hi) / 2.);
    let mut left = curve;
    let right = left.cut(split);
    hits.extend(roots(left, origin, normal, budget)?);
    hits.extend(roots(right, origin, normal, budget)?);
    hits.sort_by(f64::total_cmp);
    hits.dedup_by(|a, b| (*a - *b).abs() < 1e-10);
    Ok(hits)
}

pub(crate) fn split(edge: &Edge, hits: &[(f64, Vertex)]) -> Result<Vec<Edge>, String> {
    let mut tail = if edge.orientation() {
        edge.clone()
    } else {
        edge.inverse()
    };
    let mut pieces = Vec::new();
    for (t, vertex) in hits {
        if vertex == tail.front() || vertex == tail.back() {
            continue;
        }
        // A Line cut reparameterizes its remainder to 0..1, whereas splines
        // retain their knot parameters. Recompute successive line parameters.
        let parameter = if matches!(tail.curve(), Curve::Line(_)) {
            let a = tail.front().point();
            let d = tail.back().point() - a;
            (vertex.point() - a).dot(d) / d.magnitude2()
        } else {
            *t
        };
        let (head, next) = tail
            .cut_with_parameter(vertex, parameter)
            .ok_or("The trim intersection is too close to an edge endpoint to split safely.")?;
        pieces.push(head);
        tail = next;
    }
    pieces.push(tail);
    Ok(pieces)
}
pub(crate) fn oriented(edge: &Edge, pieces: &[Edge]) -> Vec<Edge> {
    if edge.orientation() {
        pieces.to_vec()
    } else {
        pieces.iter().rev().map(Edge::inverse).collect()
    }
}

pub(crate) fn apply(op: &Boolean, inputs: &[&Geometry]) -> Result<Geometry, String> {
    for (i, g) in inputs.iter().enumerate() {
        if g.topology != op.topology[i] {
            return Err("An Imprint curve input changed topology; reselect its reference.".into());
        }
    }
    let source = inputs[0];
    let tool = inputs[1];
    if !source.is_solid
        || tool.is_solid
        || !tool.triangles.is_empty()
        || tool.edge_count != 1
        || tool.topology.edges.len() != 1
    {
        return Err("Choose a solid face and an unextruded straight line.".into());
    }
    let element = op.operands[0].1.ok_or("Select a source face.")?;
    let plane = crate::snapping::Plane::face(source, element)
        .ok_or("Curve imprint requires a planar face.")?;
    let endpoints = tool.topology.edges[0];
    let a: Point3 = tool
        .points
        .get(endpoints[0])
        .ok_or("Missing line endpoint.")?
        .to_owned();
    let b: Point3 = tool
        .points
        .get(endpoints[1])
        .ok_or("Missing line endpoint.")?
        .to_owned();
    let length = (b - a).magnitude();
    if !length.is_finite()
        || length < EPS
        || [a, b]
            .iter()
            .any(|p| (*p - plane.origin).dot(plane.normal).abs() > EPS)
    {
        return Err("The finite line must lie on the selected face plane.".into());
    }
    let axis = (b - a) / length;
    let normal = plane.normal.cross(axis);
    if tool
        .edges
        .iter()
        .flat_map(|edge| edge.iter())
        .any(|p| (*p - a).cross(axis).magnitude() > EPS)
    {
        return Err("Imprint curve currently requires a straight line.".into());
    }
    let shell = source
        .shell
        .as_ref()
        .ok_or("Curve imprint requires retained analytic boundaries.")?;
    let mapping = source.surface_mapping()?;
    let selected: Shell = shell
        .iter()
        .zip(mapping)
        .filter(|(_, role)| **role == element)
        .map(|(f, _)| f.clone())
        .collect();
    if selected.is_empty() {
        return Err("The selected face no longer exists.".into());
    }
    let boundaries = selected.extract_boundaries();
    let mut originals = HashMap::<EdgeID, Edge>::new();
    let mut intersections = HashMap::<EdgeID, Vec<(f64, Vertex)>>::new();
    let mut virtual_pieces = HashMap::<EdgeID, Vec<Edge>>::new();
    let mut budget = 16384;
    for edge in boundaries.iter().flat_map(|w| w.iter()) {
        let absolute = if edge.orientation() {
            edge.clone()
        } else {
            edge.inverse()
        };
        let curve = absolute.curve();
        let mut hits = Vec::new();
        for t in roots(curve.clone(), a, normal, &mut budget)? {
            let point = curve.subs(t);
            let vertex = if (point - absolute.front().point()).magnitude() < EPS {
                absolute.front().clone()
            } else if (point - absolute.back().point()).magnitude() < EPS {
                absolute.back().clone()
            } else {
                Vertex::new(point)
            };
            if !hits
                .iter()
                .any(|(_, v): &(f64, Vertex)| (v.point() - point).magnitude() < EPS)
            {
                hits.push((t, vertex));
            }
        }
        hits.sort_by(|a, b| a.0.total_cmp(&b.0));
        virtual_pieces.insert(edge.id(), split(&absolute, &hits)?);
        originals.insert(edge.id(), absolute);
        intersections.insert(edge.id(), hits);
    }
    let mut events = HashMap::<VertexID, (Vertex, i32)>::new();
    let mut coincident = Vec::new();
    for edge in boundaries.iter().flat_map(|w| w.iter()) {
        for piece in oriented(edge, &virtual_pieces[&edge.id()]) {
            let curve = piece.oriented_curve();
            let (lo, hi) = curve.range_tuple();
            let side = (curve.subs((lo + hi) / 2.) - a).dot(normal);
            let front = piece.front();
            let back = piece.back();
            if side.abs() < EPS
                && [front, back]
                    .iter()
                    .all(|v| (v.point() - a).dot(normal).abs() < EPS)
            {
                let mut interval = [(front.point() - a).dot(axis), (back.point() - a).dot(axis)];
                interval.sort_by(f64::total_cmp);
                coincident.push(interval);
                continue;
            }
            for (vertex, delta) in [(front, 1), (back, -1)] {
                if (vertex.point() - a).dot(normal).abs() < EPS {
                    let event = events.entry(vertex.id()).or_insert((vertex.clone(), 0));
                    if side > EPS {
                        event.1 += delta;
                    }
                }
            }
        }
    }
    let mut events: Vec<_> = events.into_values().collect();
    events.sort_by(|x, y| {
        (x.0.point() - a)
            .dot(axis)
            .total_cmp(&(y.0.point() - a).dot(axis))
    });
    if events
        .windows(2)
        .any(|p| (p[0].0.point() - p[1].0.point()).magnitude() < EPS)
    {
        return Err("Ambiguous coincident trim vertices at the imprint line.".into());
    }
    let mut winding = 0;
    let mut connectors = Vec::new();
    let mut active = HashSet::new();
    for pair in events.windows(2) {
        winding -= pair[0].1;
        let start = (pair[0].0.point() - a).dot(axis);
        let end = (pair[1].0.point() - a).dot(axis);
        if winding <= 0
            || end - start < EPS
            || coincident
                .iter()
                .any(|range| start >= range[0] - EPS && end <= range[1] + EPS)
        {
            continue;
        }
        if end <= EPS || start >= length - EPS {
            continue;
        }
        if start < -EPS || end > length + EPS {
            return Err("The line stops inside material; extend it to the face boundary to form independent regions.".into());
        }
        active.insert(pair[0].0.id());
        active.insert(pair[1].0.id());
        connectors.push(builder::line(&pair[0].0, &pair[1].0));
    }
    if connectors.is_empty() {
        return Err("The finite line does not divide the selected face.".into());
    }
    let mut pieces = HashMap::new();
    for (id, edge) in &originals {
        let hits: Vec<_> = intersections[id]
            .iter()
            .filter(|(_, v)| active.contains(&v.id()))
            .cloned()
            .collect();
        pieces.insert(*id, split(edge, &hits)?);
    }
    let mut directed = Vec::new();
    for edge in boundaries.iter().flat_map(|w| w.iter()) {
        directed.extend(oriented(edge, &pieces[&edge.id()]));
    }
    for edge in connectors {
        directed.push(edge.inverse());
        directed.push(edge);
    }
    partition(source, element, plane, &pieces, directed)
}

pub(crate) fn partition(
    source: &Geometry,
    element: usize,
    plane: crate::snapping::Plane,
    pieces: &HashMap<EdgeID, Vec<Edge>>,
    directed: Vec<Edge>,
) -> Result<Geometry, String> {
    let shell = source
        .shell
        .as_ref()
        .ok_or("Imprint requires retained boundaries.")?;
    let regions = planar_arrangement(plane, directed)?;
    if regions.len() < 2 {
        return Err(
            "The imprint does not separate the selected face into independent regions.".into(),
        );
    }
    let mut result_shell = Shell::new();
    let mut roles = Vec::new();
    for (face, role) in shell.iter().zip(source.surface_mapping()?) {
        if *role == element {
            continue;
        }
        let wires = face
            .boundaries()
            .into_iter()
            .map(|wire| {
                wire.iter()
                    .flat_map(|edge| {
                        pieces
                            .get(&edge.id())
                            .map(|p| oriented(edge, p))
                            .unwrap_or_else(|| vec![edge.clone()])
                    })
                    .collect()
            })
            .collect();
        result_shell
            .push(Face::try_new(wires, face.oriented_surface()).map_err(|e| e.to_string())?);
        roles.push(*role);
    }
    for (i, face) in regions.into_iter().enumerate() {
        result_shell.push(face);
        roles.push(if i == 0 {
            element
        } else {
            source.face_count + i - 1
        });
    }
    if !result_shell.is_geometric_consistent() {
        return Err("The imprint cannot retain consistent analytic neighbor trims.".into());
    }
    crate::components::solid_parts(&result_shell)?;
    let mut result = crate::model::tessellate(&result_shell)?;
    crate::edge_treatments::merge_surfaces(&mut result, &roles);
    result.is_solid = true;
    result.circles = source.circles.clone();
    Ok(result)
}

/// Shared directed analytic-edge arrangement for imprint and planar set operations.
pub(crate) fn planar_arrangement(
    plane: crate::snapping::Plane,
    directed: Vec<Edge>,
) -> Result<Vec<Face>, String> {
    let mut outgoing = HashMap::<VertexID, Vec<usize>>::new();
    for (i, edge) in directed.iter().enumerate() {
        outgoing.entry(edge.front().id()).or_default().push(i);
    }
    let mut used = vec![false; directed.len()];
    let mut cycles = Vec::new();
    for start in 0..directed.len() {
        if used[start] {
            continue;
        }
        let mut wire = Wire::new();
        let mut current = start;
        loop {
            if used[current] {
                return Err("The cut forms a nonseparating or ambiguous trim.".into());
            }
            used[current] = true;
            let edge = &directed[current];
            wire.push_back(edge.clone());
            let curve = edge.oriented_curve();
            let incoming = -curve.der(curve.range_tuple().1).normalize();
            let candidates = outgoing
                .get(&edge.back().id())
                .ok_or("An imprint trim is open.")?;
            current = *candidates
                .iter()
                .filter(|i| directed[**i].id() != edge.id())
                .min_by(|x, y| {
                    let angle = |i: usize| {
                        let c = directed[i].oriented_curve();
                        let d = c.der(c.range_tuple().0).normalize();
                        (-plane.normal.dot(incoming.cross(d)))
                            .atan2(incoming.dot(d))
                            .rem_euclid(std::f64::consts::TAU)
                    };
                    angle(**x).total_cmp(&angle(**y))
                })
                .ok_or("The imprint does not separate material into closed faces.")?;
            if current == start {
                break;
            }
        }
        let poly = crate::face_operations::wire_polygon(&wire, &plane);
        let area = crate::regions::area(&poly);
        if area.abs() < 1e-12 {
            return Err("The imprint produces a degenerate region.".into());
        }
        cycles.push((wire, poly, area));
    }
    // A disconnected inner profile contributes both directions of its loop.
    // Assign its negative cycle to the smallest enclosing positive cycle,
    // never to the equal-area reverse cycle or every ancestor island.
    let mut holes = HashMap::<usize, Vec<Wire>>::new();
    for (hole, points, area) in &cycles {
        if *area >= 0. {
            continue;
        }
        let owner = cycles
            .iter()
            .enumerate()
            .filter(|(_, (_, poly, outer_area))| {
                *outer_area > -*area + 1e-12 && crate::regions::contains(points[0], poly)
            })
            .min_by(|(_, a), (_, b)| a.2.total_cmp(&b.2))
            .map(|(i, _)| i)
            .ok_or("An imprint hole has no enclosing material region.")?;
        holes.entry(owner).or_default().push(hole.clone());
    }
    let mut regions = Vec::new();
    for (index, (outer, _, area)) in cycles.iter().enumerate() {
        if *area <= 0. {
            continue;
        }
        let mut wires = vec![outer.clone()];
        wires.extend(holes.remove(&index).unwrap_or_default());
        regions.push(
            Face::try_new(
                wires,
                truck_modeling::Plane::new(
                    plane.origin,
                    plane.origin + plane.u,
                    plane.origin + plane.v,
                )
                .into(),
            )
            .map_err(|e| format!("Invalid imprint region: {e}"))?,
        );
    }
    Ok(regions)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        booleans::Kind,
        model::{Body, Shape, evaluate_scene},
        selection::SelectionMode,
    };
    fn volume(g: &Geometry) -> f64 {
        g.triangles
            .iter()
            .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
            .sum()
    }
    #[test]
    fn boundary_vertices_coincident_edges_and_tangencies_are_safe() {
        for shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            let mut source = Body::new(shape);
            source.id = 1;
            source.width = 4.;
            source.depth = 4.;
            source.extrusion = 3.;
            let horizontal = if shape == Shape::Semicircle { 1.0 } else { 0.0 };
            for (start, end, divides) in [
                ([-3., horizontal, 0.], [3., horizontal, 0.], true),
                ([-3., 2., 0.], [3., 2., 0.], false),
                ([-3., -3., 0.], [3., 3., 0.], true),
            ] {
                let mut line = Body::from_points(Shape::Line, start, end).unwrap();
                line.id = 2;
                line.position[2] = 3.;
                let mut bodies = vec![source.clone(), line];
                let scene = evaluate_scene(&bodies).unwrap();
                let face = (0..scene[0].1.face_count)
                    .find(|f| {
                        crate::edge_treatments::target(&source, &scene[0].1, *f)
                            == Ok(crate::edge_treatments::FaceTarget::Top)
                    })
                    .unwrap();
                let mut feature = Body::new(Shape::Rectangle);
                feature.id = 3;
                feature.boolean = Some(Boolean::new(
                    Kind::ImprintCurve,
                    &[(1, Some(face)), (2, None)],
                    SelectionMode::Face,
                    &scene,
                ));
                bodies.push(feature);
                let result = evaluate_scene(&bodies);
                assert_eq!(
                    result.is_ok(),
                    divides,
                    "{shape:?} {start:?}: {:?}",
                    result.err()
                );
            }
        }
    }

    #[test]
    fn transformed_imprints_regenerate_export_and_rollback() {
        use truck_modeling::Transform;
        let mut source = Body::new(Shape::Circle);
        source.id = 1;
        source.width = 4.;
        source.extrusion = 3.;
        source.position = [4., -3., 2.];
        source.rotation = [20., 30., 40.];
        source.scale = [1.5, 0.75, 2.];
        let transform = crate::regions::transform(&source);
        let line_at = |y: f64| {
            let mut line = Body::new(Shape::Line);
            line.id = 2;
            line.width = 6.;
            line.rotation = source.rotation;
            line.scale = source.scale;
            line.position = transform.transform_point(Point3::new(0., y, 3.)).into();
            line
        };
        let mut bodies = vec![source.clone(), line_at(0.25)];
        let scene = evaluate_scene(&bodies).unwrap();
        let face = (0..scene[0].1.face_count)
            .find(|f| {
                crate::edge_treatments::target(&bodies[0], &scene[0].1, *f)
                    == Ok(crate::edge_treatments::FaceTarget::Top)
            })
            .unwrap();
        let mut feature = Body::new(Shape::Rectangle);
        feature.id = 3;
        feature.boolean = Some(Boolean::new(
            Kind::ImprintCurve,
            &[(1, Some(face)), (2, None)],
            SelectionMode::Face,
            &scene,
        ));
        bodies.push(feature.clone());
        let mut model = crate::model::Model::from_bodies(bodies.clone());
        let initial = evaluate_scene(&bodies).unwrap();
        assert_eq!(initial[2].1.face_count, scene[0].1.face_count + 1);
        assert!((volume(&initial[2].1) - volume(&scene[0].1)).abs() < 0.04);
        model.edit(line_at(0.5)).unwrap();
        model.finish_edit();
        let moved = evaluate_scene(&model.bodies).unwrap();
        assert_ne!(moved[2].1.points, initial[2].1.points);
        model.undo();
        assert_eq!(model.bodies, bodies);
        model.redo();
        assert_eq!(model.bodies[1], line_at(0.5));
        let accepted = model.bodies.clone();
        let mut offset = line_at(0.5);
        offset.position[2] += 0.1;
        assert!(model.edit(offset).is_err());
        assert_eq!(model.bodies, accepted);
        let mut stale = feature.clone();
        stale.boolean.as_mut().unwrap().topology[0].edges.pop();
        assert!(model.edit(stale).is_err());
        assert_eq!(model.bodies, accepted);
        let mut keep = feature.clone();
        keep.boolean.as_mut().unwrap().keep_tools = true;
        model.edit(keep).unwrap();
        model.finish_edit();
        let kept = evaluate_scene(&model.bodies).unwrap();
        assert!(!kept[1].1.edges.is_empty());
        assert!(kept[0].1.triangles.is_empty());
        // Export the same evaluated, imprinted analytic boundary through all existing paths.
        let dir = tempfile::tempdir().unwrap();
        for format in crate::export::Format::ALL {
            let path = dir.path().join(format!("imprint.{}", format.extension()));
            crate::export::save(&accepted, format, &path).unwrap();
            assert!(std::fs::metadata(path).unwrap().len() > 100);
        }
        model.delete(3).unwrap();
        let restored = evaluate_scene(&model.bodies).unwrap();
        assert!(!restored[0].1.triangles.is_empty());
    }

    #[test]
    fn line_imprints_split_holed_faces_and_finite_concave_arms() {
        use crate::edge_treatments::FaceTarget;
        for shape in [Shape::Rectangle, Shape::Circle] {
            let mut source = Body::new(shape);
            source.id = 1;
            source.width = 6.;
            source.depth = 6.;
            source.extrusion = 3.;
            source.hollow = Some(crate::shelling::Hollow::new(&source, FaceTarget::Top).unwrap());
            let mut line = Body::from_points(Shape::Line, [-4., 0.25, 0.], [4., 0.25, 0.]).unwrap();
            line.id = 2;
            line.position[2] = 3.;
            let mut bodies = vec![source, line];
            let scene = evaluate_scene(&bodies).unwrap();
            let face = (0..scene[0].1.face_count)
                .find(|f| {
                    crate::snapping::Plane::face(&scene[0].1, *f)
                        .is_some_and(|p| p.normal.z > 0.99 && (p.origin.z - 3.).abs() < EPS)
                })
                .unwrap();
            let mut feature = Body::new(Shape::Rectangle);
            feature.id = 3;
            feature.boolean = Some(Boolean::new(
                Kind::ImprintCurve,
                &[(1, Some(face)), (2, None)],
                SelectionMode::Face,
                &scene,
            ));
            bodies.push(feature);
            let result = evaluate_scene(&bodies).unwrap();
            assert_eq!(result[2].1.face_count, scene[0].1.face_count + 1);
            assert!((volume(&result[2].1) - volume(&scene[0].1)).abs() < 0.02);
            assert!(
                result[2]
                    .1
                    .shell
                    .as_ref()
                    .unwrap()
                    .is_geometric_consistent()
            );
        }
        let mut source = Body::new(Shape::Rectangle);
        source.id = 1;
        source.width = 6.;
        source.depth = 6.;
        source.extrusion = 3.;
        let mut notch = source.clone();
        notch.id = 2;
        notch.width = 4.;
        notch.depth = 2.;
        notch.position[0] = 2.;
        let mut bodies = vec![source, notch];
        let scene = evaluate_scene(&bodies).unwrap();
        let mut cut = Body::new(Shape::Rectangle);
        cut.id = 3;
        cut.boolean = Some(Boolean::new(
            Kind::Subtract,
            &[(1, None), (2, None)],
            SelectionMode::Volume,
            &scene,
        ));
        bodies.push(cut);
        let mut line = Body::from_points(Shape::Line, [1., 0.5, 0.], [1., 4., 0.]).unwrap();
        line.id = 4;
        line.position[2] = 3.;
        bodies.push(line);
        let scene = evaluate_scene(&bodies).unwrap();
        let source = &scene[2].1;
        let face = (0..source.face_count)
            .find(|f| crate::snapping::Plane::face(source, *f).is_some_and(|p| p.normal.z > 0.99))
            .unwrap();
        let mut feature = Body::new(Shape::Rectangle);
        feature.id = 5;
        feature.boolean = Some(Boolean::new(
            Kind::ImprintCurve,
            &[(3, Some(face)), (4, None)],
            SelectionMode::Face,
            &scene,
        ));
        bodies.push(feature);
        let result = evaluate_scene(&bodies).unwrap();
        assert_eq!(result[4].1.face_count, source.face_count + 1);
        assert!((volume(&result[4].1) - volume(source)).abs() < EPS);
        // Roots on the other arm are outside the finite tool and must not split it.
        assert_eq!(
            result[4]
                .1
                .points
                .iter()
                .filter(|p| (p.x - 1.).abs() < EPS && p.y < 0.)
                .count(),
            0
        );
    }

    #[test]
    fn finite_line_splits_analytic_caps_and_keeps_neighbors_closed() {
        for shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            let mut source = Body::new(shape);
            source.id = 1;
            source.width = 4.;
            source.depth = 4.;
            source.extrusion = 3.;
            let y = if shape == Shape::Semicircle {
                0.75
            } else {
                0.25
            };
            let mut line = Body::from_points(Shape::Line, [-3., y, 3.], [3., y, 3.]).unwrap();
            line.id = 2;
            line.position[2] = 3.;
            let mut bodies = vec![source.clone(), line.clone()];
            let scene = evaluate_scene(&bodies).unwrap();
            let face = (0..scene[0].1.face_count)
                .find(|f| {
                    crate::edge_treatments::target(&source, &scene[0].1, *f)
                        == Ok(crate::edge_treatments::FaceTarget::Top)
                })
                .unwrap();
            let mut feature = Body::new(Shape::Rectangle);
            feature.id = 3;
            feature.boolean = Some(Boolean::new(
                Kind::ImprintCurve,
                &[(1, Some(face)), (2, None)],
                SelectionMode::Face,
                &scene,
            ));
            bodies.push(feature.clone());
            let result = evaluate_scene(&bodies).unwrap();
            let g = &result[2].1;
            assert_eq!(g.face_count, scene[0].1.face_count + 1, "{shape:?}");
            assert!((volume(g) - volume(&scene[0].1)).abs() < 0.02, "{shape:?}");
            assert!(g.shell.as_ref().unwrap().is_geometric_consistent());
            if shape != Shape::Rectangle {
                assert!(
                    g.shell
                        .as_ref()
                        .unwrap()
                        .edge_iter()
                        .any(|e| matches!(e.curve(), Curve::NurbsCurve(_)))
                );
            }
            assert!(result[0].1.triangles.is_empty());
            assert!(result[1].1.edges.is_empty());
            // A finite tool ending inside material is never extrapolated.
            bodies[1] = Body::from_points(Shape::Line, [-0.5, y, 3.], [0.5, y, 3.]).unwrap();
            bodies[1].id = 2;
            bodies[1].position[2] = 3.;
            assert!(
                evaluate_scene(&bodies)
                    .err()
                    .unwrap()
                    .contains("stops inside")
            );
            bodies[1] = line;
            // Both independent cap regions remain valid extrusion targets.
            for region in (0..g.face_count)
                .filter(|f| crate::snapping::Plane::face(g, *f).is_some_and(|p| p.normal.z > 0.99))
            {
                let changed = crate::components::sweep_face(&feature, g, region, 0.5).unwrap();
                let mut edited = bodies.clone();
                edited[2] = changed;
                let swept = evaluate_scene(&edited).unwrap();
                assert!(volume(&swept[2].1) > volume(g));
            }
            let project = crate::project::Project::new(
                bodies.clone(),
                crate::viewport::Camera::default(),
                true,
                Some(3),
            );
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("curve.osk");
            project.save(&path).unwrap();
            assert_eq!(crate::project::Project::load(&path).unwrap().bodies, bodies);
            let mut legacy = serde_json::to_value(&project).unwrap();
            legacy["schema_version"] = 25.into();
            assert!(
                serde_json::from_value::<crate::project::Project>(legacy)
                    .unwrap()
                    .validate()
                    .unwrap_err()
                    .contains("26")
            );
        }
    }
}
