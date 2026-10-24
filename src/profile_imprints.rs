//! Analytic coplanar closed-profile crossings; reuse the shared imprint arrangement.
use crate::{
    curve_imprints::{oriented, partition, roots, split},
    model::Geometry,
    snapping::Plane,
};
use std::collections::{HashMap, HashSet};
use truck_modeling::{
    BoundedCurve, Curve, Cut, Edge, EdgeID, EuclideanSpace, InnerSpace, Invertible, Matrix3,
    ParametricCurve, Point3, Shell, SquareMatrix, Vector3, Vertex, Wire,
};
const EPS: f64 = 1e-7;

// Convex control hull bounds are conservative. Zero/nonpositive rational weights
// require subdivision before a finite projected hull can be used.
fn controls(curve: &Curve) -> Result<Option<Vec<Point3>>, String> {
    Ok(Some(match curve {
        Curve::Line(_) => {
            let (a, b) = curve.range_tuple();
            vec![curve.subs(a), curve.subs(b)]
        }
        Curve::BSplineCurve(c) => c.control_points().clone(),
        Curve::NurbsCurve(c) => {
            if c.control_points().iter().any(|p| p.w <= 1e-14) {
                return Ok(None);
            }
            c.control_points()
                .iter()
                .map(|p| Point3::from_vec(p.truncate() / p.w))
                .collect()
        }
        Curve::IntersectionCurve(_) => {
            return Err("Profile crossings on intersection-curve trims remain unsupported.".into());
        }
    }))
}
fn bounds(curve: &Curve, plane: &Plane) -> Result<Option<[[f64; 2]; 2]>, String> {
    let Some(points) = controls(curve)? else {
        return Ok(None);
    };
    let mut result = [[f64::INFINITY, f64::NEG_INFINITY]; 2];
    for p in points {
        for (i, axis) in [plane.u, plane.v].iter().enumerate() {
            let v = (p - plane.origin).dot(*axis);
            if !v.is_finite() {
                return Err("Nonfinite trim control point.".into());
            }
            result[i][0] = result[i][0].min(v);
            result[i][1] = result[i][1].max(v);
        }
    }
    Ok(Some(result))
}
fn separated(a: &Curve, b: &Curve, plane: &Plane) -> Result<bool, String> {
    let (Some(a), Some(b)) = (controls(a)?, controls(b)?) else {
        return Ok(false);
    };
    // Any separating projection of the two control hulls excludes a true hit.
    // Chord normals resolve boxes that overlap near a transverse intersection.
    for points in [&a, &b] {
        for pair in points.windows(2) {
            let axis = plane.normal.cross(pair[1] - pair[0]);
            if axis.magnitude2() < 1e-28 {
                continue;
            }
            let axis = axis.normalize();
            let range = |points: &[Point3]| {
                points
                    .iter()
                    .map(|p| (*p - plane.origin).dot(axis))
                    .fold([f64::INFINITY, f64::NEG_INFINITY], |[lo, hi], v| {
                        [lo.min(v), hi.max(v)]
                    })
            };
            let x = range(&a);
            let y = range(&b);
            if x[1] < y[0] - 1e-12 || y[1] < x[0] - 1e-12 {
                return Ok(true);
            }
        }
    }
    Ok(false)
}
fn cross(a: Vector3, b: Vector3, plane: &Plane) -> f64 {
    a.cross(b).dot(plane.normal)
}
fn line_hits(
    line: &Curve,
    curve: &Curve,
    plane: &Plane,
    budget: &mut usize,
) -> Result<Vec<(f64, f64)>, String> {
    let (lo, hi) = line.range_tuple();
    let a = line.subs(lo);
    let b = line.subs(hi);
    let d = b - a;
    if d.magnitude2() < EPS * EPS {
        return Err("Degenerate imprint edge.".into());
    }
    let normal = plane.normal.cross(d.normalize());
    if matches!(curve, Curve::Line(_)) {
        let (first, last) = curve.range_tuple();
        let p = curve.subs(first);
        let q = curve.subs(last);
        let other = q - p;
        if (p - a).dot(normal).abs() < EPS && (q - a).dot(normal).abs() < EPS {
            let mut interval = [
                (p - a).dot(d) / d.magnitude2(),
                (q - a).dot(d) / d.magnitude2(),
            ];
            interval.sort_by(f64::total_cmp);
            let lower = interval[0].max(0.);
            let upper = interval[1].min(1.);
            if upper < lower - EPS {
                return Ok(vec![]);
            }
            return Ok([lower, upper]
                .into_iter()
                .map(|s| {
                    let point = a + d * s;
                    (
                        lo + (hi - lo) * s,
                        first + (last - first) * (point - p).dot(other) / other.magnitude2(),
                    )
                })
                .collect());
        }
    }
    let mut hits = Vec::new();
    let params = roots(curve.clone(), a, normal, budget)?;
    if params.len() > 1
        && params
            .iter()
            .all(|t| (curve.subs(*t) - a).dot(normal).abs() < EPS)
    {
        let (first, last) = curve.range_tuple();
        if (curve.subs((first + last) / 2.) - a).dot(normal).abs() < EPS {
            let mut interval = params
                .iter()
                .map(|t| (curve.subs(*t) - a).dot(d) / d.magnitude2())
                .collect::<Vec<_>>();
            interval.sort_by(f64::total_cmp);
            if interval[0].max(0.) < interval[interval.len() - 1].min(1.) - EPS {
                return Err(
                    "Coincident overlapping profile trims require a dedicated merge operation."
                        .into(),
                );
            }
        }
    }
    for t in params {
        let position = (curve.subs(t) - a).dot(d) / d.magnitude2();
        if (-EPS..=1. + EPS).contains(&position) {
            hits.push((lo + (hi - lo) * position.clamp(0., 1.), t));
        }
    }
    Ok(hits)
}

// Newton refines only a control-hull-isolated pair. It never supplies the
// candidate search, so narrow crossings are not dependent on viewport sampling.
fn refine(a: &Curve, b: &Curve, plane: &Plane) -> Option<(f64, f64)> {
    let (a0, a1) = a.range_tuple();
    let (b0, b1) = b.range_tuple();
    let mut t = (a0 + a1) / 2.;
    let mut u = (b0 + b1) / 2.;
    for _ in 0..40 {
        let error = a.subs(t) - b.subs(u);
        if error.magnitude() < 1e-9 {
            return Some((t, u));
        }
        let da = a.der(t);
        let db = b.der(u);
        let determinant = cross(da, db, plane);
        if determinant.abs() < 1e-15 * da.magnitude() * db.magnitude() {
            return None;
        }
        let dt = -cross(error, db, plane) / determinant;
        let du = -cross(error, da, plane) / determinant;
        t = (t + dt).clamp(a0, a1);
        u = (u + du).clamp(b0, b1);
    }
    None
}
fn curved_hits(
    a: Curve,
    b: Curve,
    plane: &Plane,
    budget: &mut usize,
) -> Result<Vec<(f64, f64)>, String> {
    if *budget == 0 {
        return Err("Profile intersection exceeded its subdivision budget.".into());
    }
    *budget -= 1;
    let same = |a: &Curve, b: &Curve| match (a, b) {
        (Curve::BSplineCurve(a), Curve::BSplineCurve(b)) => a == b,
        (Curve::NurbsCurve(a), Curve::NurbsCurve(b)) => a == b,
        _ => false,
    };
    let mut reversed = b.clone();
    reversed.invert();
    if same(&a, &b) || same(&a, &reversed) {
        return Err("Coincident curved trims do not form an independent imprint region.".into());
    }
    let ba = bounds(&a, plane)?;
    let bb = bounds(&b, plane)?;
    if let (Some(a), Some(b)) = (ba, bb)
        && (0..2).any(|i| a[i][1] < b[i][0] - 1e-10 || b[i][1] < a[i][0] - 1e-10)
    {
        return Ok(vec![]);
    }
    let width = |bounds: Option<[[f64; 2]; 2]>| {
        bounds.map_or(f64::INFINITY, |v| {
            (v[0][1] - v[0][0]).max(v[1][1] - v[1][0])
        })
    };
    let wa = width(ba);
    let wb = width(bb);
    if wa.max(wb) < 1e-4
        && let Some(hit) = refine(&a, &b, plane)
    {
        return Ok(vec![hit]);
    }
    let (a0, a1) = a.range_tuple();
    let (b0, b1) = b.range_tuple();
    if wa.max(wb) < 1e-3 && separated(&a, &b, plane)? {
        return Ok(vec![]);
    }
    if a1 - a0 < 1e-5 && b1 - b0 < 1e-5 {
        return refine(&a, &b, plane).map(|p| vec![p]).ok_or_else(|| {
            "Unresolved near-tangent or coincident curved trims; the model was preserved.".into()
        });
    }
    let split_a = (wa >= wb && a1 - a0 >= 1e-5) || b1 - b0 < 1e-5;
    let mut hits = if split_a {
        let mut left = a;
        let right = left.cut((a0 + a1) / 2.);
        let mut hits = curved_hits(left, b.clone(), plane, budget)?;
        hits.extend(curved_hits(right, b, plane, budget)?);
        hits
    } else {
        let mut left = b;
        let right = left.cut((b0 + b1) / 2.);
        let mut hits = curved_hits(a.clone(), left, plane, budget)?;
        hits.extend(curved_hits(a, right, plane, budget)?);
        hits
    };
    hits.sort_by(|a, b| a.0.total_cmp(&b.0));
    hits.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-8 && (a.1 - b.1).abs() < 1e-8);
    Ok(hits)
}
fn intersections(
    a: &Curve,
    b: &Curve,
    plane: &Plane,
    budget: &mut usize,
) -> Result<Vec<(f64, f64)>, String> {
    if matches!(a, Curve::Line(_)) {
        line_hits(a, b, plane, budget)
    } else if matches!(b, Curve::Line(_)) {
        line_hits(b, a, plane, budget).map(|v| v.into_iter().map(|(a, b)| (b, a)).collect())
    } else {
        curved_hits(a.clone(), b.clone(), plane, budget)
    }
}

/// Exact ray winding on retained analytic wires; no display triangles determine
/// whether a tool segment lies in the material of a holed/concave support face.
fn inside(
    wires: &[Wire],
    point: Point3,
    plane: &Plane,
    budget: &mut usize,
) -> Result<bool, String> {
    let mut winding = 0;
    for edge in wires.iter().flat_map(|w| w.iter()) {
        let c = edge.oriented_curve();
        let (lo, hi) = c.range_tuple();
        if matches!(c, Curve::Line(_)) {
            let a = c.subs(lo);
            let d = c.subs(hi) - a;
            let t = ((point - a).dot(d) / d.magnitude2()).clamp(0., 1.);
            if (a + d * t - point).magnitude() < EPS {
                return Ok(false);
            }
        }
        let mut parameters = roots(c.clone(), point, plane.v, budget)?;
        parameters.extend([lo, hi]);
        parameters.sort_by(f64::total_cmp);
        parameters.dedup_by(|a, b| (*a - *b).abs() < 1e-10);
        for pair in parameters.windows(2) {
            if (c.subs((pair[0] + pair[1]) / 2.) - point).dot(plane.v) <= 1e-10 {
                continue;
            }
            for (t, delta) in [(pair[0], -1), (pair[1], 1)] {
                let d = c.subs(t) - point;
                if d.dot(plane.v).abs() < 1e-9 && d.dot(plane.u) < -EPS {
                    winding += delta;
                }
                if d.magnitude() < EPS {
                    return Err(
                        "Coincident boundary classification is ambiguous at model tolerance."
                            .into(),
                    );
                }
            }
        }
    }
    Ok(winding > 0)
}
fn vertex_at(vertices: &mut Vec<Vertex>, point: Point3) -> Vertex {
    if let Some(v) = vertices
        .iter()
        .find(|v| (v.point() - point).magnitude() < EPS)
    {
        v.clone()
    } else {
        let v = Vertex::new(point);
        vertices.push(v.clone());
        v
    }
}

// A nondegenerate rational quadratic defines one implicit conic. Check the
// complete Bernstein coefficients of its implicit equation on both curves;
// point samples alone cannot establish that two trims share a support.
fn same_conic(a: &Curve, b: &Curve, plane: &Plane) -> bool {
    if matches!(a, Curve::Line(_) | Curve::IntersectionCurve(_))
        || matches!(b, Curve::Line(_) | Curve::IntersectionCurve(_))
    {
        return false;
    }
    let project = |p: truck_modeling::Vector4| {
        let d = p.truncate() - plane.origin.to_vec() * p.w;
        Vector3::new(d.dot(plane.u), d.dot(plane.v), p.w)
    };
    let a = a.clone().lift_up().bezier_decomposition();
    let Some(first) = a.first().filter(|c| c.degree() == 2) else {
        return false;
    };
    let c = first.control_points();
    let matrix = Matrix3::from_cols(project(c[0]), project(c[1]), project(c[2]));
    if matrix.determinant().abs()
        <= matrix.x.magnitude() * matrix.y.magnitude() * matrix.z.magnitude() * 1e-12
    {
        return false;
    }
    let Some(inverse) = matrix.invert() else {
        return false;
    };
    let binomial =
        |n: usize, k: usize| (0..k.min(n - k)).fold(1., |v, i| v * (n - i) as f64 / (i + 1) as f64);
    a.into_iter()
        .chain(b.clone().lift_up().bezier_decomposition())
        .all(|span| {
            let n = span.degree();
            if n > 32 {
                return false;
            }
            let points = span
                .control_points()
                .iter()
                .map(|p| inverse * project(*p))
                .collect::<Vec<_>>();
            let scale = points.iter().map(|p| p.magnitude2()).fold(0., f64::max);
            scale.is_finite()
                && scale > 0.
                && (0..=2 * n).all(|k| {
                    let residual = (k.saturating_sub(n)..=k.min(n))
                        .map(|i| {
                            let j = k - i;
                            let weight = binomial(n, i) * binomial(n, j) / binomial(2 * n, k);
                            weight
                                * (points[i].y * points[j].y
                                    - 2. * (points[i].x * points[j].z + points[i].z * points[j].x))
                        })
                        .sum::<f64>();
                    residual.abs() <= scale * 1e-11
                })
        })
}
fn point_parameters(
    curve: &Curve,
    point: Point3,
    plane: &Plane,
    budget: &mut usize,
) -> Result<Vec<f64>, String> {
    // Use a transverse coordinate line; try the other coordinate when a
    // tangent root cannot be isolated. Every returned parameter is validated.
    let mut result = Vec::new();
    let mut error = None;
    let mut resolved = false;
    for axis in [plane.u, plane.v] {
        match roots(curve.clone(), point, axis, budget) {
            Ok(values) => {
                resolved = true;
                result.extend(
                    values
                        .into_iter()
                        .filter(|t| (curve.subs(*t) - point).magnitude() < EPS),
                );
            }
            Err(reason) => error = Some(reason),
        }
    }
    let (lo, hi) = curve.range_tuple();
    result.extend(
        [lo, hi]
            .into_iter()
            .filter(|t| (curve.subs(*t) - point).magnitude() < EPS),
    );
    if *budget == 0 || !resolved {
        return Err(error.unwrap_or_else(|| "Coincident trim exceeded its root budget.".into()));
    }
    result.sort_by(f64::total_cmp);
    result.dedup_by(|a, b| (*a - *b).abs() < 1e-10);
    Ok(result)
}
fn conic_hits(
    a: &Curve,
    b: &Curve,
    plane: &Plane,
    budget: &mut usize,
) -> Result<Vec<(f64, f64)>, String> {
    let mut hits = Vec::new();
    let (a0, a1) = a.range_tuple();
    let (b0, b1) = b.range_tuple();
    for t in [a0, a1] {
        hits.extend(
            point_parameters(b, a.subs(t), plane, budget)?
                .into_iter()
                .map(|u| (t, u)),
        );
    }
    for u in [b0, b1] {
        hits.extend(
            point_parameters(a, b.subs(u), plane, budget)?
                .into_iter()
                .map(|t| (t, u)),
        );
    }
    Ok(hits)
}

// Call after splitting at both curves' endpoints. A piece on a proven common
// conic is either wholly on this interval or wholly outside it.
fn shared_conic_interval(
    a: &Curve,
    b: &Curve,
    plane: &Plane,
    budget: &mut usize,
) -> Result<bool, String> {
    let (lo, hi) = a.range_tuple();
    Ok(same_conic(a, b, plane)
        && !point_parameters(b, a.subs((lo + hi) / 2.), plane, budget)?.is_empty())
}

type Hits = HashMap<EdgeID, Vec<(f64, Vertex)>>;
fn crossings(
    sources: &[Edge],
    tool_wires: &[Wire],
    plane: &Plane,
) -> Result<(HashMap<EdgeID, Edge>, Hits, Hits), String> {
    let mut vertices: Vec<_> = sources
        .iter()
        .flat_map(|e| [e.front().clone(), e.back().clone()])
        .collect();
    let mut tool_edges = HashMap::<EdgeID, Edge>::new();
    for edge in tool_wires.iter().flat_map(|w| w.iter()) {
        let edge = if edge.orientation() {
            edge.clone()
        } else {
            edge.inverse()
        };
        let front = vertex_at(&mut vertices, edge.front().point());
        let back = vertex_at(&mut vertices, edge.back().point());
        tool_edges.insert(edge.id(), Edge::new(&front, &back, edge.curve()));
    }
    let mut source_hits = HashMap::<EdgeID, Vec<(f64, Vertex)>>::new();
    let mut tool_hits = HashMap::<EdgeID, Vec<(f64, Vertex)>>::new();
    let mut budget = 65536;
    for edge in sources {
        for tool in tool_wires.iter().flat_map(|w| w.iter()) {
            let reference = &tool_edges[&tool.id()];
            let a = edge.curve();
            let b = reference.curve();
            let hits = if same_conic(&a, &b, plane) {
                conic_hits(&a, &b, plane, &mut budget)?
            } else {
                intersections(&a, &b, plane, &mut budget)?
            };
            for (a, b) in hits {
                let point = edge.curve().subs(a);
                let vertex = vertex_at(&mut vertices, point);
                source_hits
                    .entry(edge.id())
                    .or_default()
                    .push((a, vertex.clone()));
                tool_hits.entry(tool.id()).or_default().push((b, vertex));
            }
        }
    }
    let order = |hits: &mut Vec<(f64, Vertex)>| {
        hits.sort_by(|a, b| a.0.total_cmp(&b.0));
        hits.dedup_by(|a, b| a.1 == b.1);
    };
    for hits in source_hits.values_mut().chain(tool_hits.values_mut()) {
        order(hits);
    }
    Ok((tool_edges, source_hits, tool_hits))
}

pub(crate) fn apply(
    source: &Geometry,
    element: usize,
    profiles: &[truck_modeling::Face],
    circles: &[crate::model::CircularEdge],
) -> Result<Geometry, String> {
    let plane = Plane::face(source, element).ok_or("Profile imprint requires a planar support.")?;
    let selected: Shell = source
        .shell
        .as_ref()
        .ok_or("Profile imprint requires retained boundaries.")?
        .iter()
        .zip(source.surface_mapping()?)
        .filter(|(_, role)| **role == element)
        .map(|(f, _)| f.clone())
        .collect();
    let boundaries = selected.extract_boundaries();
    let tools: Shell = profiles.iter().cloned().collect();
    let tool_wires = tools.extract_boundaries();
    let sources: Vec<_> = boundaries
        .iter()
        .flat_map(|w| w.iter())
        .map(|e| {
            if e.orientation() {
                e.clone()
            } else {
                e.inverse()
            }
        })
        .collect();
    let (tool_edges, source_hits, tool_hits) = crossings(&sources, &tool_wires, &plane)?;
    let mut budget = 65536;
    let mut retained = Vec::new();
    let mut complete_tools = HashSet::new();
    let mut active = HashSet::new();
    for edge in tool_wires.iter().flat_map(|w| w.iter()) {
        let pieces = split(
            &tool_edges[&edge.id()],
            tool_hits.get(&edge.id()).map_or(&[], |v| v.as_slice()),
        )?;
        let mut complete = true;
        for piece in oriented(edge, &pieces) {
            let c = piece.oriented_curve();
            let (lo, hi) = c.range_tuple();
            let mut boundary = false;
            for source in &sources {
                if shared_conic_interval(&c, &source.curve(), &plane, &mut budget)? {
                    boundary = true;
                    break;
                }
            }
            if boundary {
                continue;
            }
            if inside(&boundaries, c.subs((lo + hi) / 2.), &plane, &mut budget)? {
                active.insert(piece.front().id());
                active.insert(piece.back().id());
                retained.push(piece);
            } else {
                complete = false;
            }
        }
        if complete {
            complete_tools.insert(edge.id());
        }
    }
    if retained.is_empty() {
        return Err("The profile does not divide material on the selected face. Coincident boundaries alone do not create an independent region.".into());
    }
    let mut pieces = HashMap::new();
    for edge in &sources {
        let hits = source_hits.get(&edge.id()).map_or(vec![], |v| {
            v.iter()
                .filter(|(_, v)| active.contains(&v.id()))
                .cloned()
                .collect()
        });
        pieces.insert(edge.id(), split(edge, &hits)?);
    }
    let mut directed = Vec::new();
    for edge in boundaries.iter().flat_map(|w| w.iter()) {
        directed.extend(oriented(edge, &pieces[&edge.id()]));
    }
    for edge in retained {
        directed.push(edge.inverse());
        directed.push(edge);
    }
    let mut result = partition(source, element, plane, &pieces, directed)?;
    result.circles.extend(
        circles
            .iter()
            .filter(|circle| {
                tool_wires.iter().any(|wire| {
                    wire.iter().all(|edge| complete_tools.contains(&edge.id()))
                        && crate::face_extrusions::circle_on_wire(circle, wire)
                })
            })
            .cloned(),
    );
    Ok(result)
}

/// Coplanar face set operations retain their source trim curves. The same
/// intersection/splitting and loop assembly serve Imprint and Boolean tools.
pub(crate) fn boolean_faces(
    op: &crate::booleans::Boolean,
    geometry: &[&Geometry],
) -> Result<Geometry, String> {
    let element = op.operands[0].1.ok_or("Select a planar target face.")?;
    let plane = Plane::face(geometry[0], element).ok_or("Face Booleans require planar faces.")?;
    let mut boundaries = Vec::new();
    for (index, g) in geometry.iter().enumerate() {
        let face = op.operands[index].1.ok_or("Select a planar tool face.")?;
        let other = Plane::face(g, face).ok_or("Face Booleans require planar faces.")?;
        if plane.normal.dot(other.normal).abs() < 1. - 1e-8
            || (other.origin - plane.origin).dot(plane.normal).abs() > EPS
        {
            return Err("Face Booleans require coplanar planar faces.".into());
        }
        let shell: Shell = g
            .shell
            .as_ref()
            .ok_or("Analytic face Booleans require retained boundaries.")?
            .iter()
            .zip(g.surface_mapping()?)
            .filter(|(_, role)| **role == face)
            .map(|(f, _)| {
                if plane.normal.dot(other.normal) < 0. {
                    f.inverse()
                } else {
                    f.clone()
                }
            })
            .collect();
        boundaries.push(shell.extract_boundaries());
    }
    let sources = boundaries[0]
        .iter()
        .flat_map(|w| w.iter())
        .map(|e| {
            if e.orientation() {
                e.clone()
            } else {
                e.inverse()
            }
        })
        .collect::<Vec<_>>();
    let (tools, source_hits, tool_hits) = crossings(&sources, &boundaries[1], &plane)?;
    let mut directed = Vec::<Edge>::new();
    let mut budget = 65536;
    let mut seen_lines = HashSet::new();
    let mut seen_curves = Vec::<Edge>::new();
    for (index, wires) in boundaries.iter().enumerate() {
        for edge in wires.iter().flat_map(|w| w.iter()) {
            let original = if index == 0 {
                sources.iter().find(|e| e.id() == edge.id()).unwrap()
            } else {
                &tools[&edge.id()]
            };
            let hits = if index == 0 { &source_hits } else { &tool_hits };
            let pieces = split(original, hits.get(&edge.id()).map_or(&[], Vec::as_slice))?;
            for piece in oriented(edge, &pieces) {
                if matches!(piece.curve(), Curve::Line(_)) {
                    let pair = (piece.front().id(), piece.back().id());
                    if seen_lines.contains(&(pair.1, pair.0)) || !seen_lines.insert(pair) {
                        continue;
                    }
                }
                let curve = piece.oriented_curve();
                let (lo, hi) = curve.range_tuple();
                let middle = (lo + hi) / 2.;
                if !matches!(curve, Curve::Line(_)) {
                    let mut duplicate = false;
                    for other in &seen_curves {
                        let same_ends = (piece.front() == other.front()
                            && piece.back() == other.back())
                            || (piece.front() == other.back() && piece.back() == other.front());
                        if same_ends
                            && shared_conic_interval(&curve, &other.curve(), &plane, &mut budget)?
                        {
                            duplicate = true;
                            break;
                        }
                    }
                    if duplicate {
                        continue;
                    }
                    seen_curves.push(piece.clone());
                }
                let point = curve.subs(middle);
                let tangent = curve.der(middle);
                if tangent.magnitude2() < 1e-24 {
                    return Err("Degenerate Boolean trim tangent.".into());
                }
                let left = plane.normal.cross(tangent).normalize() * (EPS * 4.);
                let mut material = [false; 2];
                for (side, offset) in [left, -left].into_iter().enumerate() {
                    material[side] = op.kind.contains(
                        inside(&boundaries[0], point + offset, &plane, &mut budget)?,
                        inside(&boundaries[1], point + offset, &plane, &mut budget)?,
                    );
                }
                match material {
                    [true, false] => directed.push(piece),
                    [false, true] => directed.push(piece.inverse()),
                    _ => {}
                }
            }
        }
    }
    if directed.is_empty() {
        return Ok(Geometry::default());
    }
    let faces = crate::curve_imprints::planar_arrangement(plane, directed)?;
    let shell: Shell = faces.into_iter().collect();
    if !shell.is_geometric_consistent() {
        return Err("The Boolean trims are not geometrically consistent.".into());
    }
    let mut result = crate::model::tessellate(&shell)?;
    result.circles = geometry
        .iter()
        .flat_map(|g| &g.circles)
        .filter(|circle| {
            shell
                .iter()
                .flat_map(|f| f.boundaries())
                .any(|wire| crate::face_extrusions::circle_on_wire(circle, &wire))
        })
        .cloned()
        .collect();
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        booleans::{Boolean, Kind},
        edge_treatments::FaceTarget,
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
    fn coincident_rim_imprints_keep_shared_neighbors_and_independent_caps() {
        use crate::{model::Model, project::Project, viewport::Camera};
        use truck_modeling::{Matrix4, Transform};
        for shape in [Shape::Circle, Shape::Semicircle] {
            for angle in [0., 17., 90.] {
                if shape == Shape::Semicircle && angle == 0. {
                    continue;
                }
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
                    let placement = crate::regions::transform(&source);
                    let mut tool = Body::new(Shape::Semicircle);
                    tool.id = 2;
                    tool.width = 4.;
                    tool.position = placement.transform_point(Point3::new(0., 0., 3.)).into();
                    tool.rotation = source.rotation;
                    tool.scale = source.scale;
                    let g = crate::model::evaluate(&tool).unwrap();
                    let frame = crate::regions::transform(&tool);
                    tool = crate::components::transformed(
                        &tool,
                        &g,
                        SelectionMode::Face,
                        Some(0),
                        frame
                            * Matrix4::from_angle_z(truck_modeling::Deg(angle))
                            * frame.invert().unwrap(),
                    )
                    .unwrap();
                    let bodies = vec![source, tool];
                    let scene = evaluate_scene(&bodies).unwrap();
                    let cap = (0..scene[0].1.face_count)
                        .find(|f| {
                            Plane::face(&scene[0].1, *f).is_some_and(|p| {
                                p.normal
                                    .dot(placement.transform_vector(Vector3::unit_z()).normalize())
                                    > 0.99
                            })
                        })
                        .unwrap();
                    let mut feature = Body::new(Shape::Rectangle);
                    feature.id = 3;
                    feature.boolean = Some(Boolean::new(
                        Kind::Imprint,
                        &[(1, Some(cap)), (2, None)],
                        SelectionMode::Face,
                        &scene,
                    ));
                    let mut model = Model::from_bodies(bodies.clone());
                    model.commit(feature.clone()).unwrap();
                    let out = evaluate_scene(&model.bodies).unwrap();
                    let g = &out[2].1;
                    assert_eq!(g.face_count, scene[0].1.face_count + 1);
                    assert!((volume(g) - volume(&scene[0].1)).abs() < 0.08);
                    assert!(g.shell.as_ref().unwrap().is_geometric_consistent());
                    crate::components::solid_parts(g.shell.as_ref().unwrap()).unwrap();
                    let normal = placement.transform_vector(Vector3::unit_z()).normalize();
                    let caps = (0..g.face_count)
                        .filter(|f| Plane::face(g, *f).is_some_and(|p| p.normal.dot(normal) > 0.99))
                        .collect::<Vec<_>>();
                    assert_eq!(caps.len(), 2);
                    for cap in caps {
                        let mut changed = model.bodies.clone();
                        changed[2] = crate::components::sweep_face(&feature, g, cap, 0.4).unwrap();
                        let swept = evaluate_scene(&changed).unwrap();
                        assert!(volume(&swept[2].1) > volume(g));
                        assert!(swept[2].1.shell.as_ref().unwrap().is_geometric_consistent());
                        assert!(
                            !crate::export::encode(&changed, crate::export::Format::Step)
                                .unwrap()
                                .is_empty()
                        );
                        if shape == Shape::Circle && angle == 17. && !affine {
                            changed[2] =
                                crate::components::sweep_face(&feature, g, cap, -4.).unwrap();
                            let cut = evaluate_scene(&changed).unwrap();
                            assert!((volume(&cut[2].1) - volume(g) / 2.).abs() < 0.08);
                            assert!(cut[2].1.shell.as_ref().unwrap().is_geometric_consistent());
                            for format in crate::export::Format::ALL {
                                assert!(
                                    !crate::export::encode(&changed, format).unwrap().is_empty()
                                );
                            }
                            let project =
                                Project::new(changed.clone(), Camera::default(), true, Some(3));
                            assert!(changed[2].component_edits.last().unwrap().prismatic_cut);
                            let mut old = serde_json::to_value(&project).unwrap();
                            old["modeling_version"] = 29.into();
                            assert!(
                                serde_json::from_value::<Project>(old)
                                    .unwrap()
                                    .validate()
                                    .unwrap_err()
                                    .contains("30")
                            );
                            let file = tempfile::NamedTempFile::new().unwrap().into_temp_path();
                            project.save(file.as_ref()).unwrap();
                            let restored = Project::load(file.as_ref()).unwrap();
                            assert_eq!(restored.bodies, changed);
                            assert_eq!(
                                evaluate_scene(&restored.bodies).unwrap()[2].1.topology,
                                cut[2].1.topology
                            );
                        }
                    }
                    let accepted = model.bodies.clone();
                    model.undo();
                    assert_eq!(model.bodies, bodies);
                    model.redo();
                    assert_eq!(model.bodies, accepted);
                    let project = Project::new(accepted.clone(), Camera::default(), true, Some(3));
                    let file = tempfile::NamedTempFile::new().unwrap().into_temp_path();
                    project.save(file.as_ref()).unwrap();
                    assert_eq!(Project::load(file.as_ref()).unwrap().bodies, accepted);
                    let mut invalid = accepted[1].clone();
                    invalid.position[2] += 0.2;
                    assert!(model.commit(invalid).is_err());
                    assert_eq!(model.bodies, accepted);
                }
            }
        }
    }

    #[test]
    fn coincident_conic_face_sets_keep_exact_partial_and_reparameterized_arcs() {
        for (a_shape, b_shape, angle, areas) in [
            (Shape::Circle, Shape::Circle, 0., [4., 0., 4.]),
            (Shape::Circle, Shape::Circle, 17., [4., 0., 4.]),
            (Shape::Circle, Shape::Semicircle, 0., [4., 2., 2.]),
            (Shape::Semicircle, Shape::Circle, 0., [4., 0., 2.]),
            (Shape::Semicircle, Shape::Semicircle, 90., [3., 1., 1.]),
        ] {
            for affine in [false, true] {
                let mut a = Body::new(a_shape);
                a.id = 1;
                a.width = 4.;
                let mut b = Body::new(b_shape);
                b.id = 2;
                b.width = 4.;
                b.rotation[2] = angle;
                if affine {
                    // Shared nonuniform scale makes an ellipse in a rotated plane.
                    a.position = [2., -3., 1.];
                    a.rotation = [15., 25., 35.];
                    a.scale = [1.4, 0.7, 1.2];
                    let placement = crate::regions::transform(&a);
                    b.rotation = a.rotation;
                    b.scale = a.scale;
                    b.position = a.position;
                    // Keep the local arc-start rotation before nonuniform placement.
                    let g = crate::model::evaluate(&b).unwrap();
                    let world = placement
                        * truck_modeling::Matrix4::from_angle_z(truck_modeling::Deg(angle))
                        * placement.invert().unwrap();
                    b = crate::components::transformed(&b, &g, SelectionMode::Face, Some(0), world)
                        .unwrap();
                }
                let sources = vec![a, b];
                let scene = evaluate_scene(&sources).unwrap();
                for (index, kind) in [Kind::Union, Kind::Subtract, Kind::Intersect]
                    .into_iter()
                    .enumerate()
                {
                    let operation = Boolean::new(
                        kind,
                        &[(1, Some(0)), (2, Some(0))],
                        SelectionMode::Face,
                        &scene,
                    );
                    let out=crate::booleans::faces(&operation,&[&scene[0].1,&scene[1].1]).unwrap_or_else(|e|panic!("{a_shape:?} {b_shape:?} angle={angle} affine={affine} {kind:?}: {e}"));
                    let area = out
                        .triangles
                        .iter()
                        .map(|t| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
                        .sum::<f64>();
                    let expected =
                        areas[index] * std::f64::consts::PI * if affine { 1.4 * 0.7 } else { 1. };
                    assert!(
                        (area - expected).abs() < 0.08,
                        "{a_shape:?} {b_shape:?} angle={angle} affine={affine} {kind:?}: {area} != {expected}"
                    );
                    if expected > 0. {
                        assert!(out.shell.as_ref().unwrap().is_geometric_consistent());
                        assert!(
                            out.shell
                                .as_ref()
                                .unwrap()
                                .iter()
                                .flat_map(|f| f.edge_iter())
                                .any(|e| !matches!(e.curve(), Curve::Line(_)))
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn coincident_trim_chains_preserve_history_and_reject_false_supports() {
        use crate::{model::Model, project::Project, viewport::Camera};
        let mut a = Body::new(Shape::Circle);
        a.id = 1;
        a.width = 4.;
        let mut b = Body::new(Shape::Semicircle);
        b.id = 2;
        b.width = 4.;
        let mut c = b.clone();
        c.id = 3;
        c.rotation[2] = 90.;
        let mut bodies = vec![a, b, c];
        let scene = evaluate_scene(&bodies).unwrap();
        let plane = Plane::face(&scene[0].1, 0).unwrap();
        let curve = scene[0].1.shell.as_ref().unwrap()[0].boundaries()[0][0].curve();
        assert!(same_conic(&curve, &curve, &plane));
        let mut shifted = curve.clone();
        if let Curve::NurbsCurve(c) = &mut shifted {
            c.control_point_mut(1).x += 0.001;
        } else {
            panic!("Expected analytic rational arc");
        }
        assert!(
            !same_conic(&curve, &shifted, &plane),
            "Nearby curves must not be silently merged"
        );
        let mut clipped = Body::new(Shape::Rectangle);
        clipped.id = 4;
        clipped.boolean = Some(Boolean::new(
            Kind::Intersect,
            &[(1, Some(0)), (2, Some(0))],
            SelectionMode::Face,
            &scene,
        ));
        bodies.push(clipped);
        let scene = evaluate_scene(&bodies).unwrap();
        for (kind, area) in [
            (Kind::Union, 3.),
            (Kind::Subtract, 1.),
            (Kind::Intersect, 1.),
        ] {
            let mut result = Body::new(Shape::Rectangle);
            result.id = 5;
            result.boolean = Some(Boolean::new(
                kind,
                &[(4, Some(0)), (3, Some(0))],
                SelectionMode::Face,
                &scene,
            ));
            let mut model = Model::from_bodies(bodies.clone());
            model.commit(result).unwrap();
            let out = evaluate_scene(&model.bodies).unwrap();
            let measured = out[4]
                .1
                .triangles
                .iter()
                .map(|t| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
                .sum::<f64>();
            assert!((measured - area * std::f64::consts::PI).abs() < 0.08);
            assert!(out[4].1.shell.as_ref().unwrap().is_geometric_consistent());
            assert!(
                !crate::export::encode(&model.bodies, crate::export::Format::Step)
                    .unwrap()
                    .is_empty()
            );
            let accepted = model.bodies.clone();
            model.undo();
            assert_eq!(model.bodies, bodies);
            model.redo();
            assert_eq!(model.bodies, accepted);
            let project = Project::new(accepted.clone(), Camera::default(), true, Some(5));
            let file = tempfile::NamedTempFile::new().unwrap().into_temp_path();
            project.save(file.as_ref()).unwrap();
            assert_eq!(Project::load(file.as_ref()).unwrap().bodies, accepted);
            let mut invalid = accepted[2].clone();
            invalid.position[2] = 0.2;
            assert!(model.commit(invalid).is_err());
            assert_eq!(model.bodies, accepted);
        }
    }

    #[test]
    fn analytic_boolean_chains_keep_input_voids_and_split_outputs() {
        let mut a = Body::new(Shape::Circle);
        a.id = 1;
        a.width = 6.;
        let mut b = Body::new(Shape::Circle);
        b.id = 2;
        b.width = 2.;
        let mut c = Body::new(Shape::Circle);
        c.id = 3;
        c.width = 4.;
        c.position[0] = 2.5;
        let mut bodies = vec![a, b, c];
        let scene = evaluate_scene(&bodies).unwrap();
        let mut ring = Body::new(Shape::Rectangle);
        ring.id = 4;
        ring.boolean = Some(Boolean::new(
            Kind::Subtract,
            &[(1, Some(0)), (2, Some(0))],
            SelectionMode::Face,
            &scene,
        ));
        bodies.push(ring);
        let scene = evaluate_scene(&bodies).unwrap();
        assert_eq!(scene[3].1.shell.as_ref().unwrap()[0].boundaries().len(), 2);
        for kind in [Kind::Union, Kind::Subtract, Kind::Intersect] {
            let mut result = Body::new(Shape::Rectangle);
            result.id = 5;
            result.boolean = Some(Boolean::new(
                kind,
                &[(4, Some(0)), (3, Some(0))],
                SelectionMode::Face,
                &scene,
            ));
            let mut chain = bodies.clone();
            chain.push(result);
            let out = evaluate_scene(&chain).unwrap();
            assert!(out[4].1.shell.as_ref().unwrap().is_geometric_consistent());
            let plane = Plane::face(&out[4].1, 0).unwrap();
            let wires = out[4].1.shell.as_ref().unwrap().extract_boundaries();
            let mut budget = 65536;
            assert!(
                !inside(&wires, Point3::new(0., 0., 0.), &plane, &mut budget).unwrap(),
                "Original void is never filled by the offset tool"
            );
        }
        let mut a = Body::new(Shape::Circle);
        a.id = 1;
        a.width = 4.;
        let mut b = Body::new(Shape::Rectangle);
        b.id = 2;
        b.width = 1.;
        b.depth = 6.;
        let mut split = vec![a, b];
        let scene = evaluate_scene(&split).unwrap();
        let mut result = Body::new(Shape::Rectangle);
        result.id = 3;
        result.boolean = Some(Boolean::new(
            Kind::Subtract,
            &[(1, Some(0)), (2, Some(0))],
            SelectionMode::Face,
            &scene,
        ));
        split.push(result);
        let out = evaluate_scene(&split).unwrap();
        assert_eq!(out[2].1.face_count, 2);
        assert!(
            out[2]
                .1
                .shell
                .as_ref()
                .unwrap()
                .iter()
                .all(|f| f.edge_iter().any(|e| !matches!(e.curve(), Curve::Line(_))))
        );
    }

    #[test]
    fn mixed_analytic_face_booleans_and_solid_face_inputs_share_the_arrangement() {
        for a_shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            for b_shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
                let mut a = Body::new(a_shape);
                a.id = 1;
                a.width = 4.;
                a.depth = 4.;
                let mut b = Body::new(b_shape);
                b.id = 2;
                b.width = 3.;
                b.depth = 3.;
                b.position = [0.7, 0.3, 0.];
                let scene = evaluate_scene(&[a.clone(), b.clone()]).unwrap();
                for kind in [Kind::Union, Kind::Subtract, Kind::Intersect] {
                    for reverse in [false, true] {
                        let picks = if reverse {
                            [(2, Some(0)), (1, Some(0))]
                        } else {
                            [(1, Some(0)), (2, Some(0))]
                        };
                        let mut operation = Boolean::new(kind, &picks, SelectionMode::Face, &scene);
                        let operands = if reverse {
                            [&scene[1].1, &scene[0].1]
                        } else {
                            [&scene[0].1, &scene[1].1]
                        };
                        let exact = crate::booleans::faces(&operation, &operands).unwrap();
                        operation.analytic_faces = false;
                        let old = crate::booleans::faces(&operation, &operands).unwrap();
                        let area = |g: &Geometry| {
                            g.triangles
                                .iter()
                                .map(|t| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
                                .sum::<f64>()
                        };
                        assert!(
                            (area(&exact) - area(&old)).abs() < 0.08,
                            "{a_shape:?} {b_shape:?} {kind:?} reverse={reverse}"
                        );
                        assert!(!exact.is_solid);
                        assert!(exact.shell.as_ref().unwrap().is_geometric_consistent());
                    }
                }
            }
        }
        // Picking caps produces a new analytic sheet; it does not consume solids.
        let mut a = Body::new(Shape::Circle);
        a.id = 1;
        a.width = 4.;
        a.extrusion = 2.;
        let mut b = Body::new(Shape::Circle);
        b.id = 2;
        b.width = 3.;
        b.position = [1.2, 0., 0.];
        b.extrusion = 2.;
        let sources = vec![a, b];
        let scene = evaluate_scene(&sources).unwrap();
        let cap = |g: &Geometry| {
            (0..g.face_count)
                .find(|f| Plane::face(g, *f).is_some_and(|p| p.normal.z > 0.99))
                .unwrap()
        };
        let mut feature = Body::new(Shape::Rectangle);
        feature.id = 3;
        feature.boolean = Some(Boolean::new(
            Kind::Subtract,
            &[(1, Some(cap(&scene[0].1))), (2, Some(cap(&scene[1].1)))],
            SelectionMode::Face,
            &scene,
        ));
        let mut bodies = sources;
        bodies.push(feature);
        let out = evaluate_scene(&bodies).unwrap();
        assert!(out[0].1.is_solid && out[1].1.is_solid && !out[2].1.is_solid);
        assert!(
            out[2]
                .1
                .shell
                .as_ref()
                .unwrap()
                .iter()
                .flat_map(|f| f.edge_iter())
                .all(|e| !matches!(e.curve(), Curve::Line(_)))
        );
    }

    #[test]
    fn analytic_face_booleans_keep_arcs_order_holes_and_saved_behavior() {
        use crate::{model::Model, project::Project, viewport::Camera};
        use truck_modeling::Transform;
        for offset in [0., 1.2, 6.] {
            for kind in [Kind::Union, Kind::Subtract, Kind::Intersect] {
                for reverse in [false, true] {
                    let mut a = Body::new(Shape::Circle);
                    a.id = 1;
                    a.width = 4.;
                    let mut b = Body::new(Shape::Circle);
                    b.id = 2;
                    b.width = 3.;
                    b.position[0] = offset;
                    // Opposite sheet normals describe the same planar regions.
                    if reverse {
                        b.rotation[0] = 180.;
                    }
                    let sources = vec![a, b];
                    let scene = evaluate_scene(&sources).unwrap();
                    let mut feature = Body::new(Shape::Rectangle);
                    feature.id = 3;
                    feature.boolean = Some(Boolean::new(
                        kind,
                        &[(1, Some(0)), (2, Some(0))],
                        SelectionMode::Face,
                        &scene,
                    ));
                    let mut model = Model::from_bodies(sources.clone());
                    model.commit(feature.clone()).unwrap();
                    let out = evaluate_scene(&model.bodies).unwrap();
                    let g = &out[2].1;
                    let area = g
                        .triangles
                        .iter()
                        .map(|t| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
                        .sum::<f64>();
                    let pi = std::f64::consts::PI;
                    let lens = if offset == 0. {
                        pi * 2.25
                    } else if offset > 3.5 {
                        0.
                    } else {
                        let d: f64 = offset;
                        4. * ((d * d + 4. - 2.25) / (4. * d)).acos()
                            + 2.25 * ((d * d + 2.25 - 4.) / (3. * d)).acos()
                            - 0.5 * ((-d + 3.5) * (d + 0.5) * (d - 0.5) * (d + 3.5)).sqrt()
                    };
                    let expected = match kind {
                        Kind::Union => pi * 6.25 - lens,
                        Kind::Subtract => pi * 4. - lens,
                        Kind::Intersect => lens,
                        _ => unreachable!(),
                    };
                    assert!(
                        (area - expected).abs() < 0.08,
                        "{kind:?} offset={offset} reverse={reverse} area={area} expected={expected}"
                    );
                    if expected > 0. {
                        let shell = g.shell.as_ref().unwrap();
                        assert!(shell.is_geometric_consistent());
                        assert!(
                            shell
                                .iter()
                                .flat_map(|f| f.edge_iter())
                                .all(|e| !matches!(e.curve(), Curve::Line(_)))
                        );
                        if offset == 0. && kind == Kind::Subtract {
                            assert_eq!(shell[0].boundaries().len(), 2);
                            assert_eq!(g.circles.len(), 2);
                        }
                        assert!(
                            !crate::export::encode(&model.bodies, crate::export::Format::Step)
                                .unwrap()
                                .is_empty()
                        );
                    }
                    model.undo();
                    assert_eq!(model.bodies, sources);
                    model.redo();
                    assert_eq!(model.bodies.len(), 3);
                    let project =
                        Project::new(model.bodies.clone(), Camera::default(), true, Some(3));
                    let file = tempfile::NamedTempFile::new().unwrap().into_temp_path();
                    project.save(file.as_ref()).unwrap();
                    assert_eq!(Project::load(file.as_ref()).unwrap().bodies, model.bodies);
                    let mut old = serde_json::to_value(&project).unwrap();
                    old["schema_version"] = 28.into();
                    assert!(
                        serde_json::from_value::<Project>(old)
                            .unwrap()
                            .validate()
                            .unwrap_err()
                            .contains("29")
                    );
                    // Explicit false keeps earlier polygonal face evaluation.
                    let mut legacy = model.bodies.clone();
                    legacy[2].boolean.as_mut().unwrap().analytic_faces = false;
                    let legacy_scene = evaluate_scene(&legacy).unwrap();
                    if expected > 0. {
                        assert!(
                            legacy_scene[2]
                                .1
                                .shell
                                .as_ref()
                                .unwrap()
                                .iter()
                                .flat_map(|f| f.edge_iter())
                                .all(|e| matches!(e.curve(), Curve::Line(_)))
                        );
                    }
                    if offset == 1.2 && !reverse {
                        let mut changed = sources[1].clone();
                        changed.position[0] += 0.1;
                        model.commit(changed).unwrap();
                        assert_ne!(evaluate_scene(&model.bodies).unwrap()[2].1.points, g.points);
                        let stable = model.bodies.clone();
                        let mut invalid = stable[1].clone();
                        invalid.position[2] = 0.2;
                        assert!(model.commit(invalid).is_err());
                        assert_eq!(model.bodies, stable);
                        // The arrangement is independent of world XY and nonuniform scaling.
                        let mut affine = stable.clone();
                        let mut frame = sources[0].clone();
                        frame.position = [2., -3., 1.];
                        frame.rotation = [15., 25., 35.];
                        frame.scale = [1.4, 0.7, 1.2];
                        let matrix = crate::regions::transform(&frame);
                        for body in &mut affine[..2] {
                            body.position =
                                matrix.transform_point(Point3::from(body.position)).into();
                            body.rotation = frame.rotation;
                            body.scale = frame.scale;
                        }
                        let transformed = evaluate_scene(&affine).unwrap();
                        assert!(
                            transformed[2]
                                .1
                                .shell
                                .as_ref()
                                .unwrap()
                                .is_geometric_consistent()
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn coincident_and_offset_profiles_reject_without_mutating_the_model() {
        let mut source = Body::new(Shape::Circle);
        source.id = 1;
        source.width = 4.;
        source.extrusion = 3.;
        let mut tool = Body::new(Shape::Circle);
        tool.id = 2;
        tool.width = 4.;
        tool.position = [0., 0., 3.];
        let bodies = vec![source.clone(), tool];
        let scene = evaluate_scene(&bodies).unwrap();
        let face = (0..scene[0].1.face_count)
            .find(|f| {
                crate::edge_treatments::target(&source, &scene[0].1, *f) == Ok(FaceTarget::Top)
            })
            .unwrap();
        let mut feature = Body::new(Shape::Rectangle);
        feature.id = 3;
        feature.boolean = Some(Boolean::new(
            Kind::Imprint,
            &[(1, Some(face)), (2, Some(0))],
            SelectionMode::Face,
            &scene,
        ));
        let mut model = crate::model::Model::from_bodies(bodies.clone());
        assert!(
            model
                .commit(feature.clone())
                .unwrap_err()
                .contains("Coincident")
        );
        assert_eq!(model.bodies, bodies);
        let mut budget = 0;
        let curve = scene[1].1.shell.as_ref().unwrap()[0].boundaries()[0][0].curve();
        let plane = Plane::face(&scene[0].1, face).unwrap();
        assert!(
            curved_hits(curve.clone(), curve, &plane, &mut budget)
                .unwrap_err()
                .contains("budget")
        );
        feature.boolean.as_mut().unwrap().kind = Kind::Union;
        assert!(
            feature
                .validate()
                .unwrap_err()
                .contains("Analytic profile crossings")
        );
    }

    #[test]
    fn curved_crossings_retain_holes_live_affine_inputs_and_undo() {
        use truck_modeling::Transform;
        for hollow in [false, true] {
            let mut source = Body::new(Shape::Circle);
            source.id = 1;
            source.width = 6.;
            source.extrusion = 3.;
            if hollow {
                source.hollow = Some(crate::shelling::Hollow {
                    opening: FaceTarget::Top,
                    thickness: 0.4,
                    outward: false,
                    closed: false,
                });
            }
            source.position = [2., -3., 1.];
            source.rotation = [15., 25., 35.];
            source.scale = [1.4, 0.7, 1.2];
            let frame = crate::regions::transform(&source);
            let tool_at = |x: f64| {
                let mut tool = Body::new(Shape::Circle);
                tool.id = 2;
                tool.width = 3.;
                tool.position = frame.transform_point(Point3::new(x, 0.2, 3.)).into();
                tool.rotation = source.rotation;
                tool.scale = source.scale;
                tool
            };
            let mut bodies = vec![source.clone(), tool_at(2.)];
            let scene = evaluate_scene(&bodies).unwrap();
            let normal = frame.transform_vector(Vector3::unit_z()).normalize();
            let origin = frame.transform_point(Point3::new(0., 0., 3.));
            let face = (0..scene[0].1.face_count)
                .find(|f| {
                    Plane::face(&scene[0].1, *f).is_some_and(|p| {
                        p.normal.dot(normal) > 0.999 && (p.origin - origin).dot(normal).abs() < EPS
                    })
                })
                .unwrap();
            let mut feature = Body::new(Shape::Rectangle);
            feature.id = 3;
            feature.boolean = Some(Boolean::new(
                Kind::Imprint,
                &[(1, Some(face)), (2, None)],
                SelectionMode::Face,
                &scene,
            ));
            bodies.push(feature.clone());
            let out = evaluate_scene(&bodies).unwrap();
            let g = &out[2].1;
            assert!(g.face_count > scene[0].1.face_count);
            assert!((volume(g) - volume(&scene[0].1)).abs() < 0.04);
            assert!(g.shell.as_ref().unwrap().is_geometric_consistent());
            let mut model = crate::model::Model::from_bodies(bodies.clone());
            model.edit(tool_at(2.2)).unwrap();
            model.finish_edit();
            let changed = model.bodies.clone();
            assert_ne!(evaluate_scene(&changed).unwrap()[2].1.points, g.points);
            model.undo();
            assert_eq!(model.bodies, bodies);
            model.redo();
            assert_eq!(model.bodies, changed);
            let mut invalid = tool_at(2.2);
            invalid.position[2] += 0.2;
            assert!(model.edit(invalid).is_err());
            assert_eq!(model.bodies, changed);
            let mut stale = feature.clone();
            stale.boolean.as_mut().unwrap().topology[0].edges.pop();
            assert!(model.edit(stale).is_err());
            assert_eq!(model.bodies, changed);
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("curved-crossing.osk");
            let project = crate::project::Project::new(
                changed.clone(),
                crate::viewport::Camera::default(),
                true,
                Some(3),
            );
            project.save(&path).unwrap();
            assert_eq!(
                crate::project::Project::load(&path).unwrap().bodies,
                changed
            );
            let mut old = serde_json::to_value(&project).unwrap();
            old["modeling_version"] = 26.into();
            assert!(
                serde_json::from_value::<crate::project::Project>(old)
                    .unwrap()
                    .validate()
                    .unwrap_err()
                    .contains("27")
            );
            for format in crate::export::Format::ALL {
                assert!(!crate::export::encode(&changed, format).unwrap().is_empty());
            }
            let mut keep = feature.clone();
            keep.boolean.as_mut().unwrap().keep_tools = true;
            model.edit(keep).unwrap();
            model.finish_edit();
            assert!(
                !evaluate_scene(&model.bodies).unwrap()[1]
                    .1
                    .triangles
                    .is_empty()
            );
            model.delete(3).unwrap();
            assert!(
                !evaluate_scene(&model.bodies).unwrap()[0]
                    .1
                    .triangles
                    .is_empty()
            );
        }
    }
    #[test]
    fn disconnected_crossing_and_contained_analytic_regions_share_one_partition() {
        let mut source = Body::new(Shape::Circle);
        source.id = 1;
        source.width = 6.;
        source.extrusion = 3.;
        let mut crossing = Body::new(Shape::Circle);
        crossing.id = 2;
        crossing.width = 3.;
        crossing.position = [2.5, 0., 3.];
        let mut inner = crossing.clone();
        inner.id = 3;
        inner.width = 0.8;
        inner.position = [-1., 0., 3.];
        let mut bodies = vec![source.clone(), crossing, inner];
        let scene = evaluate_scene(&bodies).unwrap();
        let mut combined = Body::new(Shape::Rectangle);
        combined.id = 4;
        combined.boolean = Some(Boolean::new(
            Kind::Union,
            &[(2, Some(0)), (3, Some(0))],
            SelectionMode::Face,
            &scene,
        ));
        bodies.push(combined);
        let scene = evaluate_scene(&bodies).unwrap();
        let face = (0..scene[0].1.face_count)
            .find(|f| {
                crate::edge_treatments::target(&source, &scene[0].1, *f) == Ok(FaceTarget::Top)
            })
            .unwrap();
        let mut feature = Body::new(Shape::Rectangle);
        feature.id = 5;
        feature.boolean = Some(Boolean::new(
            Kind::Imprint,
            &[(1, Some(face)), (4, None)],
            SelectionMode::Face,
            &scene,
        ));
        bodies.push(feature);
        let out = evaluate_scene(&bodies).unwrap();
        assert!((volume(&out[4].1) - volume(&scene[0].1)).abs() < 0.03);
        assert!(out[4].1.face_count >= scene[0].1.face_count + 2);
        assert!(out[4].1.shell.as_ref().unwrap().is_geometric_consistent());
    }

    #[test]
    fn closed_profiles_cross_curved_trims_without_polygonal_substitution() {
        for host in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            for profile in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
                let mut source = Body::new(host);
                source.id = 1;
                source.width = 4.;
                source.depth = 4.;
                source.extrusion = 3.;
                let mut tool = Body::new(profile);
                tool.id = 2;
                tool.width = 3.;
                tool.depth = 1.;
                tool.position = [1., if host == Shape::Semicircle { 0.5 } else { 0. }, 3.];
                let mut bodies = vec![source.clone(), tool];
                let scene = evaluate_scene(&bodies).unwrap();
                let face = (0..scene[0].1.face_count)
                    .find(|f| {
                        crate::edge_treatments::target(&source, &scene[0].1, *f)
                            == Ok(FaceTarget::Top)
                    })
                    .unwrap();
                let mut feature = Body::new(Shape::Rectangle);
                feature.id = 3;
                feature.boolean = Some(Boolean::new(
                    Kind::Imprint,
                    &[(1, Some(face)), (2, Some(0))],
                    SelectionMode::Face,
                    &scene,
                ));
                bodies.push(feature.clone());
                let result =
                    evaluate_scene(&bodies).unwrap_or_else(|e| panic!("{host:?}/{profile:?}: {e}"));
                let g = &result[2].1;
                assert!(g.is_solid);
                assert!(g.face_count > scene[0].1.face_count);
                assert!(
                    (volume(g) - volume(&scene[0].1)).abs() < 0.03,
                    "{host:?}/{profile:?}"
                );
                assert!(g.shell.as_ref().unwrap().is_geometric_consistent());
                if host != Shape::Rectangle || profile != Shape::Rectangle {
                    assert!(
                        g.shell
                            .as_ref()
                            .unwrap()
                            .edge_iter()
                            .any(|e| matches!(e.curve(), Curve::NurbsCurve(_)))
                    );
                }
                let caps: Vec<_> = (0..g.face_count)
                    .filter(|f| {
                        Plane::face(g, *f)
                            .is_some_and(|p| p.normal.z > 0.99 && (p.origin.z - 3.).abs() < EPS)
                    })
                    .collect();
                assert!(caps.len() >= 2);
                for cap in caps {
                    let changed = crate::components::sweep_face(&feature, g, cap, 0.25).unwrap();
                    let mut edited = bodies.clone();
                    edited[2] = changed;
                    let output = evaluate_scene(&edited).unwrap();
                    assert!(volume(&output[2].1) > volume(g));
                }
            }
        }
    }
}
