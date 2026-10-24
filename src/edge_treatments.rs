//! Face-selected edge treatments on convex extrusions, rebuilt from editable parameters.
use crate::{
    components::Topology,
    model::{Body, Geometry, Shape},
};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use truck_modeling::{EuclideanSpace, InnerSpace, Point3, SquareMatrix, Transform, Vector3};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FaceTarget {
    Top,
    Bottom,
    Left,
    Right,
    Front,
    Back,
}
impl FaceTarget {
    pub fn opposite(self) -> Self {
        match self {
            Self::Top => Self::Bottom,
            Self::Bottom => Self::Top,
            Self::Left => Self::Right,
            Self::Right => Self::Left,
            Self::Front => Self::Back,
            Self::Back => Self::Front,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Top => "Top",
            Self::Bottom => "Bottom",
            Self::Left => "Left",
            Self::Right => "Right",
            Self::Front => "Front",
            Self::Back => "Back",
        }
    }
    pub(crate) fn plane(self, b: &Body) -> (Vector3, f64) {
        match self {
            Self::Top => (Vector3::unit_z(), b.extrusion),
            Self::Bottom => (-Vector3::unit_z(), 0.),
            Self::Left => (-Vector3::unit_x(), b.width / 2.),
            Self::Right => (Vector3::unit_x(), b.width / 2.),
            Self::Front => (-Vector3::unit_y(), b.depth / 2.),
            Self::Back => (Vector3::unit_y(), b.depth / 2.),
        }
    }
    /// Actual outer support after source shelling; the authored planes stay intact.
    pub(crate) fn evaluated_plane(self, b: &Body) -> (Vector3, f64) {
        let (normal, distance) = self.plane(b);
        let offset = b
            .hollow
            .as_ref()
            .filter(|op| op.outward && (op.closed || op.opening != self))
            .map_or(0., |op| op.thickness);
        (normal, distance + offset)
    }
    pub(crate) fn supported(self, b: &Body) -> bool {
        b.shape == Shape::Rectangle || matches!(self, Self::Top | Self::Bottom)
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    Chamfer,
    Bevel,
}
impl Kind {
    pub fn label(self) -> &'static str {
        match self {
            Self::Chamfer => "Chamfer",
            Self::Bevel => "Bevel",
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Style {
    Flat,
    Rounded,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PrismBevel {
    pub amount: f64,
    pub style: Style,
}
/// Distances are source-local setbacks; the angle is measured from the
/// selected face (from the adjacent face when direction is reversed).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", rename_all = "snake_case", deny_unknown_fields)]
pub enum ChamferDefinition {
    #[default]
    Equal,
    TwoDistances {
        second: f64,
    },
    DistanceAngle {
        degrees: f64,
    },
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Modifier {
    pub kind: Kind,
    pub face: FaceTarget,
    /// A neighboring source plane restricts this operation to their shared edge.
    #[serde(default)]
    pub edge: Option<FaceTarget>,
    pub amount: f64,
    pub style: Style,
    pub segments: u8,
    #[serde(default)]
    pub chamfer: ChamferDefinition,
    #[serde(default)]
    pub reverse: bool,
}
impl Modifier {
    pub fn new(kind: Kind, face: FaceTarget, b: &Body) -> Self {
        Self {
            kind,
            face,
            edge: None,
            amount: (limit(b) * 0.2).min(0.2),
            style: if kind == Kind::Chamfer {
                Style::Flat
            } else {
                Style::Rounded
            },
            segments: 8,
            chamfer: ChamferDefinition::Equal,
            reverse: false,
        }
    }
    pub fn label(&self) -> String {
        format!(
            "{} · {}{}",
            self.kind.label(),
            self.face.label(),
            self.edge
                .map_or(String::new(), |other| format!(" / {} edge", other.label()))
        )
    }
    /// (setback along selected face, setback along its neighbor).
    pub fn setbacks(&self) -> (f64, f64) {
        let second = match self.chamfer {
            ChamferDefinition::Equal => self.amount,
            ChamferDefinition::TwoDistances { second } => second,
            ChamferDefinition::DistanceAngle { degrees } => {
                self.amount * degrees.to_radians().tan()
            }
        };
        if self.reverse {
            (second, self.amount)
        } else {
            (self.amount, second)
        }
    }
}
fn limit(b: &Body) -> f64 {
    (b.width / 2.)
        .min(if b.shape == Shape::Circle {
            b.width / 2.
        } else {
            b.depth / 2.
        })
        .min(b.extrusion / 2.)
}
pub fn validate(b: &Body) -> Result<(), String> {
    if b.edge_treatments.is_empty() {
        return Ok(());
    }
    if b.extrusion <= 0. || b.shape == Shape::Line {
        return Err("Chamfer and bevel require an extruded solid.".into());
    }
    if b.edge_treatments.len() > 18 {
        return Err("At most six face perimeters and twelve source edges can be treated.".into());
    }
    for (i, m) in b.edge_treatments.iter().enumerate() {
        (|| {
        if let Some(other) = m.edge
            && (b.shape != Shape::Rectangle || m.face.plane(b).0.dot(other.plane(b).0).abs() > 1e-8)
        {
            return Err("A box edge must reference two perpendicular source planes.".into());
        }
        if !m.face.supported(b) {
            return Err("Curved profiles currently support top/bottom rim modifiers; rectangular solids support all six faces.".into());
        }
        if m.kind != Kind::Chamfer && (m.chamfer != ChamferDefinition::Equal || m.reverse) {
            return Err(
                "Asymmetric setbacks apply to Chamfer; Bevel uses its radius/distance.".into(),
            );
        }
        if let ChamferDefinition::DistanceAngle { degrees } = m.chamfer
            && (!degrees.is_finite() || degrees <= 0. || degrees >= 90.)
        {
            return Err(
                "Chamfer angle must be finite and strictly between 0 and 90 degrees.".into(),
            );
        }
        let (first, second) = m.setbacks();
        if [first, second]
            .iter()
            .any(|v| !v.is_finite() || *v <= 1e-5 || *v >= limit(b) - 1e-6)
        {
            return Err(format!(
                "Distance/radius and both chamfer setbacks must be positive and below {:.6} local units.",
                limit(b)
            ));
        }
        if !(2..=16).contains(&m.segments) {
            return Err("Rounded bevel segments must be between 2 and 16.".into());
        }
        if m.kind == Kind::Chamfer && m.style != Style::Flat {
            return Err("Chamfer uses a flat transition.".into());
        }
        if b.edge_treatments[..i]
            .iter()
            .any(|p| match (p.edge, m.edge) {
                (None, None) => p.face == m.face,
                (Some(a), Some(c)) => (p.face == m.face && a == c) || (p.face == c && a == m.face),
                (None, Some(c)) => p.face == m.face || p.face == c,
                (Some(a), None) => m.face == p.face || m.face == a,
            })
        {
            return Err(
                "This source edge/perimeter is already treated; edit or remove its operation."
                    .into(),
            );
        }
                Ok(())
        })().map_err(|reason:String|format!("Operation {} · {}: {reason}",i+1,m.label()))?;
    }

    Ok(())
}
/// Resolve selected output geometry to a source plane, never persist an output slot.
pub fn target(b: &Body, g: &Geometry, face: usize) -> Result<FaceTarget, String> {
    if b.shape == Shape::Line || b.extrusion == 0. {
        return Err("Select a face of an extruded solid.".into());
    }
    let inv = crate::regions::transform(b)
        .invert()
        .ok_or("Invalid source transform")?;
    let points: Vec<_> = g
        .triangles
        .iter()
        .zip(&g.triangle_faces)
        .filter(|(_, f)| **f == face)
        .flat_map(|(t, _)| t.iter().map(|p| inv.transform_point(*p)))
        .collect();
    if !points.is_empty() {
        for role in [
            FaceTarget::Top,
            FaceTarget::Bottom,
            FaceTarget::Left,
            FaceTarget::Right,
            FaceTarget::Front,
            FaceTarget::Back,
        ] {
            let (n, d) = role.evaluated_plane(b);
            if role.supported(b) && points.iter().all(|p| (p.to_vec().dot(n) - d).abs() < 1e-6) {
                return Ok(role);
            }
        }
    }
    Err("Select an original planar face, such as a cylinder's top or bottom cap, rather than a transition or curved wall.".into())
}

/// Circular source rims can also be selected directly in Edge mode.
pub fn rim_target(b: &Body, g: &Geometry, edge: usize) -> Result<FaceTarget, String> {
    if !matches!(b.shape, Shape::Circle | Shape::Semicircle) || b.extrusion <= 0. {
        return Err(
            "Use Face mode for rectangular solids; Edge mode supports circular top/bottom rims."
                .into(),
        );
    }
    let inv = crate::regions::transform(b)
        .invert()
        .ok_or("Invalid source transform")?;
    let points: Vec<_> = g
        .edges
        .iter()
        .zip(&g.edge_groups)
        .filter(|(_, group)| **group == edge)
        .flat_map(|(e, _)| e.iter().map(|p| inv.transform_point(*p)))
        .collect();
    if !points.is_empty() {
        for face in [FaceTarget::Top, FaceTarget::Bottom] {
            let (_, d) = face.plane(b);
            if points.iter().all(|p| {
                (if face == FaceTarget::Top {
                    p.z - d
                } else {
                    p.z
                })
                .abs()
                    < 1e-6
            }) {
                return Ok(face);
            }
        }
    }
    Err("Select a circular top or bottom rim; the modifier treats that entire rim.".into())
}

/// Resolve box edges by their two original support planes, not volatile output slots.
fn box_edge(b: &Body, g: &Geometry, edge: usize) -> Result<(FaceTarget, FaceTarget), String> {
    if b.shape != Shape::Rectangle
        || b.bore.is_some()
        || b.boolean.is_some()
        || !b.component_edits.is_empty()
    {
        return Err("Edge-local treatment requires an intact source box edge. Concave, component-deformed, and Boolean-result edges are not supported yet.".into());
    }
    let inverse = crate::regions::transform(b)
        .invert()
        .ok_or("Invalid source transform.")?;
    let points: Vec<_> = g
        .edges
        .iter()
        .zip(&g.edge_groups)
        .filter(|(_, group)| **group == edge)
        .flat_map(|(edge, _)| edge.iter().map(|p| inverse.transform_point(*p)))
        .collect();
    if points.is_empty() {
        return Err("Selected edge no longer exists.".into());
    }
    let planes: Vec<_> = [
        FaceTarget::Top,
        FaceTarget::Bottom,
        FaceTarget::Left,
        FaceTarget::Right,
        FaceTarget::Front,
        FaceTarget::Back,
    ]
    .into_iter()
    .filter(|face| {
        let (n, d) = face.plane(b);
        points.iter().all(|p| (p.to_vec().dot(n) - d).abs() < 1e-6)
    })
    .collect();
    if planes.len() != 2 {
        return Err("Select an original box edge, rather than a treatment boundary.".into());
    }
    Ok((planes[0], planes[1]))
}

/// Resolve every pick before committing: a mixed invalid selection changes nothing.
pub fn selected_operations(
    bodies: &[Body],
    scene: &[(u64, Geometry)],
    picks: &[crate::selection::Pick],
    mode: crate::selection::SelectionMode,
    kind: Kind,
) -> Result<Vec<Body>, String> {
    use crate::selection::SelectionMode;
    if picks.is_empty() || !matches!(mode, SelectionMode::Face | SelectionMode::Edge) {
        return Err("Select faces or edges to treat.".into());
    }
    let mut changed = Vec::new();
    for &(id, _) in picks {
        if changed.iter().any(|b: &Body| b.id == id) {
            continue;
        }
        let selected = picks
            .iter()
            .copied()
            .filter(|p| p.0 == id)
            .collect::<Vec<_>>();
        match source_operations(bodies, scene, &selected, mode, kind) {
            Ok(source) => changed.extend(source),
            Err(source_error) => {
                let evaluated = (|| {
                    let source = bodies.iter().find(|b| b.id == id).ok_or("Missing body.")?;
                    let mut g = scene
                        .iter()
                        .find(|g| g.0 == id)
                        .ok_or("Missing geometry.")?
                        .1
                        .clone();
                    crate::components::map_geometry(
                        &mut g,
                        crate::regions::transform(source)
                            .invert()
                            .ok_or("Invalid placement.")?,
                    );
                    let mut edges = Vec::new();
                    for (_, element) in &selected {
                        let element = element.ok_or("Select a boundary element.")?;
                        let candidates = if mode == SelectionMode::Face {
                            g.topology
                                .faces
                                .iter()
                                .enumerate()
                                .filter(|(face, _)| g.face_group(*face) == g.face_group(element))
                                .flat_map(|(_, edges)| edges.iter().copied())
                                .collect::<Vec<_>>()
                        } else {
                            vec![element]
                        };
                        for edge in candidates {
                            let edge = g.edge_group(edge);
                            if !g.internal_edges.contains(&edge) && !edges.contains(&edge) {
                                edges.push(edge);
                            }
                        }
                    }
                    let amount = g
                        .topology
                        .edges
                        .iter()
                        .map(|[a, b]| (g.points[*b] - g.points[*a]).magnitude() * 0.1)
                        .fold(0.2, f64::min);
                    let prism_chamfer = crate::components::convex_supports(&g).is_err();
                    let prism_curves = kind == Kind::Bevel || prism_chamfer;
                    let prism_bevel = (kind == Kind::Bevel).then_some(PrismBevel {
                        amount,
                        style: Style::Rounded,
                    });
                    if let Some(bevel) = prism_bevel {
                        bevel_prism(&g, &edges, bevel, prism_curves)?;
                    } else if prism_chamfer {
                        chamfer_prism(&g, &edges, [amount; 2], prism_curves)?;
                    } else {
                        chamfer_evaluated(&g, &edges, [amount; 2])?;
                    }
                    let element = *edges.first().ok_or("No boundary edges selected.")?;
                    let mut body = source.clone();
                    body.component_edits.push(crate::components::ComponentEdit {
                        evaluated: true,
                        retrim: false,
                        section_transform: false,
                        sweep: None,
                        tangent_walls: false,
                        prismatic_cut: false,
                        face_edit: None,
                        edge_chamfer: (kind == Kind::Chamfer).then_some([amount; 2]),
                        prism_chamfer: kind == Kind::Chamfer && prism_chamfer,
                        prism_bevel,
                        prism_curves,
                        mode: SelectionMode::Edge,
                        element,
                        elements: edges.into_iter().skip(1).collect(),
                        topology: g.topology.clone(),
                        matrix: truck_modeling::Matrix4::identity().into(),
                    });
                    body.validate()?;
                    Ok(body)
                })()
                .map_err(|reason: String| {
                    format!(
                        "{} · body {id}: {source_error} Evaluated boundary: {reason}",
                        kind.label()
                    )
                })?;
                changed.push(evaluated);
            }
        }
    }
    Ok(changed)
}

fn source_operations(
    bodies: &[Body],
    scene: &[(u64, Geometry)],
    picks: &[crate::selection::Pick],
    mode: crate::selection::SelectionMode,
    kind: Kind,
) -> Result<Vec<Body>, String> {
    use crate::selection::SelectionMode;
    if picks.is_empty() || !matches!(mode, SelectionMode::Face | SelectionMode::Edge) {
        return Err("Select faces or edges to treat.".into());
    }
    let mut changed: Vec<Body> = Vec::new();
    for &(id, element) in picks {
        (|| {
            let element = element.ok_or("Select faces or edges to treat.")?;
            let source = bodies
                .iter()
                .find(|b| b.id == id)
                .ok_or("Selected body no longer exists.")?;
            let geometry = &scene
                .iter()
                .find(|g| g.0 == id)
                .ok_or("Selected geometry no longer exists.")?
                .1;
            let (face, edge) = if mode == SelectionMode::Face {
                (target(source, geometry, element)?, None)
            } else if source.shape == Shape::Rectangle {
                let (face, neighbor) = box_edge(source, geometry, element)?;
                (face, Some(neighbor))
            } else {
                (rim_target(source, geometry, element)?, None)
            };
            if !changed.iter().any(|b| b.id == id) {
                changed.push(source.clone());
            }
            let body = changed.iter_mut().find(|b| b.id == id).unwrap();
            // A circular rim can span several kernel edges; one logical operation suffices.
            if body
                .edge_treatments
                .iter()
                .skip(source.edge_treatments.len())
                .any(|m| m.face == face && m.edge == edge)
            {
                return Ok(());
            }
            let mut operation = Modifier::new(kind, face, source);
            operation.edge = edge;
            body.edge_treatments.push(operation);
            body.validate()?;

            Ok(())
        })()
        .map_err(|reason: String| {
            format!(
                "{} · body {id} · {} {}: {reason}",
                kind.label(),
                mode.label(),
                element.map_or(String::new(), |e| e.saturating_add(1).to_string())
            )
        })?;
    }

    Ok(changed)
}

#[derive(Clone)]
struct Face {
    points: Vec<Point3>,
    group: usize,
}
pub(crate) fn clip_polygon(input: &[Point3], n: Vector3, d: f64) -> (Vec<Point3>, Vec<Point3>) {
    let mut points = Vec::new();
    let mut cut = Vec::new();
    for i in 0..input.len() {
        let a = input[i];
        let b = input[(i + 1) % input.len()];
        let da = a.to_vec().dot(n) - d;
        let db = b.to_vec().dot(n) - d;
        if da <= 1e-9 {
            points.push(a);
        }
        if (da > 1e-9 && db < -1e-9) || (db > 1e-9 && da < -1e-9) {
            let p = a + (b - a) * (da / (da - db));
            points.push(p);
            cut.push(p);
        } else if da.abs() <= 1e-9 {
            cut.push(a);
        }
    }
    points.dedup_by(|a, b| (*a - *b).magnitude2() < 1e-18);
    if points.len() > 1 && (points[0] - points[points.len() - 1]).magnitude2() < 1e-18 {
        points.pop();
    }
    (points, cut)
}
fn clip(faces: &mut Vec<Face>, n: Vector3, d: f64, group: usize) -> Result<(), String> {
    if !faces
        .iter()
        .flat_map(|f| &f.points)
        .any(|p| p.to_vec().dot(n) > d + 1e-9)
    {
        return Ok(());
    }
    let mut cut = Vec::new();
    let mut next = Vec::new();
    for f in faces.iter() {
        let (points, intersections) = clip_polygon(&f.points, n, d);
        cut.extend(intersections);
        if points.len() >= 3 {
            next.push(Face {
                points,
                group: f.group,
            });
        }
    }
    cut.sort_by(|a, b| {
        a.x.total_cmp(&b.x)
            .then(a.y.total_cmp(&b.y))
            .then(a.z.total_cmp(&b.z))
    });
    let mut unique = Vec::new();
    for p in cut {
        if !unique
            .iter()
            .any(|q: &Point3| (p - *q).magnitude2() < 1e-16)
        {
            unique.push(p);
        }
    }
    if unique.len() >= 3 {
        let center = unique.iter().map(|p| p.to_vec()).sum::<Vector3>() / (unique.len() as f64);
        let axis = if n.x.abs() < 0.9 {
            Vector3::unit_x()
        } else {
            Vector3::unit_y()
        };
        let u = n.cross(axis).normalize();
        let v = n.normalize().cross(u);
        unique.sort_by(|a, b| {
            let a = a.to_vec() - center;
            let b = b.to_vec() - center;
            a.dot(v)
                .atan2(a.dot(u))
                .total_cmp(&b.dot(v).atan2(b.dot(u)))
        });
        next.push(Face {
            points: unique,
            group,
        });
    }
    if next.len() > 20000 {
        return Err("Modifier geometry exceeds the face budget.".into());
    }
    *faces = next;
    Ok(())
}

fn linear_polygons(
    shell: &truck_modeling::Shell,
    roles: &[usize],
    split_holes: bool,
) -> Option<Vec<Face>> {
    let mut polygons = Vec::new();
    for (index, face) in shell.iter().enumerate() {
        let wires = face.boundaries();
        if (!split_holes && wires.len() != 1)
            || wires.is_empty()
            || face
                .edge_iter()
                .any(|edge| !matches!(edge.curve(), truck_modeling::Curve::Line(_)))
        {
            return None;
        }
        if wires.len() > 1 {
            if !matches!(face.oriented_surface(), truck_modeling::Surface::Plane(_)) {
                return None;
            }
            let geometry =
                crate::model::tessellate(&std::iter::once(face.clone()).collect()).ok()?;
            polygons.extend(geometry.triangles.into_iter().map(|triangle| Face {
                points: triangle.to_vec(),
                group: roles[index],
            }));
            continue;
        }
        let mut points = Vec::new();
        for edge in &wires[0] {
            let a = edge.front().point();
            let b = edge.back().point();
            let ab = b - a;
            if ab.magnitude2() < 1e-16 || !matches!(edge.curve(), truck_modeling::Curve::Line(_)) {
                return None;
            }
            points.push(a);
        }
        polygons.push(Face {
            points,
            group: roles[index],
        });
    }
    Some(polygons)
}

fn chamfer_corner(
    g: &Geometry,
    planes: impl Fn(usize) -> Option<(Vector3, f64)>,
    edge: usize,
) -> Result<([usize; 2], Vector3), String> {
    if edge >= g.edge_count || g.internal_edges.contains(&edge) {
        return Err("Selected chamfer edge no longer exists.".into());
    }
    let [a, b] = g.topology.edges[edge];
    let direction = g.points[b] - g.points[a];
    if direction.magnitude2() < 1e-16 {
        return Err("Chamfer edge is degenerate.".into());
    }
    let owners = g
        .topology
        .faces
        .iter()
        .enumerate()
        .filter(|(_, edges)| edges.contains(&edge))
        .map(|(face, _)| face)
        .collect::<Vec<_>>();
    let [first, second] = owners.as_slice() else {
        return Err("Chamfer requires an edge shared by exactly two faces.".into());
    };
    let (n1, _) = planes(*first).ok_or("Select an edge between two planar support faces.")?;
    let (n2, _) = planes(*second).ok_or("Select an edge between two planar support faces.")?;
    let cosine = n1.dot(n2);
    if cosine.abs() > 1. - 1e-8 {
        return Err("Coplanar or folded boundaries do not define a chamfer corner.".into());
    }
    Ok(([*first, *second], direction))
}

/// Cut the selected straight support intersections on a retained convex solid.
/// The cut planes and neighboring faces are reconstructed as shared B-rep edges.
pub(crate) fn chamfer_evaluated(
    g: &Geometry,
    selected: &[usize],
    distances: [f64; 2],
) -> Result<Geometry, String> {
    if selected.is_empty()
        || distances
            .iter()
            .any(|d| !d.is_finite() || *d <= 1e-5 || *d > 10000.)
    {
        return Err("Select edges and supply finite positive chamfer setbacks.".into());
    }
    let planes = crate::components::planar_supports(g)?;
    let shell = g
        .shell
        .as_ref()
        .ok_or("Chamfer requires retained boundaries.")?;
    if shell.connected_components().len() != 1 {
        return Err("Chamfer requires one connected convex solid.".into());
    }
    let mut faces = linear_polygons(shell, g.surface_mapping()?, false)
        .ok_or("Chamfer requires straight planar boundary loops.")?;
    let mut cuts: Vec<(Vector3, f64)> = Vec::new();
    for &edge in selected {
        let ([first, second], direction) = chamfer_corner(g, |face| Some(planes[face]), edge)?;
        let [a, _] = g.topology.edges[edge];
        let (n1, _) = planes[first];
        let (n2, _) = planes[second];
        let cosine = n1.dot(n2);
        let p = g.points[a] - (n2 - n1 * cosine).normalize() * distances[0];
        let q = g.points[a] - (n1 - n2 * cosine).normalize() * distances[1];
        let mut normal = direction.cross(q - p).normalize();
        if normal.dot(n1 + n2) < 0. {
            normal = -normal;
        }
        let distance = normal.dot(p.to_vec());
        if !cuts
            .iter()
            .any(|(n, d)| (*n - normal).magnitude() < 1e-8 && (*d - distance).abs() < 1e-7)
        {
            cuts.push((normal, distance));
        }
    }
    for (i, (normal, distance)) in cuts.into_iter().enumerate() {
        clip(&mut faces, normal, distance, g.face_count + i)?;
    }
    if (0..g.face_count).any(|group| !faces.iter().any(|f| f.group == group)) {
        return Err("Chamfer consumes a support face; reduce the setbacks.".into());
    }
    let mut result = join_linear_faces(faces)?;
    crate::components::planar_supports(&result)?;
    result.retain_circles(&g.circles);
    Ok(result)
}

/// Local full-depth corner treatment on a proven straight prism. Work on the
/// complete cap, then sweep it, so concave corners add material without CSG.
pub(crate) fn chamfer_prism(
    g: &Geometry,
    selected: &[usize],
    distances: [f64; 2],
    preserve_curves: bool,
) -> Result<Geometry, String> {
    prism_corner(g, selected, distances, None, preserve_curves)
}

pub(crate) fn bevel_prism(
    g: &Geometry,
    selected: &[usize],
    bevel: PrismBevel,
    preserve_curves: bool,
) -> Result<Geometry, String> {
    prism_corner(
        g,
        selected,
        [bevel.amount; 2],
        (bevel.style == Style::Rounded).then_some(bevel.amount),
        preserve_curves,
    )
}

fn prism_corner(
    g: &Geometry,
    selected: &[usize],
    distances: [f64; 2],
    radius: Option<f64>,
    preserve_curves: bool,
) -> Result<Geometry, String> {
    use crate::{booleans, snapping::Plane};
    use truck_modeling::builder;
    if selected.is_empty()
        || distances
            .iter()
            .any(|d| !d.is_finite() || *d <= 1e-5 || *d > 10000.)
    {
        return Err("Select edges and supply finite positive chamfer setbacks.".into());
    }
    let source = g
        .shell
        .as_ref()
        .ok_or("Treatment requires retained boundaries.")?;
    if !g.is_solid || !(4..=256).contains(&g.face_count) || source.connected_components().len() != 1
    {
        return Err("Treatment requires one connected retained solid with 4–256 faces.".into());
    }
    let mapping = g.surface_mapping()?;
    let supports = if preserve_curves {
        use truck_modeling::{ParametricSurface, Surface};
        (0..g.face_count)
            .map(|role| {
                let mut plane: Option<(Vector3, f64)> = None;
                for (face, _) in source
                    .iter()
                    .zip(mapping)
                    .filter(|(_, group)| **group == role)
                {
                    let Surface::Plane(surface) = face.oriented_surface() else {
                        return None;
                    };
                    let normal = surface.normal().normalize();
                    let distance = normal.dot(surface.subs(0., 0.).to_vec());
                    if plane.is_some_and(|(n, d)| {
                        (n - normal).magnitude() > 1e-7 || (d - distance).abs() > 1e-7
                    }) {
                        return None;
                    }
                    plane = Some((normal, distance));
                }
                plane
            })
            .collect::<Vec<_>>()
    } else {
        crate::components::straight_planar_supports(g)?
            .into_iter()
            .map(Some)
            .collect()
    };
    if supports.iter().enumerate().any(|(i, plane)| {
        plane.is_some_and(|(n, d)| {
            supports[..i].iter().enumerate().any(|(j, other)| {
                other.is_some_and(|(other, offset)| {
                    (n - other).magnitude() < 1e-7
                        && (d - offset).abs() < 1e-7
                        && g.topology.faces[i]
                            .iter()
                            .any(|edge| g.topology.faces[j].contains(edge))
                })
            })
        })
    }) {
        return Err(
            "Local treatment cannot yet preserve independently split coplanar support faces."
                .into(),
        );
    }
    let mut result = g.clone();
    for &edge in selected {
        let ([first, second], direction) = chamfer_corner(g, |face| supports[face], edge)?;
        let [a, b] = g.topology.edges[edge];
        let unit = direction.normalize();
        let plane = (0..g.face_count)
            .filter_map(|face| Plane::face(g, face))
            .find(|plane| {
                plane.normal.dot(unit) > 1. - 1e-7
                    && (g.points[b] - plane.origin).dot(unit).abs() < 1e-7
            })
            .ok_or("Local chamfer requires a complete straight-prism corner edge.")?;
        let Some((depth, caps)) = crate::face_extrusions::prismatic_section(g, plane)? else {
            return Err("Local chamfer requires two complete parallel prism caps.".into());
        };
        if caps.is_empty()
            || caps.iter().any(|cap| mapping[*cap] != mapping[caps[0]])
            || (depth - direction.magnitude()).abs() > 1e-7
        {
            return Err("Select a full-depth edge between two unsplit prism caps.".into());
        }
        let (n1, _) = supports[first].unwrap();
        let (n2, _) = supports[second].unwrap();
        let cosine = n1.dot(n2);
        let distances = radius.map_or(distances, |r| {
            [r * ((1. - cosine) / (1. + cosine)).sqrt(); 2]
        });
        if radius.is_some() || preserve_curves {
            let current=crate::face_extrusions::prismatic_section(&result,plane)?
                .ok_or("An earlier treatment changed this prism direction; select parallel full-depth edges." )?;
            if (current.0 - depth).abs() > 1e-7 {
                return Err("An earlier treatment changed the prism depth.".into());
            }
        }
        let tangent = source
            .iter()
            .zip(mapping)
            .filter(|(_, role)| **role == first)
            .flat_map(|(face, _)| face.edge_iter())
            .find_map(|e| {
                let x = e.front().point();
                let y = e.back().point();
                if (x - g.points[a]).magnitude() < 1e-7 && (y - g.points[b]).magnitude() < 1e-7 {
                    Some(direction)
                } else if (y - g.points[a]).magnitude() < 1e-7
                    && (x - g.points[b]).magnitude() < 1e-7
                {
                    Some(-direction)
                } else {
                    None
                }
            })
            .ok_or("Cannot resolve oriented chamfer edge.")?;
        let concave = n1.cross(tangent).dot(n2) > 0.;
        let sign = if concave { 1. } else { -1. };
        let p = g.points[b] + (n2 - n1 * cosine).normalize() * distances[0] * sign;
        let q = g.points[b] + (n1 - n2 * cosine).normalize() * distances[1] * sign;
        // Both setback endpoints must remain within their actual side faces,
        // not just within the infinite supporting planes.
        for (face, point, normal) in [(first, p, n1), (second, q, n2)] {
            let point = point - direction * 0.5;
            if !g
                .triangles
                .iter()
                .zip(&g.triangle_faces)
                .filter(|(_, f)| **f == face)
                .any(|(t, _)| {
                    (0..3).all(|i| (t[(i + 1) % 3] - t[i]).cross(point - t[i]).dot(normal) >= -1e-8)
                })
            {
                return Err(
                    "Chamfer setback leaves its adjacent face; reduce the distances.".into(),
                );
            }
        }
        let mut points = [g.points[b], p, q];
        if (p - g.points[b]).cross(q - g.points[b]).dot(unit) < 0. {
            points.swap(1, 2);
        }
        let vertices = points.map(builder::vertex);
        let wire = (0..3)
            .map(|i| {
                if i == 1
                    && let Some(radius) = radius
                {
                    let center = g.points[b] + (n1 + n2) * (sign * radius / (1. + cosine));
                    let middle = center + (g.points[b] - center).normalize() * radius;
                    builder::circle_arc(&vertices[1], &vertices[2], middle)
                } else {
                    builder::line(&vertices[i], &vertices[(i + 1) % 3])
                }
            })
            .collect();
        let triangle = builder::try_attach_plane(&[wire]).map_err(|e| e.to_string())?;
        let tool = crate::model::tessellate(&vec![triangle].into())?;
        let current_cap = (0..result.face_count)
            .find(|face| {
                Plane::face(&result, *face).is_some_and(|other| {
                    other.normal.dot(unit) > 1. - 1e-7
                        && (other.origin - plane.origin).dot(unit).abs() < 1e-7
                })
            })
            .ok_or("The previous chamfer changed the cap reference.")?;
        let operation = booleans::Boolean {
            kind: if concave {
                booleans::Kind::Union
            } else {
                booleans::Kind::Subtract
            },
            operands: [(1, Some(current_cap)), (2, Some(0))],
            topology: [result.topology.clone(), tool.topology.clone()],
            keep_tools: false,
            analytic_crossings: false,
            analytic_faces: true,
            analytic_prisms: false,
        };
        let sheet = crate::booleans::faces(&operation, &[&result, &tool])?;
        let area = |geometry: &Geometry, face: Option<usize>| {
            geometry
                .triangles
                .iter()
                .zip(&geometry.triangle_faces)
                .filter(|(_, f)| face.is_none_or(|face| **f == face))
                .map(|(t, _)| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() * 0.5)
                .sum::<f64>()
        };
        let change = (area(&sheet, None) - area(&result, Some(current_cap)))
            * if concave { 1. } else { -1. };
        let expected = area(&tool, None);
        if (change - expected).abs() > 1e-7 * expected.max(1.) {
            return Err("Chamfer reaches other geometry or overlaps another treatment; reduce the distances.".into());
        }
        let cap_shell = sheet.shell.ok_or("Chamfer leaves no cap material.")?;
        if cap_shell.len() != 1 {
            return Err("Chamfer splits the body; reduce the distances.".into());
        }
        let swept = builder::tsweep(&cap_shell[0].inverse(), -unit * depth);
        result = crate::operators::geometry(&swept)?;
        if radius.is_some() || preserve_curves {
            // Preserve curved bands and merge only connected coplanar patches.
            // The planar arrangement retains curved cap trims exactly.
            use truck_modeling::{ParametricSurface, Shell, Surface};
            let mut groups: Vec<(Option<Plane>, Shell)> = Vec::new();
            for face in result.shell.as_ref().unwrap() {
                if let Surface::Plane(surface) = face.oriented_surface() {
                    let origin = surface.subs(0., 0.);
                    let normal = surface.normal().normalize();
                    let u = (surface.subs(1., 0.) - origin).normalize();
                    let plane = Plane {
                        origin,
                        normal,
                        u,
                        v: normal.cross(u),
                    };
                    if let Some((_, patches)) = groups.iter_mut().find(|(other, _)| {
                        other.is_some_and(|other| {
                            (normal - other.normal).magnitude() < 1e-7
                                && (origin - other.origin).dot(normal).abs() < 1e-7
                        })
                    }) {
                        patches.push(face.clone());
                    } else {
                        groups.push((Some(plane), vec![face.clone()].into()));
                    }
                } else {
                    groups.push((None, vec![face.clone()].into()));
                }
            }
            let mut merged = Shell::default();
            for (plane, patches) in groups {
                if let Some(plane) = plane {
                    for component in patches.connected_components() {
                        let directed = component
                            .extract_boundaries()
                            .into_iter()
                            .flat_map(|wire| wire.into_iter())
                            .collect();
                        merged.extend(crate::curve_imprints::planar_arrangement(plane, directed)?);
                    }
                } else {
                    merged.extend(patches);
                }
            }
            let retained = merged
                .iter()
                .filter_map(|f| match f.oriented_surface() {
                    Surface::Plane(p) => Some((p.normal().normalize(), p.subs(0., 0.))),
                    _ => None,
                })
                .collect::<Vec<_>>();
            if supports.iter().flatten().any(|(n, d)| {
                !retained.iter().any(|(other, origin)| {
                    (*n - *other).magnitude() < 1e-7 && (n.dot(origin.to_vec()) - *d).abs() < 1e-7
                })
            }) {
                return Err("Bevel consumes an existing support face; reduce its radius.".into());
            }
            crate::components::solid_parts(&merged)?;
            if preserve_curves {
                // Preserve outer-loop position, but order independent holes
                // before the shared solid canonicalizer rotates/sorts faces.
                let key = |p: Point3| [p.x, p.y, p.z].map(|v| (v * 1e7).round() as i64);
                let merged = merged
                    .iter()
                    .map(|face| {
                        let mut wires = face.boundaries();
                        wires[1..].sort_by_cached_key(|wire| {
                            wire.iter().map(|e| key(e.front().point())).min()
                        });
                        truck_modeling::Face::new(wires, face.oriented_surface())
                    })
                    .collect::<Shell>();
                let solid = truck_modeling::Solid::try_new(merged.connected_components())
                    .map_err(|e| e.to_string())?;
                result = crate::operators::geometry(&solid)?;
            } else {
                result = crate::model::tessellate(&merged)?;
                result.is_solid = true;
            }
            continue;
        }
        let after = crate::components::straight_planar_supports(&result)?;
        if supports.iter().flatten().any(|(n, d)| {
            !after.iter().any(|(other, offset)| {
                (*n - *other).magnitude() < 1e-7 && (*d - *offset).abs() < 1e-7
            })
        }) {
            return Err("Chamfer consumes an existing support face; reduce the distances.".into());
        }
        // Reconcile collinear cap fragments into connected logical walls. The
        // shared joiner keeps disconnected regions on one plane independent.
        let roles = after
            .iter()
            .map(|(n, d)| {
                after
                    .iter()
                    .position(|(other, offset)| {
                        (*n - *other).magnitude() < 1e-7 && (*d - *offset).abs() < 1e-7
                    })
                    .unwrap()
            })
            .collect::<Vec<_>>();
        let polygons = linear_polygons(result.shell.as_ref().unwrap(), &roles, true)
            .ok_or("Cannot reconstruct chamfer planar faces.")?;
        result = join_linear_faces(polygons)?;
        result = planar_regions(result)?;
    }
    result.retain_circles(&g.circles);
    Ok(result)
}

/// Reconcile a planar retained result into one face per connected logical region.
pub(crate) fn planar_regions(result: Geometry) -> Result<Geometry, String> {
    use crate::snapping::Plane;
    let source = result
        .shell
        .as_ref()
        .ok_or("No retained planar material remains.")?;
    let roles = result.surface_mapping()?;
    let mut merged = truck_modeling::Shell::default();
    for role in 0..result.face_count {
        let patches = source
            .iter()
            .zip(roles)
            .filter(|(_, group)| **group == role)
            .map(|(face, _)| face.clone())
            .collect::<truck_modeling::Shell>();
        let directed = patches
            .extract_boundaries()
            .into_iter()
            .flat_map(|wire| wire.into_iter())
            .collect();
        let faces = crate::curve_imprints::planar_arrangement(
            Plane::face(&result, role).ok_or("Missing planar chamfer region.")?,
            directed,
        )?;
        if faces.len() != 1 {
            return Err("Chamfer creates an ambiguous planar region.".into());
        }
        merged.extend(faces);
    }
    crate::components::solid_parts(&merged)?;
    let mut result = crate::model::tessellate(&merged)?;
    result.is_solid = true;
    Ok(result)
}

/// Exact convex planar intersection, including coincident supports and unequal depths.
pub(crate) fn intersect_convex(a: &Geometry, b: &Geometry) -> Result<Option<Geometry>, String> {
    let (Ok(ap), Ok(bp)) = (
        crate::components::planar_supports(a),
        crate::components::planar_supports(b),
    ) else {
        return Ok(None);
    };
    let inside = |g: &Geometry, planes: &[(Vector3, f64)]| {
        planes
            .iter()
            .all(|(n, d)| g.points.iter().all(|p| n.dot(p.to_vec()) <= *d + 1e-9))
    };
    if inside(a, &bp) {
        return Ok(Some(a.clone()));
    }
    if inside(b, &ap) {
        return Ok(Some(b.clone()));
    }
    let Some(mut faces) = linear_polygons(
        a.shell
            .as_ref()
            .ok_or("Intersection requires an exact boundary.")?,
        a.surface_mapping()?,
        false,
    ) else {
        return Ok(None);
    };
    for (index, (n, d)) in bp.into_iter().enumerate() {
        clip(&mut faces, n, d, a.face_count + index)?;
    }
    let Some(origin) = faces.first().and_then(|f| f.points.first()).copied() else {
        return Ok(Some(Geometry::default()));
    };
    let volume = faces
        .iter()
        .map(|f| {
            (1..f.points.len().saturating_sub(1))
                .map(|i| {
                    (f.points[0] - origin)
                        .dot((f.points[i] - origin).cross(f.points[i + 1] - origin))
                        / 6.
                })
                .sum::<f64>()
        })
        .sum::<f64>();
    if volume.abs() < 1e-10 {
        return Ok(Some(Geometry::default()));
    }
    let mut result = geometry(faces)?;
    result.shell = Some(crate::components::boundary_shell(&result, true)?);
    result.is_solid = true;
    Ok(Some(result))
}

/// Exact linear-face subtraction. Unsupported curved/nonconvex inputs return
/// None so callers retain the kernel path; no mesh Boolean is substituted.
pub(crate) fn subtract_convex(
    g: &Geometry,
    tool: &truck_modeling::Shell,
    roles: &[usize],
) -> Result<Option<Geometry>, String> {
    fn plane(face: &Face) -> Option<(Vector3, f64)> {
        let a = *face.points.first()?;
        let normal = face
            .points
            .windows(2)
            .map(|p| (p[0] - a).cross(p[1] - a))
            .sum::<Vector3>();
        if normal.magnitude2() < 1e-20 {
            return None;
        }
        let normal = normal.normalize();
        let d = normal.dot(a.to_vec());
        face.points
            .iter()
            .all(|p| (normal.dot(p.to_vec()) - d).abs() < 1e-6)
            .then_some((normal, d))
    }
    fn convex(faces: &[Face], planes: &[(Vector3, f64)]) -> bool {
        planes.iter().all(|(n, d)| {
            faces
                .iter()
                .flat_map(|f| &f.points)
                .all(|p| n.dot(p.to_vec()) <= *d + 1e-7)
        })
    }
    let shell = g.shell.as_ref().ok_or("Missing exact solid boundary.")?;
    let Some(body) = linear_polygons(shell, roles, false) else {
        return Ok(None);
    };
    let tool_roles = (g.face_count..g.face_count + tool.len()).collect::<Vec<_>>();
    let Some(cutter) = linear_polygons(tool, &tool_roles, false) else {
        return Ok(None);
    };
    let Some(body_planes) = body.iter().map(plane).collect::<Option<Vec<_>>>() else {
        return Ok(None);
    };
    let Some(tool_planes) = cutter.iter().map(plane).collect::<Option<Vec<_>>>() else {
        return Ok(None);
    };
    if body_planes.len() > 512
        || tool_planes.len() > 256
        || !convex(&body, &body_planes)
        || !convex(&cutter, &tool_planes)
    {
        return Ok(None);
    }
    // Preserve the original boundary and references for disjoint/touching tools.
    if (0..3).any(|axis| {
        let bounds = |faces: &[Face]| {
            faces
                .iter()
                .flat_map(|f| &f.points)
                .map(|p| p[axis])
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                    (lo.min(v), hi.max(v))
                })
        };
        let (alo, ahi) = bounds(&body);
        let (blo, bhi) = bounds(&cutter);
        ahi <= blo + 1e-9 || bhi <= alo + 1e-9
    }) {
        return Ok(Some(g.clone()));
    }
    let mut output = Vec::new();
    for face in body {
        let mut remainder = face.points;
        for (n, d) in &tool_planes {
            let outside = clip_polygon(&remainder, -*n, -*d).0;
            if outside.len() >= 3 && remainder.iter().any(|p| n.dot(p.to_vec()) > *d + 1e-9) {
                let candidate = Face {
                    points: outside,
                    group: face.group,
                };
                if plane(&candidate).is_some() {
                    output.push(candidate);
                }
            }
            remainder = clip_polygon(&remainder, *n, *d).0;
            if remainder.len() < 3 {
                break;
            }
        }
    }
    for (face, (n, d)) in cutter.into_iter().zip(&tool_planes) {
        // Coincident outer boundaries are removed material, not new zero-thickness walls.
        if body_planes.iter().any(|(other, offset)| {
            ((*other - *n).magnitude() < 1e-7 && (*offset - *d).abs() < 1e-7)
                || ((*other + *n).magnitude() < 1e-7 && (*offset + *d).abs() < 1e-7)
        }) {
            continue;
        }
        let mut points = face.points;
        for (n, d) in &body_planes {
            points = clip_polygon(&points, *n, *d).0;
            if points.len() < 3 {
                break;
            }
        }
        points.reverse();
        let candidate = Face {
            points,
            group: face.group,
        };
        if plane(&candidate).is_some() {
            output.push(candidate);
        }
    }
    if output.is_empty() {
        return Ok(Some(Geometry::default()));
    }
    if output.len() > 4096 {
        return Err("Planar subtraction exceeds the face budget.".into());
    }
    join_linear_faces(output).map(Some)
}

/// Reconcile a planar face partition with every adjacent straight boundary.
pub(crate) fn imprint_linear(
    g: &Geometry,
    face: usize,
    remainder: &Geometry,
    inner: &Geometry,
) -> Result<Option<Geometry>, String> {
    let Some(mut output) = linear_polygons(
        g.shell
            .as_ref()
            .ok_or("Imprint requires a retained boundary.")?,
        g.surface_mapping()?,
        true,
    ) else {
        return Ok(None);
    };
    output.retain(|f| f.group != face);
    for (part, role) in [(remainder, face), (inner, g.face_count)] {
        let shell = part
            .shell
            .as_ref()
            .ok_or("Imprint partition has no boundary.")?;
        let Some(polygons) = linear_polygons(shell, &vec![role; shell.len()], true) else {
            return Ok(None);
        };
        output.extend(polygons);
    }
    if output.len() > 4096 {
        return Err("Imprint exceeds the face budget.".into());
    }
    join_linear_faces(output).map(Some)
}

fn join_linear_faces(output: Vec<Face>) -> Result<Geometry, String> {
    // Reconcile cut T-junctions before constructing shared topological edges.
    let mut points = Vec::new();
    let mut point_map = HashMap::new();
    let mut polygons = Vec::new();
    for face in &output {
        polygons.push(
            face.points
                .iter()
                .map(|p| {
                    *point_map
                        .entry([p.x, p.y, p.z].map(|v| (v * 1e7).round() as i64))
                        .or_insert_with(|| {
                            let i = points.len();
                            points.push(*p);
                            i
                        })
                })
                .collect::<Vec<_>>(),
        );
    }
    if points.len() > 8192 {
        return Err("Planar subtraction exceeds the vertex budget.".into());
    }
    if polygons
        .iter()
        .map(Vec::len)
        .sum::<usize>()
        .saturating_mul(points.len())
        > 4_000_000
    {
        return Err("Planar subtraction exceeds the edge reconciliation budget.".into());
    }
    let vertices = points
        .iter()
        .copied()
        .map(truck_modeling::builder::vertex)
        .collect::<Vec<_>>();
    let mut edges = HashMap::new();
    let mut joined = truck_modeling::Shell::default();
    let mut groups = Vec::new();
    for (polygon, face) in polygons.iter().zip(&output) {
        let mut wire = truck_modeling::Wire::default();
        for i in 0..polygon.len() {
            let a = polygon[i];
            let b = polygon[(i + 1) % polygon.len()];
            let ab = points[b] - points[a];
            let length = ab.magnitude2();
            if length < 1e-16 {
                continue;
            }
            let mut split = points
                .iter()
                .enumerate()
                .filter_map(|(index, p)| {
                    let t = (*p - points[a]).dot(ab) / length;
                    (t > 1e-8 && t < 1. - 1e-8 && (*p - points[a] - ab * t).magnitude() < 1e-7)
                        .then_some((t, index))
                })
                .collect::<Vec<_>>();
            split.push((0., a));
            split.push((1., b));
            split.sort_by(|a, b| a.0.total_cmp(&b.0));
            for pair in split.windows(2) {
                let a = pair[0].1;
                let b = pair[1].1;
                let key = if a < b { (a, b) } else { (b, a) };
                let edge = edges.entry(key).or_insert_with(|| {
                    truck_modeling::builder::line(&vertices[key.0], &vertices[key.1])
                });
                wire.push_back(if a < b { edge.clone() } else { edge.inverse() });
            }
        }
        joined.push(
            truck_modeling::builder::try_attach_plane(&[wire])
                .map_err(|e| format!("Cannot construct planar cut face: {e}"))?,
        );
        groups.push(face.group);
    }
    crate::components::solid_parts(&joined)?;
    let mut result = crate::model::tessellate(&joined)?;
    // A cut may disconnect regions that belonged to one original plane.
    // Merge only neighboring fragments, keeping those regions independently selectable.
    let mut parents = (0..joined.len()).collect::<Vec<_>>();
    let root = |mut i: usize, parents: &[usize]| {
        while parents[i] != i {
            i = parents[i];
        }
        i
    };
    let mut owners = HashMap::new();
    for (index, face) in joined.iter().enumerate() {
        for edge in face.edge_iter() {
            if let Some(&other) = owners.get(&edge.id()) {
                if groups[index] == groups[other] {
                    let a = root(index, &parents);
                    let b = root(other, &parents);
                    parents[b] = a;
                }
            } else {
                owners.insert(edge.id(), index);
            }
        }
    }
    let connected = (0..joined.len())
        .map(|i| root(i, &parents))
        .collect::<Vec<_>>();
    merge_surfaces(&mut result, &connected);
    result.is_solid = true;
    Ok(result)
}

pub fn evaluate(b: &Body, split: bool) -> Result<Geometry, String> {
    validate(b)?;
    if split {
        return Err("Face modifiers on partitioned profiles are not supported yet; move the cutting profile away or disable its cutting flag.".into());
    }
    if b.shape == Shape::Circle {
        return cylinder(b);
    }
    let profile: Vec<[f64; 2]> = match b.shape {
        Shape::Rectangle => vec![
            [-b.width / 2., -b.depth / 2.],
            [b.width / 2., -b.depth / 2.],
            [b.width / 2., b.depth / 2.],
            [-b.width / 2., b.depth / 2.],
        ],
        Shape::Circle | Shape::Semicircle => {
            let count = if b.shape == Shape::Circle { 64 } else { 32 };
            (0..if b.shape == Shape::Circle {
                count
            } else {
                count + 1
            })
                .map(|i| {
                    let t = i as f64 * std::f64::consts::TAU / 64.;
                    [b.width / 2. * t.cos(), b.width / 2. * t.sin()]
                })
                .collect()
        }
        Shape::Line => return Err("Lines do not define a solid.".into()),
    };
    let bottom: Vec<_> = profile
        .iter()
        .map(|p| Point3::new(p[0], p[1], 0.))
        .collect();
    let top: Vec<_> = bottom
        .iter()
        .map(|p| *p + Vector3::unit_z() * b.extrusion)
        .collect();
    let mut faces = vec![
        Face {
            points: bottom.iter().rev().copied().collect(),
            group: 0,
        },
        Face {
            points: top.clone(),
            group: 1,
        },
    ];
    let mut sides = Vec::new();
    for i in 0..profile.len() {
        let j = (i + 1) % profile.len();
        faces.push(Face {
            points: vec![bottom[i], bottom[j], top[j], top[i]],
            group: if b.shape == Shape::Rectangle {
                2 + i
            } else if b.shape == Shape::Semicircle && j == 0 {
                3
            } else {
                2
            },
        });
        let edge = bottom[j] - bottom[i];
        let n = Vector3::new(edge.y, -edge.x, 0.).normalize();
        sides.push((n, n.dot(bottom[i].to_vec())));
    }
    for (modifier_index, m) in b.edge_treatments.iter().enumerate() {
        let (normal, d) = m.face.plane(b);
        let neighbors: Vec<_> = if let Some(other) = m.edge {
            vec![other.plane(b)]
        } else if matches!(m.face, FaceTarget::Top | FaceTarget::Bottom) {
            sides.clone()
        } else {
            let axis = if normal.x.abs() > 0.5 {
                Vector3::unit_y()
            } else {
                Vector3::unit_x()
            };
            let span = if normal.x.abs() > 0.5 {
                b.depth / 2.
            } else {
                b.width / 2.
            };
            vec![
                (axis, span),
                (-axis, span),
                (Vector3::unit_z(), b.extrusion),
                (-Vector3::unit_z(), 0.),
            ]
        };
        for (neighbor_index, (neighbor, edge_d)) in neighbors.into_iter().enumerate() {
            let group = 100
                + modifier_index * 8
                + if b.shape == Shape::Rectangle {
                    neighbor_index
                } else if b.shape == Shape::Semicircle && neighbor_index == profile.len() - 1 {
                    1
                } else {
                    0
                };
            if m.kind == Kind::Chamfer || m.style == Style::Flat {
                let (first, second) = m.setbacks();
                let length = first.hypot(second);
                let (a, c) = (first / length, second / length);
                clip(
                    &mut faces,
                    normal * a + neighbor * c,
                    d * a + edge_d * c - first * second / length,
                    group,
                )?;
            } else {
                for step in 1..m.segments {
                    let angle = std::f64::consts::FRAC_PI_2 * step as f64 / m.segments as f64;
                    let a = angle.cos();
                    let c = angle.sin();
                    clip(
                        &mut faces,
                        normal * a + neighbor * c,
                        (d - m.amount) * a + (edge_d - m.amount) * c + m.amount,
                        group,
                    )?;
                }
            }
        }
    }
    // A source face must survive. This also makes source dimension edits transactional.
    for m in &b.edge_treatments {
        let (n, d) = m.face.plane(b);
        if !faces.iter().any(|f| {
            f.points
                .iter()
                .all(|p| (p.to_vec().dot(n) - d).abs() < 1e-7)
        }) {
            return Err(format!(
                "{} consumes its source face or overlaps another treatment; reduce its setbacks/radius.",
                m.label()
            ));
        }
    }
    let mut g = geometry(faces)?;
    crate::components::weld_geometry(&mut g);
    g.shell = Some(crate::components::boundary_shell(&g, true)?);
    crate::components::map_geometry(&mut g, crate::regions::transform(b));
    Ok(g)
}
/// Revolve the authored radial section: circular rims and roundovers remain analytic.
fn cylinder(b: &Body) -> Result<Geometry, String> {
    use truck_modeling::{Rad, Shell, Solid, Wire, builder};
    let radius = b.width / 2.;
    let bottom = b
        .edge_treatments
        .iter()
        .find(|m| m.face == FaceTarget::Bottom);
    let top = b.edge_treatments.iter().find(|m| m.face == FaceTarget::Top);
    let (lower_radial, lower) = bottom.map_or((0., 0.), Modifier::setbacks);
    let (upper_radial, upper) = top.map_or((0., 0.), Modifier::setbacks);
    let vertex = |r, z| builder::vertex(Point3::new(r, 0., z));
    let mut edges = Vec::new();
    let mut current = vertex(radius - lower_radial, 0.);
    if let Some(m) = bottom {
        let next = vertex(radius, lower);
        edges.push(if m.kind == Kind::Bevel && m.style == Style::Rounded {
            builder::circle_arc(
                &current,
                &next,
                Point3::new(
                    radius - lower + lower / std::f64::consts::SQRT_2,
                    0.,
                    lower - lower / std::f64::consts::SQRT_2,
                ),
            )
        } else {
            builder::line(&current, &next)
        });
        current = next;
    }
    let next = vertex(radius, b.extrusion - upper);
    edges.push(builder::line(&current, &next));
    current = next;
    if let Some(m) = top {
        let next = vertex(radius - upper_radial, b.extrusion);
        edges.push(if m.kind == Kind::Bevel && m.style == Style::Rounded {
            builder::circle_arc(
                &current,
                &next,
                Point3::new(
                    radius - upper + upper / std::f64::consts::SQRT_2,
                    0.,
                    b.extrusion - upper + upper / std::f64::consts::SQRT_2,
                ),
            )
        } else {
            builder::line(&current, &next)
        });
    }
    let wire: Wire = edges.into();
    let mut shell: Shell = builder::rsweep(
        &wire.inverse(),
        Point3::new(0., 0., 0.),
        Vector3::unit_z(),
        Rad(std::f64::consts::TAU),
    );
    for boundary in shell.extract_boundaries() {
        shell.push(
            builder::try_attach_plane(&[boundary.inverse()])
                .map_err(|e| format!("Cannot close cylinder modifier: {e}"))?,
        );
    }
    Solid::try_new(vec![shell.clone()])
        .map_err(|e| format!("Cannot build cylinder modifier: {e}"))?;
    let mut g = crate::model::tessellate(&shell)?;
    // Truck splits revolutions into patches; expose each authored surface as one face.
    let mut roles = vec![0; g.face_count];
    for (face, role) in roles.iter_mut().enumerate() {
        let mut min = f64::INFINITY;
        let mut max = f64::NEG_INFINITY;
        for (t, owner) in g.triangles.iter().zip(&g.triangle_faces) {
            if *owner == face {
                for p in t {
                    min = min.min(p.z);
                    max = max.max(p.z);
                }
            }
        }
        *role = if max.abs() < 1e-7 {
            0
        } else if (min - b.extrusion).abs() < 1e-7 {
            1
        } else if lower > 0. && max <= lower + 1e-7 {
            3
        } else if upper > 0. && min >= b.extrusion - upper - 1e-7 {
            4
        } else {
            2
        };
    }
    merge_surfaces(&mut g, &roles);
    crate::components::weld_geometry(&mut g);
    crate::components::boundary_shell(&g, true)?;
    crate::components::map_geometry(&mut g, crate::regions::transform(b));
    Ok(g)
}
pub(crate) fn merge_surfaces(g: &mut Geometry, roles: &[usize]) {
    let mut groups = HashMap::new();
    let faces: Vec<_> = roles
        .iter()
        .map(|role| {
            let next = groups.len();
            *groups.entry(*role).or_insert(next)
        })
        .collect();
    let mut owners = vec![Vec::new(); g.edge_count];
    for (face, edges) in g.topology.faces.iter().enumerate() {
        for edge in edges {
            owners[*edge].push(faces[face]);
        }
    }
    let mut topology = Topology {
        edges: Vec::new(),
        faces: vec![Vec::new(); groups.len()],
    };
    let mut edge_map = HashMap::new();
    for (old, mut faces) in owners.into_iter().enumerate() {
        let boundary = faces.len() == 1;
        faces.sort_unstable();
        faces.dedup();
        if faces.len() < 2 && !boundary {
            continue;
        }
        let index = topology.edges.len();
        edge_map.insert(old, index);
        topology.edges.push(g.topology.edges[old]);
        for face in faces {
            topology.faces[face].push(index);
        }
    }
    let mut edges = Vec::new();
    let mut edge_groups = Vec::new();
    for (edge, old) in g.edges.iter().zip(&g.edge_groups) {
        if let Some(index) = edge_map.get(old) {
            edges.push(*edge);
            edge_groups.push(*index);
        }
    }
    for face in &mut g.triangle_faces {
        *face = faces[*face];
    }
    g.topology = topology;
    g.edges = edges;
    g.edge_groups = edge_groups;
    g.surface_roles = faces;
    g.face_count = groups.len();
    g.edge_count = edge_map.len();
}

/// Remove convex planar constraints and extend the surviving neighboring planes.
pub(crate) fn heal_planar_faces(g: &Geometry, removed: &[usize]) -> Result<Geometry, String> {
    let planes = crate::components::planar_supports(g)?
        .into_iter()
        .enumerate()
        .filter(|(i, _)| !removed.contains(i))
        .collect::<Vec<_>>();
    convex_from_planes(&planes)
}

pub(crate) fn convex_from_planes(planes: &[(usize, (Vector3, f64))]) -> Result<Geometry, String> {
    if !(4..=64).contains(&planes.len()) {
        return Err("Plane reconstruction requires 4–64 convex planar supports.".into());
    }
    let mut corners = Vec::<Point3>::new();
    for i in 0..planes.len() {
        for j in i + 1..planes.len() {
            for k in j + 1..planes.len() {
                let (a, da) = planes[i].1;
                let (b, db) = planes[j].1;
                let (c, dc) = planes[k].1;
                let determinant = a.dot(b.cross(c));
                if determinant.abs() < 1e-10 {
                    continue;
                }
                let point = Point3::from_vec(
                    (b.cross(c) * da + c.cross(a) * db + a.cross(b) * dc) / determinant,
                );
                if [point.x, point.y, point.z]
                    .iter()
                    .any(|p| !p.is_finite() || p.abs() > 1e6)
                {
                    continue;
                }
                if planes
                    .iter()
                    .all(|(_, (n, d))| n.dot(point.to_vec()) <= *d + 1e-7)
                    && !corners.iter().any(|p| (*p - point).magnitude2() < 1e-14)
                {
                    corners.push(point);
                }
            }
        }
    }
    let mut faces = Vec::new();
    for &(group, (n, d)) in planes {
        let mut points = corners
            .iter()
            .copied()
            .filter(|p| (n.dot(p.to_vec()) - d).abs() < 1e-7)
            .collect::<Vec<_>>();
        if points.len() < 3 {
            continue;
        }
        let center = points.iter().map(|p| p.to_vec()).sum::<Vector3>() / points.len() as f64;
        let u = (points[0].to_vec() - center).normalize();
        let v = n.cross(u);
        points.sort_by(|a, b| {
            let a = a.to_vec() - center;
            let b = b.to_vec() - center;
            a.dot(v)
                .atan2(a.dot(u))
                .total_cmp(&b.dot(v).atan2(b.dot(u)))
        });
        faces.push(Face { points, group });
    }
    let mut result = geometry(faces)?;
    result.shell = Some(
        crate::components::boundary_shell(&result, true)
            .map_err(|e| format!("The remaining planes cannot heal a closed solid: {e}"))?,
    );
    result.is_solid = true;
    Ok(result)
}

fn geometry(faces: Vec<Face>) -> Result<Geometry, String> {
    let mut g = Geometry {
        topology: Topology::default(),
        ..Default::default()
    };
    let mut vertices = HashMap::new();
    let mut groups = HashMap::new();
    let mut edges: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for f in faces {
        let mut indices = Vec::new();
        for p in f.points {
            let key = [p.x, p.y, p.z].map(|v| (v * 1e7).round() as i64);
            let index = *vertices.entry(key).or_insert_with(|| {
                let index = g.points.len();
                g.points.push(p);
                index
            });
            if indices.last() != Some(&index) {
                indices.push(index);
            }
        }
        if indices.first() == indices.last() {
            indices.pop();
        }
        if indices.len() < 3 {
            continue;
        }
        let face = *groups.entry(f.group).or_insert_with(|| {
            let index = g.face_count;
            g.face_count += 1;
            g.topology.faces.push(Vec::new());
            index
        });
        for i in 0..indices.len() {
            let a = indices[i];
            let b = indices[(i + 1) % indices.len()];
            edges
                .entry(if a < b { (a, b) } else { (b, a) })
                .or_default()
                .push(face);
        }
        for i in 1..indices.len() - 1 {
            let t = [
                g.points[indices[0]],
                g.points[indices[i]],
                g.points[indices[i + 1]],
            ];
            if (t[1] - t[0]).cross(t[2] - t[0]).magnitude2() > 1e-20 {
                g.triangles.push(t);
                g.triangle_faces.push(face);
            }
        }
    }
    // Approximation seams within one surface are not authored selectable edges.
    let mut edges: Vec<_> = edges.into_iter().collect();
    edges.sort_by_key(|(key, _)| *key);
    for ((a, b), mut owners) in edges {
        owners.sort_unstable();
        owners.dedup();
        if owners.len() < 2 {
            continue;
        }
        let index = g.edge_count;
        g.edge_count += 1;
        g.edges.push([g.points[a], g.points[b]]);
        g.edge_groups.push(index);
        g.topology.edges.push([a, b]);
        for face in owners {
            g.topology.faces[face].push(index);
        }
    }
    let mut remap = HashMap::new();
    let mut points = Vec::new();
    for edge in &mut g.topology.edges {
        for vertex in edge {
            *vertex = *remap.entry(*vertex).or_insert_with(|| {
                let index = points.len();
                points.push(g.points[*vertex]);
                index
            });
        }
    }
    g.points = points;
    if g.triangles.is_empty() {
        return Err("Modifier leaves no solid geometry.".into());
    }
    Ok(g)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        model::{Model, evaluate, evaluate_scene},
        project::Project,
        viewport::Camera,
    };
    fn volume(g: &Geometry) -> f64 {
        g.triangles
            .iter()
            .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
            .sum()
    }
    // Integrate retained cap curves rather than measuring the 0.01-tolerance
    // viewport triangles when checking an analytic fillet's small volume.
    fn prism_volume(g: &Geometry, axis: Vector3) -> f64 {
        use truck_modeling::{BoundedCurve, ParametricCurve, Surface};
        let origin = g.points[0];
        let axis = axis.normalize();
        let mut area = 0.;
        for face in g.shell.as_ref().unwrap() {
            if !matches!(face.oriented_surface(),Surface::Plane(p) if p.normal().normalize().dot(axis)>1.-1e-7)
            {
                continue;
            }
            for edge in face.edge_iter() {
                let curve = edge.oriented_curve();
                let (lo, hi) = curve.range_tuple();
                let step = (hi - lo) / 128.;
                let value = |i| {
                    let t = lo + step * i as f64;
                    (curve.subs(t) - origin).cross(curve.der(t)).dot(axis) * 0.5
                };
                area += (value(0)
                    + value(128)
                    + (1..128)
                        .map(|i| value(i) * if i % 2 == 0 { 2. } else { 4. })
                        .sum::<f64>())
                    * step
                    / 3.;
            }
        }
        let (lo, hi) = g
            .points
            .iter()
            .map(|p| (*p - origin).dot(axis))
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                (lo.min(v), hi.max(v))
            });
        area * (hi - lo)
    }
    #[test]
    fn prismatic_concave_and_outer_corners_chamfer_locally_with_history() {
        use crate::{booleans, components, selection::SelectionMode};
        use truck_modeling::{Deg, Matrix4};
        for hole in [false, true] {
            for affine in [false, true] {
                for inner in [false, true] {
                    let mut a = Body::new(Shape::Rectangle);
                    a.id = 1;
                    a.width = 6.;
                    a.depth = 6.;
                    a.extrusion = 3.;
                    let mut b = a.clone();
                    b.id = 2;
                    b.width = if hole { 2. } else { 4. };
                    b.depth = 2.;
                    b.position[0] = if hole { 0. } else { 2. };
                    let transform = if affine {
                        Matrix4::from_angle_z(Deg(23.))
                            * Matrix4::from_angle_x(Deg(17.))
                            * Matrix4::from_nonuniform_scale(1.5, 0.75, 2.)
                    } else {
                        Matrix4::identity()
                    };
                    if affine {
                        for body in [&mut a, &mut b] {
                            body.position = transform
                                .transform_point(Point3::from(body.position))
                                .into();
                            body.rotation = [17., 0., 23.];
                            body.scale = [1.5, 0.75, 2.];
                        }
                    }
                    let mut bodies = vec![a, b];
                    let source = evaluate_scene(&bodies).unwrap();
                    let mut feature = Body::new(Shape::Rectangle);
                    feature.id = 3;
                    feature.boolean = Some(booleans::Boolean::new(
                        booleans::Kind::Subtract,
                        &[(1, None), (2, None)],
                        SelectionMode::Volume,
                        &source,
                    ));
                    bodies.push(feature);
                    let scene = evaluate_scene(&bodies).unwrap();
                    let g = &scene[2].1;
                    let inverse = transform.invert().unwrap();
                    let target = if inner {
                        if hole { [-1., -1.] } else { [0., -1.] }
                    } else {
                        [-3., -3.]
                    };
                    let edge = g
                        .topology
                        .edges
                        .iter()
                        .position(|[a, b]| {
                            let p = inverse.transform_point(g.points[*a]);
                            let q = inverse.transform_point(g.points[*b]);
                            (p.x - target[0]).abs() < 1e-6
                                && (p.y - target[1]).abs() < 1e-6
                                && (q.x - target[0]).abs() < 1e-6
                                && (q.y - target[1]).abs() < 1e-6
                                && (p.z - q.z).abs() > 2.9
                        })
                        .unwrap_or_else(|| panic!("Missing corner: hole={hole}, affine={affine}, inner={inner}, points={:?}",g.points));
                    let mut changes = selected_operations(
                        &bodies,
                        &scene,
                        &[(3, Some(edge))],
                        SelectionMode::Edge,
                        Kind::Chamfer,
                    )
                    .unwrap();
                    let edit = changes[0].component_edits.last_mut().unwrap();
                    assert!(edit.prism_chamfer);
                    edit.edge_chamfer = Some([0.2, 0.3]);
                    let mut model = Model::from_bodies(bodies.clone());
                    model.commit_changes(&changes).unwrap();
                    let evaluated = evaluate_scene(&model.bodies).unwrap();
                    let result = &evaluated[2].1;
                    let expected = 0.5 * 0.2 * 0.3 * 3. * if affine { 2. } else { 1. };
                    assert!(
                        (volume(result) - volume(g) - expected * if inner { 1. } else { -1. })
                            .abs()
                            < 1e-6,
                        "hole={hole} affine={affine} inner={inner}"
                    );
                    assert_eq!(result.face_count, g.face_count + 1);
                    assert!(result.shell.as_ref().unwrap().is_geometric_consistent());
                    components::solid_parts(result.shell.as_ref().unwrap()).unwrap();
                    assert_eq!(result.shell.as_ref().unwrap().len(), result.face_count);
                    let before_planes = components::straight_planar_supports(g).unwrap();
                    let band = components::straight_planar_supports(result)
                        .unwrap()
                        .iter()
                        .position(|(n, d)| {
                            !before_planes.iter().any(|(other, offset)| {
                                (*n - *other).magnitude() < 1e-7 && (*d - *offset).abs() < 1e-7
                            })
                        })
                        .unwrap();
                    let normal = components::straight_planar_supports(result).unwrap()[band].0;
                    let moved = components::transformed(
                        &changes[0],
                        result,
                        SelectionMode::Face,
                        Some(band),
                        Matrix4::from_translation(normal * 0.02),
                    )
                    .unwrap();
                    let mut moved_bodies = bodies.clone();
                    moved_bodies[2] = moved;
                    assert!(evaluate_scene(&moved_bodies).is_ok());
                    for format in [
                        crate::export::Format::Stl,
                        crate::export::Format::Glb,
                        crate::export::Format::Step,
                        crate::export::Format::Dxf,
                    ] {
                        assert!(
                            !crate::export::encode(&model.bodies, format)
                                .unwrap()
                                .is_empty()
                        );
                    }
                    let dir = tempfile::tempdir().unwrap();
                    let path = dir.path().join("concave-chamfer.osk");
                    let project =
                        Project::new(model.bodies.clone(), Camera::default(), true, Some(3));
                    project.save(&path).unwrap();
                    assert_eq!(Project::load(&path).unwrap().bodies, model.bodies);
                    let mut old = serde_json::to_value(&project).unwrap();
                    old["schema_version"] = 32.into();
                    old["modeling_version"] = 32.into();
                    old["bodies"][2]["component_edits"][0]["prism_curves"] = false.into();
                    assert!(
                        serde_json::from_value::<Project>(old)
                            .unwrap()
                            .validate()
                            .unwrap_err()
                            .contains("version 33")
                    );
                    let before = model.bodies.clone();
                    let mut invalid = changes[0].clone();
                    invalid.component_edits.last_mut().unwrap().edge_chamfer = Some([5.; 2]);
                    assert!(model.commit(invalid).is_err());
                    assert_eq!(model.bodies, before);
                    let mut beveled = selected_operations(
                        &bodies,
                        &scene,
                        &[(3, Some(edge))],
                        SelectionMode::Edge,
                        Kind::Bevel,
                    )
                    .unwrap();
                    beveled[0]
                        .component_edits
                        .last_mut()
                        .unwrap()
                        .prism_bevel
                        .as_mut()
                        .unwrap()
                        .amount = 0.25;
                    let mut bevel_model = Model::from_bodies(bodies.clone());
                    bevel_model.commit_changes(&beveled).unwrap();
                    let bevel_scene = evaluate_scene(&bevel_model.bodies).unwrap();
                    let rounded = &bevel_scene[2].1;
                    assert_eq!(rounded.face_count, g.face_count + 1);
                    assert_eq!(rounded.shell.as_ref().unwrap().len(), rounded.face_count);
                    assert!(rounded.shell.as_ref().unwrap().is_geometric_consistent());
                    // Check the retained arc geometry, independent of display accuracy.
                    use truck_modeling::{BoundedCurve, Curve, ParametricCurve, Surface};
                    assert_eq!(
                        rounded
                            .shell
                            .as_ref()
                            .unwrap()
                            .iter()
                            .filter(|f| !matches!(f.oriented_surface(), Surface::Plane(_)))
                            .count(),
                        1
                    );
                    let planes = components::straight_planar_supports(g).unwrap();
                    let ([first, second], axis) =
                        chamfer_corner(g, |face| Some(planes[face]), edge).unwrap();
                    let [a, _] = g.topology.edges[edge];
                    let n1 = planes[first].0;
                    let n2 = planes[second].0;
                    let center = g.points[a]
                        + (n1 + n2) * (if inner { 0.25 } else { -0.25 }) / (1. + n1.dot(n2));
                    let axis = axis.normalize();
                    let mut arcs = std::collections::HashSet::new();
                    for e in rounded
                        .shell
                        .as_ref()
                        .unwrap()
                        .edge_iter()
                        .filter(|e| matches!(e.curve(), Curve::NurbsCurve(_)))
                    {
                        arcs.insert(e.id());
                        let curve = e.oriented_curve();
                        let (lo, hi) = curve.range_tuple();
                        for i in 0..=32 {
                            let delta = curve.subs(lo + (hi - lo) * i as f64 / 32.) - center;
                            assert!((delta.cross(axis).magnitude() - 0.25).abs() < 1e-6);
                        }
                    }
                    assert_eq!(arcs.len(), 2, "Exactly one analytic arc per cap");
                    let expected = (1. - std::f64::consts::FRAC_PI_4)
                        * 0.25f64.powi(2)
                        * 3.
                        * if affine { 2. } else { 1. };
                    assert!(
                        (prism_volume(rounded, axis)
                            - prism_volume(g, axis)
                            - expected * if inner { 1. } else { -1. })
                        .abs()
                            < 1e-7
                    );
                    for format in [
                        crate::export::Format::Stl,
                        crate::export::Format::Glb,
                        crate::export::Format::Step,
                        crate::export::Format::Dxf,
                    ] {
                        assert!(
                            !crate::export::encode(&bevel_model.bodies, format)
                                .unwrap()
                                .is_empty()
                        );
                    }
                    let project =
                        Project::new(bevel_model.bodies.clone(), Camera::default(), true, Some(3));
                    project.save(&path).unwrap();
                    assert_eq!(Project::load(&path).unwrap().bodies, bevel_model.bodies);
                    let mut old = serde_json::to_value(&project).unwrap();
                    old["schema_version"] = 33.into();
                    old["modeling_version"] = 33.into();
                    old["bodies"][2]["component_edits"][0]["prism_curves"] = false.into();
                    assert!(
                        serde_json::from_value::<Project>(old)
                            .unwrap()
                            .validate()
                            .unwrap_err()
                            .contains("version 34")
                    );
                    let bevel_before = bevel_model.bodies.clone();
                    let mut invalid = beveled[0].clone();
                    invalid
                        .component_edits
                        .last_mut()
                        .unwrap()
                        .prism_bevel
                        .as_mut()
                        .unwrap()
                        .amount = 10.;
                    assert!(bevel_model.commit(invalid).is_err());
                    assert_eq!(bevel_model.bodies, bevel_before);
                    let mut flat = beveled[0].clone();
                    flat.component_edits
                        .last_mut()
                        .unwrap()
                        .prism_bevel
                        .as_mut()
                        .unwrap()
                        .style = Style::Flat;
                    bevel_model.commit(flat).unwrap();
                    let flat_scene = evaluate_scene(&bevel_model.bodies).unwrap();
                    let expected = 0.5 * 0.25f64.powi(2) * 3. * if affine { 2. } else { 1. };
                    assert!(
                        (volume(&flat_scene[2].1)
                            - volume(g)
                            - expected * if inner { 1. } else { -1. })
                        .abs()
                            < 1e-6
                    );
                    bevel_model.undo();
                    assert_eq!(bevel_model.bodies, bevel_before);
                    bevel_model.undo();
                    assert_eq!(bevel_model.bodies, bodies);
                    bevel_model.redo();
                    assert_eq!(bevel_model.bodies, bevel_before);
                    let repeated_edge = (0..rounded.edge_count)
                        .find(|edge| {
                            chamfer_corner(
                                rounded,
                                |face| {
                                    crate::snapping::Plane::face(rounded, face)
                                        .map(|p| (p.normal, p.normal.dot(p.origin.to_vec())))
                                },
                                *edge,
                            )
                            .is_ok_and(|(_, direction)| {
                                direction.normalize().dot(axis).abs() > 1. - 1e-7
                            }) && (!(!hole && !affine && inner)
                                || rounded.topology.edges[*edge].iter().all(|v| {
                                    rounded.points[*v].x.abs() < 1e-7
                                        && (rounded.points[*v].y - 1.).abs() < 1e-7
                                }))
                        })
                        .unwrap();
                    for kind in [Kind::Chamfer, Kind::Bevel] {
                        let mut repeated = selected_operations(
                            &bevel_model.bodies,
                            &bevel_scene,
                            &[(3, Some(repeated_edge))],
                            SelectionMode::Edge,
                            kind,
                        )
                        .unwrap();
                        assert!(repeated[0].component_edits.last().unwrap().prism_curves);
                        if let Some(bevel) =
                            &mut repeated[0].component_edits.last_mut().unwrap().prism_bevel
                        {
                            bevel.amount = 0.1;
                        } else {
                            repeated[0].component_edits.last_mut().unwrap().edge_chamfer =
                                Some([0.1, 0.15]);
                        }
                        let mut repeated_model = Model::from_bodies(bevel_model.bodies.clone());
                        repeated_model.commit_changes(&repeated).unwrap();
                        let evaluated = evaluate_scene(&repeated_model.bodies).unwrap();
                        let result = &evaluated[2].1;
                        assert_eq!(result.face_count, rounded.face_count + 1);
                        assert_eq!(
                            result
                                .shell
                                .as_ref()
                                .unwrap()
                                .iter()
                                .filter(|f| !matches!(f.oriented_surface(), Surface::Plane(_)))
                                .count(),
                            if kind == Kind::Bevel { 2 } else { 1 }
                        );
                        // The earlier circular band keeps its original radius and axis.
                        let mut old_arcs = std::collections::HashSet::new();
                        for e in result
                            .shell
                            .as_ref()
                            .unwrap()
                            .edge_iter()
                            .filter(|e| matches!(e.curve(), Curve::NurbsCurve(_)))
                        {
                            let curve = e.oriented_curve();
                            let (lo, hi) = curve.range_tuple();
                            if ((curve.subs((lo + hi) * 0.5) - center)
                                .cross(axis)
                                .magnitude()
                                - 0.25)
                                .abs()
                                > 1e-7
                            {
                                continue;
                            }
                            old_arcs.insert(e.id());
                            for i in 0..=32 {
                                assert!(
                                    ((curve.subs(lo + (hi - lo) * i as f64 / 32.) - center)
                                        .cross(axis)
                                        .magnitude()
                                        - 0.25)
                                        .abs()
                                        < 1e-7
                                );
                            }
                        }
                        assert_eq!(old_arcs.len(), 2);
                        let project = Project::new(
                            repeated_model.bodies.clone(),
                            Camera::default(),
                            true,
                            Some(3),
                        );
                        project.save(&path).unwrap();
                        assert_eq!(Project::load(&path).unwrap().bodies, repeated_model.bodies);
                        let mut old = serde_json::to_value(&project).unwrap();
                        old["schema_version"] = 34.into();
                        old["modeling_version"] = 34.into();
                        assert!(
                            serde_json::from_value::<Project>(old)
                                .unwrap()
                                .validate()
                                .unwrap_err()
                                .contains("version 35")
                        );
                        let before = repeated_model.bodies.clone();
                        let mut invalid = repeated[0].clone();
                        invalid.component_edits.last_mut().unwrap().prism_curves = false;
                        assert!(repeated_model.commit(invalid).is_err());
                        assert_eq!(repeated_model.bodies, before);
                        repeated_model.undo();
                        assert_eq!(repeated_model.bodies, bevel_model.bodies);
                        repeated_model.redo();
                        assert_eq!(repeated_model.bodies, before);
                        for format in [
                            crate::export::Format::Stl,
                            crate::export::Format::Glb,
                            crate::export::Format::Step,
                            crate::export::Format::Dxf,
                        ] {
                            assert!(
                                !crate::export::encode(&repeated_model.bodies, format)
                                    .unwrap()
                                    .is_empty()
                            );
                        }
                    }
                    if !hole
                        && !affine
                        && inner
                        && let Some(path) = std::env::var_os("OSK_REPEAT_TREATMENT_REVIEW_PROJECT")
                    {
                        let mut review = Project::new(
                            bevel_model.bodies.clone(),
                            Camera::default(),
                            true,
                            Some(3),
                        );
                        review.selection_mode = SelectionMode::Edge;
                        review.selected_element = Some(repeated_edge);
                        review.selection = vec![(3, Some(repeated_edge))];
                        review.save(std::path::Path::new(&path)).unwrap();
                    }
                    model.undo();
                    assert_eq!(model.bodies, bodies);
                    model.redo();
                    assert_eq!(model.bodies, before);
                    if !hole && !affine && inner {
                        let other = g
                            .topology
                            .edges
                            .iter()
                            .position(|[a, b]| {
                                let p = g.points[*a];
                                let q = g.points[*b];
                                p.x.abs() < 1e-7
                                    && q.x.abs() < 1e-7
                                    && (p.y - 1.).abs() < 1e-7
                                    && (q.y - 1.).abs() < 1e-7
                                    && (p.z - q.z).abs() > 2.9
                            })
                            .unwrap();
                        let mut both = selected_operations(
                            &bodies,
                            &scene,
                            &[(3, Some(edge)), (3, Some(other))],
                            SelectionMode::Edge,
                            Kind::Chamfer,
                        )
                        .unwrap();
                        assert_eq!(both[0].component_edits.len(), 1);
                        both[0].component_edits[0].edge_chamfer = Some([0.2, 0.3]);
                        let mut multiple = bodies.clone();
                        multiple[2] = both[0].clone();
                        let combined = evaluate_scene(&multiple).unwrap();
                        assert!(
                            (volume(&combined[2].1) - volume(g) - 2. * 0.5 * 0.2 * 0.3 * 3.).abs()
                                < 1e-6
                        );
                        assert_eq!(combined[2].1.face_count, g.face_count + 2);
                        let mut rounded_pair = selected_operations(
                            &bodies,
                            &scene,
                            &[(3, Some(edge)), (3, Some(other))],
                            SelectionMode::Edge,
                            Kind::Bevel,
                        )
                        .unwrap();
                        rounded_pair[0].component_edits[0]
                            .prism_bevel
                            .as_mut()
                            .unwrap()
                            .amount = 0.25;
                        let mut multiple = bodies.clone();
                        multiple[2] = rounded_pair[0].clone();
                        let combined = evaluate_scene(&multiple).unwrap();
                        assert_eq!(combined[2].1.face_count, g.face_count + 2);
                        let expected =
                            2. * (1. - std::f64::consts::FRAC_PI_4) * 0.25f64.powi(2) * 3.;
                        assert!(
                            (prism_volume(&combined[2].1, Vector3::unit_z())
                                - prism_volume(g, Vector3::unit_z())
                                - expected)
                                .abs()
                                < 1e-7
                        );
                        rounded_pair[0].component_edits[0]
                            .prism_bevel
                            .as_mut()
                            .unwrap()
                            .amount = 1.2;
                        let before = model.bodies.clone();
                        assert!(model.commit_changes(&rounded_pair).is_err());
                        assert_eq!(model.bodies, before);
                        both[0].component_edits[0].edge_chamfer = Some([1.2; 2]);
                        let before = model.bodies.clone();
                        assert!(model.commit_changes(&both).is_err());
                        assert_eq!(model.bodies, before);
                        if let Some(path) = std::env::var_os("OSK_CONCAVE_CHAMFER_REVIEW_PROJECT")
                            .or_else(|| std::env::var_os("OSK_PRISM_BEVEL_REVIEW_PROJECT"))
                        {
                            let mut review =
                                Project::new(bodies.clone(), Camera::default(), true, Some(3));
                            review.selection_mode = SelectionMode::Edge;
                            review.selected_element = Some(edge);
                            review.selection = vec![(3, Some(edge))];
                            review.save(std::path::Path::new(&path)).unwrap();
                        }
                    }
                }
            }
        }
    }
    #[test]
    fn prism_treatments_preserve_unrelated_circular_through_holes() {
        use crate::{booleans, selection::SelectionMode};
        let mut base = Body::new(Shape::Rectangle);
        base.id = 1;
        base.width = 6.;
        base.depth = 6.;
        base.extrusion = 3.;
        let mut tool = Body::new(Shape::Circle);
        tool.id = 2;
        tool.width = 1.;
        tool.extrusion = 3.;
        tool.position[0] = -1.;
        let mut bodies = vec![base, tool];
        let scene = evaluate_scene(&bodies).unwrap();
        let mut feature = Body::new(Shape::Rectangle);
        feature.id = 3;
        feature.boolean = Some(booleans::Boolean::new(
            booleans::Kind::Subtract,
            &[(1, None), (2, None)],
            SelectionMode::Volume,
            &scene,
        ));
        bodies.push(feature);
        let mut tool = bodies[1].clone();
        tool.id = 4;
        tool.position[0] = 1.;
        bodies.push(tool);
        let scene = evaluate_scene(&bodies).unwrap();
        let mut feature = Body::new(Shape::Rectangle);
        feature.id = 5;
        feature.boolean = Some(booleans::Boolean::new(
            booleans::Kind::Subtract,
            &[(3, None), (4, None)],
            SelectionMode::Volume,
            &scene,
        ));
        bodies.push(feature);
        let scene = evaluate_scene(&bodies).unwrap();
        let g = &scene[4].1;
        assert_eq!(g.circles.len(), 4);
        let edge = g
            .topology
            .edges
            .iter()
            .position(|[a, b]| {
                [a, b].iter().all(|v| {
                    let p = g.points[**v];
                    (p.x + 3.).abs() < 1e-7 && (p.y + 3.).abs() < 1e-7
                }) && (g.points[*a].z - g.points[*b].z).abs() > 2.9
            })
            .unwrap();
        for kind in [Kind::Chamfer, Kind::Bevel] {
            let changes = selected_operations(
                &bodies,
                &scene,
                &[(5, Some(edge))],
                SelectionMode::Edge,
                kind,
            )
            .unwrap();
            assert!(changes[0].component_edits[0].prism_curves);
            let mut model = Model::from_bodies(bodies.clone());
            model.commit_changes(&changes).unwrap();
            let evaluated = evaluate_scene(&model.bodies).unwrap();
            let result = &evaluated[4].1;
            assert_eq!(result.circles.len(), 4);
            for circle in &g.circles {
                assert!(
                    result
                        .circles
                        .iter()
                        .any(|other| (circle.center - other.center).magnitude() < 1e-7)
                );
            }
            assert!(result.shell.as_ref().unwrap().is_geometric_consistent());
            crate::components::solid_parts(result.shell.as_ref().unwrap()).unwrap();
            assert!(prism_volume(result, Vector3::unit_z()) < prism_volume(g, Vector3::unit_z()));
            let next = result
                .topology
                .edges
                .iter()
                .position(|[a, b]| {
                    [a, b].iter().all(|v| {
                        let p = result.points[**v];
                        (p.x - 3.).abs() < 1e-7 && (p.y + 3.).abs() < 1e-7
                    }) && (result.points[*a].z - result.points[*b].z).abs() > 2.9
                })
                .unwrap();
            let repeated = selected_operations(
                &model.bodies,
                &evaluated,
                &[(5, Some(next))],
                SelectionMode::Edge,
                Kind::Chamfer,
            )
            .unwrap();
            model.commit_changes(&repeated).unwrap();
            assert_eq!(evaluate_scene(&model.bodies).unwrap()[4].1.circles.len(), 4);
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("two-holes.osk");
            Project::new(model.bodies.clone(), Camera::default(), true, Some(5))
                .save(&path)
                .unwrap();
            assert_eq!(Project::load(&path).unwrap().bodies, model.bodies);

            for format in [
                crate::export::Format::Stl,
                crate::export::Format::Glb,
                crate::export::Format::Step,
                crate::export::Format::Dxf,
            ] {
                assert!(
                    !crate::export::encode(&model.bodies, format)
                        .unwrap()
                        .is_empty()
                );
            }
        }
    }

    #[test]
    fn analytic_prism_bevel_uses_radius_and_tangency_at_oblique_corners() {
        use crate::{booleans, selection::SelectionMode};
        use truck_modeling::{BoundedCurve, Curve, ParametricCurve};
        let mut a = Body::new(Shape::Rectangle);
        a.id = 1;
        a.width = 4.;
        a.depth = 4.;
        a.extrusion = 3.;
        let mut b = a.clone();
        b.id = 2;
        b.position[0] = 1.;
        b.rotation[2] = 25.;
        let mut bodies = vec![a, b];
        let scene = evaluate_scene(&bodies).unwrap();
        let mut feature = Body::new(Shape::Rectangle);
        feature.id = 3;
        feature.boolean = Some(booleans::Boolean::new(
            booleans::Kind::Intersect,
            &[(1, None), (2, None)],
            SelectionMode::Volume,
            &scene,
        ));
        bodies.push(feature);
        let scene = evaluate_scene(&bodies).unwrap();
        let g = &scene[2].1;
        let planes = crate::components::straight_planar_supports(g).unwrap();
        let edge = (0..g.edge_count)
            .find(|edge| {
                chamfer_corner(g, |face| Some(planes[face]), *edge).is_ok_and(|([a, b], axis)| {
                    axis.normalize().z.abs() > 0.99 && planes[a].0.dot(planes[b].0).abs() > 0.1
                })
            })
            .unwrap();
        let ([a, b], axis) = chamfer_corner(g, |face| Some(planes[face]), edge).unwrap();
        let cosine = planes[a].0.dot(planes[b].0);
        let mut changes = selected_operations(
            &bodies,
            &scene,
            &[(3, Some(edge))],
            SelectionMode::Edge,
            Kind::Bevel,
        )
        .unwrap();
        changes[0].component_edits[0]
            .prism_bevel
            .as_mut()
            .unwrap()
            .amount = 0.2;
        let mut model = Model::from_bodies(bodies.clone());
        model.commit_changes(&changes).unwrap();
        let result = evaluate_scene(&model.bodies).unwrap();
        let result = &result[2].1;
        let expected = 0.2f64.powi(2)
            * (((1. - cosine) / (1. + cosine)).sqrt() - cosine.acos() * 0.5)
            * axis.magnitude();
        assert!((prism_volume(g, axis) - prism_volume(result, axis) - expected).abs() < 1e-7);
        for edge in result
            .shell
            .as_ref()
            .unwrap()
            .edge_iter()
            .filter(|e| matches!(e.curve(), Curve::NurbsCurve(_)))
        {
            let curve = edge.oriented_curve();
            let (lo, hi) = curve.range_tuple();
            for t in [lo, hi] {
                let point = curve.subs(t);
                let tangent = curve.der(t).normalize();
                assert!([planes[a], planes[b]].iter().any(|(normal, offset)| {
                    (normal.dot(point.to_vec()) - *offset).abs() < 1e-7
                        && normal.dot(tangent).abs() < 1e-7
                }));
            }
        }
        let mut invalid = changes[0].clone();
        invalid.component_edits[0].edge_chamfer = Some([0.2; 2]);
        assert!(invalid.validate().is_err());
        invalid = changes[0].clone();
        invalid.component_edits[0]
            .prism_bevel
            .as_mut()
            .unwrap()
            .amount = f64::NAN;
        assert!(invalid.validate().is_err());
    }

    #[test]
    fn evaluated_planar_chamfers_replay_edit_move_export_and_rollback() {
        use crate::{booleans, components, selection::SelectionMode};
        use truck_modeling::{Deg, Matrix4};
        let mut a = Body::new(Shape::Rectangle);
        a.id = 1;
        a.width = 4.;
        a.depth = 4.;
        a.extrusion = 3.;
        let mut b = a.clone();
        b.id = 2;
        b.position[0] = 1.;
        let mut bodies = vec![a, b];
        let scene = evaluate_scene(&bodies).unwrap();
        let mut feature = Body::new(Shape::Rectangle);
        feature.id = 3;
        feature.boolean = Some(booleans::Boolean::new(
            booleans::Kind::Intersect,
            &[(1, None), (2, None)],
            SelectionMode::Volume,
            &scene,
        ));
        bodies.push(feature.clone());
        let scene = evaluate_scene(&bodies).unwrap();
        let g = &scene[2].1;
        let edge = g
            .topology
            .edges
            .iter()
            .position(|[a, b]| (g.points[*a].z - g.points[*b].z).abs() > 2.9)
            .unwrap();
        let mut changes = selected_operations(
            &bodies,
            &scene,
            &[(3, Some(edge))],
            SelectionMode::Edge,
            Kind::Chamfer,
        )
        .unwrap();
        assert!(changes[0].edge_treatments.is_empty());
        changes[0].component_edits.last_mut().unwrap().edge_chamfer = Some([0.2, 0.4]);
        let mut model = Model::from_bodies(bodies.clone());
        model.commit_changes(&changes).unwrap();
        let evaluated = evaluate_scene(&model.bodies).unwrap();
        let result = &evaluated[2].1;
        assert_eq!(result.face_count, g.face_count + 1);
        assert!((volume(g) - volume(result) - 0.5 * 0.2 * 0.4 * 3.).abs() < 1e-6);
        assert!(result.shell.as_ref().unwrap().is_geometric_consistent());
        // Both asymmetric distance order and non-right dihedral angles rebuild
        // exact planar boundaries; retained chamfer faces stay movable.
        let mut reversed = changes[0].clone();
        reversed.component_edits.last_mut().unwrap().edge_chamfer = Some([0.4, 0.2]);
        let mut reverse_bodies = bodies.clone();
        reverse_bodies[2] = reversed;
        let reverse = evaluate_scene(&reverse_bodies).unwrap();
        assert!((volume(&reverse[2].1) - volume(result)).abs() < 1e-6);
        assert_ne!(reverse[2].1.points, result.points);
        let band = result.face_count - 1;
        let normal = components::planar_supports(result).unwrap()[band].0;
        let moved = components::transformed(
            &changes[0],
            result,
            SelectionMode::Face,
            Some(band),
            Matrix4::from_translation(normal * 0.05),
        )
        .unwrap();
        let mut moved_bodies = bodies.clone();
        moved_bodies[2] = moved;
        let moved_scene = evaluate_scene(&moved_bodies).unwrap();
        assert!(volume(&moved_scene[2].1) > volume(result));
        let rotated = components::transformed(
            &changes[0],
            result,
            SelectionMode::Face,
            Some(band),
            Matrix4::from_angle_z(Deg(1.)),
        )
        .unwrap();
        let mut rotated_bodies = bodies.clone();
        rotated_bodies[2] = rotated;
        assert!(evaluate_scene(&rotated_bodies).is_ok());
        for format in [
            crate::export::Format::Stl,
            crate::export::Format::Glb,
            crate::export::Format::Step,
            crate::export::Format::Dxf,
        ] {
            assert!(
                !crate::export::encode(&model.bodies, format)
                    .unwrap()
                    .is_empty()
            );
        }
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("chamfer.osk");
        let project = Project::new(model.bodies.clone(), Camera::default(), true, Some(3));
        project.save(&path).unwrap();
        assert_eq!(Project::load(&path).unwrap().bodies, model.bodies);
        let mut old = serde_json::to_value(&project).unwrap();
        old["schema_version"] = 31.into();
        old["modeling_version"] = 31.into();
        assert!(
            serde_json::from_value::<Project>(old)
                .unwrap()
                .validate()
                .unwrap_err()
                .contains("version 32")
        );
        let mut invalid = changes[0].clone();
        invalid.component_edits.last_mut().unwrap().edge_chamfer = Some([20.; 2]);
        let before = model.bodies.clone();
        assert!(model.commit(invalid).is_err());
        assert_eq!(model.bodies, before);
        model.undo();
        assert_eq!(model.bodies, bodies);
        model.redo();
        assert_eq!(model.bodies, before);
        let mut removed = changes[0].clone();
        removed.component_edits.clear();
        model.commit(removed).unwrap();
        assert!((volume(&evaluate_scene(&model.bodies).unwrap()[2].1) - volume(g)).abs() < 1e-6);
        let mut invalid = changes[0].clone();
        invalid.component_edits.last_mut().unwrap().edge_chamfer = Some([f64::NAN, 0.2]);
        assert!(invalid.validate().is_err());
        let top = components::planar_supports(g)
            .unwrap()
            .iter()
            .position(|(n, _)| n.z > 0.99)
            .unwrap();
        let perimeter = selected_operations(
            &bodies,
            &scene,
            &[(3, Some(top))],
            SelectionMode::Face,
            Kind::Chamfer,
        )
        .unwrap();
        assert_eq!(
            perimeter[0].component_edits.last().unwrap().elements.len(),
            3
        );
        let mut perimeter_bodies = bodies.clone();
        perimeter_bodies[2] = perimeter[0].clone();
        assert_eq!(
            evaluate_scene(&perimeter_bodies).unwrap()[2].1.face_count,
            10
        );
        assert!(
            selected_operations(
                &bodies,
                &scene,
                &[(3, Some(edge)), (3, Some(usize::MAX))],
                SelectionMode::Edge,
                Kind::Chamfer
            )
            .is_err()
        );
        // Nonuniform source placement is inverted before applying local setbacks.
        let mut source = bodies[0].clone();
        source.position = [2., -1., 4.];
        source.rotation = [13., 19., 27.];
        source.scale = [1.5, 0.75, 2.];
        source
            .edge_treatments
            .push(Modifier::new(Kind::Chamfer, FaceTarget::Top, &source));
        let source_scene = evaluate_scene(&[source.clone()]).unwrap();
        let transition = (0..source_scene[0].1.edge_count)
            .find(|edge| box_edge(&source, &source_scene[0].1, *edge).is_err())
            .unwrap();
        let change = selected_operations(
            &[source.clone()],
            &source_scene,
            &[(1, Some(transition))],
            SelectionMode::Edge,
            Kind::Chamfer,
        )
        .unwrap();
        assert!(volume(&evaluate(&change[0]).unwrap()) < volume(&source_scene[0].1));
        let mut stale = change[0].clone();
        stale
            .component_edits
            .last_mut()
            .unwrap()
            .topology
            .edges
            .pop();
        assert!(evaluate_scene(&[stale]).is_err());
        if let Some(path) = std::env::var_os("OSK_EVALUATED_CHAMFER_REVIEW_PROJECT") {
            let mut review = Project::new(bodies.clone(), Camera::default(), true, Some(3));
            review.selection_mode = SelectionMode::Edge;
            review.selected_element = Some(edge);
            review.selection = vec![(3, Some(edge))];
            review.save(std::path::Path::new(&path)).unwrap();
        }
    }
    #[test]
    fn planar_intersection_handles_coincident_unequal_depth_and_affine_inputs() {
        use truck_modeling::{Deg, Matrix4};
        let mut a = Body::new(Shape::Rectangle);
        a.width = 4.;
        a.depth = 3.;
        a.extrusion = 3.;
        let mut b = Body::new(Shape::Rectangle);
        b.width = 3.;
        b.depth = 2.;
        b.extrusion = 4.;
        b.position = [1., 0.5, -1.];
        let transform = Matrix4::from_angle_z(Deg(13.))
            * Matrix4::from_angle_y(Deg(17.))
            * Matrix4::from_nonuniform_scale(1.5, 0.75, 2.);
        for affine in [false, true] {
            let mut ga = evaluate(&a).unwrap();
            let mut gb = evaluate(&b).unwrap();
            if affine {
                crate::components::map_geometry(&mut ga, transform);
                crate::components::map_geometry(&mut gb, transform);
            }
            for (first, second) in [(&ga, &gb), (&gb, &ga)] {
                let result = intersect_convex(first, second).unwrap().unwrap();
                assert!((volume(&result) - 15. * if affine { 2.25 } else { 1. }).abs() < 1e-6);
                assert!(result.is_solid);
                crate::components::solid_parts(result.shell.as_ref().unwrap()).unwrap();
                crate::components::planar_supports(&result).unwrap();
            }
            let identical = intersect_convex(&ga, &ga).unwrap().unwrap();
            assert_eq!(identical.topology, ga.topology);
            assert_eq!(identical.triangles, ga.triangles);
        }
        let ga = evaluate(&a).unwrap();
        for x in [3.5, 20.] {
            b.position[0] = x;
            assert!(
                intersect_convex(&ga, &evaluate(&b).unwrap())
                    .unwrap()
                    .unwrap()
                    .triangles
                    .is_empty()
            );
        }
        b.width = 1.;
        b.depth = 1.;
        b.extrusion = 1.;
        b.position = [0., 0., 1.];
        let gb = evaluate(&b).unwrap();
        assert_eq!(
            intersect_convex(&ga, &gb).unwrap().unwrap().triangles,
            gb.triangles
        );
        let mut curved = Body::new(Shape::Circle);
        curved.extrusion = 3.;
        assert!(
            intersect_convex(&ga, &evaluate(&curved).unwrap())
                .unwrap()
                .is_none()
        );
    }
    #[test]
    fn selected_box_edges_and_multi_input_are_local_atomic_and_persistent() {
        use crate::selection::SelectionMode;
        let mut source = Body::new(Shape::Rectangle);
        source.id = 1;
        source.width = 4.;
        source.depth = 5.;
        source.extrusion = 6.;
        let scene = evaluate_scene(&[source.clone()]).unwrap();
        let original = &scene[0].1;
        assert_eq!(original.edge_count, 12);
        for edge in 0..12 {
            let (a, b) = box_edge(&source, original, edge).unwrap();
            assert_ne!(a, b);
            let changes = selected_operations(
                &[source.clone()],
                &scene,
                &[(1, Some(edge))],
                SelectionMode::Edge,
                Kind::Chamfer,
            )
            .unwrap();
            let operation = &changes[0].edge_treatments[0];
            assert_eq!(operation.edge, Some(b));
            let geometry = evaluate(&changes[0]).unwrap();
            assert_eq!(geometry.face_count, 7);
            let vertices = original.topology.edges[edge];
            let length = (original.points[vertices[1]] - original.points[vertices[0]]).magnitude();
            let expected = 120. - 0.5 * operation.amount * operation.amount * length;
            assert!((volume(&geometry) - expected).abs() < 1e-6);
            for format in [
                crate::export::Format::Stl,
                crate::export::Format::Glb,
                crate::export::Format::Step,
                crate::export::Format::Dxf,
            ] {
                assert!(!crate::export::encode(&changes, format).unwrap().is_empty());
            }
            let mut duplicate = changes[0].clone();
            let mut other = operation.clone();
            other.face = b;
            other.edge = Some(a);
            duplicate.edge_treatments.push(other);
            assert!(duplicate.validate().is_err());
            let transition = (0..geometry.edge_count)
                .find(|edge| box_edge(&changes[0], &geometry, *edge).is_err())
                .unwrap();
            let further = selected_operations(
                &changes,
                &[(1, geometry.clone())],
                &[(1, Some(transition))],
                SelectionMode::Edge,
                Kind::Chamfer,
            )
            .unwrap();
            assert!(
                further[0]
                    .component_edits
                    .last()
                    .unwrap()
                    .edge_chamfer
                    .is_some()
            );
            assert!(volume(&evaluate(&further[0]).unwrap()) < volume(&geometry));
        }
        let picks: Vec<_> = (0..12).map(|edge| (1, Some(edge))).collect();
        let changes = selected_operations(
            &[source.clone()],
            &scene,
            &picks,
            SelectionMode::Edge,
            Kind::Chamfer,
        )
        .unwrap();
        assert_eq!(changes[0].edge_treatments.len(), 12);
        let mut model = Model::from_bodies(vec![source.clone()]);
        model.commit_changes(&changes).unwrap();
        let geometry = evaluate_scene(&model.bodies).unwrap();
        assert!(volume(&geometry[0].1) > 100.);
        let file = tempfile::NamedTempFile::new().unwrap().into_temp_path();
        let project = Project::new(model.bodies.clone(), Camera::default(), true, Some(1));
        project.save(file.as_ref()).unwrap();
        assert_eq!(Project::load(file.as_ref()).unwrap().bodies, model.bodies);
        let mut old = serde_json::to_value(&project).unwrap();
        old["schema_version"] = 8.into();
        old["modeling_version"] = 8.into();
        assert!(
            serde_json::from_value::<Project>(old)
                .unwrap()
                .validate()
                .is_err()
        );
        model.undo();
        assert_eq!(model.bodies, vec![source.clone()]);
        model.redo();
        assert_eq!(model.bodies, changes);
        let mut second = source.clone();
        second.id = 2;
        second.position[0] = 8.;
        let sources = vec![source.clone(), second];
        let scene = evaluate_scene(&sources).unwrap();
        let changes = selected_operations(
            &sources,
            &scene,
            &[(1, Some(0)), (2, Some(1))],
            SelectionMode::Edge,
            Kind::Bevel,
        )
        .unwrap();
        assert_eq!(changes.len(), 2);
        let mut model = Model::from_bodies(sources.clone());
        model.commit_changes(&changes).unwrap();
        model.undo();
        assert_eq!(model.bodies, sources);
        assert!(
            selected_operations(
                &sources,
                &scene,
                &[(1, Some(0)), (2, Some(usize::MAX))],
                SelectionMode::Edge,
                Kind::Bevel
            )
            .is_err()
        );
        assert_eq!(model.bodies, sources);
        let picks: Vec<_> = (0..6)
            .filter(|f| {
                matches!(
                    target(&source, &scene[0].1, *f),
                    Ok(FaceTarget::Top | FaceTarget::Bottom)
                )
            })
            .map(|f| (1, Some(f)))
            .collect();
        let changes =
            selected_operations(&sources, &scene, &picks, SelectionMode::Face, Kind::Chamfer)
                .unwrap();
        assert_eq!(changes[0].edge_treatments.len(), 2);
        evaluate_scene(&changes).unwrap();
        let mut posed = source.clone();
        posed.position = [1., 2., 3.];
        posed.rotation = [15., 25., 35.];
        posed.scale = [1.5, 0.75, 2.];
        let scene = evaluate_scene(&[posed.clone()]).unwrap();
        for edge in 0..scene[0].1.edge_count {
            box_edge(&posed, &scene[0].1, edge).unwrap();
        }
    }
    #[test]
    fn asymmetric_chamfers_preserve_setbacks_direction_history_and_snapping() {
        for shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            let mut source = Body::new(shape);
            source.id = 1;
            source.width = 4.;
            source.depth = 6.;
            source.extrusion = 4.;
            source.position = [2., -3., 5.];
            source.rotation = [15., 25., 35.];
            source.scale = [1.5, 0.75, 2.];
            for face in [
                FaceTarget::Top,
                FaceTarget::Bottom,
                FaceTarget::Left,
                FaceTarget::Right,
                FaceTarget::Front,
                FaceTarget::Back,
            ] {
                if !face.supported(&source) {
                    continue;
                }
                for reverse in [false, true] {
                    let mut body = source.clone();
                    let mut chamfer = Modifier::new(Kind::Chamfer, face, &body);
                    chamfer.amount = 0.25;
                    chamfer.chamfer = ChamferDefinition::TwoDistances { second: 0.5 };
                    chamfer.reverse = reverse;
                    let (first, second) = chamfer.setbacks();
                    body.edge_treatments.push(chamfer);
                    let g = evaluate(&body).unwrap();
                    let inv = crate::regions::transform(&body).invert().unwrap();
                    let (normal, d) = face.plane(&body);
                    let selected: Vec<_> = g
                        .triangles
                        .iter()
                        .zip(&g.triangle_faces)
                        .filter(|(_, f)| target(&body, &g, **f) == Ok(face))
                        .flat_map(|(t, _)| t.iter().map(|p| inv.transform_point(*p)))
                        .collect();
                    assert!(!selected.is_empty());
                    if shape == Shape::Rectangle {
                        for axis in [Vector3::unit_x(), Vector3::unit_y(), Vector3::unit_z()]
                            .into_iter()
                            .filter(|axis| axis.dot(normal).abs() < 0.5)
                        {
                            let (lo, hi) = if axis.x > 0.5 {
                                (-2., 2.)
                            } else if axis.y > 0.5 {
                                (-3., 3.)
                            } else {
                                (0., 4.)
                            };
                            let min = selected
                                .iter()
                                .map(|p| p.to_vec().dot(axis))
                                .fold(f64::INFINITY, f64::min);
                            let max = selected
                                .iter()
                                .map(|p| p.to_vec().dot(axis))
                                .fold(f64::NEG_INFINITY, f64::max);
                            assert!(
                                (min - lo - first).abs() < 1e-6 && (max - hi + first).abs() < 1e-6,
                                "{face:?} {first}: {min}..{max}"
                            );
                        }
                    }
                    assert!(
                        g.points.iter().map(|p| inv.transform_point(*p)).any(|p| (p
                            .to_vec()
                            .dot(normal)
                            - (d - second))
                            .abs()
                            < 1e-6)
                    );
                    if shape == Shape::Circle {
                        assert_eq!(g.face_count, 4);
                        let z = if face == FaceTarget::Top { 4. } else { 0. };
                        assert!(g.circles.iter().any(|c| {
                            let center = inv.transform_point(c.center);
                            let u = inv.transform_vector(c.u);
                            (center.z - z).abs() < 1e-6
                                && (u.magnitude() - (2. - first)).abs() < 1e-6
                        }));
                    }
                    if face == FaceTarget::Top && !reverse {
                        for format in [
                            crate::export::Format::Stl,
                            crate::export::Format::Glb,
                            crate::export::Format::Step,
                            crate::export::Format::Dxf,
                        ] {
                            assert!(
                                !crate::export::encode(&[body.clone()], format)
                                    .unwrap()
                                    .is_empty()
                            );
                        }
                    }
                    let mut angle_body = body.clone();
                    angle_body.edge_treatments[0].chamfer = ChamferDefinition::DistanceAngle {
                        degrees: 2_f64.atan().to_degrees(),
                    };
                    let angle = evaluate(&angle_body).unwrap();
                    assert!((volume(&g) - volume(&angle)).abs() < 1e-6);
                    let mut model = Model::from_bodies(vec![source.clone()]);
                    model.commit(body.clone()).unwrap();
                    let project =
                        Project::new(model.bodies.clone(), Camera::default(), true, Some(1));
                    let file = tempfile::NamedTempFile::new().unwrap().into_temp_path();
                    project.save(file.as_ref()).unwrap();
                    assert_eq!(Project::load(file.as_ref()).unwrap().bodies, model.bodies);
                    let mut legacy = serde_json::to_value(&project).unwrap();
                    legacy["schema_version"] = 7.into();
                    legacy["modeling_version"] = 7.into();
                    assert!(
                        serde_json::from_value::<Project>(legacy.clone())
                            .unwrap()
                            .validate()
                            .is_err()
                    );
                    legacy["bodies"][0]["edge_treatments"][0]
                        .as_object_mut()
                        .unwrap()
                        .remove("chamfer");
                    legacy["bodies"][0]["edge_treatments"][0]
                        .as_object_mut()
                        .unwrap()
                        .remove("reverse");
                    serde_json::from_value::<Project>(legacy)
                        .unwrap()
                        .validate()
                        .unwrap();
                    let before = model.bodies.clone();
                    let mut bad = body.clone();
                    bad.edge_treatments[0].chamfer =
                        ChamferDefinition::DistanceAngle { degrees: 90. };
                    assert!(model.commit(bad).is_err());
                    assert_eq!(model.bodies, before);
                    model.undo();
                    assert_eq!(model.bodies, vec![source.clone()]);
                    model.redo();
                    assert_eq!(model.bodies, vec![body]);
                }
            }
        }
    }
    #[test]
    fn face_and_cylinder_rim_modifiers_are_editable_closed_and_persistent() {
        for shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            let mut b = Body::new(shape);
            b.id = 1;
            b.width = 4.;
            b.depth = 3.;
            b.extrusion = 4.;
            let base = evaluate(&b).unwrap();
            let original = volume(&base);
            let top = (0..base.face_count)
                .find(|f| target(&b, &base, *f) == Ok(FaceTarget::Top))
                .unwrap();
            assert_eq!(target(&b, &base, top).unwrap(), FaceTarget::Top);
            if shape != Shape::Rectangle {
                let edge = (0..base.edge_count)
                    .find(|e| rim_target(&b, &base, *e) == Ok(FaceTarget::Top))
                    .unwrap();
                assert_eq!(rim_target(&b, &base, edge).unwrap(), FaceTarget::Top);
            }
            for kind in [Kind::Chamfer, Kind::Bevel] {
                b.edge_treatments = vec![Modifier::new(kind, FaceTarget::Top, &b)];
                let g = evaluate(&b).unwrap_or_else(|e| panic!("{shape:?}/{kind:?}: {e}"));
                if shape == Shape::Circle {
                    assert_eq!(
                        g.face_count, 4,
                        "a rim is one surface, not its render subdivisions"
                    );
                }
                if shape == Shape::Rectangle && kind == Kind::Bevel {
                    assert_eq!(g.face_count, 10, "four bevel strips plus six source faces");
                }
                let v = volume(&g);
                assert!(v > 0. && v < original);
                crate::components::boundary_shell(&g, true).unwrap();
                assert!(
                    g.triangles
                        .iter()
                        .flatten()
                        .all(|p| p.z >= -1e-8 && p.z <= b.extrusion + 1e-8)
                );
                if shape == Shape::Circle {
                    let face = (0..g.face_count)
                        .find(|f| target(&b, &g, *f) == Ok(FaceTarget::Top))
                        .unwrap();
                    let max_radius = g
                        .triangles
                        .iter()
                        .zip(&g.triangle_faces)
                        .filter(|(_, f)| **f == face)
                        .flat_map(|(t, _)| t.iter())
                        .map(|p| p.x.hypot(p.y))
                        .fold(0., f64::max);
                    assert!(
                        (max_radius - (b.width / 2. - b.edge_treatments[0].amount)).abs() < 1e-6
                    );
                }
                b.edge_treatments[0].amount *= 1.5;
                let edited = evaluate(&b).unwrap();
                assert!(volume(&edited) < v);
                if kind == Kind::Bevel {
                    b.edge_treatments[0].style = Style::Flat;
                    let flat = evaluate(&b).unwrap();
                    assert!((volume(&flat) - volume(&edited)).abs() > 1e-5);
                    b.edge_treatments[0].style = Style::Rounded;
                }
                b.edge_treatments
                    .push(Modifier::new(kind, FaceTarget::Bottom, &b));
                evaluate(&b).unwrap_or_else(|e| panic!("two rims {shape:?}/{kind:?}: {e}"));
                let project = Project::new(vec![b.clone()], Camera::default(), true, Some(1));
                let dir = tempfile::tempdir().unwrap();
                let path = dir.path().join("modifiers.osk");
                project.save(&path).unwrap();
                let loaded = Project::load(&path).unwrap();
                assert_eq!(loaded.bodies, project.bodies);
                if shape == Shape::Circle {
                    for format in crate::export::Format::ALL {
                        let output = dir.path().join(format!("cylinder.{}", format.extension()));
                        crate::export::save(&loaded.bodies, format, &output).unwrap();
                        assert!(std::fs::metadata(output).unwrap().len() > 100);
                    }
                }
                let mut legacy = serde_json::to_value(&project).unwrap();
                legacy["schema_version"] = 4.into();
                legacy["modeling_version"] = 4.into();
                std::fs::write(&path, serde_json::to_vec(&legacy).unwrap()).unwrap();
                assert!(
                    Project::load(&path).is_err(),
                    "old versions must reject nonempty modifiers"
                );
                legacy["bodies"][0]
                    .as_object_mut()
                    .unwrap()
                    .remove("edge_treatments");
                std::fs::write(&path, serde_json::to_vec(&legacy).unwrap()).unwrap();
                let migrated = Project::load(&path).unwrap();
                assert!(migrated.bodies[0].edge_treatments.is_empty());
                assert!((volume(&evaluate(&migrated.bodies[0]).unwrap()) - original).abs() < 1e-8);
                let mut model = Model::from_bodies(loaded.bodies);
                let committed = model.bodies[0].clone();
                let mut invalid = committed.clone();
                invalid.edge_treatments[0].amount = 100.;
                assert!(model.commit(invalid).is_err());
                assert_eq!(model.bodies[0], committed);
                let mut removed = committed.clone();
                removed.edge_treatments.clear();
                model.commit(removed).unwrap();
                assert!((volume(&evaluate(&model.bodies[0]).unwrap()) - original).abs() < 1e-8);
                model.undo();
                assert_eq!(model.bodies[0], committed);
                model.redo();
                assert!(model.bodies[0].edge_treatments.is_empty());
            }
        }
        let mut b = Body::new(Shape::Rectangle);
        b.id = 1;
        b.width = 4.;
        b.depth = 5.;
        b.extrusion = 3.;
        b.position = [1., 2., 3.];
        b.rotation = [15., 30., 45.];
        b.scale = [1., 2., 0.5];
        for face in [
            FaceTarget::Top,
            FaceTarget::Bottom,
            FaceTarget::Left,
            FaceTarget::Right,
            FaceTarget::Front,
            FaceTarget::Back,
        ] {
            b.edge_treatments
                .push(Modifier::new(Kind::Chamfer, face, &b));
            let g = evaluate(&b).unwrap_or_else(|e| panic!("box face {face:?}: {e}"));
            assert!(volume(&g) > 0.);
        }
        let mut cutter = Body::new(Shape::Circle);
        cutter.id = 2;
        cutter.cuts_profiles = true;
        cutter.position = b.position;
        cutter.rotation = b.rotation;
        cutter.scale = b.scale;
        assert!(evaluate_scene(&[b, cutter]).is_err());
    }
}
