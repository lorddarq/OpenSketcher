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
    if !(4..=64).contains(&planes.len()) {
        return Err("Heal requires 4–64 remaining convex planar supports.".into());
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
    for (group, (n, d)) in planes {
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
            assert!(
                selected_operations(
                    &changes,
                    &[(1, geometry)],
                    &[(1, Some(transition))],
                    SelectionMode::Edge,
                    Kind::Chamfer
                )
                .is_err()
            );
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
