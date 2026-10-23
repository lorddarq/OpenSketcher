//! Solid operators act on kernel boundaries, never on viewport triangles.
use crate::edge_treatments::FaceTarget;
use crate::model::{Body, Geometry, Shape, tessellate};
use serde::{Deserialize, Serialize};
use truck_modeling::{
    EuclideanSpace, InnerSpace, Matrix4, Point3, Shell, Solid, SquareMatrix, Transform, Vector3,
};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BoreMode {
    Generate,
    Borer,
    GeneratorAndBorer,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Extent {
    #[default]
    Distance,
    ThroughAll,
    UpToNext {
        target: u64,
    },
    UpToFace {
        target: u64,
        face: FaceTarget,
    },
}
impl Extent {
    pub fn references(self, attached: u64) -> String {
        match self {
            Self::Distance => "Distance".into(),
            Self::ThroughAll => format!("Through all · attached body {attached}"),
            Self::UpToNext { target } => format!("Up to next · end body {target}"),
            Self::UpToFace { target, face } => {
                format!("Up to face · end body {target} / {}", face.label())
            }
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Distance => "Distance",
            Self::ThroughAll => "Through all",
            Self::UpToNext { .. } => "Up to next",
            Self::UpToFace { .. } => "Up to face",
        }
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Bore {
    pub target: u64,
    pub face: FaceTarget,
    /// Initial orthonormal support frame, used to preserve profile attachment.
    pub reference_frame: [[f64; 4]; 4],
    pub travel: f64,
    pub mode: BoreMode,
    #[serde(default)]
    pub extent: Extent,
    #[serde(default)]
    pub through_targets: Vec<u64>,
}
impl BoreMode {
    pub fn label(self) -> &'static str {
        match self {
            Self::Generate => "New body",
            Self::Borer => "Borer only",
            Self::GeneratorAndBorer => "Generator + borer",
        }
    }
}
impl Bore {
    pub fn new(target: &Body, face: FaceTarget) -> Result<Self, String> {
        Ok(Self {
            target: target.id,
            face,
            reference_frame: support_frame(target, face)?.into(),
            travel: 0.,
            mode: BoreMode::Generate,
            extent: Extent::Distance,
            through_targets: Vec::new(),
        })
    }
    pub fn validate(&self, profile: &Body) -> Result<(), String> {
        let frame = Matrix4::from(self.reference_frame);
        let mut targets = std::collections::HashSet::new();
        if self.through_targets.len() > 64
            || (!self.through_targets.is_empty() && self.extent != Extent::ThroughAll)
            || self.through_targets.iter().any(|id| {
                *id == 0
                    || *id == self.target
                    || (profile.id != 0 && *id >= profile.id)
                    || !targets.insert(*id)
            })
        {
            return Err("Through-all scope requires up to 64 distinct earlier target IDs, excluding the attached body, and the Through all extent.".into());
        }

        if self.target == 0 || self.target >= profile.id && profile.id != 0 {
            return Err("A bore must reference an earlier target body.".into());
        }
        if let Extent::UpToNext { target } | Extent::UpToFace { target, .. } = self.extent
            && (target == 0 || profile.id != 0 && target >= profile.id)
        {
            return Err(
                "Extent must reference an earlier body; cyclic/forward references are not allowed."
                    .into(),
            );
        }
        if !self.travel.is_finite() || self.travel.abs() > 10000. {
            return Err("Bore travel must be finite and within ±10,000 mm.".into());
        }
        if profile.shape == Shape::Line
            || profile.extrusion != 0.
            || crate::components::has_source_edits(profile)
            || !profile.edge_treatments.is_empty()
            || profile.cuts_profiles
        {
            return Err("Boring requires an independent closed profile without component edits or edge treatments.".into());
        }
        let axes = [frame.x.truncate(), frame.y.truncate(), frame.z.truncate()];
        if self
            .reference_frame
            .iter()
            .flatten()
            .any(|v| !v.is_finite() || v.abs() > 1e6)
            || [frame.x.w, frame.y.w, frame.z.w]
                .iter()
                .any(|v| v.abs() > 1e-10)
            || (frame.w.w - 1.).abs() > 1e-10
            || axes.iter().any(|v| (v.magnitude2() - 1.).abs() > 1e-8)
            || axes[0].cross(axes[1]).dot(axes[2]) < 1. - 1e-8
        {
            return Err("Invalid bore support frame.".into());
        }
        Ok(())
    }
}

pub fn support_frame(body: &Body, face: FaceTarget) -> Result<Matrix4, String> {
    if body.bore.is_some()
        || body.extrusion <= 0.
        || body.shape == Shape::Line
        || !face.supported(body)
        || crate::components::has_source_edits(body)
    {
        return Err(
            "Bore support currently requires an original planar face of an undeformed solid."
                .into(),
        );
    }
    let placement = crate::regions::transform(body);
    let (n, d) = face.evaluated_plane(body);
    let normal = placement.transform_vector(n).normalize();
    let u = placement
        .transform_vector(if n.z.abs() > 0.5 {
            Vector3::unit_x()
        } else {
            Vector3::unit_z()
        })
        .normalize();
    let v = normal.cross(u);
    let origin = placement.transform_point(Point3::from_vec(n * d));
    Ok(Matrix4::from_cols(
        u.extend(0.),
        v.extend(0.),
        normal.extend(0.),
        origin.to_homogeneous(),
    ))
}

pub fn attachment_delta(target: &Body, bore: &Bore) -> Result<Matrix4, String> {
    Ok(support_frame(target, bore.face)?
        * Matrix4::from(bore.reference_frame)
            .invert()
            .ok_or("Invalid bore reference frame.")?)
}
pub fn resolve_profile(profile: &Body, target: &Body, bore: &Bore) -> Result<Body, String> {
    let frame = support_frame(target, bore.face)?;
    let delta = attachment_delta(target, bore)?;
    let mut source = profile.clone();
    source.bore = None;
    let resolved = crate::components::transformed(
        &source,
        &Geometry::default(),
        crate::selection::SelectionMode::Volume,
        None,
        delta,
    )?;
    let normal = crate::regions::transform(&resolved)
        .transform_vector(Vector3::unit_z())
        .normalize();
    if normal.dot(frame.z.truncate()) < 1. - 1e-7
        || (Point3::from(resolved.position) - Point3::from_vec(frame.w.truncate()))
            .dot(normal)
            .abs()
            > 1e-6
    {
        return Err("Bore profile must remain on its referenced support plane, with the same outward normal.".into());
    }
    Ok(resolved)
}

fn swept(profile: &Body, start: f64, end: f64) -> Result<Geometry, String> {
    if (end - start).abs() < 1e-8 {
        return Ok(Geometry::default());
    }
    crate::model::build(&sweep_body(profile, start, end), None)
}
fn sweep_body(profile: &Body, start: f64, end: f64) -> Body {
    let mut body = profile.clone();
    body.bore = None;
    let normal = crate::regions::transform(profile)
        .transform_vector(Vector3::unit_z())
        .normalize();
    body.position = (Point3::from(body.position) + normal * start.min(end)).into();
    body.extrusion = (end - start).abs() / profile.scale[2];
    // Generated extent has its own checked world-unit limit; primitive UI bounds
    // do not limit a derived sweep to 100 local units.
    body
}

fn target_thickness(target: &Body, face: FaceTarget) -> f64 {
    let (n, _) = face.plane(target);
    let local = if n.z.abs() > 0.5 {
        target.extrusion
    } else if n.x.abs() > 0.5 {
        target.width
    } else {
        target.depth
    };
    let local = local + face.evaluated_plane(target).1 - face.plane(target).1
        + face.opposite().evaluated_plane(target).1
        - face.opposite().plane(target).1;
    crate::regions::transform(target)
        .transform_vector(n * local)
        .magnitude()
}
fn limit_face(
    target: u64,
    face: FaceTarget,
    bodies: &[Body],
    scene: &[(u64, Geometry)],
) -> Result<crate::snapping::Plane, String> {
    let body = bodies
        .iter()
        .find(|b| b.id == target)
        .ok_or_else(|| format!("Extent target body {target} no longer exists."))?;
    let g = &scene
        .iter()
        .find(|g| g.0 == target)
        .ok_or_else(|| format!("Extent target body {target} has no geometry."))?
        .1;
    if !(0..g.face_count).any(|f| crate::edge_treatments::target(body, g, f) == Ok(face)) {
        return Err(format!(
            "Extent target body {target} / {} no longer exists.",
            face.label()
        ));
    }
    let frame = support_frame(body, face)?;
    Ok(crate::snapping::Plane {
        origin: Point3::from_vec(frame.w.truncate()),
        u: frame.x.truncate(),
        v: frame.y.truncate(),
        normal: frame.z.truncate(),
    })
}
fn plane_depths(
    profile: &Body,
    plane: crate::snapping::Plane,
    direction: f64,
) -> Result<[f64; 2], String> {
    let transform = crate::regions::transform(profile);
    let axis = transform.transform_vector(Vector3::unit_z()).normalize();
    let denominator = axis.dot(plane.normal);
    if denominator.abs() < 1e-8 {
        return Err("Sweep direction is parallel to the end plane.".into());
    }
    let center = Point3::from(profile.position).to_vec().dot(plane.normal);
    let a = profile.width / 2. * transform.x.truncate().dot(plane.normal);
    let b = if profile.shape == Shape::Rectangle {
        profile.depth / 2.
    } else {
        profile.width / 2.
    } * transform.y.truncate().dot(plane.normal);
    let [low, high] = match profile.shape {
        Shape::Rectangle => [-a.abs() - b.abs(), a.abs() + b.abs()],
        Shape::Circle => [-a.hypot(b), a.hypot(b)],
        Shape::Semicircle => {
            let mut values = vec![a, -a];
            let angle = b.atan2(a).rem_euclid(std::f64::consts::TAU);
            for t in [
                angle,
                (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU),
            ] {
                if t <= std::f64::consts::PI {
                    values.push(a * t.cos() + b * t.sin());
                }
            }
            [
                values.iter().copied().fold(f64::INFINITY, f64::min),
                values.into_iter().fold(f64::NEG_INFINITY, f64::max),
            ]
        }
        Shape::Line => return Err("Termination requires a closed profile.".into()),
    };
    let d = plane.origin.to_vec().dot(plane.normal);
    let values = [
        (d - center - low) / denominator * direction,
        (d - center - high) / denominator * direction,
    ];
    let bounds = [values[0].min(values[1]), values[0].max(values[1])];
    Ok(bounds)
}
pub(crate) fn plane_travel(
    profile: &Body,
    plane: crate::snapping::Plane,
    direction: f64,
) -> Result<[f64; 2], String> {
    let bounds = plane_depths(profile, plane, direction)?;
    if bounds.iter().any(|v| !v.is_finite()) || bounds[0] <= 1e-7 || bounds[1] > 10000. {
        return Err("The whole profile must reach the end plane ahead in the chosen direction, within 10,000 mm.".into());
    }
    Ok(bounds)
}

pub(crate) fn terminated(
    profile: &Body,
    start: f64,
    plane: crate::snapping::Plane,
    direction: f64,
) -> Result<Geometry, String> {
    use truck_modeling::builder;
    let placement = crate::regions::transform(profile);
    let normal = placement.transform_vector(Vector3::unit_z()).normalize();
    let project = plane
        .projection_along(normal)
        .ok_or("Sweep direction is parallel to the end plane.")?;
    let mut entry = builder::transformed(&crate::model::profile_wire(profile), placement);
    if direction < 0. {
        entry = entry.inverse();
    }
    entry = builder::transformed(&entry, Matrix4::from_translation(normal * start));
    let end = builder::transformed(&entry, project);
    let mut shell = builder::try_wire_homotopy(&entry, &end)
        .map_err(|e| format!("Cannot terminate the swept boundary: {e}"))?;
    let side_count = shell.len();
    for wire in shell.extract_boundaries() {
        shell.push(builder::try_attach_plane(&[wire.inverse()]).map_err(|e| e.to_string())?);
    }
    Solid::try_new(shell.connected_components())
        .map_err(|e| format!("Terminated sweep is not closed: {e}"))?;
    let mut g = tessellate(&shell)?;
    if profile.shape == Shape::Circle {
        let roles = (0..g.face_count)
            .map(|i| {
                if i < side_count {
                    0
                } else {
                    i - side_count + 1
                }
            })
            .collect::<Vec<_>>();
        crate::edge_treatments::merge_surfaces(&mut g, &roles);
        let center = Point3::from(profile.position) + normal * start;
        let radius = profile.width / 2.;
        let u = placement.transform_vector(Vector3::unit_x() * radius);
        let v = placement.transform_vector(Vector3::unit_y() * radius);
        g.circles.push(crate::model::CircularEdge { center, u, v });
        g.circles.push(crate::model::CircularEdge {
            center: project.transform_point(center),
            u: project.transform_vector(u),
            v: project.transform_vector(v),
        });
    }
    g.is_solid = true;
    Ok(g)
}

fn source_cap(target: &Body, face: FaceTarget) -> bool {
    matches!(face, FaceTarget::Top | FaceTarget::Bottom)
        && target.edge_treatments.is_empty()
        && target.hollow.is_none()
        && target.taper.is_none()
        && target.face_extrusions.is_empty()
}
fn terminated_pocket(
    target: &Body,
    base: &Geometry,
    profile: &Body,
    face: FaceTarget,
    plane: crate::snapping::Plane,
) -> Result<Option<Geometry>, String> {
    if !source_cap(target, face) || !profile_fits(target, profile)? {
        return Ok(None);
    }
    let shell = base
        .shell
        .as_ref()
        .ok_or("Pocket requires a retained boundary.")?;
    let role = (0..base.face_count)
        .find(|f| crate::edge_treatments::target(target, base, *f) == Ok(face))
        .ok_or("Pocket support face no longer exists.")?;
    let mut roles = base.surface_mapping()?.to_vec();
    let indices = roles
        .iter()
        .enumerate()
        .filter(|(_, f)| **f == role)
        .map(|(i, _)| i)
        .collect::<Vec<_>>();
    if indices.len() != 1 {
        return Ok(None);
    }
    let tool = terminated(profile, 0., plane, -1.)?;
    let inner = tool.shell.as_ref().unwrap();
    let normal = crate::regions::transform(profile)
        .transform_vector(Vector3::unit_z())
        .normalize();
    let origin = Point3::from(profile.position);
    let entry = inner
        .iter()
        .position(|f| {
            f.edge_iter()
                .all(|e| (e.front().point() - origin).dot(normal).abs() < 1e-7)
        })
        .ok_or("Terminated pocket entry is missing.")?;
    let mut wires = shell[indices[0]].boundaries();
    wires.extend(inner[entry].boundaries().iter().map(|w| w.inverse()));
    let ring = truck_modeling::builder::try_attach_plane(&wires).map_err(|e| e.to_string())?;
    let mut joined = shell.clone();
    joined.remove(indices[0]);
    roles.remove(indices[0]);
    joined.push(ring);
    roles.push(role);
    let floor = roles.iter().copied().max().unwrap_or(0) + 1;
    for (index, f) in inner.iter().enumerate() {
        if index == entry {
            continue;
        }
        joined.push(f.inverse());
        let end = f
            .edge_iter()
            .all(|e| (e.front().point() - plane.origin).dot(plane.normal).abs() < 1e-7);
        roles.push(if end {
            floor
        } else if profile.shape == Shape::Circle {
            floor + 1
        } else {
            floor + 1 + index
        });
    }
    Solid::try_new(joined.connected_components())
        .map_err(|e| format!("Invalid terminated pocket boundary: {e}"))?;
    let mut g = tessellate(&joined)?;
    crate::edge_treatments::merge_surfaces(&mut g, &roles);
    g.is_solid = true;
    g.circles = base.circles.clone();
    g.circles.extend(tool.circles);
    Ok(Some(g))
}

/// Resolve termination from current dependencies; manual travel remains an authored value.
#[cfg(test)]
pub fn resolved_travel(
    profile: &Body,
    bore: &Bore,
    bodies: &[Body],
    scene: &[(u64, Geometry)],
) -> Result<f64, String> {
    resolved_extent(profile, bore, bodies, scene).map(|(travel, _)| travel)
}
fn resolved_extent(
    profile: &Body,
    bore: &Bore,
    bodies: &[Body],
    scene: &[(u64, Geometry)],
) -> Result<(f64, Option<crate::snapping::Plane>), String> {
    if bore.extent == Extent::Distance {
        return Ok((bore.travel, None));
    }
    let direction = if bore.travel == 0. {
        if bore.mode == BoreMode::Generate {
            1.
        } else {
            -1.
        }
    } else {
        bore.travel.signum()
    };
    let normal = crate::regions::transform(profile)
        .transform_vector(Vector3::unit_z())
        .normalize();
    let origin = Point3::from(profile.position);
    let lookup = |id| {
        bodies
            .iter()
            .find(|b| b.id == id)
            .ok_or_else(|| format!("Extent target body {id} no longer exists."))
    };
    let geometry = |id| {
        scene
            .iter()
            .find(|g| g.0 == id)
            .map(|g| &g.1)
            .ok_or_else(|| format!("Extent target body {id} has no geometry."))
    };
    let mut end = None;
    let travel = match bore.extent {
        Extent::Distance => unreachable!(),
        Extent::ThroughAll => {
            if direction > 0. {
                return Err(format!(
                    "Attached target body {} lies inward of its support; reverse the extent direction.",
                    bore.target
                ));
            }
            if bore.through_targets.is_empty() {
                -target_thickness(lookup(bore.target)?, bore.face)
            } else {
                let mut farthest = 0_f64;
                for id in std::iter::once(&bore.target).chain(&bore.through_targets) {
                    let body = lookup(*id)?;
                    let g = geometry(*id)?;
                    if body.construction
                        || body.boolean.is_some()
                        || !g.is_solid
                        || g.triangles.is_empty()
                    {
                        return Err(format!(
                            "Through-all body {id} must be an earlier evaluated standalone solid."
                        ));
                    }
                    let bounds = g
                        .projection_range(normal)
                        .map_err(|e| format!("Through-all body {id}: {e}"))?;
                    let inward = origin.to_vec().dot(normal) - bounds[0];
                    if inward <= 1e-7 {
                        return Err(format!(
                            "Through-all body {id} does not lie inward of the profile."
                        ));
                    }
                    farthest = farthest.max(inward);
                }
                -farthest
            }
        }
        Extent::UpToFace { target, face } => {
            let plane = limit_face(target, face, bodies, scene)?;
            if plane.normal.cross(normal).magnitude2() > 1e-16 {
                end = Some(plane);
                direction * plane_travel(profile, plane, direction)?[1]
            } else {
                (plane.origin - origin).dot(normal)
            }
        }
        Extent::UpToNext { target } => {
            let body = lookup(target)?;
            let g = geometry(target)?;
            let pristine = body.bore.is_none()
                && body.boolean.is_none()
                && body.component_edits.is_empty()
                && body.edge_treatments.is_empty()
                && body.hollow.is_none()
                && body.taper.is_none()
                && body.face_extrusions.is_empty()
                && !bodies.iter().any(|b| {
                    b.id < profile.id
                        && b.bore.as_ref().is_some_and(|op| {
                            (op.target == target || op.through_targets.contains(&target))
                                && op.mode != BoreMode::Generate
                        })
                })
                && crate::regions::boundaries(body, bodies)?.is_none();
            let mut candidates = Vec::<([f64; 2], crate::snapping::Plane, Vec<usize>)>::new();
            for face in 0..g.face_count {
                let Some(plane) = crate::snapping::Plane::face(g, face) else {
                    continue;
                };
                if plane.normal.dot(normal).abs() < 1e-8 {
                    continue;
                }
                let depths = plane_depths(profile, plane, direction)?;
                if depths[1] <= 1e-7 {
                    continue;
                }
                if let Some((_, _, faces)) = candidates.iter_mut().find(|(_, p, _)| {
                    p.normal.dot(plane.normal).abs() > 1. - 1e-8
                        && (p.origin - plane.origin).dot(plane.normal).abs() < 1e-7
                }) {
                    faces.push(face);
                } else {
                    candidates.push((depths, plane, vec![face]));
                }
            }
            candidates.sort_by(|a, b| a.0[0].total_cmp(&b.0[0]));
            let mut found = None;
            let mut partial = false;
            for (depths, plane, faces) in candidates {
                let parallel = plane.normal.cross(normal).magnitude2() <= 1e-16;
                let cap = faces.iter().any(|f| {
                    matches!(
                        crate::edge_treatments::target(body, g, *f),
                        Ok(FaceTarget::Top | FaceTarget::Bottom)
                    )
                });
                let (covered, overlap) =
                    if pristine && parallel && cap && profile_fits(body, profile)? {
                        (true, true)
                    } else {
                        crate::booleans::profile_coverage(profile, g, &faces, plane)?
                    };
                if !covered {
                    partial |= overlap;
                    continue;
                }
                if depths[0] <= 1e-7 || depths[1] > 10000. {
                    return Err(format!(
                        "Next faces on body {target} must lie ahead of the whole profile within 10,000 mm."
                    ));
                }
                // Another finite surface may be nearer for only part of the
                // profile even when its infinite plane has a farther minimum.
                // Clip actual boundaries between the start and proposed stop.
                let ray = normal * direction;
                let sign = ray.dot(plane.normal).signum();
                let project = plane.projection_along(normal).unwrap();
                let mut nearer = Geometry::default();
                for (triangle, face) in g.triangles.iter().zip(&g.triangle_faces) {
                    if faces.contains(face) {
                        continue;
                    }
                    let points = crate::edge_treatments::clip_polygon(
                        triangle,
                        -ray,
                        -origin.to_vec().dot(ray) - 1e-7,
                    )
                    .0;
                    let points = crate::edge_treatments::clip_polygon(
                        &points,
                        plane.normal * sign,
                        plane.origin.to_vec().dot(plane.normal) * sign - 1e-7,
                    )
                    .0;
                    for i in 1..points.len().saturating_sub(1) {
                        let triangle = [points[0], points[i], points[i + 1]]
                            .map(|p| project.transform_point(p));
                        if (triangle[1] - triangle[0])
                            .cross(triangle[2] - triangle[0])
                            .magnitude2()
                            > 1e-20
                        {
                            nearer.triangles.push(triangle);
                            nearer.triangle_faces.push(0);
                        }
                    }
                }
                if !nearer.triangles.is_empty()
                    && crate::booleans::profile_coverage(profile, &nearer, &[0], plane)?.1
                {
                    return Err(format!(
                        "Next faces on body {target} only partially terminate the profile on one plane; nearer surfaces require varying/curved termination."
                    ));
                }
                if !parallel {
                    end = Some(plane);
                }
                found = Some(direction * depths[1]);
                break;
            }
            found.ok_or_else(||if partial {
                format!("Next faces on body {target} only partially terminate the profile; select a complete end face or split the profile.")
            } else {format!("Extent target body {target} has no next planar face fully covering the profile in this direction.")})?
        }
    };
    if !travel.is_finite() || travel * direction <= 1e-7 || travel.abs() > 10000. {
        return Err("Extent must terminate ahead of the profile, within 10,000 mm; check its target and direction.".into());
    }
    Ok((travel, end))
}

pub fn apply_bores(bodies: &[Body], scene: &mut [(u64, Geometry)]) -> Result<(), String> {
    let mut modified_targets = std::collections::HashSet::new();
    let mut profiles: Vec<_> = bodies.iter().filter(|b| b.bore.is_some()).collect();
    profiles.sort_by_key(|b| b.id);
    for profile in profiles {
        let bore = profile.bore.as_ref().unwrap();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| { (|| {
            let target = bodies
                .iter()
                .find(|b| b.id == bore.target)
                .ok_or("Referenced target body no longer exists.")?;
            let target_index = scene
                .iter()
                .position(|(id, _)| *id == target.id)
                .ok_or("Referenced target geometry is missing.")?;
            let target_geometry = &scene[target_index].1;
            if !(0..target_geometry.face_count).any(|f| {
                crate::edge_treatments::target(target, target_geometry, f) == Ok(bore.face)
            }) {
                return Err("Referenced support face no longer exists.".into());
            }
            let resolved = resolve_profile(profile, target, bore)?;
            let thickness=target_thickness(target,bore.face);
            let (travel,end)=resolved_extent(&resolved,bore,bodies,scene).map_err(|e|format!("{}{}: {e}",bore.extent.references(bore.target),if bore.through_targets.is_empty(){String::new()}else{format!(" · scoped bodies {:?}",bore.through_targets)}))?;
            let output = if travel.abs() < 1e-8 {
                crate::model::build(&resolved, None)?
            } else if bore.mode == BoreMode::Generate {
                if let Some(plane)=end {terminated(&resolved,0.,plane,travel.signum())?}else{swept(&resolved, 0., travel)?}
            } else {
                if travel < 0. {
                    let margin = (thickness * 0.02).max(0.02);
                    let overrun = if -travel >= thickness - 1e-7 {
                        margin
                    } else {
                        0.
                    };
                    let tool=if let Some(plane)=end {terminated(&resolved,margin,plane,-1.)?.shell.unwrap()}else {
                        crate::model::primitive_shell(&sweep_body(&resolved,(if bore.through_targets.is_empty(){travel.max(-thickness)}else{travel})-overrun,margin))?
                    };
                    let prismatic = if !modified_targets.contains(&target.id)&&crate::regions::boundaries(target,bodies)?.is_none() {
                        if let Some(plane)=end {
                            let depths=plane_travel(&resolved,plane,-1.)?;
                            if depths[0]>=thickness-1e-7 {contained_profile(target,target_geometry,&resolved,bore.face,thickness)?}
                            else if depths[1]<thickness-1e-7 {terminated_pocket(target,target_geometry,&resolved,bore.face,plane)?}
                            else {None}
                        }else{contained_profile(target,target_geometry,&resolved,bore.face,-travel)?}
                    }else{None};
                    scene[target_index].1 = if let Some(g) = prismatic {
                        g
                    } else {
                        difference(target_geometry, &tool)?
                    };
                    scene[target_index].1.is_solid = true;
                    modified_targets.insert(target.id);
                    for id in &bore.through_targets {
                        let index=scene.iter().position(|(owner,_)|owner==id).ok_or_else(||format!("Through-all body {id} no longer exists."))?;
                        let source=bodies.iter().find(|b|b.id==*id).unwrap();
                        let mut direct=None;
                        let normal=crate::regions::transform(&resolved).transform_vector(Vector3::unit_z()).normalize();
                        if !modified_targets.contains(id)&&crate::regions::boundaries(source,bodies)?.is_none()
                            && scene[index].1.projection_range(normal)?[1]<=Point3::from(resolved.position).to_vec().dot(normal)+1e-7 {
                            for face in [FaceTarget::Top,FaceTarget::Bottom] {
                                if support_frame(source,face).is_ok_and(|frame|frame.z.truncate().dot(normal)>1.-1e-7) {
                                    direct=contained_profile(source,&scene[index].1,&resolved,face,target_thickness(source,face))?;
                                    break;
                                }
                            }
                        }
                        scene[index].1=if let Some(g)=direct {g}else{difference(&scene[index].1,&tool).map_err(|e|format!("Through-all body {id}: {e}"))?};
                        scene[index].1.is_solid=true;modified_targets.insert(*id);
                    }
                }
                if bore.mode == BoreMode::GeneratorAndBorer {
                    if let Some(plane)=end {
                        if travel>0. {terminated(&resolved,0.,plane,1.)?}
                        else {
                            let depths=plane_travel(&resolved,plane,-1.)?;
                            if depths[1]<=thickness+1e-7 {Geometry::default()}
                            else if depths[0]>thickness+1e-7 {terminated(&resolved,-thickness,plane,-1.)?}
                            else {
                                let full=terminated(&resolved,0.,plane,-1.)?;
                                let before=crate::model::primitive_shell(&sweep_body(&resolved,-thickness,0.02))?;
                                difference(&full,&before)?
                            }
                        }
                    }else if !bore.through_targets.is_empty() {Geometry::default()}
                    else if travel > 0. {
                        swept(&resolved, 0., travel)?
                    } else if -travel > thickness + 1e-7 {
                        swept(&resolved, travel, -thickness)?
                    } else {
                        Geometry::default()
                    }
                } else {
                    Geometry::default()
                }
            };
            // Profile ID owns the emerging body; it remains distinct from target.
            let index = scene
                .iter()
                .position(|(id, _)| *id == profile.id)
                .ok_or("Bore feature is missing from scene.")?;
            scene[index].1 = output;
            Ok(())
        })() })).unwrap_or_else(|_|Err("The geometry kernel could not evaluate this operation; the committed model was preserved.".into()));
        result.map_err(|e: String| {
            format!(
                "{} · {} · target body {} / {}: {e}",
                profile.label(),
                bore.mode.label(),
                bore.target,
                bore.face.label()
            )
        })?;
    }
    Ok(())
}

/// Conservative whole-region containment shared by pocket construction and extent resolution.
fn profile_fits(target: &Body, profile: &Body) -> Result<bool, String> {
    let relative = crate::regions::transform(target)
        .invert()
        .ok_or("Invalid target transform.")?
        * crate::regions::transform(profile);
    // A sampled outline plus a display-sized clearance rejects thin pipe walls
    // and needlessly sends them back to CSG. Bound circular profiles analytically.
    let analytic = if target.shape == Shape::Circle && profile.shape == Shape::Circle {
        let (x, y) = (relative.x.truncate(), relative.y.truncate());
        let (a, b, c) = (
            x.x * x.x + x.y * x.y,
            x.x * y.x + x.y * y.y,
            y.x * y.x + y.y * y.y,
        );
        let radius =
            profile.width * 0.5 * ((a + c + ((a - c) * (a - c) + 4. * b * b).sqrt()) * 0.5).sqrt();
        Some(
            relative.w.x.hypot(relative.w.y) + radius
                < target.width * 0.5 - 2. * truck_modeling::TOLERANCE,
        )
    } else if target.shape == Shape::Rectangle && profile.shape == Shape::Circle {
        let r = profile.width * 0.5;
        Some(
            relative.w.x.abs() + r * relative.x.x.hypot(relative.y.x)
                < target.width * 0.5 - 2. * truck_modeling::TOLERANCE
                && relative.w.y.abs() + r * relative.x.y.hypot(relative.y.y)
                    < target.depth * 0.5 - 2. * truck_modeling::TOLERANCE,
        )
    } else {
        None
    };
    if analytic == Some(false) {
        return Ok(false);
    }
    if analytic.is_none() {
        let outer = crate::regions::profile(target);
        let margin = 0.01
            * relative
                .x
                .truncate()
                .magnitude()
                .max(relative.y.truncate().magnitude())
                .max(1.);
        for p in crate::regions::profile(profile) {
            let p = relative.transform_point(Point3::new(p[0], p[1], 0.));
            if !crate::regions::contains([p.x, p.y], &outer) {
                return Ok(false);
            }
            for i in 0..outer.len() {
                let a = outer[i];
                let b = outer[(i + 1) % outer.len()];
                let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                let t = (((p.x - a[0]) * dx + (p.y - a[1]) * dy) / (dx * dx + dy * dy).max(1e-20))
                    .clamp(0., 1.);
                if (p.x - a[0] - t * dx).hypot(p.y - a[1] - t * dy) < margin {
                    return Ok(false);
                }
            }
        }
    }
    Ok(true)
}

/// Contained blind and through bores have known boundaries. Construct them
/// directly instead of sending concentric analytic surfaces through 3D CSG.
fn contained_profile(
    target: &Body,
    base: &Geometry,
    profile: &Body,
    face: FaceTarget,
    depth: f64,
) -> Result<Option<Geometry>, String> {
    if !source_cap(target, face) {
        return Ok(None);
    }
    let inverse = crate::regions::transform(target)
        .invert()
        .ok_or("Invalid target transform.")?;
    let relative = inverse * crate::regions::transform(profile);
    if !profile_fits(target, profile)? {
        return Ok(None);
    }
    let z = relative.transform_point(Point3::new(0., 0., 0.)).z;
    let placement = Matrix4::from_translation(Vector3::new(0., 0., -z)) * relative;
    let inner =
        truck_modeling::builder::transformed(&crate::model::profile_wire(profile), placement);
    let inner = if relative.z.z > 0. {
        inner.inverse()
    } else {
        inner
    };
    let depth = (depth
        / crate::regions::transform(target)
            .transform_vector(Vector3::unit_z())
            .magnitude())
    .min(target.extrusion);
    let shell = pocket_shell(
        crate::model::profile_wire(target),
        inner,
        target.extrusion,
        face,
        depth,
    )?;
    let shell = truck_modeling::builder::transformed(&shell, crate::regions::transform(target));
    let mut g = tessellate(&shell)?;
    g.circles = base.circles.clone();
    if profile.shape == Shape::Circle {
        let frame = support_frame(target, face)?;
        let normal = frame.z.truncate();
        let center = Point3::from(profile.position);
        let entry = center + normal * (Point3::from_vec(frame.w.truncate()) - center).dot(normal);
        let vector =
            crate::regions::transform(target).transform_vector(face.plane(target).0 * depth);
        let placement = crate::regions::transform(profile);
        let radius = profile.width / 2.;
        for center in [entry, entry - vector] {
            g.circles.push(crate::model::CircularEdge {
                center,
                u: placement.transform_vector(Vector3::unit_x() * radius),
                v: placement.transform_vector(Vector3::unit_y() * radius),
            });
        }
    }
    g.is_solid = true;
    Ok(Some(g))
}
/// Close shared outer/inner wires into an exact blind pocket or through opening.
/// The inner wire is clockwise in the XY plane.
pub(crate) fn pocket_shell(
    outer: truck_modeling::Wire,
    inner: truck_modeling::Wire,
    height: f64,
    face: FaceTarget,
    depth: f64,
) -> Result<Shell, String> {
    if depth < height - 1e-7 {
        let mut shell: Shell = truck_modeling::builder::tsweep(&outer, Vector3::unit_z() * height);
        let start = if face == FaceTarget::Top {
            height - depth
        } else {
            0.
        };
        let inner = truck_modeling::builder::transformed(
            &inner,
            Matrix4::from_translation(Vector3::unit_z() * start),
        );
        let walls: Shell = truck_modeling::builder::tsweep(&inner, Vector3::unit_z() * depth);
        shell.extend(walls);
        let mut planes: Vec<(f64, Vec<truck_modeling::Wire>)> = Vec::new();
        for wire in shell.extract_boundaries() {
            let z = wire[0].front().point().z;
            if let Some((_, wires)) = planes
                .iter_mut()
                .find(|(height, _)| (*height - z).abs() < 1e-8)
            {
                wires.push(wire.inverse());
            } else {
                planes.push((z, vec![wire.inverse()]));
            }
        }
        for (_, wires) in planes {
            shell.push(
                truck_modeling::builder::try_attach_plane(&wires).map_err(|e| e.to_string())?,
            );
        }
        Solid::try_new(vec![shell.clone()]).map_err(|e| format!("Invalid pocket boundary: {e}"))?;
        Ok(shell)
    } else {
        let face = truck_modeling::builder::try_attach_plane(&[outer, inner])
            .map_err(|e| e.to_string())?;
        crate::model::extrude_face(&face, height)
    }
}

pub(crate) fn solid(g: &Geometry) -> Result<Solid, String> {
    let shell = g
        .shell
        .clone()
        .ok_or("This operation requires a retained kernel boundary.")?;
    Solid::try_new(shell.connected_components()).map_err(|e| format!("Invalid solid boundary: {e}"))
}

pub(crate) fn geometry(solid: &Solid) -> Result<Geometry, String> {
    // Kernel connected-components can return faces in hash order. Canonicalize
    // boundaries so stored component slots identify the same geometry on replay.
    let point_key = |p: Point3| [p.x, p.y, p.z].map(|v| (v * 1e7).round() as i64);
    let mut faces: Vec<_> = solid
        .boundaries()
        .iter()
        .flat_map(|s| s.iter())
        .map(|face| {
            let boundaries = face
                .boundaries()
                .into_iter()
                .map(|wire| {
                    let mut edges: Vec<_> = wire.into_iter().collect();
                    if let Some(index) = (0..edges.len()).min_by_key(|i| {
                        (
                            point_key(edges[*i].front().point()),
                            point_key(edges[*i].back().point()),
                        )
                    }) {
                        edges.rotate_left(index);
                    }
                    edges.into_iter().collect::<truck_modeling::Wire>()
                })
                .collect();
            truck_modeling::Face::new(boundaries, face.oriented_surface())
        })
        .collect();
    faces.sort_by_cached_key(|face| {
        face.boundaries()
            .iter()
            .flat_map(|wire| wire.iter().map(|edge| point_key(edge.front().point())))
            .collect::<Vec<_>>()
    });
    let shell: Shell = faces.into_iter().collect();
    if shell.is_empty() {
        return Ok(Geometry::default());
    }
    let mut g = tessellate(&shell)?;
    g.is_solid = true;
    Ok(g)
}

pub fn difference(target: &Geometry, tool: &Shell) -> Result<Geometry, String> {
    if target.triangles.is_empty() {
        return Ok(target.clone());
    }
    if let Some(result) =
        crate::edge_treatments::subtract_convex(target, tool, target.surface_mapping()?)?
    {
        return Ok(result);
    }
    let target_solid = solid(target)?;
    let mut tool = Solid::try_new(tool.connected_components()).map_err(|e| e.to_string())?;
    tool.not();
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        truck_shapeops::and(&target_solid, &tool, 0.01)
    }))
    .map_err(|_| "The kernel could not subtract this tool; the committed model was preserved.")?
    .ok_or("The kernel could not resolve the solid intersection.")?;
    geometry(&result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Body, Shape, evaluate};
    use truck_modeling::{EuclideanSpace, InnerSpace};
    fn volume(g: &Geometry) -> f64 {
        g.triangles
            .iter()
            .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
            .sum()
    }
    #[test]
    fn angled_planar_termination_keeps_exact_caps_and_blind_pockets() {
        for shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            let mut base = Body::new(Shape::Rectangle);
            base.id = 1;
            base.width = 4.;
            base.depth = 4.;
            base.extrusion = 6.;
            let mut end = Body::new(Shape::Rectangle);
            end.id = 2;
            end.position = [6., 0., 2.5];
            end.rotation = [15., 0., 0.];
            end.extrusion = 0.5;
            let mut profile = Body::new(shape);
            profile.id = 3;
            profile.width = 1.;
            profile.depth = 1.;
            profile.position = [0., 0., 6.];
            let mut bore = Bore::new(&base, FaceTarget::Top).unwrap();
            bore.travel = -1.;
            bore.extent = Extent::UpToFace {
                target: 2,
                face: FaceTarget::Top,
            };
            profile.bore = Some(bore);
            let plane = limit_face(
                2,
                FaceTarget::Top,
                &[end.clone()],
                &[(2, evaluate(&end).unwrap())],
            )
            .unwrap();
            let area = match shape {
                Shape::Rectangle => 1.,
                Shape::Circle => std::f64::consts::PI / 4.,
                _ => std::f64::consts::PI / 8.,
            };
            let centroid = if shape == Shape::Semicircle {
                2. / (3. * std::f64::consts::PI)
            } else {
                0.
            };
            let expected = area
                * (6. - 2.5 - 0.5 / 15f64.to_radians().cos() - 15f64.to_radians().tan() * centroid);
            for mode in [
                BoreMode::Generate,
                BoreMode::Borer,
                BoreMode::GeneratorAndBorer,
            ] {
                profile.bore.as_mut().unwrap().mode = mode;
                let scene =
                    crate::model::evaluate_scene(&[base.clone(), end.clone(), profile.clone()])
                        .unwrap();
                assert!((volume(&scene[1].1) - volume(&evaluate(&end).unwrap())).abs() < 1e-6);
                if mode == BoreMode::Generate {
                    let g = &scene[2].1;
                    assert!(
                        (volume(g) - expected).abs() < 0.04,
                        "{shape:?}: {} vs {expected}",
                        volume(g)
                    );
                    assert!(g.points.iter().all(|p| (p.z - 6.).abs() < 1e-6
                        || (*p - plane.origin).dot(plane.normal).abs() < 1e-6));
                } else {
                    assert!(scene[2].1.triangles.is_empty());
                    assert!(
                        (96. - volume(&scene[0].1) - expected).abs() < 0.04,
                        "{shape:?}: {}",
                        volume(&scene[0].1)
                    );
                    Solid::try_new(scene[0].1.shell.as_ref().unwrap().connected_components())
                        .unwrap();
                }
            }
        }
    }
    #[test]
    fn automatic_extents_follow_targets_preserve_scope_and_roundtrip() {
        for shape in [Shape::Rectangle, Shape::Circle] {
            let mut base = Body::new(shape);
            base.id = 1;
            base.width = 4.;
            base.depth = 4.;
            base.extrusion = 3.;
            base.position = [3., -2., 4.];
            base.rotation = [15., 25., 35.];
            base.scale = [1.5, 0.75, 2.];
            let frame = support_frame(&base, FaceTarget::Top).unwrap();
            let normal = frame.z.truncate();
            let thickness = target_thickness(&base, FaceTarget::Top);
            let mut end = base.clone();
            end.id = 2;
            end.position = (Point3::from(base.position) - normal * (thickness + 2.)).into();
            let mut profile = Body::new(Shape::Circle);
            profile.id = 3;
            profile.width = 1.;
            profile.position = frame.w.truncate().into();
            profile.rotation = crate::regions::rotation_angles(truck_modeling::Matrix3::from_cols(
                frame.x.truncate(),
                frame.y.truncate(),
                normal,
            ));
            let mut bore = Bore::new(&base, FaceTarget::Top).unwrap();
            bore.mode = BoreMode::GeneratorAndBorer;
            bore.travel = -0.1;
            profile.bore = Some(bore);
            let end_volume = volume(&evaluate(&end).unwrap());
            for extent in [
                Extent::ThroughAll,
                Extent::UpToNext { target: 1 },
                Extent::UpToFace {
                    target: 1,
                    face: FaceTarget::Bottom,
                },
                Extent::UpToNext { target: 2 },
                Extent::UpToFace {
                    target: 2,
                    face: FaceTarget::Top,
                },
            ] {
                profile.bore.as_mut().unwrap().extent = extent;
                let bodies = vec![base.clone(), end.clone(), profile.clone()];
                let scene = crate::model::evaluate_scene(&bodies).unwrap();
                let expected = if shape == Shape::Rectangle {
                    4. * 4. * 3. * 2.25
                } else {
                    std::f64::consts::PI * 4. * 3. * 2.25
                };
                assert!(
                    (volume(&scene[0].1) - (expected - std::f64::consts::PI * 0.25 * thickness))
                        .abs()
                        < 0.4,
                    "{extent:?}"
                );
                assert!((volume(&scene[1].1) - end_volume).abs() < 1e-6);
                let emerging = matches!(
                    extent,
                    Extent::UpToNext { target: 2 } | Extent::UpToFace { target: 2, .. }
                );
                assert_eq!(scene[2].1.triangles.is_empty(), !emerging);
                if emerging {
                    assert!((volume(&scene[2].1) - std::f64::consts::PI * 0.5).abs() < 0.04);
                }
                let project = crate::project::Project::new(
                    bodies.clone(),
                    crate::viewport::Camera::default(),
                    true,
                    Some(3),
                );
                let file = tempfile::NamedTempFile::new().unwrap();
                project.save(file.path()).unwrap();
                assert_eq!(
                    crate::project::Project::load(file.path()).unwrap().bodies,
                    bodies
                );
                let mut old = serde_json::to_value(&project).unwrap();
                old["schema_version"] = 9.into();
                old["modeling_version"] = 9.into();
                assert!(
                    serde_json::from_value::<crate::project::Project>(old.clone())
                        .unwrap()
                        .validate()
                        .is_err()
                );
                old["bodies"][2]["bore"]
                    .as_object_mut()
                    .unwrap()
                    .remove("extent");
                serde_json::from_value::<crate::project::Project>(old)
                    .unwrap()
                    .validate()
                    .unwrap();
                for format in [
                    crate::export::Format::Stl,
                    crate::export::Format::Glb,
                    crate::export::Format::Step,
                    crate::export::Format::Dxf,
                ] {
                    assert!(!crate::export::encode(&bodies, format).unwrap().is_empty());
                }
            }
            for mode in [BoreMode::Generate, BoreMode::Borer] {
                profile.bore.as_mut().unwrap().mode = mode;
                profile.bore.as_mut().unwrap().extent = Extent::UpToFace {
                    target: 2,
                    face: FaceTarget::Top,
                };
                let scene =
                    crate::model::evaluate_scene(&[base.clone(), end.clone(), profile.clone()])
                        .unwrap();
                assert_eq!(scene[2].1.triangles.is_empty(), mode == BoreMode::Borer);
                if mode == BoreMode::Generate {
                    assert!((volume(&scene[0].1) - volume(&evaluate(&base).unwrap())).abs() < 1e-6);
                    assert!(
                        (volume(&scene[2].1) - std::f64::consts::PI * 0.25 * (thickness + 2.))
                            .abs()
                            < 0.1
                    );
                }
            }
            profile.bore.as_mut().unwrap().mode = BoreMode::GeneratorAndBorer;
            profile.bore.as_mut().unwrap().extent = Extent::UpToFace {
                target: 2,
                face: FaceTarget::Top,
            };
            let mut model =
                crate::model::Model::from_bodies(vec![base.clone(), end.clone(), profile.clone()]);
            let before = model.bodies.clone();
            assert!(model.delete(2).is_err());
            assert_eq!(model.bodies, before);
            let mut grown = base.clone();
            grown.extrusion = 4.;
            model.commit(grown.clone()).unwrap();
            let attached =
                resolve_profile(&profile, &grown, profile.bore.as_ref().unwrap()).unwrap();
            let scene = crate::model::evaluate_scene(&model.bodies).unwrap();
            assert!(
                (resolved_travel(
                    &attached,
                    profile.bore.as_ref().unwrap(),
                    &model.bodies,
                    &scene
                )
                .unwrap()
                    + 10.)
                    .abs()
                    < 1e-6
            );
            model.undo();
            assert_eq!(model.bodies, before);
            model.redo();
            assert_eq!(model.bodies[0], grown);
            let mut reversed = profile.clone();
            reversed.bore.as_mut().unwrap().travel = 1.;
            assert!(model.commit(reversed).is_err());
            let mut cyclic = profile.clone();
            cyclic.bore.as_mut().unwrap().extent = Extent::UpToNext { target: 3 };
            assert!(cyclic.validate().is_err());
        }
    }
    #[test]
    fn next_angled_faces_preserve_finite_coverage_history_and_exports() {
        for shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            let mut base = Body::new(Shape::Rectangle);
            base.id = 1;
            base.width = 4.;
            base.depth = 4.;
            base.extrusion = 6.;
            let mut end = Body::new(Shape::Rectangle);
            end.id = 2;
            end.width = 4.;
            end.depth = 4.;
            end.extrusion = 0.5;
            end.position = [0., 0., 2.5];
            end.rotation = [15., 0., 0.];
            let mut profile = Body::new(shape);
            profile.id = 3;
            profile.width = 1.;
            profile.depth = 1.;
            profile.position = [0., 0., 6.];
            let mut bore = Bore::new(&base, FaceTarget::Top).unwrap();
            bore.extent = Extent::UpToNext { target: 2 };
            bore.travel = -1.;
            profile.bore = Some(bore);
            let area = match shape {
                Shape::Rectangle => 1.,
                Shape::Circle => std::f64::consts::PI / 4.,
                _ => std::f64::consts::PI / 8.,
            };
            let centroid = if shape == Shape::Semicircle {
                2. / (3. * std::f64::consts::PI)
            } else {
                0.
            };
            let depth =
                6. - 2.5 - 0.5 / 15f64.to_radians().cos() - 15f64.to_radians().tan() * centroid;
            for mode in [
                BoreMode::Generate,
                BoreMode::Borer,
                BoreMode::GeneratorAndBorer,
            ] {
                profile.bore.as_mut().unwrap().mode = mode;
                let bodies = vec![base.clone(), end.clone(), profile.clone()];
                let scene = crate::model::evaluate_scene(&bodies).unwrap();
                assert_eq!(scene[1].1.triangles, evaluate(&end).unwrap().triangles);
                if mode == BoreMode::Generate {
                    assert!((volume(&scene[2].1) - area * depth).abs() < 0.04);
                    assert!((volume(&scene[0].1) - 96.).abs() < 1e-6);
                    if shape == Shape::Circle {
                        assert_eq!(scene[2].1.face_count, 3);
                        assert_eq!(scene[2].1.circles.len(), 2);
                    }
                } else {
                    assert!(scene[2].1.triangles.is_empty());
                    assert!((96. - volume(&scene[0].1) - area * depth).abs() < 0.04);
                }
                for (_, g) in scene.iter().filter(|(_, g)| !g.triangles.is_empty()) {
                    Solid::try_new(g.shell.as_ref().unwrap().connected_components()).unwrap();
                }
                if shape == Shape::Circle {
                    for format in [
                        crate::export::Format::Stl,
                        crate::export::Format::Glb,
                        crate::export::Format::Step,
                        crate::export::Format::Dxf,
                    ] {
                        assert!(!crate::export::encode(&bodies, format).unwrap().is_empty());
                    }
                }
            }
            let bodies = vec![base.clone(), end.clone(), profile.clone()];
            let mut model = crate::model::Model::from_bodies(bodies.clone());
            let mut moved = end.clone();
            moved.position[2] += 0.5;
            model.commit(moved.clone()).unwrap();
            let scene = crate::model::evaluate_scene(&model.bodies).unwrap();
            assert!((96. - volume(&scene[0].1) - area * (depth - 0.5)).abs() < 0.04);
            model.undo();
            assert_eq!(model.bodies, bodies);
            model.redo();
            assert_eq!(model.bodies[1], moved);
            let project = crate::project::Project::new(
                model.bodies.clone(),
                crate::viewport::Camera::default(),
                true,
                Some(3),
            );
            let file = tempfile::NamedTempFile::new().unwrap();
            project.save(file.path()).unwrap();
            assert_eq!(
                crate::project::Project::load(file.path()).unwrap().bodies,
                model.bodies
            );
            let committed = model.bodies.clone();
            moved.width = 0.25;
            assert!(model.commit(moved).unwrap_err().contains("partially"));
            assert_eq!(model.bodies, committed);
            let mut disjoint = end.clone();
            disjoint.position[0] = 6.;
            assert!(model.commit(disjoint).is_err());
            assert_eq!(model.bodies, committed);
            profile.bore.as_mut().unwrap().travel = 1.;
            assert!(model.commit(profile.clone()).is_err());
            assert_eq!(model.bodies, committed);
            profile.bore.as_mut().unwrap().travel = -1.;
            end.position[2] = -2.;
            let scene = crate::model::evaluate_scene(&[base, end.clone(), profile]).unwrap();
            assert!((96. - volume(&scene[0].1) - area * 6.).abs() < 0.06);
            let beyond = 2. - 0.5 / 15f64.to_radians().cos() - 15f64.to_radians().tan() * centroid;
            assert!((volume(&scene[2].1) - area * beyond).abs() < 0.04);
        }
    }
    #[test]
    fn next_plane_checks_finite_nearer_surfaces_instead_of_plane_order() {
        let mut base = Body::new(Shape::Rectangle);
        base.id = 1;
        base.extrusion = 1.;
        let mut end = base.clone();
        end.id = 2;
        end.position[2] = -20.;
        let mut profile = Body::new(Shape::Rectangle);
        profile.id = 3;
        let mut bore = Bore::new(&base, FaceTarget::Top).unwrap();
        bore.extent = Extent::UpToNext { target: 2 };
        bore.travel = 1.;
        let mut stop = Body::new(Shape::Rectangle);
        stop.width = 4.;
        stop.depth = 4.;
        stop.position[2] = 3.;
        stop.rotation[1] = 45.;
        let stop = evaluate(&stop).unwrap();
        let mut patch = Body::new(Shape::Rectangle);
        patch.width = 0.5;
        patch.depth = 2.;
        patch.position = [0.75, 0., 2.5];
        for (x, blocked) in [(0.75, false), (-0.75, true)] {
            patch.position[0] = x;
            let patch = evaluate(&patch).unwrap();
            let mut geometry = Geometry {
                triangles: stop.triangles.clone(),
                triangle_faces: vec![0; stop.triangles.len()],
                face_count: 2,
                ..Default::default()
            };
            geometry
                .triangle_faces
                .extend(vec![1; patch.triangles.len()]);
            geometry.triangles.extend(patch.triangles);
            let result = resolved_extent(
                &profile,
                &bore,
                &[base.clone(), end.clone()],
                &[(2, geometry)],
            );
            if blocked {
                assert!(result.unwrap_err().contains("nearer"));
            } else {
                let (travel, plane) = result.unwrap();
                assert!((travel - 4.).abs() < 1e-6);
                assert!(plane.is_some());
            }
        }
        assert!(
            crate::snapping::Plane::ground()
                .projection_along(Vector3::unit_x())
                .is_none()
        );
    }
    #[test]
    fn next_extent_rejects_partial_missing_and_angled_termination() {
        let mut outline = Body::new(Shape::Rectangle);
        outline.width = 3.;
        outline.depth = 1.;
        outline.position[2] = 1.;
        let mut patch = Body::new(Shape::Rectangle);
        patch.position[0] = -1.;
        let left = evaluate(&patch).unwrap();
        patch.position[0] = 1.;
        let right = evaluate(&patch).unwrap();
        let mut joined = Geometry::default();
        joined.triangles = left.triangles.clone();
        joined.triangle_faces = vec![0; joined.triangles.len()];
        joined.triangles.extend(right.triangles.iter());
        joined.triangle_faces.extend(vec![1; right.triangles.len()]);
        assert_eq!(
            crate::booleans::profile_coverage(
                &outline,
                &joined,
                &[0, 1],
                crate::snapping::Plane::ground()
            )
            .unwrap(),
            (true, true)
        );
        assert_eq!(
            crate::booleans::profile_coverage(
                &outline,
                &joined,
                &[0],
                crate::snapping::Plane::ground()
            )
            .unwrap(),
            (false, true)
        );
        let mut base = Body::new(Shape::Rectangle);
        base.id = 1;
        base.width = 4.;
        base.depth = 4.;
        base.extrusion = 3.;
        let mut profile = Body::new(Shape::Circle);
        profile.id = 2;
        profile.width = 1.;
        profile.position = [2., 0., 3.];
        let mut bore = Bore::new(&base, FaceTarget::Top).unwrap();
        bore.mode = BoreMode::Borer;
        bore.travel = -1.;
        bore.extent = Extent::UpToNext { target: 1 };
        profile.bore = Some(bore);
        let reason = crate::model::evaluate_scene(&[base.clone(), profile.clone()])
            .err()
            .unwrap();
        assert!(reason.contains("partially"), "{reason}");
        profile.position[0] = 5.;
        assert!(crate::model::evaluate_scene(&[base.clone(), profile.clone()]).is_err());
        profile.position[0] = 0.;
        profile.bore.as_mut().unwrap().extent = Extent::UpToFace {
            target: 1,
            face: FaceTarget::Right,
        };
        assert!(
            crate::model::evaluate_scene(&[base.clone(), profile.clone()])
                .err()
                .unwrap()
                .contains("parallel")
        );
        profile.bore.as_mut().unwrap().extent = Extent::ThroughAll;
        profile.bore.as_mut().unwrap().travel = 1.;
        assert!(
            crate::model::evaluate_scene(&[base, profile])
                .err()
                .unwrap()
                .contains("reverse")
        );
    }
    #[test]
    fn concentric_pipe_bore_stays_fast_while_depth_changes() {
        let mut base = Body::new(Shape::Circle);
        base.id = 1;
        base.width = 4.;
        base.extrusion = 2.;
        let mut cutter = Body::new(Shape::Circle);
        cutter.id = 2;
        cutter.width = 2.;
        cutter.position = [0., 0., 2.];
        cutter.bore = Some(Bore::new(&base, FaceTarget::Top).unwrap());
        cutter.bore.as_mut().unwrap().mode = BoreMode::Borer;
        for depth in [0.1, 1., 1.99, 2., 3.] {
            eprintln!("concentric bore depth {depth}");
            cutter.bore.as_mut().unwrap().travel = -depth;
            let started = std::time::Instant::now();
            let scene = crate::model::evaluate_scene(&[base.clone(), cutter.clone()]).unwrap();
            assert!(
                (volume(&scene[0].1)
                    - (8. * std::f64::consts::PI - std::f64::consts::PI * depth.min(2.)))
                .abs()
                    < 0.15
            );
            eprintln!("finished in {:?}", started.elapsed());
            assert!(
                started.elapsed() < std::time::Duration::from_secs(5),
                "contained bore took {:?}",
                started.elapsed()
            );
            Solid::try_new(scene[0].1.shell.as_ref().unwrap().connected_components()).unwrap();
        }
    }
    #[test]
    fn contained_pockets_cover_bottom_thin_walls_and_transformed_supports() {
        for shape in [Shape::Circle, Shape::Rectangle] {
            let mut base = Body::new(shape);
            base.id = 1;
            base.width = 4.;
            base.depth = 4.;
            base.extrusion = 2.;
            base.position = [4., -3., 2.];
            base.rotation = [15., 25., 35.];
            base.scale = [1.5, 0.75, 2.];
            for face in [FaceTarget::Top, FaceTarget::Bottom] {
                let frame = support_frame(&base, face).unwrap();
                let mut cutter = Body::new(Shape::Circle);
                cutter.id = 2;
                cutter.width = 1.;
                // Build the authored profile directly in the support frame.
                cutter.position = frame.w.truncate().into();
                cutter.rotation =
                    crate::regions::rotation_angles(truck_modeling::Matrix3::from_cols(
                        frame.x.truncate(),
                        frame.y.truncate(),
                        frame.z.truncate(),
                    ));
                let mut bore = Bore::new(&base, face).unwrap();
                bore.travel = -1.;
                bore.mode = BoreMode::Borer;
                cutter.bore = Some(bore);
                let scene = crate::model::evaluate_scene(&[base.clone(), cutter.clone()]).unwrap();
                let expected = if shape == Shape::Circle {
                    std::f64::consts::PI * 4. * 2. * 2.25
                } else {
                    4. * 4. * 2. * 2.25
                };
                assert!(
                    (volume(&scene[0].1) - (expected - std::f64::consts::PI * 0.25)).abs() < 0.3
                );
                let mut mesh = scene[0].1.clone();
                crate::components::weld_geometry(&mut mesh);
                crate::components::boundary_shell(&mesh, true)
                    .unwrap_or_else(|e| panic!("{shape:?} {face:?}: {e}"));
            }
        }
        let mut base = Body::new(Shape::Circle);
        base.id = 1;
        base.width = 4.;
        base.extrusion = 2.;
        let mut cutter = Body::new(Shape::Circle);
        cutter.width = 3.99;
        cutter.position[2] = 2.;
        let base_geometry = evaluate(&base).unwrap();
        for depth in [0.5, 2.] {
            let g = contained_profile(&base, &base_geometry, &cutter, FaceTarget::Top, depth)
                .unwrap()
                .expect("thin pipe must avoid CSG");
            assert!(g.is_solid);
            let mut mesh = g.clone();
            crate::components::weld_geometry(&mut mesh);
            crate::components::boundary_shell(&mesh, true).unwrap();
        }
    }
    #[test]
    fn kernel_circular_pocket_and_notch() {
        let mut base = Body::new(Shape::Rectangle);
        base.width = 4.;
        base.depth = 4.;
        base.extrusion = 3.;
        let base = evaluate(&base).unwrap();
        for x in [0., 2.] {
            let mut tool = Body::new(Shape::Circle);
            tool.width = 1.;
            tool.extrusion = 1.01;
            tool.position = [x, 0., 2.];
            let out = difference(&base, &crate::model::primitive_shell(&tool).unwrap()).unwrap();
            let expected = 48. - std::f64::consts::PI * 0.25 * (if x == 0. { 1. } else { 0.5 });
            assert!(
                (volume(&out) - expected).abs() < 0.03,
                "volume {} expected {expected}",
                volume(&out)
            );
            let mut out = out;
            crate::components::weld_geometry(&mut out);
            crate::components::boundary_shell(&out, true).unwrap();
        }
    }
    #[test]
    fn bore_history_targets_and_separate_outputs_regenerate() {
        use crate::{
            model::{Model, evaluate_scene},
            project::Project,
            viewport::Camera,
        };
        let mut base = Body::new(Shape::Rectangle);
        base.id = 1;
        base.width = 4.;
        base.depth = 4.;
        base.extrusion = 3.;
        let mut profile = Body::new(Shape::Circle);
        profile.id = 2;
        profile.width = 1.;
        profile.position = [0., 0., 3.];
        profile.bore = Some(Bore::new(&base, FaceTarget::Top).unwrap());
        profile.bore.as_mut().unwrap().mode = BoreMode::GeneratorAndBorer;
        let mut model = Model::from_bodies(vec![base.clone(), profile.clone()]);
        for travel in [-1., -3., -4.5] {
            let mut changed = profile.clone();
            changed.bore.as_mut().unwrap().travel = travel;
            model.commit(changed).unwrap();
            let scene = evaluate_scene(&model.bodies).unwrap();
            let expected = 48. - std::f64::consts::PI * 0.25 * (-travel).min(3.);
            assert!(
                (volume(&scene[0].1) - expected).abs() < 0.05,
                "travel {travel}: {} != {expected}",
                volume(&scene[0].1)
            );
            assert_eq!(scene[1].0, 2);
            if travel < -3. {
                assert!(scene[1].1.is_solid);
                assert!((volume(&scene[1].1) - std::f64::consts::PI * 0.25 * 1.5).abs() < 0.03);
                assert!(scene[1].1.triangles.iter().flatten().all(|p| p.z <= 1e-7));
            } else {
                assert!(scene[1].1.triangles.is_empty());
            }
        }
        let generated = model.bodies.clone();
        for format in crate::export::Format::ALL {
            crate::export::encode(&generated, format).unwrap();
        }
        let mut thick = base.clone();
        thick.extrusion = 100.;
        let mut long = profile.clone();
        long.position = [0., 0., 100.];
        long.bore = Some(Bore::new(&thick, FaceTarget::Top).unwrap());
        long.bore.as_mut().unwrap().mode = BoreMode::GeneratorAndBorer;
        long.bore.as_mut().unwrap().travel = -150.;
        let long_scene = evaluate_scene(&[thick, long]).unwrap();
        assert!((volume(&long_scene[1].1) - std::f64::consts::PI * 0.25 * 50.).abs() < 0.5);
        assert!(
            long_scene[1]
                .1
                .triangles
                .iter()
                .flatten()
                .all(|p| p.z >= -50. - 1e-6 && p.z <= 1e-6)
        );
        let mut only = model.bodies[1].clone();
        only.bore.as_mut().unwrap().mode = BoreMode::Borer;
        model.commit(only).unwrap();
        assert!(
            evaluate_scene(&model.bodies).unwrap()[1]
                .1
                .triangles
                .is_empty()
        );
        model.undo();
        assert_eq!(model.bodies, generated);
        model.redo();
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("bore.osk");
        let project = Project::new(model.bodies.clone(), Camera::default(), true, Some(2));
        project.save(&path).unwrap();
        let loaded = Project::load(&path).unwrap();
        assert_eq!(loaded.bodies, model.bodies);
        let mut legacy = serde_json::to_value(&project).unwrap();
        legacy["schema_version"] = 5.into();
        legacy["modeling_version"] = 5.into();
        std::fs::write(&path, serde_json::to_vec(&legacy).unwrap()).unwrap();
        assert!(Project::load(&path).is_err());
        let before = model.bodies.clone();
        assert!(model.delete(1).is_err());
        assert_eq!(model.bodies, before);
        model.delete(2).unwrap();
        assert!((volume(&evaluate_scene(&model.bodies).unwrap()[0].1) - 48.).abs() < 1e-6);
        model.undo();
        assert_eq!(model.bodies, before);
        let mut bad = model.bodies[1].clone();
        bad.bore.as_mut().unwrap().target = 999;
        assert!(model.commit(bad).is_err());
        assert_eq!(model.bodies, before);
    }
    #[test]
    fn cylinder_rim_notch_and_parent_attachment_use_semantic_faces() {
        use crate::model::evaluate_scene;
        let mut base = Body::new(Shape::Circle);
        base.id = 1;
        base.width = 4.;
        base.extrusion = 3.;
        let mut profile = Body::new(Shape::Circle);
        profile.id = 2;
        profile.width = 1.;
        profile.position = [2., 0., 3.];
        let mut bore = Bore::new(&base, FaceTarget::Top).unwrap();
        bore.mode = BoreMode::Borer;
        bore.travel = -1.;
        profile.bore = Some(bore);
        let scene = evaluate_scene(&[base.clone(), profile.clone()]).unwrap();
        let (r, big, d) = (0.5_f64, 2_f64, 2_f64);
        let lens = r * r * ((d * d + r * r - big * big) / (2. * d * r)).acos()
            + big * big * ((d * d + big * big - r * r) / (2. * d * big)).acos()
            - 0.5 * ((-d + r + big) * (d + r - big) * (d - r + big) * (d + r + big)).sqrt();
        assert!((volume(&scene[0].1) - (std::f64::consts::PI * 12. - lens)).abs() < 0.15);
        let mut closed = scene[0].1.clone();
        crate::components::weld_geometry(&mut closed);
        crate::components::boundary_shell(&closed, true).unwrap();
        // A source height change moves the attached sketch plane, rather than
        // leaving it behind at the original cap or rebinding to an output index.
        base.extrusion = 4.;
        base.position = [2., -3., 1.];
        base.rotation = [10., 25., 32.];
        profile.bore.as_mut().unwrap().mode = BoreMode::Generate;
        profile.bore.as_mut().unwrap().travel = 1.;
        let resolved = resolve_profile(&profile, &base, profile.bore.as_ref().unwrap()).unwrap();
        let expected = crate::regions::transform(&base).transform_point(Point3::new(2., 0., 4.));
        assert!((Point3::from(resolved.position) - expected).magnitude() < 1e-6);
        let bodies = vec![base.clone(), profile.clone()];
        let scene = evaluate_scene(&bodies).unwrap();
        let translation = Matrix4::from_translation(Vector3::new(1., 0., 0.));
        let moved = crate::components::transform_selection(
            &bodies,
            &scene,
            crate::selection::SelectionMode::Volume,
            &[(1, None), (2, None)],
            translation,
        )
        .unwrap();
        assert_eq!(
            moved[1], profile,
            "selected attached child follows its parent once"
        );
        let attached =
            resolve_profile(&moved[1], &moved[0], moved[1].bore.as_ref().unwrap()).unwrap();
        assert!(
            (Point3::from(attached.position) - translation.transform_point(expected)).magnitude()
                < 1e-6
        );
        let mut bad = profile.clone();
        bad.bore.as_mut().unwrap().reference_frame[0][0] = 2.;
        assert!(bad.validate().is_err());
    }
    #[test]
    fn boundary_crossing_rectangular_bores_share_exact_subtraction() {
        let mut target = Body::new(Shape::Rectangle);
        target.id = 1;
        target.width = 4.;
        target.depth = 3.;
        target.extrusion = 5.;
        for mode in [BoreMode::Borer, BoreMode::GeneratorAndBorer] {
            for travel in [-2., -5., -7.] {
                let mut profile = Body::new(Shape::Rectangle);
                profile.id = 2;
                profile.width = 2.;
                profile.depth = 2.;
                profile.position = [1.5, 0., 5.];
                let mut bore = Bore::new(&target, FaceTarget::Top).unwrap();
                bore.mode = mode;
                bore.travel = travel;
                profile.bore = Some(bore);
                let bodies = vec![target.clone(), profile];
                let scene = crate::model::evaluate_scene(&bodies).unwrap();
                assert!((volume(&scene[0].1) - (60. - 3. * (-travel).min(5.))).abs() < 1e-6);
                let emergence = if mode == BoreMode::GeneratorAndBorer {
                    4. * (-travel - 5.).max(0.)
                } else {
                    0.
                };
                assert!((volume(&scene[1].1) - emergence).abs() < 1e-6);
                let shell = scene[0].1.shell.as_ref().unwrap();
                assert!(
                    shell
                        .iter()
                        .flat_map(|f| f.edge_iter())
                        .all(|e| matches!(e.curve(), truck_modeling::Curve::Line(_)))
                );
                crate::components::solid_parts(shell).unwrap();
                let mut mesh = scene[0].1.clone();
                crate::components::weld_geometry(&mut mesh);
                crate::components::boundary_shell(&mesh, true).unwrap();
                let dir = tempfile::tempdir().unwrap();
                let path = dir.path().join("rectangular-notch.osk");
                crate::project::Project::new(
                    bodies.clone(),
                    crate::viewport::Camera::default(),
                    true,
                    Some(1),
                )
                .save(&path)
                .unwrap();
                assert_eq!(crate::project::Project::load(&path).unwrap().bodies, bodies);
                for format in crate::export::Format::ALL {
                    assert!(!crate::export::encode(&bodies, format).unwrap().is_empty());
                }
            }
        }
    }
    #[test]
    fn through_all_scope_cuts_only_explicit_bodies_and_regenerates_atomically() {
        use crate::{
            model::{Model, evaluate_scene},
            project::Project,
            viewport::Camera,
        };
        for shape in [Shape::Rectangle, Shape::Circle] {
            let mut upper = Body::new(shape);
            upper.id = 1;
            upper.width = 4.;
            upper.depth = 4.;
            upper.extrusion = 2.;
            upper.position = [0., 0., 4.];
            let mut lower = upper.clone();
            lower.id = 2;
            lower.position = [0., 0., 0.];
            let mut unchecked = upper.clone();
            unchecked.id = 3;
            unchecked.extrusion = 1.;
            unchecked.position = [0., 0., 2.5];
            let initial = vec![upper.clone(), lower.clone(), unchecked.clone()];
            let original = evaluate_scene(&initial).unwrap();
            let base = volume(&original[0].1);
            for mode in [
                BoreMode::Borer,
                BoreMode::GeneratorAndBorer,
                BoreMode::Generate,
            ] {
                let mut profile = Body::new(Shape::Circle);
                profile.id = 4;
                profile.width = 1.;
                profile.position = [0., 0., 6.];
                let mut bore = Bore::new(&upper, FaceTarget::Top).unwrap();
                bore.mode = mode;
                bore.travel = -1.;
                bore.extent = Extent::ThroughAll;
                bore.through_targets = vec![2];
                profile.bore = Some(bore);
                let mut bodies = initial.clone();
                bodies.push(profile.clone());
                let scene = evaluate_scene(&bodies).unwrap();
                let resolved =
                    resolve_profile(&profile, &upper, profile.bore.as_ref().unwrap()).unwrap();
                assert!(
                    (resolved_travel(
                        &resolved,
                        profile.bore.as_ref().unwrap(),
                        &bodies,
                        &original
                    )
                    .unwrap()
                        + 6.)
                        .abs()
                        < 1e-8
                );
                for index in [0, 1] {
                    let expected = if mode == BoreMode::Generate {
                        base
                    } else {
                        base - std::f64::consts::PI * 0.5
                    };
                    assert!(
                        (volume(&scene[index].1) - expected).abs() < 0.15,
                        "{shape:?} {mode:?} body {index}"
                    );
                    if mode != BoreMode::Generate {
                        assert_eq!(
                            scene[index].1.circles.len(),
                            if shape == Shape::Circle { 4 } else { 2 },
                            "Entry/exit circle snapping follows every cut target"
                        );
                    }
                }
                assert_eq!(
                    scene[2].1.triangles, original[2].1.triangles,
                    "Unchecked bodies remain unchanged even between targets"
                );
                if mode == BoreMode::Generate {
                    assert!((volume(&scene[3].1) - std::f64::consts::PI * 1.5).abs() < 0.2);
                    let low = scene[3]
                        .1
                        .points
                        .iter()
                        .map(|p| p.z)
                        .fold(f64::INFINITY, f64::min);
                    let high = scene[3]
                        .1
                        .points
                        .iter()
                        .map(|p| p.z)
                        .fold(f64::NEG_INFINITY, f64::max);
                    assert!(
                        low.abs() < 1e-8 && (high - 6.).abs() < 1e-8,
                        "End planes remain exact independently of display chord tolerance"
                    );
                } else {
                    assert!(
                        scene[3].1.triangles.is_empty(),
                        "Automatic Through all ends at scope; it does not fill gaps or emit an extension"
                    );
                }
                let mut model = Model::from_bodies(initial.clone());
                model.commit(profile.clone()).unwrap();
                let accepted = model.bodies.clone();
                model.undo();
                assert_eq!(model.bodies, initial);
                model.redo();
                assert_eq!(model.bodies, accepted);
                let dir = tempfile::tempdir().unwrap();
                let path = dir.path().join("scope.osk");
                let project = Project::new(accepted.clone(), Camera::default(), true, Some(1));
                project.save(&path).unwrap();
                assert_eq!(Project::load(&path).unwrap().bodies, accepted);
                let mut old = serde_json::to_value(project).unwrap();
                old["schema_version"] = 20.into();
                old["modeling_version"] = 20.into();
                std::fs::write(&path, serde_json::to_vec(&old).unwrap()).unwrap();
                assert!(
                    Project::load(&path)
                        .unwrap_err()
                        .contains("Multiple-body Through all requires .osk version 21")
                );
                old["bodies"][3]["bore"]
                    .as_object_mut()
                    .unwrap()
                    .remove("through_targets");
                std::fs::write(&path, serde_json::to_vec(&old).unwrap()).unwrap();
                assert!(
                    Project::load(&path).unwrap().bodies[3]
                        .bore
                        .as_ref()
                        .unwrap()
                        .through_targets
                        .is_empty()
                );
                for format in crate::export::Format::ALL {
                    assert!(!crate::export::encode(&accepted, format).unwrap().is_empty());
                }
                for ids in [vec![1], vec![2, 2], vec![9]] {
                    let mut invalid = profile.clone();
                    invalid.bore.as_mut().unwrap().through_targets = ids;
                    assert!(model.commit(invalid).is_err());
                    assert_eq!(model.bodies, accepted);
                }
                let mut invalid = profile.clone();
                invalid.bore.as_mut().unwrap().extent = Extent::Distance;
                assert!(model.commit(invalid).is_err());
                assert_eq!(model.bodies, accepted);
                assert!(model.delete(2).is_err());
                assert_eq!(model.bodies, accepted);
                let mut deeper = lower.clone();
                deeper.position[2] = -2.;
                model.commit(deeper).unwrap();
                let regenerated = evaluate_scene(&model.bodies).unwrap();
                let resolved = resolve_profile(
                    &model.bodies[3],
                    &model.bodies[0],
                    model.bodies[3].bore.as_ref().unwrap(),
                )
                .unwrap();
                assert!(
                    (resolved_travel(
                        &resolved,
                        model.bodies[3].bore.as_ref().unwrap(),
                        &model.bodies,
                        &regenerated
                    )
                    .unwrap()
                        + 8.)
                        .abs()
                        < 1e-8
                );
                let mut detached = profile.clone();
                detached.bore = None;
                model.commit(detached).unwrap();
                let restored = evaluate_scene(&model.bodies).unwrap();
                assert!(
                    (volume(&restored[0].1) - base).abs() < 1e-6
                        && (volume(&restored[1].1) - base).abs() < 1e-6
                );
            }
        }
    }
    #[test]
    fn analytic_scope_bounds_cover_curved_and_affine_geometry() {
        for shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            for rounded in [false, true] {
                let mut b = Body::new(shape);
                b.id = 1;
                b.width = 4.;
                b.depth = 3.;
                b.extrusion = 5.;
                b.position = [2., -3., 1.];
                b.rotation = [17., 31., 43.];
                b.scale = [1.5, 0.75, 2.];
                if rounded {
                    b.edge_treatments
                        .push(crate::edge_treatments::Modifier::new(
                            crate::edge_treatments::Kind::Bevel,
                            FaceTarget::Top,
                            &b,
                        ));
                }
                let g = evaluate(&b).unwrap();
                let direction = Vector3::new(0.3, -0.4, 0.5).normalize();
                let bounds = g.projection_range(direction).unwrap();
                assert!(g.triangles.iter().flatten().all(|p| {
                    let v = p.to_vec().dot(direction);
                    (bounds[0] - 1e-6..=bounds[1] + 1e-6).contains(&v)
                }));
                if shape == Shape::Circle && !rounded {
                    let m = crate::regions::transform(&b);
                    let radius = b.width / 2.;
                    let radial = radius
                        * direction
                            .dot(m.x.truncate())
                            .hypot(direction.dot(m.y.truncate()));
                    let height = b.extrusion * direction.dot(m.z.truncate());
                    let center = Point3::from(b.position).to_vec().dot(direction);
                    assert!(
                        bounds[0] <= center - radial + height.min(0.) + 1e-7
                            && bounds[1] >= center + radial + height.max(0.) - 1e-7,
                        "Analytic full-cylinder extrema cannot lie outside the support bound"
                    );
                }
                assert!(g.projection_range(Vector3::new(f64::NAN, 0., 0.)).is_err());
            }
        }
    }
}
