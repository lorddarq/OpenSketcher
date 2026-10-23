//! Ordered, persistent set operations; inputs remain authored and undoable.
use crate::{
    components::Topology,
    model::{Body, Geometry},
    selection::{Pick, SelectionMode},
};
use serde::{Deserialize, Serialize};
use truck_modeling::{InnerSpace, Point3, Shell, Transform, builder};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Kind {
    Union,
    Subtract,
    Intersect,
}
impl Kind {
    pub fn label(self) -> &'static str {
        match self {
            Self::Union => "Union",
            Self::Subtract => "Subtract",
            Self::Intersect => "Intersect",
        }
    }
    fn contains(self, a: bool, b: bool) -> bool {
        match self {
            Self::Union => a || b,
            Self::Subtract => a && !b,
            Self::Intersect => a && b,
        }
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Boolean {
    pub kind: Kind,
    pub operands: [Pick; 2],
    pub topology: [Topology; 2],
    pub keep_tools: bool,
}
impl Boolean {
    pub fn new(kind: Kind, picks: &[Pick], mode: SelectionMode, scene: &[(u64, Geometry)]) -> Self {
        let operands = std::array::from_fn(|i| {
            (
                picks[i].0,
                if mode == SelectionMode::Face {
                    picks[i].1
                } else {
                    None
                },
            )
        });
        Self {
            kind,
            operands,
            topology: std::array::from_fn(|i| {
                scene
                    .iter()
                    .find(|g| g.0 == operands[i].0)
                    .map(|g| g.1.topology.clone())
                    .unwrap_or_default()
            }),
            keep_tools: false,
        }
    }
    pub fn references(&self) -> String {
        let label = |(id, face): Pick| match face {
            Some(index) => format!("body {id} face {}", index.saturating_add(1)),
            None => format!("body {id} volume"),
        };
        format!(
            "target {} · tool {}",
            label(self.operands[0]),
            label(self.operands[1])
        )
    }
    pub fn validate(&self, body: &Body) -> Result<(), String> {
        if self.operands[0] == self.operands[1]
            || self
                .operands
                .iter()
                .any(|p| p.0 == 0 || (body.id != 0 && p.0 >= body.id))
            || self.operands[0].1.is_some() != self.operands[1].1.is_some()
        {
            return Err("Choose two distinct earlier volumes, or two coplanar faces.".into());
        }
        if body.bore.is_some()
            || crate::components::has_source_edits(body)
            || !body.edge_treatments.is_empty()
            || body.cuts_profiles
            || body.extrusion != 0.
            || body.position != [0.; 3]
            || body.rotation != [0.; 3]
            || body.scale != [1.; 3]
        {
            return Err("Edit Boolean source geometry through its operands.".into());
        }
        Ok(())
    }
}

pub fn apply(bodies: &[Body], scene: &mut [(u64, Geometry)]) -> Result<(), String> {
    // Resolve before suppressing inputs so chains and shared tools stay evaluable.
    let mut suppressed = Vec::new();
    let mut features: Vec<_> = bodies.iter().filter(|b| b.boolean.is_some()).collect();
    features.sort_by_key(|b| b.id);
    for body in features {
        let op = body.boolean.as_ref().unwrap();
        (|| {
            let operands: Vec<_> = op
                .operands
                .iter()
                .map(|p| {
                    scene
                        .iter()
                        .find(|g| g.0 == p.0)
                        .map(|g| &g.1)
                        .ok_or("A Boolean operand was removed.")
                })
                .collect::<Result<_, _>>()?;
            let mut result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                if op.operands[0].1.is_some() {
                    faces(op, &operands)
                } else {
                    prisms(op, operands[0], operands[1]).and_then(|g| match g {
                        Some(g) => Ok(g),
                        None => volumes(op.kind, operands[0], operands[1]),
                    })
                }
            }))
            .map_err(|_| {
                "The kernel could not evaluate this Boolean; the committed model was preserved."
                    .to_owned()
            })??;
            crate::components::apply_evaluated(body, &mut result)?;
            scene.iter_mut().find(|g| g.0 == body.id).unwrap().1 = result;
            // Face operations produce a sheet; a solid containing a picked face remains intact.
            for (i, p) in op.operands.iter().enumerate() {
                if (i == 0 || !op.keep_tools) && (p.1.is_none() || !operands_solid(scene, p.0)) {
                    suppressed.push(*p);
                }
            }
            Ok(())
        })()
        .map_err(|reason: String| format!("{} · {}: {reason}", body.label(), op.references()))?;
    }
    for (id, g) in scene {
        let removed: Vec<_> = suppressed.iter().filter(|p| p.0 == *id).collect();
        if removed.is_empty() {
            continue;
        }
        if removed.iter().any(|p| p.1.is_none()) || g.face_count == 1 {
            *g = Geometry::default();
            continue;
        }
        let source = g
            .shell
            .as_ref()
            .filter(|s| s.len() == g.face_count)
            .ok_or_else(||format!("Boolean input body {id}: cannot consume this sheet face without a retained face boundary."))?;
        let shell: Shell = source
            .iter()
            .enumerate()
            .filter(|(i, _)| !removed.iter().any(|p| p.1 == Some(*i)))
            .map(|(_, f)| f.clone())
            .collect();
        *g = if shell.is_empty() {
            Geometry::default()
        } else {
            crate::model::tessellate(&shell)?
        };
    }
    Ok(())
}
fn operands_solid(scene: &[(u64, Geometry)], id: u64) -> bool {
    scene
        .iter()
        .find(|g| g.0 == id)
        .is_some_and(|g| g.1.is_solid)
}
// Equal-depth prisms have coincident cap planes, a degenerate case for Truck CSG.
// Perform the same set operation on their cap regions and sweep once.
fn prisms(op: &Boolean, a: &Geometry, b: &Geometry) -> Result<Option<Geometry>, String> {
    if !a.is_solid || !b.is_solid {
        return Ok(None);
    }
    let bounds = |g: &Geometry| {
        g.points
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                (lo.min(p.z), hi.max(p.z))
            })
    };
    let (lo, hi) = bounds(a);
    let (blo, bhi) = bounds(b);
    if (lo - blo).abs() > 1e-7 || (hi - bhi).abs() > 1e-7 {
        return Ok(None);
    }
    let cap = |g: &Geometry| {
        (0..g.face_count).find(|f| {
            let ts: Vec<_> = g
                .triangles
                .iter()
                .zip(&g.triangle_faces)
                .filter(|(_, id)| *id == f)
                .collect();
            !ts.is_empty()
                && ts
                    .iter()
                    .all(|(t, _)| t.iter().all(|p| (p.z - hi).abs() < 1e-7))
        })
    };
    let (Some(af), Some(bf)) = (cap(a), cap(b)) else {
        return Ok(None);
    };
    // A straight prism's side vertices lie only on the two cap planes.
    if [a, b].iter().any(|g| {
        g.points
            .iter()
            .any(|p| (p.z - lo).abs() > 1e-7 && (p.z - hi).abs() > 1e-7)
    }) {
        return Ok(None);
    }
    // Matching top/bottom footprints exclude taper, chamfers, and non-prismatic solids.
    for g in [a, b] {
        if g.points
            .iter()
            .filter(|p| (p.z - hi).abs() < 1e-7)
            .any(|p| {
                !g.points.iter().any(|q| {
                    (q.z - lo).abs() < 1e-7 && (p.x - q.x).abs() < 1e-7 && (p.y - q.y).abs() < 1e-7
                })
            })
        {
            return Ok(None);
        }
    }
    let mut face_op = op.clone();
    face_op.operands[0].1 = Some(af);
    face_op.operands[1].1 = Some(bf);
    face_op.topology = [a.topology.clone(), b.topology.clone()];
    let sheet = faces(&face_op, &[a, b])?;
    let Some(cap_shell) = sheet.shell else {
        return Ok(Some(Geometry::default()));
    };
    let mut shell = Shell::default();
    for face in &cap_shell {
        let swept = builder::tsweep(
            &face.inverse(),
            truck_modeling::Vector3::new(0., 0., lo - hi),
        );
        shell.extend(swept.boundaries().iter().flat_map(|s| s.iter().cloned()));
    }
    let solid =
        truck_modeling::Solid::try_new(shell.connected_components()).map_err(|e| e.to_string())?;
    crate::operators::geometry(&solid).map(Some)
}
fn volumes(kind: Kind, a: &Geometry, b: &Geometry) -> Result<Geometry, String> {
    if !a.is_solid || !b.is_solid {
        return Err(
            "Volume Booleans require two closed solids. Use Face selection for sheets.".into(),
        );
    }
    let result = match kind {
        Kind::Union => truck_shapeops::or(
            &crate::operators::solid(a)?,
            &crate::operators::solid(b)?,
            0.01,
        ),
        Kind::Intersect => truck_shapeops::and(
            &crate::operators::solid(a)?,
            &crate::operators::solid(b)?,
            0.01,
        ),
        Kind::Subtract => {
            return crate::operators::difference(
                a,
                b.shell
                    .as_ref()
                    .ok_or("The tool requires a retained kernel boundary.")?,
            );
        }
    }
    .ok_or("The kernel could not resolve these Boolean operands.")?;
    crate::operators::geometry(&result)
}
/// Check complete termination using the same planar region difference as face Booleans.
pub(crate) fn profile_coverage(
    profile: &Body,
    g: &Geometry,
    faces: &[usize],
    travel: f64,
) -> Result<(bool, bool), String> {
    let mut moved = profile.clone();
    moved.position = (truck_modeling::Point3::from(profile.position)
        + crate::regions::transform(profile)
            .transform_vector(truck_modeling::Vector3::unit_z())
            .normalize()
            * travel)
        .into();
    let mesh = crate::model::build(&moved, None)?;
    face_coverage(&mesh, g, faces)
}
pub(crate) fn face_coverage(
    mesh: &Geometry,
    g: &Geometry,
    faces: &[usize],
) -> Result<(bool, bool), String> {
    let triangles: Vec<_> = g
        .triangles
        .iter()
        .zip(&g.triangle_faces)
        .filter(|(_, face)| faces.contains(face))
        .map(|(t, _)| *t)
        .collect();
    let target = Geometry {
        triangle_faces: vec![0; triangles.len()],
        triangles,
        ..Default::default()
    };
    let mut operation = Boolean {
        kind: Kind::Subtract,
        operands: [(1, Some(0)), (2, Some(0))],
        topology: [mesh.topology.clone(), target.topology.clone()],
        keep_tools: false,
    };
    if self::faces(&operation, &[mesh, &target])?
        .triangles
        .is_empty()
    {
        return Ok((true, true));
    }
    operation.kind = Kind::Intersect;
    Ok((
        false,
        !self::faces(&operation, &[mesh, &target])?
            .triangles
            .is_empty(),
    ))
}
fn faces(op: &Boolean, geometry: &[&Geometry]) -> Result<Geometry, String> {
    let mut triangles = Vec::new();
    for (i, g) in geometry.iter().enumerate() {
        if g.topology != op.topology[i] {
            return Err("A Boolean face changed topology; reselect the intended face.".into());
        }
        let selected: Vec<_> = g
            .triangles
            .iter()
            .zip(&g.triangle_faces)
            .filter(|(_, f)| Some(**f) == op.operands[i].1)
            .map(|(t, _)| *t)
            .collect();
        if selected.is_empty() {
            return Err("The selected Boolean face no longer exists.".into());
        }
        triangles.push(selected);
    }
    let t = triangles[0][0];
    let origin = t[0];
    let u = (t[1] - origin).normalize();
    let n = (t[1] - origin).cross(t[2] - origin).normalize();
    let v = n.cross(u);
    let project = |p: Point3| {
        let d = p - origin;
        [d.dot(u), d.dot(v)]
    };
    if triangles
        .iter()
        .flatten()
        .flatten()
        .any(|p| (*p - origin).dot(n).abs() > 1e-6)
    {
        return Err("Face Booleans require coplanar planar faces. Curved or crossing faces need trim/split operations.".into());
    }
    let polygons: Vec<Vec<[f64; 2]>> = triangles
        .iter()
        .flatten()
        .map(|t| t.iter().map(|p| project(*p)).collect())
        .collect();
    let split = triangles[0].len();
    let inside = |p, range: std::ops::Range<usize>| {
        range
            .into_iter()
            .any(|i| crate::regions::contains(p, &polygons[i]))
    };
    let loops = crate::regions::trace(&polygons, |p| {
        op.kind
            .contains(inside(p, 0..split), inside(p, split..polygons.len()))
    })?;
    let wires: Vec<truck_modeling::Wire> = loops
        .iter()
        .map(|polygon| {
            let vertices: Vec<_> = polygon
                .iter()
                .map(|p| builder::vertex(origin + u * p[0] + v * p[1]))
                .collect();
            (0..vertices.len())
                .map(|i| builder::line(&vertices[i], &vertices[(i + 1) % vertices.len()]))
                .collect()
        })
        .collect();
    let mut shell = Shell::default();
    for (i, outer) in loops
        .iter()
        .enumerate()
        .filter(|(_, p)| crate::regions::area(p) > 0.)
    {
        let mut boundaries = vec![wires[i].clone()];
        for (j, hole) in loops
            .iter()
            .enumerate()
            .filter(|(_, p)| crate::regions::area(p) < 0.)
        {
            if crate::regions::contains(hole[0], outer) {
                boundaries.push(wires[j].clone());
            }
        }
        shell.push(builder::try_attach_plane(&boundaries).map_err(|e| e.to_string())?);
    }
    if shell.is_empty() {
        Ok(Geometry::default())
    } else {
        crate::model::tessellate(&shell)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Model, Shape, evaluate_scene};
    fn measure(g: &Geometry) -> f64 {
        g.triangles
            .iter()
            .map(|t| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
            .sum()
    }
    #[test]
    fn ordered_faces_and_volumes_roundtrip_and_undo() {
        for solid in [false, true] {
            let mut a = Body::new(Shape::Rectangle);
            a.id = 1;
            a.width = 4.;
            a.extrusion = if solid { 2. } else { 0. };
            let mut b = Body::new(Shape::Rectangle);
            b.id = 2;
            b.position = [1.5, 0., 0.];
            b.extrusion = a.extrusion;
            let source = vec![a, b];
            let scene = evaluate_scene(&source).unwrap();
            let mode = if solid {
                SelectionMode::Volume
            } else {
                SelectionMode::Face
            };
            let element = if solid { None } else { Some(0) };
            for kind in [Kind::Union, Kind::Subtract, Kind::Intersect] {
                let mut feature = Body::new(Shape::Rectangle);
                feature.boolean = Some(Boolean::new(
                    kind,
                    &[(1, element), (2, element)],
                    mode,
                    &scene,
                ));
                let mut model = Model::from_bodies(source.clone());
                let id = model.commit(feature.clone()).unwrap();
                let evaluated = evaluate_scene(&model.bodies).unwrap();
                assert!(evaluated[0].1.triangles.is_empty());
                assert!(evaluated[1].1.triangles.is_empty());
                assert!(!evaluated[2].1.triangles.is_empty());
                assert_eq!(evaluated[2].1.is_solid, solid);
                if solid {
                    use truck_modeling::EuclideanSpace;
                    let volume: f64 = evaluated[2]
                        .1
                        .triangles
                        .iter()
                        .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                        .sum();
                    let expected = match kind {
                        Kind::Union => 18.,
                        Kind::Subtract => 10.,
                        Kind::Intersect => 6.,
                    };
                    assert!(
                        (volume - expected).abs() < 1e-6,
                        "{kind:?}: volume {volume}"
                    );
                    crate::components::boundary_shell(&evaluated[2].1, true).unwrap();
                }
                if !solid {
                    let expected = match kind {
                        Kind::Union => 9.,
                        Kind::Subtract => 5.,
                        Kind::Intersect => 3.,
                    };
                    assert!((measure(&evaluated[2].1) - expected).abs() < 1e-6);
                }
                let project = crate::project::Project::new(
                    model.bodies.clone(),
                    crate::viewport::Camera::default(),
                    true,
                    Some(id),
                );
                let restored: crate::project::Project =
                    serde_json::from_str(&serde_json::to_string(&project).unwrap()).unwrap();
                restored.validate().unwrap();
                let file = tempfile::NamedTempFile::new().unwrap();
                project.save(file.path()).unwrap();
                let loaded = crate::project::Project::load(file.path()).unwrap();
                assert_eq!(loaded.bodies, model.bodies);
                if solid {
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

                model.undo();
                assert_eq!(model.bodies, source);
                model.redo();
                assert_eq!(model.bodies.len(), 3);
                feature.id = id;
                let op = feature.boolean.as_mut().unwrap();
                op.operands.swap(0, 1);
                op.topology.swap(0, 1);
                op.keep_tools = true;
                model.commit(feature).unwrap();
                let swapped = evaluate_scene(&model.bodies).unwrap();
                assert!(!swapped[0].1.triangles.is_empty());
                if !solid && kind == Kind::Subtract {
                    assert!((measure(&swapped[2].1) - 1.).abs() < 1e-6);
                }
            }
        }
    }
    #[test]
    fn empty_disconnected_and_contained_results_preserve_inputs_on_error() {
        for solid in [false, true] {
            let mut a = Body::new(Shape::Rectangle);
            a.id = 1;
            a.width = 4.;
            a.depth = 4.;
            a.extrusion = if solid { 2. } else { 0. };
            let mut b = Body::new(Shape::Rectangle);
            b.id = 2;
            b.extrusion = a.extrusion;
            for x in [0., 5.] {
                b.position[0] = x;
                let sources = vec![a.clone(), b.clone()];
                let scene = evaluate_scene(&sources).unwrap();
                let element = if solid { None } else { Some(0) };
                let mode = if solid {
                    SelectionMode::Volume
                } else {
                    SelectionMode::Face
                };
                for kind in [Kind::Union, Kind::Intersect, Kind::Subtract] {
                    let mut result = Body::new(Shape::Rectangle);
                    result.boolean = Some(Boolean::new(
                        kind,
                        &[(1, element), (2, element)],
                        mode,
                        &scene,
                    ));
                    let mut model = Model::from_bodies(sources.clone());
                    model.commit(result).unwrap();
                    let evaluated = evaluate_scene(&model.bodies).unwrap();
                    assert_eq!(
                        evaluated[2].1.triangles.is_empty(),
                        x == 5. && kind == Kind::Intersect
                    );
                    if solid && !evaluated[2].1.triangles.is_empty() {
                        crate::components::boundary_shell(&evaluated[2].1, true).unwrap();
                    }
                    let before = model.bodies.clone();
                    assert!(model.delete(1).is_err());
                    assert_eq!(model.bodies, before);
                }
            }
            if !solid {
                b.position[2] = 1.;
                let sources = vec![a, b];
                let scene = evaluate_scene(&sources).unwrap();
                let mut result = Body::new(Shape::Rectangle);
                result.boolean = Some(Boolean::new(
                    Kind::Subtract,
                    &[(1, Some(0)), (2, Some(0))],
                    SelectionMode::Face,
                    &scene,
                ));
                let mut model = Model::from_bodies(sources.clone());
                assert!(model.commit(result).is_err());
                assert_eq!(model.bodies, sources);
            }
        }
    }
}
