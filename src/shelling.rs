//! Exact inward/outward shelling of source prisms, with a removable planar opening.
use crate::{
    edge_treatments::FaceTarget,
    model::{Body, Geometry, Shape},
};
use serde::{Deserialize, Serialize};
use truck_modeling::{
    EuclideanSpace, InnerSpace, Matrix4, Point3, Transform, Vector3, Wire, builder,
};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvaluatedShell {
    pub thickness: f64,
    pub outward: bool,
    pub closed: bool,
}
impl EvaluatedShell {
    pub fn validate(self) -> Result<(), String> {
        if !self.thickness.is_finite() || self.thickness <= 1e-5 || self.thickness > 10000. {
            return Err(
                "Shell thickness must be finite, positive and at most 10000 local units.".into(),
            );
        }
        Ok(())
    }
    pub fn new(g: &Geometry) -> Result<Self, String> {
        let planes = crate::components::planar_supports(g)?;
        let center = g.points.iter().map(|p| p.to_vec()).sum::<Vector3>() / g.points.len() as f64;
        let thickness = planes
            .iter()
            .map(|(n, d)| (*d - n.dot(center)) * 0.1)
            .fold(0.2, f64::min);
        let operation = Self {
            thickness,
            outward: false,
            closed: false,
        };
        operation.validate()?;
        Ok(operation)
    }
}

/// Offset retained convex planes, then reuse exact planar solid subtraction.
pub(crate) fn evaluated(
    g: &Geometry,
    openings: &[usize],
    operation: EvaluatedShell,
) -> Result<Geometry, String> {
    operation.validate()?;
    let planes = crate::components::planar_supports(g)?;
    if openings.is_empty() || openings.iter().any(|face| *face >= planes.len()) {
        return Err("Select valid planar opening faces for Shell.".into());
    }
    if g.shell
        .as_ref()
        .is_none_or(|shell| shell.connected_components().len() != 1)
    {
        return Err("Evaluated Shell requires one retained convex planar solid.".into());
    }
    let offset = |inner: bool| {
        planes
            .iter()
            .copied()
            .enumerate()
            .map(|(face, (n, d))| {
                let opening = !operation.closed && openings.contains(&face);
                let distance = if inner {
                    d - if !operation.outward && !opening {
                        operation.thickness
                    } else {
                        0.
                    }
                } else {
                    d + if operation.outward && !opening {
                        operation.thickness
                    } else {
                        0.
                    }
                };
                (face, (n, distance))
            })
            .collect::<Vec<_>>()
    };
    let outer = if operation.outward {
        crate::edge_treatments::convex_from_planes(&offset(false))?
    } else {
        g.clone()
    };
    let inner = if operation.outward {
        g.clone()
    } else {
        crate::edge_treatments::convex_from_planes(&offset(true))?
    };
    let outer = crate::edge_treatments::planar_regions(outer)?;
    let inner = crate::edge_treatments::planar_regions(inner)?;
    let result = crate::edge_treatments::subtract_convex(
        &outer,
        inner
            .shell
            .as_ref()
            .ok_or("Missing shell cavity boundary.")?,
        outer.surface_mapping()?,
    )?
    .ok_or("Shell requires exact planar boundaries.")?;
    let result = crate::edge_treatments::planar_regions(result)?;
    crate::components::solid_parts(result.shell.as_ref().ok_or("Shell leaves no material.")?)?;
    crate::operators::geometry(&crate::operators::solid(&result)?)
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Hollow {
    pub opening: FaceTarget,
    pub thickness: f64,
    #[serde(default)]
    pub outward: bool,
    #[serde(default)]
    pub closed: bool,
}
impl Hollow {
    pub fn new(body: &Body, opening: FaceTarget) -> Result<Self, String> {
        let (working, _) = frame(body, opening);
        let cross = if body.shape == Shape::Rectangle {
            working.width.min(working.depth) / 2.
        } else if body.shape == Shape::Semicircle {
            working.width / 4.
        } else {
            working.width / 2.
        };
        let operation = Self {
            opening,
            closed: body.hollow.as_ref().is_some_and(|op| op.closed),
            outward: body.hollow.as_ref().is_some_and(|op| op.outward),
            thickness: body
                .hollow
                .as_ref()
                .map_or((cross.min(working.extrusion) * 0.1).min(0.2), |operation| {
                    operation.thickness
                }),
        };
        let mut candidate = body.clone();
        candidate.hollow = Some(operation.clone());
        candidate.validate()?;
        Ok(operation)
    }
}
fn frame(body: &Body, opening: FaceTarget) -> (Body, Matrix4) {
    let (normal, _) = opening.plane(body);
    let u = if normal.z > 0.5 {
        Vector3::unit_x()
    } else if normal.z < -0.5 {
        -Vector3::unit_x()
    } else {
        Vector3::unit_z()
    };
    let v = normal.cross(u);
    let extent = |axis: Vector3| {
        axis.x.abs() * body.width + axis.y.abs() * body.depth + axis.z.abs() * body.extrusion
    };
    let mut working = Body::new(body.shape);
    working.width = extent(u);
    working.depth = extent(v);
    working.extrusion = extent(normal);
    let start = Vector3::new(0., 0., body.extrusion / 2.) - normal * working.extrusion / 2.;
    let matrix = crate::regions::transform(body)
        * Matrix4::from_cols(
            u.extend(0.),
            v.extend(0.),
            normal.extend(0.),
            start.extend(1.),
        );
    (working, matrix)
}
pub fn validate(body: &Body) -> Result<(), String> {
    let Some(operation) = &body.hollow else {
        return Ok(());
    };
    if body.shape == Shape::Line
        || body.extrusion <= 0.
        || !operation.opening.supported(body)
        || body.bore.is_some()
        || body.boolean.is_some()
        || !body.edge_treatments.is_empty()
        || crate::components::has_source_edits(body)
    {
        return Err("Shell currently requires a source box, cylinder or semicylinder without edge treatments, source deformation or Boolean operations.".into());
    }
    let (working, _) = frame(body, operation.opening);
    let maximum = if operation.outward {
        100.00001
    } else if body.shape == Shape::Rectangle {
        (working.width.min(working.depth) / 2.)
            .min(working.extrusion / if operation.closed { 2. } else { 1. })
    } else if body.shape == Shape::Semicircle {
        (working.width / 4.).min(working.extrusion / if operation.closed { 2. } else { 1. })
    } else {
        (working.width / 2.).min(working.extrusion / if operation.closed { 2. } else { 1. })
    };
    if !operation.thickness.is_finite()
        || operation.thickness <= 1e-5
        || operation.thickness >= maximum - 1e-5
    {
        return Err(format!(
            "Shell thickness must be finite, greater than 0.00001 and less than {maximum:.5} local units, preserving valid wall geometry."
        ));
    }
    Ok(())
}
pub fn evaluate(body: &Body, partitioned: bool) -> Result<Geometry, String> {
    if partitioned {
        return Err("Shell does not yet support partitioned source profiles.".into());
    }
    validate(body)?;
    let operation = body.hollow.as_ref().ok_or("Missing shell operation.")?;
    geometry(body, operation.thickness, 0., 0.)
}
// Back-wall cuts can change back thickness independently of the side walls.
// Keep this derived value out of authored Shell parameters/history.
fn geometry(body: &Body, back: f64, trim_back: f64, trim_opening: f64) -> Result<Geometry, String> {
    let operation = body.hollow.as_ref().ok_or("Missing shell operation.")?;
    let t = operation.thickness;
    let (mut working, mut matrix) = frame(body, operation.opening);
    let inset = if operation.outward { 0. } else { t };
    let inner = section(&working, inset);
    let outer = section(&working, if operation.outward { -t } else { 0. });
    if operation.outward {
        matrix = matrix * Matrix4::from_translation(Vector3::unit_z() * (trim_back - t));
        working.extrusion += t * if operation.closed { 2. } else { 1. } - trim_back;
    }
    working.extrusion -= trim_opening;
    let core_only = working.extrusion <= 1e-7;
    let inner_top = working.extrusion - if operation.closed { t } else { 0. };
    let shell = if operation.closed {
        let cap = builder::try_attach_plane(&[outer]).map_err(|e| e.to_string())?;
        let mut shell = crate::model::extrude_face(&cap, working.extrusion)?;
        let inner =
            builder::transformed(&inner, Matrix4::from_translation(Vector3::unit_z() * back));
        let cap = builder::try_attach_plane(&[inner]).map_err(|e| e.to_string())?;
        shell.extend(
            crate::model::extrude_face(&cap, inner_top - back)?
                .iter()
                .map(|face| face.inverse()),
        );
        crate::operators::solid(&Geometry {
            shell: Some(shell.clone()),
            ..Default::default()
        })?;
        shell
    } else if core_only {
        let cap = builder::try_attach_plane(&[inner]).map_err(|e| e.to_string())?;
        crate::model::extrude_face(&cap, back)?
    } else {
        crate::operators::pocket_shell(
            outer,
            inner.inverse(),
            working.extrusion,
            FaceTarget::Top,
            working.extrusion - back,
        )?
    };
    let mut g = crate::model::tessellate(&shell)?;
    if body.shape == Shape::Circle {
        let outer_radius = working.width / 2. + if operation.outward { t } else { 0. };
        let inner_radius = working.width / 2. - inset;
        let roles: Vec<_> = (0..g.face_count)
            .map(|face| {
                let points: Vec<_> = g
                    .triangles
                    .iter()
                    .zip(&g.triangle_faces)
                    .filter(|(_, f)| **f == face)
                    .flat_map(|(t, _)| t.iter())
                    .collect();
                if points.iter().all(|p| p.z.abs() < 1e-6) {
                    0
                } else if points
                    .iter()
                    .all(|p| (p.z - working.extrusion).abs() < 1e-6)
                {
                    1
                } else if points.iter().all(|p| (p.z - back).abs() < 1e-6) {
                    2
                } else if operation.closed && points.iter().all(|p| (p.z - inner_top).abs() < 1e-6)
                {
                    5
                } else if points
                    .iter()
                    .all(|p| (p.x.hypot(p.y) - outer_radius).abs() < 1e-5)
                {
                    3
                } else {
                    4
                }
            })
            .collect();
        crate::edge_treatments::merge_surfaces(&mut g, &roles);
        for z in if core_only {
            Vec::new()
        } else {
            vec![0., working.extrusion]
        } {
            let r = outer_radius;
            g.circles.push(crate::model::CircularEdge {
                center: Point3::new(0., 0., z),
                u: Vector3::unit_x() * r,
                v: Vector3::unit_y() * r,
            });
        }
        for z in if core_only {
            [0., back]
        } else {
            [back, inner_top]
        } {
            let r = inner_radius;
            g.circles.push(crate::model::CircularEdge {
                center: Point3::new(0., 0., z),
                u: Vector3::unit_x() * r,
                v: Vector3::unit_y() * r,
            });
        }
    }
    crate::components::map_geometry(&mut g, matrix);
    g.is_solid = true;
    Ok(g)
}

// Exact parallel offsets of the original cross-section, including the chord.
fn section(working: &Body, inset: f64) -> Wire {
    if working.shape == Shape::Semicircle {
        let r = working.width / 2. - inset;
        let x = (r * r - inset * inset).sqrt();
        let a = builder::vertex(Point3::new(x, inset, 0.));
        let b = builder::vertex(Point3::new(-x, inset, 0.));
        vec![
            builder::circle_arc(&a, &b, Point3::new(0., r, 0.)),
            builder::line(&b, &a),
        ]
        .into()
    } else {
        let mut inner = working.clone();
        inner.width -= 2. * inset;
        inner.depth -= 2. * inset;
        crate::model::profile_wire(&inner)
    }
}

/// Retracting an annular opening preserves the central material below its cavity.
pub(crate) fn retract_opening(
    body: &Body,
    face: FaceTarget,
    distance: f64,
) -> Result<Option<Geometry>, String> {
    let Some(operation) = &body.hollow else {
        return Ok(None);
    };
    if operation.closed || distance >= 0. || face != operation.opening {
        return Ok(None);
    }
    geometry(body, operation.thickness, 0., -distance).map(Some)
}

/// Subtract a full back-cap slice without a coplanar Boolean calculation.
pub(crate) fn retract_back(
    body: &Body,
    face: FaceTarget,
    distance: f64,
) -> Result<Option<Geometry>, String> {
    let Some(operation) = &body.hollow else {
        return Ok(None);
    };
    if operation.closed || distance >= 0. || face != operation.opening.opposite() {
        return Ok(None);
    }
    let (working, _) = frame(body, operation.opening);
    if operation.outward {
        return if -distance >= working.extrusion + operation.thickness - 1e-7 {
            Ok(Some(Geometry::default()))
        } else {
            geometry(
                body,
                (operation.thickness + distance).max(0.),
                -distance,
                0.,
            )
            .map(Some)
        };
    }
    if -distance >= working.extrusion - 1e-7 {
        return Ok(Some(Geometry::default()));
    }
    let mut source = body.clone();
    source.hollow = None;
    source.face_extrusions.clear();
    source.component_edits.clear();
    let world = crate::regions::transform(body).transform_vector(face.plane(body).0 * distance);
    let source = crate::components::offset_source_face(
        &source,
        face,
        -truck_modeling::InnerSpace::magnitude(world),
    )?;
    let mut source = source;
    source.hollow = Some(operation.clone());
    geometry(&source, (operation.thickness + distance).max(0.), 0., 0.).map(Some)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        model::{Model, evaluate},
        project::Project,
        viewport::Camera,
    };
    use truck_modeling::{EuclideanSpace, InnerSpace, Solid, SquareMatrix, Transform};
    #[test]
    fn evaluated_planar_shells_offset_treated_boolean_and_moved_solids() {
        use crate::{
            booleans, components,
            face_operations::{self, FaceEdit},
            model::evaluate_scene,
            selection::SelectionMode,
        };
        let volume = |g: &Geometry| {
            g.triangles
                .iter()
                .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                .sum::<f64>()
        };
        for kind in 0..5 {
            for affine in [false, true] {
                if kind == 2 && affine {
                    continue;
                }
                let mut body = Body::new(Shape::Rectangle);
                body.id = 1;
                body.width = 6.;
                body.depth = 5.;
                body.extrusion = 4.;
                if affine {
                    body.position = [2., -1., 3.];
                    body.rotation = [11., 17., 23.];
                    body.scale = [1.5, 0.75, 2.];
                }
                if kind == 1 || kind == 4 {
                    let mut modifier = crate::edge_treatments::Modifier::new(
                        crate::edge_treatments::Kind::Chamfer,
                        FaceTarget::Top,
                        &body,
                    );
                    modifier.amount = 0.3;
                    if kind == 1 {
                        modifier.edge = Some(FaceTarget::Right);
                    }
                    body.edge_treatments.push(modifier);
                }
                if kind == 3 {
                    let g = evaluate(&body).unwrap();
                    let face = (0..g.face_count)
                        .find(|f| {
                            crate::edge_treatments::target(&body, &g, *f) == Ok(FaceTarget::Top)
                        })
                        .unwrap();
                    body = components::transformed(
                        &body,
                        &g,
                        SelectionMode::Face,
                        Some(face),
                        Matrix4::from_translation(
                            crate::regions::transform(&body)
                                .transform_vector(Vector3::unit_z() * 0.25),
                        ),
                    )
                    .unwrap();
                }
                let mut bodies = vec![body.clone()];
                if kind == 2 {
                    let mut tool = body.clone();
                    tool.id = 2;
                    tool.width = 4.;
                    tool.depth = 4.;
                    tool.position[0] = 1.;
                    bodies.push(tool);
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
                }
                let id = bodies.last().unwrap().id;
                let owner = bodies.last().unwrap();
                let placement = crate::regions::transform(owner);
                let inverse = placement.invert().unwrap();
                let scene = evaluate_scene(&bodies).unwrap();
                let g = &scene.last().unwrap().1;
                let mut local = g.clone();
                components::map_geometry(&mut local, inverse);
                let planes = components::planar_supports(&local).unwrap();
                EvaluatedShell::new(&local).unwrap();
                for side in [false, true] {
                    for outward in [false, true] {
                        for closed in [false, true] {
                            let opening = planes
                                .iter()
                                .position(|(n, _)| if side { n.x > 0.99 } else { n.z > 0.99 })
                                .unwrap();
                            let operation = EvaluatedShell {
                                thickness: 0.1,
                                outward,
                                closed,
                            };
                            let changes = face_operations::selected(
                                &bodies,
                                &scene,
                                &[(id, Some(opening))],
                                FaceEdit::Shell(operation),
                            )
                            .unwrap();
                            let mut model = Model::from_bodies(bodies.clone());
                            model.commit_changes(&changes).unwrap();
                            let output = evaluate_scene(&model.bodies).unwrap();
                            let result = &output.last().unwrap().1;
                            assert!(result.is_solid);
                            assert!(result.shell.as_ref().unwrap().is_geometric_consistent());
                            assert_eq!(result.shell.as_ref().unwrap().len(), result.face_count);
                            assert_eq!(
                                result.face_count,
                                planes.len() * 2 - if closed { 0 } else { 1 }
                            );
                            assert_eq!(
                                result.shell.as_ref().unwrap().connected_components().len(),
                                if closed { 2 } else { 1 }
                            );
                            components::solid_parts(result.shell.as_ref().unwrap()).unwrap();
                            if kind == 1 && !affine && !side && !outward && !closed {
                                let input = planes.iter().find(|(n, _)| n.x > 0.99).unwrap();
                                let inner = components::straight_planar_supports(result)
                                    .unwrap()
                                    .iter()
                                    .position(|(n, d)| {
                                        (*n + input.0).magnitude() < 1e-7
                                            && (*d + input.1 - 0.1).abs() < 1e-7
                                    })
                                    .unwrap();
                                let changed = components::transformed(
                                    &changes[0],
                                    result,
                                    SelectionMode::Face,
                                    Some(inner),
                                    Matrix4::from_translation(-input.0 * 0.02),
                                )
                                .unwrap();
                                let mut edited = model.bodies.clone();
                                *edited.last_mut().unwrap() = changed;
                                let edited_scene = evaluate_scene(&edited).unwrap();
                                assert!(volume(&edited_scene.last().unwrap().1) > volume(result));
                                let dir = tempfile::tempdir().unwrap();
                                let path = dir.path().join("shell-wall-edit.osk");
                                Project::new(edited.clone(), Camera::default(), true, Some(id))
                                    .save(&path)
                                    .unwrap();
                                assert_eq!(Project::load(&path).unwrap().bodies, edited);
                            }
                            let mut working = result.clone();
                            components::map_geometry(&mut working, inverse);
                            let after = components::straight_planar_supports(&working).unwrap();
                            for (face, (n, d)) in planes.iter().enumerate() {
                                let open = !closed && face == opening;
                                let outer = *d + if outward && !open { 0.1 } else { 0. };
                                assert!(
                                    after
                                        .iter()
                                        .any(|(other, offset)| (*n - *other).magnitude() < 1e-6
                                            && (outer - *offset).abs() < 1e-6)
                                );
                                if !open {
                                    let inner = *d - if outward { 0. } else { 0.1 };
                                    assert!(
                                        after
                                            .iter()
                                            .any(|(other, offset)| (*n + *other).magnitude()
                                                < 1e-6
                                                && (inner + *offset).abs() < 1e-6)
                                    );
                                }
                            }
                            if kind != 1 && kind != 4 {
                                let (w, d, h) = (
                                    if kind == 2 { 4. } else { 6. },
                                    if kind == 2 { 4. } else { 5. },
                                    if kind == 3 { 4.25 } else { 4. },
                                );
                                let expected = if outward {
                                    (w + if !closed && side { 0.1 } else { 0.2 })
                                        * (d + 0.2)
                                        * (h + if !closed && !side { 0.1 } else { 0.2 })
                                        - w * d * h
                                } else {
                                    w * d * h
                                        - (w - if !closed && side { 0.1 } else { 0.2 })
                                            * (d - 0.2)
                                            * (h - if !closed && !side { 0.1 } else { 0.2 })
                                };
                                assert!(
                                    (volume(result) - expected * if affine { 2.25 } else { 1. })
                                        .abs()
                                        < 1e-5,
                                    "kind={kind} side={side} outward={outward} closed={closed}"
                                );
                            }
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
                            let path = dir.path().join("evaluated-shell.osk");
                            let project = Project::new(
                                model.bodies.clone(),
                                Camera::default(),
                                true,
                                Some(id),
                            );
                            project.save(&path).unwrap();
                            assert_eq!(Project::load(&path).unwrap().bodies, model.bodies);
                            let mut old = serde_json::to_value(&project).unwrap();
                            old["schema_version"] = 35.into();
                            old["modeling_version"] = 35.into();
                            assert!(
                                serde_json::from_value::<Project>(old)
                                    .unwrap()
                                    .validate()
                                    .unwrap_err()
                                    .contains("version 36")
                            );
                            let before = model.bodies.clone();
                            let mut invalid = changes[0].clone();
                            invalid.component_edits.last_mut().unwrap().face_edit =
                                Some(FaceEdit::Shell(EvaluatedShell {
                                    thickness: 100.,
                                    outward: false,
                                    closed,
                                }));
                            assert!(model.commit(invalid).is_err());
                            assert_eq!(model.bodies, before);
                            model.undo();
                            assert_eq!(model.bodies, bodies);
                            model.redo();
                            assert_eq!(model.bodies, before);
                            let mut removed = changes[0].clone();
                            removed.component_edits.pop();
                            model.commit(removed).unwrap();
                            assert!(
                                (volume(&evaluate_scene(&model.bodies).unwrap().last().unwrap().1)
                                    - volume(g))
                                .abs()
                                    < 1e-6
                            );
                            if kind == 1
                                && !affine
                                && !side
                                && !outward
                                && !closed
                                && let Some(path) =
                                    std::env::var_os("OSK_EVALUATED_SHELL_REVIEW_PROJECT")
                            {
                                let mut review =
                                    Project::new(bodies.clone(), Camera::default(), true, Some(id));
                                review.selection_mode = SelectionMode::Face;
                                review.selected_element = Some(opening);
                                review.selection = vec![(id, Some(opening))];
                                review.save(std::path::Path::new(&path)).unwrap();
                            }
                        }
                    }
                }
                assert!(
                    evaluated(
                        &local,
                        &(0..planes.len()).collect::<Vec<_>>(),
                        EvaluatedShell {
                            thickness: 0.1,
                            outward: false,
                            closed: false
                        }
                    )
                    .is_err()
                );
            }
        }
        assert!(
            EvaluatedShell {
                thickness: f64::NAN,
                outward: false,
                closed: false
            }
            .validate()
            .is_err()
        );
    }

    #[test]
    fn primitive_shells_preserve_exact_thickness_openings_history_and_exports() {
        for shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            for opening in [
                FaceTarget::Top,
                FaceTarget::Bottom,
                FaceTarget::Left,
                FaceTarget::Right,
                FaceTarget::Front,
                FaceTarget::Back,
            ] {
                let mut b = Body::new(shape);
                b.id = 1;
                b.width = 6.;
                b.depth = 4.;
                b.extrusion = 5.;
                b.position = [3., -2., 1.];
                b.rotation = [15., 25., 35.];
                b.scale = [1.5, 0.75, 2.];
                if !opening.supported(&b) {
                    continue;
                }
                let original = b.clone();
                b.hollow = Some(Hollow {
                    opening,
                    thickness: 0.25,
                    outward: false,
                    closed: false,
                });
                assert_eq!(
                    Hollow::new(&b, opening.opposite()).unwrap().thickness,
                    0.25,
                    "Changing the opening preserves the existing thickness"
                );
                let g = evaluate(&b).unwrap();
                assert!(g.is_solid);
                Solid::try_new(vec![g.shell.clone().unwrap()]).unwrap();
                let (working, matrix) = frame(&b, opening);
                let inverse = matrix.invert().unwrap();
                let volume = g
                    .triangles
                    .iter()
                    .map(|t| t.map(|p| inverse.transform_point(p)))
                    .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                    .sum::<f64>();
                let t = 0.25;
                let h = working.extrusion;
                let r = working.width / 2.;
                let expected = match shape {
                    Shape::Rectangle => {
                        working.width * working.depth * h
                            - (working.width - 2. * t) * (working.depth - 2. * t) * (h - t)
                    }
                    Shape::Circle => std::f64::consts::PI * (r * r * h - (r - t).powi(2) * (h - t)),
                    Shape::Semicircle => {
                        let q = r - t;
                        let inner = q * q * (t / q).acos() - t * (q * q - t * t).sqrt();
                        std::f64::consts::PI * r * r * h / 2. - inner * (h - t)
                    }
                    Shape::Line => unreachable!(),
                };
                assert!(
                    (volume - expected).abs() < 0.4,
                    "{shape:?} {opening:?}: {volume} vs {expected}"
                );
                if shape == Shape::Circle {
                    assert_eq!(
                        g.face_count, 5,
                        "Cylindrical patches must remain logical surfaces"
                    );
                    assert_eq!(g.circles.len(), 4);
                }
                if shape == Shape::Semicircle {
                    let x = ((r - t).powi(2) - t * t).sqrt();
                    assert!(
                        g.points.iter().any(|p| {
                            let p = inverse.transform_point(*p);
                            (p.x - x).abs() < 1e-6 && (p.y - t).abs() < 1e-6
                        }),
                        "Semicircle shell offsets its original arc and diameter, rather than translating a smaller half-disc"
                    );
                }
                if shape == Shape::Circle && opening == FaceTarget::Top {
                    let mut extended = b.clone();
                    extended
                        .face_extrusions
                        .push(crate::face_extrusions::Extrusion {
                            face: opening,
                            distance: 1.,
                        });
                    let output = evaluate(&extended).unwrap();
                    assert_eq!(
                        output.face_count,
                        g.face_count + 2,
                        "Extending an annular cap keeps the inner and outer walls separate"
                    );
                    Solid::try_new(vec![output.shell.unwrap()]).unwrap();
                }
                let mut retracted = b.clone();
                retracted
                    .face_extrusions
                    .push(crate::face_extrusions::Extrusion {
                        face: opening,
                        distance: -0.1,
                    });
                let cut = evaluate(&retracted).unwrap();
                Solid::try_new(vec![cut.shell.clone().unwrap()]).unwrap();
                let measured = cut
                    .triangles
                    .iter()
                    .map(|t| t.map(|p| inverse.transform_point(p)))
                    .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                    .sum::<f64>();
                let outer = if shape == Shape::Rectangle {
                    working.width * working.depth
                } else {
                    std::f64::consts::PI * r * r * if shape == Shape::Semicircle { 0.5 } else { 1. }
                };
                let inner = (outer * h - expected) / (h - t);
                assert!(
                    (measured - (expected - (outer - inner) * 0.1)).abs() < 0.4,
                    "Retracting the rim keeps the original cavity floor"
                );
                for amount in [h - 0.1, h, h + 1.] {
                    let mut rim = b.clone();
                    rim.face_extrusions.push(crate::face_extrusions::Extrusion {
                        face: opening,
                        distance: -amount,
                    });
                    let cut = evaluate(&rim).unwrap();
                    Solid::try_new(vec![cut.shell.clone().unwrap()]).unwrap();
                    let measured = cut
                        .triangles
                        .iter()
                        .map(|t| t.map(|p| inverse.transform_point(p)))
                        .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                        .sum::<f64>();
                    let expected = inner * t + (outer - inner) * (h - amount).max(0.);
                    assert!(
                        (measured - expected).abs() < 0.4,
                        "Annular rim retraction preserves the back core"
                    );
                }
                for amount in [0.1, 0.5, h + 1.] {
                    let mut back = b.clone();
                    back.face_extrusions
                        .push(crate::face_extrusions::Extrusion {
                            face: opening.opposite(),
                            distance: -amount,
                        });
                    let cut = evaluate(&back).unwrap();
                    if amount >= h {
                        assert!(cut.triangles.is_empty());
                        continue;
                    }
                    Solid::try_new(vec![cut.shell.clone().unwrap()]).unwrap();
                    let measured = cut
                        .triangles
                        .iter()
                        .map(|t| t.map(|p| inverse.transform_point(p)))
                        .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                        .sum::<f64>();
                    let removed = outer * amount.min(t) + (outer - inner) * (amount - t).max(0.);
                    assert!(
                        (measured - (expected - removed)).abs() < 0.4,
                        "Back-cap cuts preserve side thickness and open the cavity after crossing its floor"
                    );
                    if shape == Shape::Circle {
                        let old = matrix.transform_point(Point3::new(0., 0., t));
                        assert_eq!(
                            cut.circles
                                .iter()
                                .any(|c| (c.center - old).magnitude() < 1e-6),
                            amount < t,
                            "Circular references must follow the actual back/floor geometry"
                        );
                    }
                }
                let mut model = Model::from_bodies(vec![original.clone()]);
                model.edit(b.clone()).unwrap();
                model.finish_edit();
                model.undo();
                assert_eq!(model.bodies, vec![original]);
                model.redo();
                assert_eq!(model.bodies, vec![b.clone()]);
                let mut thicker = b.clone();
                thicker.hollow.as_mut().unwrap().thickness = 0.4;
                model.edit(thicker.clone()).unwrap();
                model.finish_edit();
                model.undo();
                assert_eq!(model.bodies, vec![b.clone()]);
                model.redo();
                assert_eq!(model.bodies, vec![thicker.clone()]);
                let mut restored = thicker.clone();
                restored.hollow = None;
                model.edit(restored).unwrap();
                model.finish_edit();
                model.undo();
                assert_eq!(model.bodies, vec![thicker]);
                model.undo();
                assert_eq!(model.bodies, vec![b.clone()]);
                let mut invalid = b.clone();
                invalid.hollow.as_mut().unwrap().thickness = 3.;
                assert!(model.edit(invalid).is_err());
                assert_eq!(model.bodies, vec![b.clone()]);
                let dir = tempfile::tempdir().unwrap();
                let path = dir.path().join("shell.osk");
                Project::new(vec![b.clone()], Camera::default(), true, Some(1))
                    .save(&path)
                    .unwrap();
                assert_eq!(Project::load(&path).unwrap().bodies, vec![b.clone()]);
                if shape == Shape::Circle && opening == FaceTarget::Top {
                    let mut old = serde_json::to_value(Project::new(
                        vec![b.clone()],
                        Camera::default(),
                        true,
                        Some(1),
                    ))
                    .unwrap();
                    old["schema_version"] = 13.into();
                    old["modeling_version"] = 13.into();
                    std::fs::write(&path, serde_json::to_vec(&old).unwrap()).unwrap();
                    assert!(
                        Project::load(&path)
                            .err()
                            .unwrap()
                            .contains("Shell operations require .osk version 14")
                    );
                    old["bodies"][0].as_object_mut().unwrap().remove("hollow");
                    std::fs::write(&path, serde_json::to_vec(&old).unwrap()).unwrap();
                    assert!(
                        Project::load(&path).unwrap().bodies[0].hollow.is_none(),
                        "Legacy documents restore the original solid"
                    );
                }
                for format in [
                    crate::export::Format::Stl,
                    crate::export::Format::Glb,
                    crate::export::Format::Step,
                    crate::export::Format::Dxf,
                ] {
                    assert!(
                        !crate::export::encode(&[b.clone()], format)
                            .unwrap()
                            .is_empty()
                    );
                }
                let points = &g.points;
                for (index, p) in points.iter().enumerate() {
                    let p = inverse.transform_point(*p);
                    assert!(
                        p.z >= -1e-6 && p.z <= h + 1e-6,
                        "vertex {index} leaves source bounds"
                    );
                }
            }
        }
        let mut b = Body::new(Shape::Rectangle);
        b.id = 1;
        b.extrusion = 3.;
        b.hollow = Some(Hollow {
            opening: FaceTarget::Top,
            thickness: f64::NAN,
            outward: false,
            closed: false,
        });
        assert!(b.validate().is_err());
    }
    #[test]
    fn outward_shells_keep_cavity_and_opening_with_exact_offsets() {
        for shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            for opening in [
                FaceTarget::Top,
                FaceTarget::Bottom,
                FaceTarget::Left,
                FaceTarget::Right,
                FaceTarget::Front,
                FaceTarget::Back,
            ] {
                let mut body = Body::new(shape);
                body.id = 1;
                body.width = 6.;
                body.depth = 4.;
                body.extrusion = 5.;
                body.position = [2., -3., 1.];
                body.rotation = [15., 25., 35.];
                body.scale = [1.5, 0.75, 2.];
                if !opening.supported(&body) {
                    continue;
                }
                let original = body.clone();
                let t = 0.4;
                body.hollow = Some(Hollow {
                    opening,
                    thickness: t,
                    outward: true,
                    closed: false,
                });
                assert!(Hollow::new(&body, opening.opposite()).unwrap().outward);
                let (working, matrix) = frame(&body, opening);
                let inv = matrix.invert().unwrap();
                let r = working.width / 2.;
                let h = working.extrusion;
                let inner = match shape {
                    Shape::Rectangle => working.width * working.depth,
                    Shape::Circle => std::f64::consts::PI * r * r,
                    Shape::Semicircle => std::f64::consts::PI * r * r / 2.,
                    _ => unreachable!(),
                };
                let outer = match shape {
                    Shape::Rectangle => (working.width + 2. * t) * (working.depth + 2. * t),
                    Shape::Circle => std::f64::consts::PI * (r + t).powi(2),
                    Shape::Semicircle => {
                        let q = r + t;
                        q * q * (-t / q).acos() + t * (q * q - t * t).sqrt()
                    }
                    _ => unreachable!(),
                };
                let expected = outer * (h + t) - inner * h;
                let measure = |g: &Geometry| {
                    g.triangles
                        .iter()
                        .map(|tri| tri.map(|p| inv.transform_point(p)))
                        .map(|tri| tri[0].to_vec().dot(tri[1].to_vec().cross(tri[2].to_vec())) / 6.)
                        .sum::<f64>()
                };
                let g = evaluate(&body).unwrap();
                Solid::try_new(vec![g.shell.clone().unwrap()]).unwrap();
                assert!(
                    (measure(&g) - expected).abs() < 0.6,
                    "{shape:?} {opening:?}: {} vs {expected}",
                    measure(&g)
                );
                let points: Vec<_> = g.points.iter().map(|p| inv.transform_point(*p)).collect();
                assert!(points.iter().all(|p| p.z >= -t - 1e-6 && p.z <= h + 1e-6));
                assert!(
                    points.iter().any(|p| p.z.abs() < 1e-6),
                    "Original back plane remains the cavity floor"
                );
                if shape == Shape::Semicircle {
                    let x = ((r + t).powi(2) - t * t).sqrt();
                    assert!(
                        points
                            .iter()
                            .any(|p| (p.x - x).abs() < 1e-6 && (p.y + t).abs() < 1e-6)
                    );
                }
                if shape == Shape::Circle {
                    assert_eq!(g.face_count, 5);
                    assert_eq!(g.circles.len(), 4);
                    for (z, radius) in [(-t, r + t), (h, r + t), (0., r), (h, r)] {
                        assert!(g.circles.iter().any(|c| {
                            let c0 = inv.transform_point(c.center);
                            (c0.z - z).abs() < 1e-6
                                && (inv.transform_vector(c.u).magnitude() - radius).abs() < 1e-6
                        }));
                    }
                }
                // Both outer caps retain semantic picking, including the shifted back cap.
                for face in [opening, opening.opposite()] {
                    assert!(
                        (0..g.face_count)
                            .any(|i| crate::face_extrusions::target(&body, &g, i) == Ok(face))
                    );
                    let support = crate::operators::support_frame(&body, face).unwrap();
                    let (n, d) = face.evaluated_plane(&body);
                    let expected_origin =
                        crate::regions::transform(&body).transform_point(Point3::from_vec(n * d));
                    assert!(
                        (Point3::from_vec(support.w.truncate()) - expected_origin).magnitude()
                            < 1e-8
                    );
                    let mut profile = Body::new(Shape::Circle);
                    profile.id = 2;
                    profile.width = 0.5;
                    profile.position = support.w.truncate().into();
                    profile.rotation =
                        crate::regions::rotation_angles(truck_modeling::Matrix3::from_cols(
                            support.x.truncate(),
                            support.y.truncate(),
                            support.z.truncate(),
                        ));
                    let mut bore = crate::operators::Bore::new(&body, face).unwrap();
                    bore.travel = -1.;
                    bore.extent = crate::operators::Extent::ThroughAll;
                    profile.bore = Some(bore.clone());
                    let travel = crate::operators::resolved_travel(
                        &profile,
                        &bore,
                        &[body.clone(), profile.clone()],
                        &[(body.id, g.clone())],
                    )
                    .unwrap();
                    let expected_travel = matrix
                        .transform_vector(Vector3::unit_z() * (h + t))
                        .magnitude();
                    assert!(
                        (travel + expected_travel).abs() < 1e-8,
                        "Through all includes the expanded back wall"
                    );
                    for distance in [0.1, -0.1, -0.7, -(h + t + 1.)] {
                        let mut edited = body.clone();
                        edited
                            .face_extrusions
                            .push(crate::face_extrusions::Extrusion { face, distance });
                        let cut = evaluate(&edited).unwrap_or_else(|e| {
                            panic!("{shape:?} {opening:?} {face:?} {distance}: {e}")
                        });
                        if distance < -(h + t) && face != opening {
                            assert!(cut.triangles.is_empty());
                            continue;
                        }
                        Solid::try_new(vec![cut.shell.clone().unwrap()]).unwrap();
                        let expected = if face == opening {
                            expected + (outer - inner) * distance.max(-(h + t))
                        } else if distance >= 0. {
                            expected + outer * distance
                        } else {
                            expected
                                - outer * (-distance).min(t)
                                - (outer - inner) * (-distance - t).max(0.)
                        };
                        assert!(
                            (measure(&cut) - expected).abs() < 0.7,
                            "{shape:?} {opening:?} {face:?} {distance}: {} vs {expected}",
                            measure(&cut)
                        );
                    }
                }
                let mut model = Model::from_bodies(vec![original.clone()]);
                model.edit(body.clone()).unwrap();
                model.finish_edit();
                model.undo();
                assert_eq!(model.bodies, vec![original]);
                model.redo();
                assert_eq!(model.bodies, vec![body.clone()]);
                let dir = tempfile::tempdir().unwrap();
                let path = dir.path().join("outward.osk");
                let project = Project::new(vec![body.clone()], Camera::default(), true, Some(1));
                project.save(&path).unwrap();
                assert_eq!(Project::load(&path).unwrap().bodies, vec![body.clone()]);
                let mut old = serde_json::to_value(project).unwrap();
                old["schema_version"] = 15.into();
                old["modeling_version"] = 15.into();
                std::fs::write(&path, serde_json::to_vec(&old).unwrap()).unwrap();
                assert!(
                    Project::load(&path)
                        .unwrap_err()
                        .contains("Outward shells require .osk version 16")
                );
                old["bodies"][0]["hollow"]
                    .as_object_mut()
                    .unwrap()
                    .remove("outward");
                std::fs::write(&path, serde_json::to_vec(&old).unwrap()).unwrap();
                assert!(
                    !Project::load(&path).unwrap().bodies[0]
                        .hollow
                        .as_ref()
                        .unwrap()
                        .outward
                );
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
                for thickness in [f64::NAN, f64::INFINITY, 0., -0.1, 101.] {
                    let mut invalid = body.clone();
                    invalid.hollow.as_mut().unwrap().thickness = thickness;
                    assert!(model.edit(invalid).is_err());
                    assert_eq!(model.bodies, vec![body.clone()]);
                }
            }
        }
    }
    #[test]
    fn closed_shells_retain_voids_thickness_history_and_cad_exports() {
        for shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            for outward in [false, true] {
                let mut body = Body::new(shape);
                body.id = 1;
                body.width = 6.;
                body.depth = 4.;
                body.extrusion = 5.;
                body.position = [3., -2., 1.];
                body.rotation = [15., 25., 35.];
                body.scale = [1.5, 0.75, 2.];
                let original = body.clone();
                let t = 0.3;
                body.hollow = Some(Hollow {
                    opening: FaceTarget::Top,
                    thickness: t,
                    outward,
                    closed: true,
                });
                let g = evaluate(&body).unwrap();
                let shell = g.shell.as_ref().unwrap();
                let parts = crate::components::solid_parts(shell).unwrap();
                assert_eq!(parts.len(), 1);
                assert_eq!(parts[0].boundaries().len(), 2);
                let inverse = crate::regions::transform(&body).invert().unwrap();
                let volume = g
                    .triangles
                    .iter()
                    .map(|tri| tri.map(|p| inverse.transform_point(p)))
                    .map(|tri| tri[0].to_vec().dot(tri[1].to_vec().cross(tri[2].to_vec())) / 6.)
                    .sum::<f64>();
                let area = |inset: f64| match shape {
                    Shape::Rectangle => (body.width - 2. * inset) * (body.depth - 2. * inset),
                    Shape::Circle => std::f64::consts::PI * (body.width / 2. - inset).powi(2),
                    Shape::Semicircle => {
                        let r = body.width / 2. - inset;
                        r * r * (inset / r).acos() - inset * (r * r - inset * inset).sqrt()
                    }
                    _ => unreachable!(),
                };
                let expected = if outward {
                    area(-t) * (body.extrusion + 2. * t) - area(0.) * body.extrusion
                } else {
                    area(0.) * body.extrusion - area(t) * (body.extrusion - 2. * t)
                };
                assert!(
                    (volume - expected).abs() < 0.5,
                    "{shape:?} outward={outward}: {volume} vs {expected}"
                );
                if shape == Shape::Circle {
                    assert_eq!(g.face_count, 6);
                    assert_eq!(g.circles.len(), 4);
                }
                // Mesh reconciliation must accept cavity orientation, but reject inversion.
                let mut print = g.clone();
                crate::components::weld_geometry(&mut print);
                crate::components::boundary_shell(&print, true).unwrap();
                for tri in &mut print.triangles {
                    tri.swap(1, 2);
                }
                assert!(crate::components::boundary_shell(&print, true).is_err());
                for format in [
                    crate::export::Format::Stl,
                    crate::export::Format::Glb,
                    crate::export::Format::Step,
                    crate::export::Format::Dxf,
                ] {
                    let bytes = crate::export::encode(&[body.clone()], format).unwrap();
                    assert!(!bytes.is_empty());
                    if format == crate::export::Format::Step {
                        let text = String::from_utf8(bytes).unwrap();
                        assert_eq!(text.matches("BREP_WITH_VOIDS(").count(), 1);
                        assert_eq!(
                            text.matches("MANIFOLD_SOLID_BREP(").count(),
                            0,
                            "The cavity must not become a second filled body"
                        );
                    }
                }
                let mut island = Body::new(Shape::Rectangle);
                island.position = [25., 0., 0.];
                island.extrusion = 1.;
                let mut compound = shell.clone();
                compound.extend(crate::model::primitive_shell(&island).unwrap());
                let groups = crate::components::solid_parts(&compound).unwrap();
                assert_eq!(groups.len(), 2);
                assert!(groups.iter().any(|s| s.boundaries().len() == 2));
                assert!(groups.iter().any(|s| s.boundaries().len() == 1));
                let mut model = Model::from_bodies(vec![original.clone()]);
                model.edit(body.clone()).unwrap();
                model.finish_edit();
                model.undo();
                assert_eq!(model.bodies, vec![original]);
                model.redo();
                assert_eq!(model.bodies, vec![body.clone()]);
                let mut opened = body.clone();
                opened.hollow.as_mut().unwrap().closed = false;
                model.edit(opened).unwrap();
                model.finish_edit();
                model.undo();
                assert_eq!(model.bodies, vec![body.clone()]);
                let dir = tempfile::tempdir().unwrap();
                let path = dir.path().join("closed.osk");
                let project = Project::new(vec![body.clone()], Camera::default(), true, Some(1));
                project.save(&path).unwrap();
                assert_eq!(Project::load(&path).unwrap().bodies, vec![body.clone()]);
                let mut old = serde_json::to_value(project).unwrap();
                old["schema_version"] = 16.into();
                old["modeling_version"] = 16.into();
                std::fs::write(&path, serde_json::to_vec(&old).unwrap()).unwrap();
                assert!(
                    Project::load(&path)
                        .unwrap_err()
                        .contains("Closed shells require .osk version 17")
                );
                old["bodies"][0]["hollow"]
                    .as_object_mut()
                    .unwrap()
                    .remove("closed");
                std::fs::write(&path, serde_json::to_vec(&old).unwrap()).unwrap();
                assert!(
                    !Project::load(&path).unwrap().bodies[0]
                        .hollow
                        .as_ref()
                        .unwrap()
                        .closed
                );
                if !outward {
                    let mut invalid = body.clone();
                    invalid.hollow.as_mut().unwrap().thickness = 2.6;
                    assert!(model.edit(invalid).is_err());
                    assert_eq!(model.bodies, vec![body]);
                }
            }
        }
    }
}
