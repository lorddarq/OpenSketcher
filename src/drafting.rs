//! Exact source-prism and evaluated planar draft, anchored to a neutral plane.
use crate::{
    edge_treatments::FaceTarget,
    model::{Body, Geometry, Shape},
};
use serde::{Deserialize, Serialize};
use truck_modeling::{
    EuclideanSpace, InnerSpace, Matrix4, Point3, Solid, SquareMatrix, Transform, Vector3, builder,
};

/// Absolute wall angle measured from a direction perpendicular to the neutral face.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvaluatedDraft {
    pub degrees: f64,
    pub neutral: usize,
    #[serde(default)]
    pub planar_loops: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub neutral_plane: Option<u64>,
}
impl EvaluatedDraft {
    pub fn validate(self) -> Result<(), String> {
        if self.neutral_plane == Some(0) {
            return Err("Draft requires a valid construction-plane identifier.".into());
        }
        if !self.degrees.is_finite() || self.degrees.abs() > 80. {
            return Err("Draft requires a finite angle within ±80 degrees.".into());
        }
        Ok(())
    }
    pub fn new(g: &Geometry, selected: &[usize]) -> Result<Self, String> {
        let planes = crate::components::straight_planar_supports(g)?;
        if selected.is_empty() || selected.iter().any(|f| *f >= planes.len()) {
            return Err("Select planar faces to draft.".into());
        }
        // Suggest the largest adjacent neutral face; the inspector can change it.
        let neutral = (0..planes.len())
            .filter(|f| !selected.contains(f))
            .filter(|f| {
                selected.iter().all(|side| {
                    planes[*f].0.dot(planes[*side].0).abs() < 1. - 1e-8
                        && g.topology.faces[*f]
                            .iter()
                            .any(|edge| g.topology.faces[*side].contains(edge))
                })
            })
            .max_by(|a, b| {
                let area = |face| {
                    g.triangles
                        .iter()
                        .zip(&g.triangle_faces)
                        .filter(|(_, role)| **role == face)
                        .map(|(t, _)| (t[1] - t[0]).cross(t[2] - t[0]).magnitude())
                        .sum::<f64>()
                };
                area(*a).total_cmp(&area(*b)).then_with(|| b.cmp(a))
            })
            .ok_or(
                "No shared neutral face is available; select side walls rather than opposing caps.",
            )?;
        Ok(Self {
            degrees: 3.,
            neutral,
            planar_loops: true,
            neutral_plane: None,
        })
    }
}

pub(crate) fn evaluated(
    g: &Geometry,
    selected: &[usize],
    operation: EvaluatedDraft,
) -> Result<Geometry, String> {
    evaluated_with_plane(g, selected, operation, None, false)
}

/// Resolve an authored, independent construction plane into owner-local coordinates.
pub(crate) fn reference_plane(
    owner: &Body,
    bodies: &[Body],
    id: u64,
) -> Result<(Vector3, f64), String> {
    let reference = bodies
        .iter()
        .find(|b| b.id == id)
        .ok_or_else(|| format!("Neutral construction plane {id} was removed."))?;
    if !reference.construction {
        return Err(format!(
            "Neutral reference {id} must be a construction plane."
        ));
    }
    let placement = crate::regions::transform(owner)
        .invert()
        .ok_or("Invalid Draft owner placement.")?
        * crate::regions::transform(reference);
    let normal = placement
        .transform_vector(Vector3::unit_x())
        .cross(placement.transform_vector(Vector3::unit_y()))
        .normalize();
    let origin = placement.transform_point(Point3::new(0., 0., 0.));
    Ok((normal, normal.dot(origin.to_vec())))
}

pub(crate) fn evaluated_with_plane(
    g: &Geometry,
    selected: &[usize],
    operation: EvaluatedDraft,
    reference: Option<(Vector3, f64)>,
    convex_rebuild: bool,
) -> Result<Geometry, String> {
    operation.validate()?;
    if operation.neutral_plane.is_some() != reference.is_some() {
        return Err("Draft requires its resolved neutral construction plane.".into());
    }
    let mut planes = if operation.planar_loops {
        crate::components::straight_planar_supports(g)?
    } else {
        crate::components::planar_supports(g)?
    };
    let (axis, height) = match reference {
        Some(plane) => plane,
        None => *planes
            .get(operation.neutral)
            .ok_or("Draft neutral face no longer exists.")?,
    };
    if selected.is_empty()
        || reference.is_none() && selected.contains(&operation.neutral)
        || selected.iter().any(|f| *f >= planes.len())
    {
        return Err("Select draft faces distinct from the neutral face.".into());
    }
    let (sine, cosine) = operation.degrees.to_radians().sin_cos();
    for (face, plane) in planes.iter_mut().enumerate() {
        if !selected.contains(&face) {
            continue;
        }
        let (normal, distance) = *plane;
        let parallel = normal.dot(axis);
        let tangent = normal - axis * parallel;
        let length = tangent.magnitude();
        if length < 1e-8 {
            return Err(
                "Draft walls must intersect the neutral plane; parallel caps cannot be drafted."
                    .into(),
            );
        }
        // Keep the original intersection line with the neutral support fixed.
        // Positive angles widen on the negative side of the oriented neutral plane.
        *plane = (
            tangent / length * cosine + axis * sine,
            (distance - parallel * height) / length * cosine + height * sine,
        );
    }
    let result = if operation.planar_loops
        && (planes.len() > 64 || crate::components::convex_supports(g).is_err())
    {
        let grouped;
        let input = if g
            .shell
            .as_ref()
            .is_some_and(|shell| shell.len() == g.face_count)
        {
            g
        } else {
            grouped = crate::edge_treatments::planar_regions(g.clone())?;
            &grouped
        };
        crate::components::retrim_planes(input, &planes)?
    } else {
        let indexed = planes.into_iter().enumerate().collect::<Vec<_>>();
        crate::edge_treatments::planar_regions(crate::edge_treatments::convex_from_planes(
            &indexed,
        )?)?
    };
    if result.face_count != g.face_count
        && !(convex_rebuild && g.face_count <= 64 && crate::components::convex_supports(g).is_ok())
    {
        return Err(
            "Draft would consume a support face; reduce the angle or change the neutral face."
                .into(),
        );
    }
    let shell = result
        .shell
        .as_ref()
        .ok_or("Draft leaves no closed material.")?;
    crate::components::solid_parts(shell)?;
    crate::operators::geometry(&crate::operators::solid(&result)?)
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Draft {
    pub degrees: f64,
    pub neutral: FaceTarget,
}
pub fn validate(body: &Body) -> Result<(), String> {
    let Some(operation) = &body.taper else {
        return Ok(());
    };
    if !matches!(
        body.shape,
        Shape::Rectangle | Shape::Circle | Shape::Semicircle
    ) || body.extrusion <= 0.
        || body.bore.is_some()
        || body.boolean.is_some()
        || body.hollow.is_some()
        || !body.edge_treatments.is_empty()
        || crate::components::has_source_edits(body)
    {
        return Err("Draft currently requires a source box, cylinder or semicylinder without shell, edge treatments, source deformation or Boolean operations.".into());
    }
    if !matches!(operation.neutral, FaceTarget::Top | FaceTarget::Bottom)
        || !operation.degrees.is_finite()
        || operation.degrees.abs() > 80.
    {
        return Err(
            "Draft requires a top/bottom neutral cap and a finite angle within ±80 degrees.".into(),
        );
    }
    let offset = body.extrusion * operation.degrees.to_radians().tan();
    if body.width + 2. * offset <= 1e-5
        || body.shape == Shape::Rectangle && body.depth + 2. * offset <= 1e-5
        || body.shape == Shape::Semicircle && body.width / 2. + 2. * offset <= 1e-5
    {
        return Err(
            "Draft angle would collapse or invert the opposite cap; reduce the inward angle."
                .into(),
        );
    }
    Ok(())
}
pub fn evaluate(body: &Body, partitioned: bool) -> Result<Geometry, String> {
    validate(body)?;
    if partitioned {
        return Err("Draft does not yet support partitioned source profiles.".into());
    }
    let operation = body.taper.as_ref().ok_or("Missing draft operation.")?;
    let delta = body.extrusion * operation.degrees.to_radians().tan();
    let bottom = if operation.neutral == FaceTarget::Bottom {
        0.
    } else {
        delta
    };
    let top = if operation.neutral == FaceTarget::Top {
        0.
    } else {
        delta
    };
    let wire = |offset: f64, z: f64| {
        let mut profile = Body::new(body.shape);
        profile.width = body.width + 2. * offset;
        profile.depth = body.depth + 2. * offset;
        builder::transformed(
            &crate::model::profile_wire(&profile),
            Matrix4::from_translation(Vector3::unit_z() * z),
        )
    };
    let mut shell = if body.shape == Shape::Semicircle {
        semicylinder(body, bottom, top)?
    } else {
        builder::try_wire_homotopy(&wire(bottom, 0.), &wire(top, body.extrusion))
            .map_err(|e| format!("Cannot draft the profile: {e}"))?
    };
    for boundary in shell.extract_boundaries() {
        shell.push(builder::try_attach_plane(&[boundary.inverse()]).map_err(|e| e.to_string())?);
    }
    Solid::try_new(vec![shell.clone()])
        .map_err(|e| format!("Draft did not produce a closed solid: {e}"))?;
    let mut geometry = crate::model::tessellate(&shell)?;
    if body.shape == Shape::Circle {
        let roles: Vec<_> = (0..geometry.face_count)
            .map(|face| {
                let points: Vec<_> = geometry
                    .triangles
                    .iter()
                    .zip(&geometry.triangle_faces)
                    .filter(|(_, f)| **f == face)
                    .flat_map(|(t, _)| t.iter())
                    .collect();
                if points.iter().all(|p| p.z.abs() < 1e-6) {
                    0
                } else if points.iter().all(|p| (p.z - body.extrusion).abs() < 1e-6) {
                    1
                } else {
                    2
                }
            })
            .collect();
        crate::edge_treatments::merge_surfaces(&mut geometry, &roles);
        for (offset, z) in [(bottom, 0.), (top, body.extrusion)] {
            let r = body.width / 2. + offset;
            geometry.circles.push(crate::model::CircularEdge {
                center: Point3::new(0., 0., z),
                u: Vector3::unit_x() * r,
                v: Vector3::unit_y() * r,
            });
        }
    }
    crate::components::map_geometry(&mut geometry, crate::regions::transform(body));
    geometry.is_solid = true;
    Ok(geometry)
}

fn semicylinder(body: &Body, bottom: f64, top: f64) -> Result<truck_modeling::Shell, String> {
    use truck_modeling::{
        BSplineCurve, Curve, Edge, Face, Invertible, KnotVec, Line, Processor, RevolutedCurve,
        Surface, Wire,
    };
    let r = body.width / 2.;
    let h = body.extrusion;
    let section = |offset: f64, z: f64| {
        let x = (r * r + 2. * r * offset).sqrt();
        let right = builder::vertex(Point3::new(x, -offset, z));
        let left = builder::vertex(Point3::new(-x, -offset, z));
        let arc = builder::circle_arc(&right, &left, Point3::new(0., r + offset, z));
        let line = builder::line(&left, &right);
        (right, left, arc, line)
    };
    let (br, bl, ba, bd) = section(bottom, 0.);
    let (tr, tl, ta, td) = section(top, h);
    let operation = body.taper.as_ref().unwrap();
    let offset = bottom + top;
    let x = (r * r + 2. * r * offset).sqrt();
    // Conjugate form avoids cancellation at tiny/zero angles. The corner is
    // the exact intersection x²=R²+2 R tan(angle) distance with y=-tan(angle) distance.
    let q = 2. * r * offset / (x + r);
    let dz = 2. * r * h / (x + r);
    let corner = |bottom: &truck_modeling::Vertex, top: &truck_modeling::Vertex, sign: f64| {
        let (start, end, zsign) = if operation.neutral == FaceTarget::Bottom {
            (bottom, top, 1.)
        } else {
            (top, bottom, -1.)
        };
        let control = start.point() + Vector3::new(sign * q, -q, zsign * dz) / 2.;
        let curve = Curve::BSplineCurve(BSplineCurve::new(
            KnotVec::bezier_knot(2),
            vec![start.point(), control, end.point()],
        ));
        let edge = Edge::new(start, end, curve);
        if zsign > 0. { edge } else { edge.inverse() }
    };
    let right = corner(&br, &tr, 1.);
    let left = corner(&bl, &tl, -1.);
    let generator = Curve::Line(Line(
        Point3::new(0., -(r + bottom), 0.),
        Point3::new(0., -(r + top), h),
    ));
    // Put the periodic seam on -Y, outside the trimmed half-cone for both signs.
    let surface = Surface::RevolutedCurve(Processor::new(RevolutedCurve::by_revolution(
        generator,
        Point3::new(0., 0., 0.),
        Vector3::unit_z(),
    )))
    .inverse();
    let curved: Wire = vec![ba, left.clone(), ta.inverse(), right.inverse()].into();
    let flat: Wire = vec![bd, right, td.inverse(), left.inverse()].into();
    Ok(vec![
        Face::new(vec![curved], surface),
        builder::try_attach_plane(&[flat]).map_err(|e| e.to_string())?,
    ]
    .into_iter()
    .collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        model::{Model, evaluate, evaluate_scene},
        project::Project,
        viewport::Camera,
    };
    use truck_modeling::{
        BoundedCurve, Curve, EuclideanSpace, InnerSpace, ParametricCurve, ParametricSurface,
        SquareMatrix, Surface, Transform,
    };
    #[test]
    fn evaluated_planar_draft_keeps_neutral_trace_angles_and_ordered_history() {
        use crate::{
            booleans, components,
            face_operations::{self, FaceEdit},
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
                if kind == 1 {
                    let mut chamfer = crate::edge_treatments::Modifier::new(
                        crate::edge_treatments::Kind::Chamfer,
                        FaceTarget::Top,
                        &body,
                    );
                    chamfer.edge = Some(FaceTarget::Right);
                    chamfer.amount = 0.3;
                    body.edge_treatments.push(chamfer);
                }
                if kind == 3 {
                    let g = evaluate(&body).unwrap();
                    let top = (0..g.face_count)
                        .find(|f| {
                            crate::edge_treatments::target(&body, &g, *f) == Ok(FaceTarget::Top)
                        })
                        .unwrap();
                    body = components::transformed(
                        &body,
                        &g,
                        SelectionMode::Face,
                        Some(top),
                        Matrix4::from_translation(
                            crate::regions::transform(&body)
                                .transform_vector(Vector3::unit_z() * 0.25),
                        ),
                    )
                    .unwrap();
                }
                if kind == 4 {
                    body.taper = Some(Draft {
                        degrees: 9.,
                        neutral: FaceTarget::Bottom,
                    });
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
                    let mut result = Body::new(Shape::Rectangle);
                    result.id = 3;
                    result.boolean = Some(booleans::Boolean::new(
                        booleans::Kind::Intersect,
                        &[(1, None), (2, None)],
                        SelectionMode::Volume,
                        &scene,
                    ));
                    bodies.push(result);
                }
                let owner = bodies.last().unwrap();
                let id = owner.id;
                let inverse = crate::regions::transform(owner).invert().unwrap();
                let scene = evaluate_scene(&bodies).unwrap();
                let g = &scene.last().unwrap().1;
                let mut local = g.clone();
                components::map_geometry(&mut local, inverse);
                let planes = components::planar_supports(&local).unwrap();
                let wall = planes.iter().position(|(n, _)| n.y > 0.9).unwrap();
                EvaluatedDraft::new(&local, &[wall]).unwrap();
                for reference in 0..3 {
                    let side = reference == 1;
                    let skew = reference == 2;
                    if skew && kind != 1 {
                        continue;
                    }
                    if kind == 4 && side {
                        continue;
                    }
                    let neutral = planes
                        .iter()
                        .position(|(n, _)| {
                            if skew {
                                n.x > 0.1 && n.z > 0.1
                            } else if side {
                                n.x < -0.99
                            } else {
                                n.z < -0.99
                            }
                        })
                        .unwrap();
                    let (axis, height) = planes[neutral];
                    let (normal, distance) = planes[wall];
                    for degrees in [-1.5, 0., 3.] {
                        let operation = EvaluatedDraft {
                            degrees,
                            neutral,
                            planar_loops: false,
                            neutral_plane: None,
                        };
                        let mut changes = face_operations::selected(
                            &bodies,
                            &scene,
                            &[(id, Some(wall))],
                            FaceEdit::Draft(operation),
                        )
                        .unwrap();
                        changes[0]
                            .component_edits
                            .last_mut()
                            .unwrap()
                            .convex_rebuild = false;
                        let mut model = Model::from_bodies(bodies.clone());
                        model.commit_changes(&changes).unwrap();
                        let evaluated_scene = evaluate_scene(&model.bodies).unwrap();
                        let mut output = evaluated_scene.last().unwrap().1.clone();
                        components::map_geometry(&mut output, inverse);
                        assert!(output.is_solid);
                        assert!(output.shell.as_ref().unwrap().is_geometric_consistent());
                        assert_eq!(output.face_count, planes.len());
                        assert_eq!(
                            output.shell.as_ref().unwrap().len(),
                            planes.len(),
                            "Retained regions, not triangle patches"
                        );
                        components::solid_parts(output.shell.as_ref().unwrap()).unwrap();
                        let result_planes = components::planar_supports(&output).unwrap();
                        let tangent = (normal - axis * normal.dot(axis)).normalize();
                        let &(new_normal, new_distance) = result_planes
                            .iter()
                            .find(|(n, _)| n.dot(tangent) > 0.9)
                            .unwrap();
                        assert!(
                            (new_normal.dot(axis).asin().to_degrees() - degrees).abs() < 1e-8,
                            "Requested absolute wall angle"
                        );
                        assert!(
                            new_normal.cross(tangent).dot(axis).abs() < 1e-8,
                            "Wall only tilts across its neutral trace"
                        );
                        // The entire original wall/neutral intersection line is fixed.
                        let trace = local
                            .points
                            .iter()
                            .filter(|p| {
                                (axis.dot(p.to_vec()) - height).abs() < 1e-6
                                    && (normal.dot(p.to_vec()) - distance).abs() < 1e-6
                            })
                            .collect::<Vec<_>>();
                        assert!(trace.len() >= 2);
                        for p in trace {
                            assert!((new_normal.dot(p.to_vec()) - new_distance).abs() < 1e-7);
                            assert!(output.points.iter().any(|q| (*q - *p).magnitude() < 1e-6));
                        }
                        for (face, (n, d)) in planes.iter().enumerate().filter(|(f, _)| *f != wall)
                        {
                            assert!(
                                result_planes
                                    .iter()
                                    .any(|(m, e)| (*m - *n).magnitude() < 1e-7
                                        && (*e - *d).abs() < 1e-7),
                                "Untouched support {face}"
                            );
                        }
                        if kind == 0 {
                            let extra = if side {
                                4. * 6_f64.powi(2)
                            } else {
                                6. * 4_f64.powi(2)
                            } * degrees.to_radians().tan()
                                / 2.;
                            assert!(
                                (volume(&output) - (120. + extra)).abs() < 1e-6,
                                "One-wall wedge has exact material volume"
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
                        let path = dir.path().join("evaluated-draft.osk");
                        let project =
                            Project::new(model.bodies.clone(), Camera::default(), true, Some(id));
                        project.save(&path).unwrap();
                        assert_eq!(Project::load(&path).unwrap().bodies, model.bodies);
                        let mut old = serde_json::to_value(&project).unwrap();
                        old["schema_version"] = 36.into();
                        old["modeling_version"] = 36.into();
                        assert!(
                            serde_json::from_value::<Project>(old)
                                .unwrap()
                                .validate()
                                .unwrap_err()
                                .contains("version 37")
                        );
                        if kind == 0 && !affine && !side && degrees == 0. {
                            let mut legacy = serde_json::to_value(&project).unwrap();
                            legacy["schema_version"] = 37.into();
                            legacy["modeling_version"] = 37.into();
                            legacy["bodies"][0]["component_edits"][0]["face_edit"]["draft"]
                                .as_object_mut()
                                .unwrap()
                                .remove("planar_loops");
                            std::fs::write(&path, serde_json::to_vec(&legacy).unwrap()).unwrap();
                            assert_eq!(
                                Project::load(&path).unwrap().bodies,
                                model.bodies,
                                "Missing v38 flag retains v37 replay"
                            );
                        }
                        let accepted = model.bodies.clone();
                        let mut invalid = changes[0].clone();
                        invalid.component_edits.last_mut().unwrap().face_edit =
                            Some(FaceEdit::Draft(EvaluatedDraft {
                                degrees: f64::NAN,
                                neutral,
                                planar_loops: false,
                                neutral_plane: None,
                            }));
                        assert!(model.commit(invalid).is_err());
                        assert_eq!(model.bodies, accepted);
                        let mut collapsed = changes[0].clone();
                        collapsed.component_edits.last_mut().unwrap().face_edit =
                            Some(FaceEdit::Draft(EvaluatedDraft {
                                degrees: -80.,
                                neutral,
                                planar_loops: false,
                                neutral_plane: None,
                            }));
                        if kind == 0 {
                            assert!(model.commit(collapsed).is_err());
                            assert_eq!(
                                model.bodies, accepted,
                                "Rejected geometric collapse preserves the committed body"
                            );
                        }
                        model.undo();
                        assert_eq!(model.bodies, bodies);
                        model.redo();
                        assert_eq!(model.bodies, accepted);
                        let mut removed = changes[0].clone();
                        removed.component_edits.pop();
                        model.commit(removed).unwrap();
                        assert_eq!(model.bodies, bodies);
                        // A later exact face move must replay after the drafted boundary.
                        if kind == 1 && !affine && !side && !skew && degrees == 3. {
                            let top = result_planes.iter().position(|(n, _)| n.z > 0.99).unwrap();
                            let next = components::transformed(
                                &changes[0],
                                &evaluated_scene.last().unwrap().1,
                                SelectionMode::Face,
                                Some(top),
                                Matrix4::from_translation(Vector3::unit_z() * 0.1),
                            )
                            .unwrap();
                            model.commit(next).unwrap();
                            let project = Project::new(
                                model.bodies.clone(),
                                Camera::default(),
                                true,
                                Some(id),
                            );
                            project.save(&path).unwrap();
                            assert_eq!(Project::load(&path).unwrap().bodies, model.bodies);
                            if let Some(path) =
                                std::env::var_os("OSK_EVALUATED_DRAFT_REVIEW_PROJECT")
                            {
                                let mut review =
                                    Project::new(bodies.clone(), Camera::default(), true, Some(id));
                                review.selection_mode = SelectionMode::Face;
                                review.selected_element = Some(wall);
                                review.selection = vec![(id, Some(wall))];
                                review.save(std::path::Path::new(&path)).unwrap();
                            }
                        }
                    }
                }
                assert!(
                    evaluated(
                        &local,
                        &[wall],
                        EvaluatedDraft {
                            degrees: 3.,
                            planar_loops: false,
                            neutral_plane: None,
                            neutral: wall
                        }
                    )
                    .is_err()
                );
                assert!(
                    evaluated(
                        &local,
                        &[wall],
                        EvaluatedDraft {
                            degrees: 3.,
                            planar_loops: false,
                            neutral_plane: None,
                            neutral: usize::MAX
                        }
                    )
                    .is_err()
                );
            }
        }
        // Multiple selected walls, absolute angle reset on a sloped source, and collapse rejection.
        let mut body = Body::new(Shape::Rectangle);
        body.id = 1;
        body.width = 6.;
        body.depth = 5.;
        body.extrusion = 4.;
        body.taper = Some(Draft {
            degrees: 9.,
            neutral: FaceTarget::Bottom,
        });
        let g = evaluate(&body).unwrap();
        let planes = components::planar_supports(&g).unwrap();
        let neutral = planes.iter().position(|(n, _)| n.z < -0.99).unwrap();
        let walls = planes
            .iter()
            .enumerate()
            .filter(|(_, (n, _))| n.z.abs() < 0.9)
            .map(|(f, _)| f)
            .collect::<Vec<_>>();
        let reset = evaluated(
            &g,
            &walls,
            EvaluatedDraft {
                degrees: 0.,
                neutral,
                planar_loops: false,
                neutral_plane: None,
            },
        )
        .unwrap();
        assert!(
            (volume(&reset) - 120.).abs() < 1e-6,
            "Zero is perpendicular, not a relative rotation"
        );
        assert!(
            evaluated(
                &g,
                &walls,
                EvaluatedDraft {
                    degrees: -80.,
                    planar_loops: false,
                    neutral_plane: None,
                    neutral
                }
            )
            .is_err()
        );
        let cap = planes.iter().position(|(n, _)| n.z > 0.99).unwrap();
        assert!(
            evaluated(
                &g,
                &[cap],
                EvaluatedDraft {
                    degrees: 3.,
                    planar_loops: false,
                    neutral_plane: None,
                    neutral
                }
            )
            .is_err()
        );
        let mut curved = Body::new(Shape::Circle);
        curved.extrusion = 4.;
        assert!(EvaluatedDraft::new(&evaluate(&curved).unwrap(), &[2]).is_err());
    }

    #[test]
    fn concave_draft_preserves_notches_pockets_holes_and_later_edits() {
        use crate::{
            booleans, components,
            face_operations::{self, FaceEdit},
            selection::SelectionMode,
        };
        let volume = |g: &Geometry| {
            g.triangles
                .iter()
                .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                .sum::<f64>()
        };
        for kind in 0..3 {
            for affine in [false, true] {
                let mut target = Body::new(Shape::Rectangle);
                target.id = 1;
                target.width = 6.;
                target.depth = 6.;
                target.extrusion = 3.;
                let mut tool = target.clone();
                tool.id = 2;
                tool.width = if kind == 0 { 4. } else { 2. };
                tool.depth = 2.;
                tool.position[0] = if kind == 0 { 2. } else { 0. };
                if kind == 2 {
                    tool.position[2] = 1.;
                    tool.extrusion = 2.;
                }
                let mut owner = Body::new(Shape::Rectangle);
                owner.id = 3;
                let mut frame = owner.clone();
                if affine {
                    frame.position = [2., -1., 3.];
                    frame.rotation = [11., 17., 23.];
                    frame.scale = [1.5, 0.75, 2.];
                    let placement = crate::regions::transform(&frame);
                    for b in [&mut target, &mut tool] {
                        b.position = placement.transform_point(Point3::from(b.position)).into();
                        b.rotation = frame.rotation;
                        b.scale = frame.scale;
                    }
                }
                let mut bodies = vec![target, tool];
                let source = evaluate_scene(&bodies).unwrap();
                owner.boolean = Some(booleans::Boolean::new(
                    booleans::Kind::Subtract,
                    &[(1, None), (2, None)],
                    SelectionMode::Volume,
                    &source,
                ));
                bodies.push(owner.clone());
                let inverse = crate::regions::transform(&frame).invert().unwrap();
                let scene = evaluate_scene(&bodies).unwrap();
                let g = &scene[2].1;
                let mut local = g.clone();
                components::map_geometry(&mut local, inverse);
                assert!(components::convex_supports(&local).is_err());
                let planes = components::straight_planar_supports(&local).unwrap();
                let wall = planes
                    .iter()
                    .position(|(n, d)| {
                        n.x > 0.99 && (*d - if kind == 0 { 0. } else { -1. }).abs() < 1e-6
                    })
                    .unwrap();
                let suggestion = EvaluatedDraft::new(&local, &[wall]).unwrap();
                assert!(suggestion.planar_loops);
                for top in [false, true] {
                    if kind == 2 && !top {
                        continue;
                    }
                    let neutral = planes
                        .iter()
                        .position(|(n, d)| {
                            if top {
                                n.z > 0.99 && (*d - 3.).abs() < 1e-6
                            } else {
                                n.z < -0.99
                            }
                        })
                        .unwrap();
                    for many in [false, true] {
                        let faces = if many {
                            planes
                                .iter()
                                .enumerate()
                                .filter(|(_, (n, d))| n.z.abs() < 0.1 && *d < 1.01)
                                .map(|(face, _)| face)
                                .collect::<Vec<_>>()
                        } else {
                            vec![wall]
                        };
                        assert_eq!(
                            faces.len(),
                            if many {
                                if kind == 0 { 3 } else { 4 }
                            } else {
                                1
                            }
                        );
                        for degrees in [-3., 0., 5.] {
                            let operation = EvaluatedDraft {
                                degrees,
                                neutral,
                                planar_loops: true,
                                neutral_plane: None,
                            };
                            assert!(
                                evaluated(
                                    &local,
                                    &faces,
                                    EvaluatedDraft {
                                        planar_loops: false,
                                        neutral_plane: None,
                                        ..operation
                                    }
                                )
                                .is_err(),
                                "v37 stays convex-only"
                            );
                            let picks = faces.iter().map(|f| (3, Some(*f))).collect::<Vec<_>>();
                            let changes = face_operations::selected(
                                &bodies,
                                &scene,
                                &picks,
                                FaceEdit::Draft(operation),
                            )
                            .unwrap();
                            let mut model = Model::from_bodies(bodies.clone());
                            model.commit_changes(&changes).unwrap();
                            let output = evaluate_scene(&model.bodies).unwrap();
                            let mut result = output[2].1.clone();
                            components::map_geometry(&mut result, inverse);
                            assert!(result.is_solid);
                            assert!(result.shell.as_ref().unwrap().is_geometric_consistent());
                            components::solid_parts(result.shell.as_ref().unwrap()).unwrap();
                            assert_eq!(result.face_count, local.face_count);
                            assert_eq!(result.shell.as_ref().unwrap().len(), result.face_count);
                            let after = components::straight_planar_supports(&result).unwrap();
                            let axis = planes[neutral].0;
                            let world_planes =
                                components::straight_planar_supports(&output[2].1).unwrap();
                            let world_axis = crate::regions::transform(&frame)
                                .transform_vector(axis)
                                .normalize();
                            for (index, (n, d)) in planes.iter().enumerate() {
                                if faces.contains(&index) {
                                    let tangent = (*n - axis * n.dot(axis)).normalize();
                                    let trace = tangent * *d + axis * planes[neutral].1;
                                    let matched = after
                                        .iter()
                                        .position(|(m, e)| {
                                            m.dot(tangent) > 0.9 && (m.dot(trace) - *e).abs() < 1e-7
                                        })
                                        .unwrap();
                                    assert!(
                                        (world_planes[matched]
                                            .0
                                            .dot(world_axis)
                                            .asin()
                                            .to_degrees()
                                            - degrees)
                                            .abs()
                                            < 1e-7
                                    );
                                } else {
                                    assert!(
                                        after.iter().any(|(m, e)| (*m - *n).magnitude() < 1e-7
                                            && (*e - *d).abs() < 1e-7)
                                    );
                                }
                            }
                            let tangent = degrees.to_radians().tan();
                            let tx = tangent * frame.scale[2] / frame.scale[0];
                            let ty = tangent * frame.scale[2] / frame.scale[1];
                            let expected = if !many {
                                [90., 96., 100.][kind] + if kind == 2 { 4. } else { 9. } * tx
                            } else {
                                match kind {
                                    0 => 90. + 9. * tx + 27. * ty - 18. * tx * ty,
                                    1 => 96. + 18. * (tx + ty) - 36. * tx * ty,
                                    _ => 100. + 8. * (tx + ty) - 32. / 3. * tx * ty,
                                }
                            };
                            assert!(
                                (volume(&result) - expected).abs() < 1e-6,
                                "kind={kind} affine={affine} top={top} many={many} degrees={degrees}: {} vs {expected}",
                                volume(&result)
                            );
                            // A through-hole has two annular caps; a pocket has only an annular top.
                            let roles = result.surface_mapping().unwrap();
                            for (i, face) in result.shell.as_ref().unwrap().iter().enumerate() {
                                let role = roles[i];
                                if after[role].0.z.abs() < 0.99 {
                                    continue;
                                }
                                let annular = kind == 1
                                    || kind == 2
                                        && after[role].0.z > 0.99
                                        && (after[role].1 - 3.).abs() < 1e-6;
                                assert_eq!(
                                    face.boundaries().len(),
                                    if annular { 2 } else { 1 },
                                    "Hole loops stay holes"
                                );
                            }
                            for format in crate::export::Format::ALL {
                                assert!(
                                    !crate::export::encode(&model.bodies, format)
                                        .unwrap()
                                        .is_empty()
                                );
                            }
                            let dir = tempfile::tempdir().unwrap();
                            let path = dir.path().join("concave-draft.osk");
                            let project = Project::new(
                                model.bodies.clone(),
                                Camera::default(),
                                true,
                                Some(3),
                            );
                            project.save(&path).unwrap();
                            assert_eq!(Project::load(&path).unwrap().bodies, model.bodies);
                            let mut old = serde_json::to_value(&project).unwrap();
                            old["schema_version"] = 37.into();
                            old["modeling_version"] = 37.into();
                            assert!(
                                serde_json::from_value::<Project>(old)
                                    .unwrap()
                                    .validate()
                                    .unwrap_err()
                                    .contains("version 38")
                            );
                            let accepted = model.bodies.clone();
                            let mut invalid = changes[0].clone();
                            invalid.component_edits.last_mut().unwrap().face_edit =
                                Some(FaceEdit::Draft(EvaluatedDraft {
                                    degrees: 80.,
                                    ..operation
                                }));
                            assert!(model.commit(invalid).is_err());
                            assert_eq!(model.bodies, accepted);
                            model.undo();
                            assert_eq!(model.bodies, bodies);
                            model.redo();
                            assert_eq!(model.bodies, accepted);
                            let mut removed = changes[0].clone();
                            removed.component_edits.pop();
                            model.commit(removed).unwrap();
                            assert_eq!(model.bodies, bodies);
                            if kind == 1 && !affine && top && !many && degrees == 5. {
                                let top_face = after
                                    .iter()
                                    .position(|(n, d)| n.z > 0.99 && (*d - 3.).abs() < 1e-6)
                                    .unwrap();
                                let moved = components::transformed(
                                    &changes[0],
                                    &output[2].1,
                                    SelectionMode::Face,
                                    Some(top_face),
                                    Matrix4::from_translation(Vector3::unit_z() * 0.2),
                                )
                                .unwrap();
                                model.commit(moved).unwrap();
                                Project::new(
                                    model.bodies.clone(),
                                    Camera::default(),
                                    true,
                                    Some(3),
                                )
                                .save(&path)
                                .unwrap();
                                assert_eq!(Project::load(&path).unwrap().bodies, model.bodies);
                                if let Some(path) =
                                    std::env::var_os("OSK_CONCAVE_DRAFT_REVIEW_PROJECT")
                                {
                                    let mut review = Project::new(
                                        bodies.clone(),
                                        Camera::default(),
                                        true,
                                        Some(3),
                                    );
                                    review.selection_mode = SelectionMode::Face;
                                    review.selected_element = Some(wall);
                                    review.selection = vec![(3, Some(wall))];
                                    review.save(std::path::Path::new(&path)).unwrap();
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn live_neutral_planes_regenerate_draft_with_history_and_reference_validation() {
        use crate::{
            booleans, components,
            face_operations::{self, FaceEdit},
            selection::SelectionMode,
        };
        let volume = |g: &Geometry| {
            g.triangles
                .iter()
                .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                .sum::<f64>()
        };
        for kind in 0..3 {
            for affine in [false, true] {
                if kind == 1 && affine {
                    continue;
                }
                for before in [false, true] {
                    if kind == 1 && before {
                        continue;
                    }
                    let mut owner = Body::new(Shape::Rectangle);
                    owner.id = if before { 2 } else { 1 };
                    owner.width = 6.;
                    owner.depth = 6.;
                    owner.extrusion = 4.;
                    if affine {
                        owner.position = [2., -1., 3.];
                        owner.rotation = [11., 17., 23.];
                        owner.scale = [1.5, 0.75, 2.];
                    }
                    if kind == 2 {
                        let mut treatment = crate::edge_treatments::Modifier::new(
                            crate::edge_treatments::Kind::Chamfer,
                            FaceTarget::Top,
                            &owner,
                        );
                        treatment.edge = Some(FaceTarget::Right);
                        treatment.amount = 0.3;
                        owner.edge_treatments.push(treatment);
                    }
                    let mut bodies = vec![owner.clone()];
                    if kind == 1 {
                        bodies[0].extrusion = 3.;
                        let mut tool = bodies[0].clone();
                        tool.id = 2;
                        tool.width = 2.;
                        tool.depth = 2.;
                        bodies.push(tool);
                        let source = evaluate_scene(&bodies).unwrap();
                        owner = Body::new(Shape::Rectangle);
                        owner.id = 3;
                        owner.boolean = Some(booleans::Boolean::new(
                            booleans::Kind::Subtract,
                            &[(1, None), (2, None)],
                            SelectionMode::Volume,
                            &source,
                        ));
                        bodies.push(owner.clone());
                    }
                    let mut plane = Body::new(Shape::Rectangle);
                    plane.id = if before { 1 } else { 9 };
                    plane.construction = true;
                    plane.rotation = owner.rotation;
                    plane.scale = [2., 3., 0.5];
                    let placement = crate::regions::transform(&owner);
                    let inverse = placement.invert().unwrap();
                    plane.position = placement.transform_point(Point3::new(0., 0., 2.)).into();
                    bodies.push(plane.clone());
                    let scene = evaluate_scene(&bodies).unwrap();
                    let source = &scene.iter().find(|p| p.0 == owner.id).unwrap().1;
                    let mut local = source.clone();
                    components::map_geometry(&mut local, inverse);
                    let planes = components::straight_planar_supports(&local).unwrap();
                    let wall = planes
                        .iter()
                        .position(|(n, d)| {
                            if kind == 2 {
                                n.y > 0.99
                            } else {
                                n.x > 0.99 && (*d - if kind == 1 { -1. } else { 3. }).abs() < 1e-6
                            }
                        })
                        .unwrap();
                    let operation = EvaluatedDraft {
                        degrees: 5.,
                        neutral: wall,
                        planar_loops: true,
                        neutral_plane: Some(plane.id),
                    };
                    assert!(
                        EvaluatedDraft {
                            neutral_plane: Some(0),
                            ..operation
                        }
                        .validate()
                        .is_err()
                    );
                    let changed = face_operations::selected(
                        &bodies,
                        &scene,
                        &[(owner.id, Some(wall))],
                        FaceEdit::Draft(operation),
                    )
                    .unwrap();
                    let mut model = Model::from_bodies(bodies.clone());
                    model.commit_changes(&changed).unwrap();
                    assert!(
                        evaluate(&changed[0]).is_err(),
                        "Standalone evaluation cannot guess an external reference"
                    );
                    for height in [0., 2., 3.] {
                        let mut moved = plane.clone();
                        moved.position = placement
                            .transform_point(Point3::new(0., 0., height))
                            .into();
                        model.commit(moved).unwrap();
                        let output = evaluate_scene(&model.bodies).unwrap();
                        let result = &output.iter().find(|p| p.0 == owner.id).unwrap().1;
                        assert!(result.is_solid);
                        assert!(result.shell.as_ref().unwrap().is_geometric_consistent());
                        components::solid_parts(result.shell.as_ref().unwrap()).unwrap();
                        let mut result = result.clone();
                        components::map_geometry(&mut result, inverse);
                        let slope = 5_f64.to_radians().tan();
                        let (depth, span) = if kind == 1 { (3., 2.) } else { (4., 6.) };
                        let expected = volume(&local) + span * slope * (height * depth - depth * depth / 2.)
                            // Integrate the missing triangular X/Z strip of the 0.30 chamfer.
                            + if kind==2 { slope*(0.1755-0.045*height) }else{0.};
                        assert!(
                            (volume(&result) - expected).abs() < 1e-6,
                            "kind={kind} affine={affine} before={before} h={height}: {} vs {expected}",
                            volume(&result)
                        );
                        let supports = components::straight_planar_supports(&result).unwrap();
                        let direction = if kind == 2 {
                            Vector3::unit_y()
                        } else {
                            Vector3::unit_x()
                        };
                        let offset = if kind == 1 { -1. } else { 3. };
                        let trace = direction * offset + Vector3::unit_z() * height;
                        let &(normal, distance) = supports
                            .iter()
                            .find(|(n, d)| {
                                n.dot(direction) > 0.99 && (n.dot(trace) - *d).abs() < 1e-7
                            })
                            .unwrap();
                        assert!(
                            (normal.dot(Vector3::unit_z()).asin().to_degrees() - 5.).abs() < 1e-7
                        );
                        assert!((normal.dot(trace) - distance).abs() < 1e-7);
                        for format in crate::export::Format::ALL {
                            assert!(
                                !crate::export::encode(&model.bodies, format)
                                    .unwrap()
                                    .is_empty()
                            );
                        }
                        let dir = tempfile::tempdir().unwrap();
                        let path = dir.path().join("live-draft.osk");
                        let project = Project::new(
                            model.bodies.clone(),
                            Camera::default(),
                            true,
                            Some(owner.id),
                        );
                        project.save(&path).unwrap();
                        assert_eq!(Project::load(&path).unwrap().bodies, model.bodies);
                        let mut old = serde_json::to_value(&project).unwrap();
                        old["schema_version"] = 38.into();
                        old["modeling_version"] = 38.into();
                        assert!(
                            serde_json::from_value::<Project>(old)
                                .unwrap()
                                .validate()
                                .unwrap_err()
                                .contains("version 39")
                        );
                    }
                    let accepted = model.bodies.clone();
                    assert!(model.delete(plane.id).is_err());
                    assert_eq!(model.bodies, accepted);
                    let mut invalid = model
                        .bodies
                        .iter()
                        .find(|b| b.id == plane.id)
                        .unwrap()
                        .clone();
                    invalid.construction = false;
                    assert!(model.commit(invalid).is_err());
                    assert_eq!(model.bodies, accepted);
                    // Tilt the live neutral support; its trace and absolute requested angle both change.
                    if kind == 0 && !affine && !before {
                        let mut flipped = model
                            .bodies
                            .iter()
                            .find(|b| b.id == plane.id)
                            .unwrap()
                            .clone();
                        flipped.rotation[0] += 180.;
                        model.commit(flipped).unwrap();
                        let flipped_scene = evaluate_scene(&model.bodies).unwrap();
                        assert!(
                            (volume(&flipped_scene.iter().find(|p| p.0 == owner.id).unwrap().1)
                                - (144. - 24. * 5_f64.to_radians().tan()))
                            .abs()
                                < 1e-6,
                            "Flipping the live plane reverses the taper direction"
                        );
                        model.undo();
                        assert_eq!(model.bodies, accepted);
                        let mut tilted = model
                            .bodies
                            .iter()
                            .find(|b| b.id == plane.id)
                            .unwrap()
                            .clone();
                        tilted.rotation[1] = 20.;
                        model.commit(tilted).unwrap();
                        let output = evaluate_scene(&model.bodies).unwrap();
                        let result = &output.iter().find(|p| p.0 == owner.id).unwrap().1;
                        let supports = components::straight_planar_supports(result).unwrap();
                        let angle = (-15_f64).to_radians();
                        let &(normal, _) = supports.iter().find(|(n, _)| n.x > 0.9).unwrap();
                        assert!(
                            (normal.x - angle.cos()).abs() < 1e-7
                                && (normal.z - angle.sin()).abs() < 1e-7
                        );
                        let trace_z = 3. - 3. * 20_f64.to_radians().tan();
                        let expected =
                            144. + 6. * (-15_f64).to_radians().tan() * (trace_z * 4. - 8.);
                        assert!((volume(result) - expected).abs() < 1e-6);
                        model.undo();
                        assert_eq!(model.bodies, accepted);
                        model.redo();
                        assert_ne!(model.bodies, accepted);
                        model.undo();
                        if let Some(path) = std::env::var_os("OSK_LIVE_DRAFT_REVIEW_PROJECT") {
                            let mut review = Project::new(
                                bodies.clone(),
                                Camera::default(),
                                true,
                                Some(owner.id),
                            );
                            review.selection_mode = SelectionMode::Face;
                            review.selected_element = Some(wall);
                            review.selection = vec![(owner.id, Some(wall))];
                            review.save(std::path::Path::new(&path)).unwrap();
                        }
                    }
                    let mut restored = changed[0].clone();
                    restored.component_edits.pop();
                    model.commit(restored).unwrap();
                    assert!(
                        model.delete(plane.id).is_ok(),
                        "Removing Draft releases the reference"
                    );
                }
            }
        }
    }

    #[test]
    fn convex_draft_consumes_redundant_supports_without_losing_history() {
        use crate::{
            components,
            face_operations::{self, FaceEdit},
            selection::SelectionMode,
        };
        let volume = |g: &Geometry| {
            g.triangles
                .iter()
                .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                .sum::<f64>()
        };
        for chamfer in [false, true] {
            for affine in [false, true] {
                let mut body = Body::new(Shape::Rectangle);
                body.id = 1;
                body.width = 6.;
                body.depth = 6.;
                body.extrusion = 4.;
                if affine {
                    body.position = [2., -1., 3.];
                    body.rotation = [11., 17., 23.];
                    body.scale = [1.5, 0.75, 2.];
                }
                if chamfer {
                    let mut modifier = crate::edge_treatments::Modifier::new(
                        crate::edge_treatments::Kind::Chamfer,
                        FaceTarget::Top,
                        &body,
                    );
                    modifier.amount = 0.3;
                    body.edge_treatments.push(modifier);
                }
                let bodies = vec![body.clone()];
                let scene = evaluate_scene(&bodies).unwrap();
                let g = &scene[0].1;
                let inverse = crate::regions::transform(&body).invert().unwrap();
                let mut local = g.clone();
                components::map_geometry(&mut local, inverse);
                let planes = components::planar_supports(&local).unwrap();
                let wall = planes.iter().position(|(n, _)| n.x > 0.99).unwrap();
                let neutral = planes.iter().position(|(n, _)| n.z < -0.99).unwrap();
                for degrees in [-80., -6., 3.] {
                    let operation = EvaluatedDraft {
                        degrees,
                        neutral,
                        planar_loops: true,
                        neutral_plane: None,
                    };
                    let changes = face_operations::selected(
                        &bodies,
                        &scene,
                        &[(1, Some(wall))],
                        FaceEdit::Draft(operation),
                    )
                    .unwrap();
                    assert!(changes[0].component_edits.last().unwrap().convex_rebuild);
                    let mut model = Model::from_bodies(bodies.clone());
                    model.commit_changes(&changes).unwrap();
                    let output = evaluate_scene(&model.bodies).unwrap();
                    let mut result = output[0].1.clone();
                    components::map_geometry(&mut result, inverse);
                    assert!(result.is_solid);
                    assert!(result.shell.as_ref().unwrap().is_geometric_consistent());
                    components::solid_parts(result.shell.as_ref().unwrap()).unwrap();
                    assert_eq!(result.shell.as_ref().unwrap().len(), result.face_count);
                    if degrees == -80. {
                        assert_eq!(
                            result.face_count, 5,
                            "The top cap and all chamfer bands are redundant"
                        );
                        let slope = degrees.to_radians().tan();
                        let height = -6. / slope;
                        let expected = 6. * (6. * height + slope * height * height / 2.);
                        assert!(
                            (volume(&result) - expected).abs() < 1e-6,
                            "The surviving wedge has the exact integrated volume"
                        );
                    } else if chamfer && degrees == -6. {
                        assert_eq!(
                            result.face_count,
                            g.face_count - 1,
                            "Only the right transition support disappears"
                        );
                    } else {
                        assert_eq!(result.face_count, g.face_count);
                    }
                    let after = components::planar_supports(&result).unwrap();
                    let normal =
                        Vector3::new(degrees.to_radians().cos(), 0., -degrees.to_radians().sin());
                    let distance = 3. * degrees.to_radians().cos();
                    assert!(
                        after.iter().any(|(n, d)| (*n - normal).magnitude() < 1e-7
                            && (*d - distance).abs() < 1e-7)
                    );
                    for (n, d) in after
                        .iter()
                        .filter(|(n, _)| (*n - normal).magnitude() > 1e-7)
                    {
                        assert!(
                            planes.iter().any(
                                |(m, e)| (*m - *n).magnitude() < 1e-7 && (*e - *d).abs() < 1e-7
                            ),
                            "Surviving untouched supports stay fixed"
                        );
                    }
                    let mut old = changes[0].clone();
                    old.component_edits.last_mut().unwrap().convex_rebuild = false;
                    if result.face_count < g.face_count {
                        assert!(
                            evaluate(&old).is_err(),
                            "Legacy Draft continues to reject support consumption"
                        );
                    }
                    for format in crate::export::Format::ALL {
                        assert!(
                            !crate::export::encode(&model.bodies, format)
                                .unwrap()
                                .is_empty()
                        );
                    }
                    let dir = tempfile::tempdir().unwrap();
                    let path = dir.path().join("convex-draft.osk");
                    let project =
                        Project::new(model.bodies.clone(), Camera::default(), true, Some(1));
                    project.save(&path).unwrap();
                    assert_eq!(Project::load(&path).unwrap().bodies, model.bodies);
                    let mut old = serde_json::to_value(&project).unwrap();
                    old["schema_version"] = 40.into();
                    old["modeling_version"] = 40.into();
                    assert!(
                        serde_json::from_value::<Project>(old)
                            .unwrap()
                            .validate()
                            .unwrap_err()
                            .contains("version 41")
                    );
                    let accepted = model.bodies.clone();
                    model.undo();
                    assert_eq!(model.bodies, bodies);
                    model.redo();
                    assert_eq!(model.bodies, accepted);
                    let side = after.iter().position(|(n, _)| n.y > 0.99).unwrap();
                    let placement = crate::regions::transform(&body);
                    let moved = components::transformed(
                        &changes[0],
                        &output[0].1,
                        SelectionMode::Face,
                        Some(side),
                        placement * Matrix4::from_translation(Vector3::unit_y() * 0.1) * inverse,
                    )
                    .unwrap();
                    model.commit(moved).unwrap();
                    Project::new(model.bodies.clone(), Camera::default(), true, Some(1))
                        .save(&path)
                        .unwrap();
                    assert_eq!(Project::load(&path).unwrap().bodies, model.bodies);
                    let mut removed = changes[0].clone();
                    removed.component_edits.pop();
                    model.commit(removed).unwrap();
                    assert_eq!(model.bodies, bodies);
                    if chamfer
                        && !affine
                        && degrees == -6.
                        && let Some(path) = std::env::var_os("OSK_CONVEX_DRAFT_REVIEW_PROJECT")
                    {
                        let mut review =
                            Project::new(bodies.clone(), Camera::default(), true, Some(1));
                        review.selection_mode = SelectionMode::Face;
                        review.selected_element = Some(wall);
                        review.selection = vec![(1, Some(wall))];
                        review.save(std::path::Path::new(&path)).unwrap();
                    }
                }
            }
        }
    }

    #[test]
    fn source_draft_keeps_neutral_cap_exact_surfaces_history_and_extents() {
        for shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            for neutral in [FaceTarget::Bottom, FaceTarget::Top] {
                for degrees in [-7., 0., 9.] {
                    let mut b = Body::new(shape);
                    b.id = 1;
                    b.width = 6.;
                    b.depth = 4.;
                    b.extrusion = 5.;
                    b.position = [3., -2., 1.];
                    b.rotation = [15., 25., 35.];
                    b.scale = [1.5, 0.75, 2.];
                    let original = b.clone();
                    b.taper = Some(Draft { degrees, neutral });
                    let g = evaluate(&b).unwrap();
                    assert!(g.is_solid);
                    Solid::try_new(vec![g.shell.clone().unwrap()]).unwrap();
                    let inverse = crate::regions::transform(&b).invert().unwrap();
                    let volume = g
                        .triangles
                        .iter()
                        .map(|t| t.map(|p| inverse.transform_point(p)))
                        .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                        .sum::<f64>();
                    let d = b.extrusion * degrees.to_radians().tan();
                    let expected = if shape == Shape::Circle {
                        let r = b.width / 2.;
                        std::f64::consts::PI * b.extrusion * (r * r + r * d + d * d / 3.)
                    } else if shape == Shape::Semicircle {
                        let r = b.width / 2.;
                        let area = |fraction: f64| {
                            let q = d * fraction;
                            let radius = r + q;
                            radius * radius * (-q / radius).acos() + q * (r * r + 2. * r * q).sqrt()
                        };
                        let n = 256;
                        let sum = area(0.)
                            + area(1.)
                            + (1..n)
                                .map(|i| {
                                    area(i as f64 / n as f64) * if i % 2 == 0 { 2. } else { 4. }
                                })
                                .sum::<f64>();
                        sum * b.extrusion / (3. * n as f64)
                    } else {
                        b.extrusion
                            * (b.width * b.depth + (b.width + b.depth) * d + 4. * d * d / 3.)
                    };
                    // Mesh volume is approximate at the viewport's 0.01 chord
                    // tolerance. Check the exact NURBS cone independently below.
                    let tolerance = if shape != Shape::Rectangle {
                        let r = b.width / 2. + d.max(0.);
                        std::f64::consts::PI * b.extrusion * (2. * r * 0.01 + 0.01_f64.powi(2))
                    } else {
                        1e-8
                    };
                    assert!(
                        (volume - expected).abs() < tolerance,
                        "{shape:?} {neutral:?} {degrees}: {volume} vs {expected}"
                    );
                    let z = if neutral == FaceTarget::Bottom {
                        0.
                    } else {
                        b.extrusion
                    };
                    assert!(
                        g.points.iter().any(|p| {
                            let p = inverse.transform_point(*p);
                            (p.z - z).abs() < 1e-6 && (p.x - b.width / 2.).abs() < 1e-6
                        }),
                        "Neutral cap retains its original dimensions/plane"
                    );
                    assert_eq!(
                        g.face_count,
                        match shape {
                            Shape::Circle => 3,
                            Shape::Semicircle => 4,
                            _ => 6,
                        }
                    );
                    if shape == Shape::Circle {
                        assert_eq!(g.circles.len(), 2);
                    }
                    for face in g.shell.as_ref().unwrap() {
                        let surface = face.oriented_surface();
                        let normal = |u, v| {
                            inverse
                                .transform_vector(surface.uder(u, v))
                                .cross(inverse.transform_vector(surface.vder(u, v)))
                        };
                        let slope = if neutral == FaceTarget::Bottom {
                            -degrees.to_radians().tan()
                        } else {
                            degrees.to_radians().tan()
                        };
                        if matches!(surface, Surface::Plane(_)) {
                            let n = normal(0.5, 0.5);
                            if n.x.hypot(n.y) > 1e-8 {
                                assert!((n.z / n.x.hypot(n.y) - slope).abs() < 1e-8);
                            }
                            continue;
                        }
                        let (ur, vr) = surface.try_range_tuple();
                        let (u0, u1) = ur.unwrap();
                        let (v0, v1) = vr.unwrap();
                        let midpoint =
                            inverse.transform_point(surface.subs((u0 + u1) / 2., (v0 + v1) / 2.));
                        let midfraction = if neutral == FaceTarget::Bottom {
                            midpoint.z / b.extrusion
                        } else {
                            1. - midpoint.z / b.extrusion
                        };
                        let flat = shape == Shape::Semicircle
                            && (midpoint.y + d * midfraction).abs() < 1e-8;
                        for a in [0.17, 0.5, 0.83] {
                            for c in [0.2, 0.6, 0.9] {
                                let u = u0 + (u1 - u0) * a;
                                let v = v0 + (v1 - v0) * c;
                                let p = inverse.transform_point(surface.subs(u, v));
                                let fraction = if neutral == FaceTarget::Bottom {
                                    p.z / b.extrusion
                                } else {
                                    1. - p.z / b.extrusion
                                };
                                if shape != Shape::Rectangle && !flat {
                                    assert!(
                                        (p.x.hypot(p.y) - (b.width / 2. + d * fraction)).abs()
                                            < 1e-8,
                                        "Retained surface is an exact cone/cylinder"
                                    );
                                }
                                let n = normal(u, v);
                                assert!(
                                    (n.z / n.x.hypot(n.y) - slope).abs() < 1e-8,
                                    "Surface retains requested draft angle"
                                );
                            }
                        }
                    }
                    if shape == Shape::Circle {
                        for ring in &g.circles {
                            let centre = inverse.transform_point(ring.center);
                            let radius = inverse.transform_vector(ring.u).magnitude();
                            let r = if (centre.z - z).abs() < 1e-6 {
                                b.width / 2.
                            } else {
                                b.width / 2. + d
                            };
                            assert!((radius - r).abs() < 1e-6);
                        }
                    }
                    if shape == Shape::Semicircle {
                        let mut seen = std::collections::HashSet::new();
                        for edge in g.shell.as_ref().unwrap().edge_iter() {
                            if let Curve::BSplineCurve(curve) = edge.oriented_curve() {
                                seen.insert(edge.id());
                                let (a, c) = curve.range_tuple();
                                for fraction in [0.13, 0.5, 0.87] {
                                    let p =
                                        inverse.transform_point(curve.subs(a + (c - a) * fraction));
                                    let z = if neutral == FaceTarget::Bottom {
                                        p.z
                                    } else {
                                        b.extrusion - p.z
                                    };
                                    let q = d * z / b.extrusion;
                                    assert!((p.y + q).abs() < 1e-8);
                                    assert!(
                                        (p.x.hypot(p.y) - (b.width / 2. + q)).abs() < 1e-8,
                                        "Corner lies on both the exact cone and the drafted diameter plane"
                                    );
                                }
                            }
                        }
                        assert_eq!(
                            seen.len(),
                            2,
                            "The two corner trims are exact shared parabolic edges"
                        );
                    }
                    let mut model = Model::from_bodies(vec![original.clone()]);
                    model.edit(b.clone()).unwrap();
                    model.finish_edit();
                    model.undo();
                    assert_eq!(model.bodies, vec![original]);
                    model.redo();
                    assert_eq!(model.bodies, vec![b.clone()]);
                    let mut invalid = b.clone();
                    invalid.taper.as_mut().unwrap().degrees = -80.;
                    assert!(model.edit(invalid).is_err());
                    assert_eq!(model.bodies, vec![b.clone()]);
                    let dir = tempfile::tempdir().unwrap();
                    let path = dir.path().join("draft.osk");
                    let project = Project::new(vec![b.clone()], Camera::default(), true, Some(1));
                    project.save(&path).unwrap();
                    assert_eq!(Project::load(&path).unwrap().bodies, vec![b.clone()]);
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
                    let mut old = serde_json::to_value(&project).unwrap();
                    old["schema_version"] = 14.into();
                    old["modeling_version"] = 14.into();
                    std::fs::write(&path, serde_json::to_vec(&old).unwrap()).unwrap();
                    assert!(
                        Project::load(&path)
                            .err()
                            .unwrap()
                            .contains("Draft operations require .osk version 15")
                    );
                    old["bodies"][0].as_object_mut().unwrap().remove("taper");
                    std::fs::write(&path, serde_json::to_vec(&old).unwrap()).unwrap();
                    assert!(Project::load(&path).unwrap().bodies[0].taper.is_none());
                }
            }
        }
        // Extent coverage must use the actual drafted cap, not its neutral
        // primitive radius. This profile fits only the expanded top cap.
        let mut support = Body::new(Shape::Rectangle);
        support.id = 1;
        support.width = 10.;
        support.depth = 10.;
        support.extrusion = 1.;
        support.position[2] = 10.;
        let mut end = Body::new(Shape::Circle);
        end.id = 2;
        end.width = 6.;
        end.extrusion = 5.;
        end.taper = Some(Draft {
            degrees: 9.,
            neutral: FaceTarget::Bottom,
        });
        let mut profile = Body::new(Shape::Circle);
        profile.id = 3;
        profile.width = 0.5;
        profile.position = [3.4, 0., 10.];
        profile.rotation[0] = 180.;
        let mut bore = crate::operators::Bore::new(&support, FaceTarget::Bottom).unwrap();
        bore.travel = 5.;
        bore.extent = crate::operators::Extent::UpToNext { target: 2 };
        profile.bore = Some(bore);
        let scene = evaluate_scene(&[support, end, profile]).unwrap();
        assert!(scene[2].1.points.iter().any(|p| (p.z - 5.).abs() < 1e-6));
        // A shell's open annular rim cannot terminate a profile through its hole.
        let mut support = Body::new(Shape::Rectangle);
        support.id = 1;
        support.extrusion = 1.;
        let mut end = Body::new(Shape::Circle);
        end.id = 2;
        end.width = 4.;
        end.extrusion = 3.;
        end.position[2] = 3.;
        end.hollow = Some(crate::shelling::Hollow {
            outward: false,
            closed: false,
            opening: FaceTarget::Bottom,
            thickness: 0.25,
        });
        let mut profile = Body::new(Shape::Circle);
        profile.id = 3;
        profile.width = 1.;
        profile.position[2] = 1.;
        let mut bore = crate::operators::Bore::new(&support, FaceTarget::Top).unwrap();
        bore.travel = 5.;
        bore.extent = crate::operators::Extent::UpToNext { target: 2 };
        profile.bore = Some(bore);
        let scene = evaluate_scene(&[support, end, profile]).unwrap();
        assert!(scene[2].1.points.iter().any(|p| (p.z - 5.75).abs() < 1e-6));
        assert!(!scene[2].1.points.iter().any(|p| (p.z - 3.).abs() < 1e-6));
        let mut b = Body::new(Shape::Rectangle);
        b.extrusion = 3.;
        for degrees in [f64::NAN, f64::INFINITY, 81., -80.] {
            b.taper = Some(Draft {
                degrees,
                neutral: FaceTarget::Bottom,
            });
            assert!(b.validate().is_err());
        }
        b.taper = Some(Draft {
            degrees: 3.,
            neutral: FaceTarget::Left,
        });
        assert!(b.validate().is_err());
    }
}
