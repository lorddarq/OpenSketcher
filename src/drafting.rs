//! Exact source-prism draft, anchored to a neutral cap.
use crate::{
    edge_treatments::FaceTarget,
    model::{Body, Geometry, Shape},
};
use serde::{Deserialize, Serialize};
use truck_modeling::{Matrix4, Point3, Solid, Vector3, builder};

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
