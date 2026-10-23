//! Sweeps of evaluated planar faces, preserving the existing treated boundary.
use crate::{
    edge_treatments::FaceTarget,
    model::{Body, Geometry},
};
use serde::{Deserialize, Serialize};
use truck_modeling::{EuclideanSpace, InnerSpace, Solid, SquareMatrix, Transform, builder};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Extrusion {
    pub face: FaceTarget,
    pub distance: f64,
}
pub fn validate(b: &Body) -> Result<(), String> {
    if b.face_extrusions.len() > 64
        || b.face_extrusions
            .iter()
            .any(|e| !e.distance.is_finite() || e.distance == 0. || e.distance.abs() > 100.)
    {
        return Err(
            "Face extrusion requires a nonzero distance within ±100 mm (maximum 64 operations)."
                .into(),
        );
    }
    if !b.face_extrusions.is_empty()
        && (crate::components::has_source_edits(b) || b.bore.is_some() || b.boolean.is_some())
    {
        return Err(
            "Face extrusion of component-deformed or Boolean geometry is not supported yet.".into(),
        );
    }
    Ok(())
}
pub fn target(b: &Body, g: &Geometry, face: usize) -> Result<FaceTarget, String> {
    let inverse = crate::regions::transform(b)
        .invert()
        .ok_or("Invalid placement.")?;
    let points: Vec<_> = g
        .triangles
        .iter()
        .zip(&g.triangle_faces)
        .filter(|(_, f)| **f == face)
        .flat_map(|(t, _)| t.iter().map(|p| inverse.transform_point(*p)))
        .collect();
    for role in [
        FaceTarget::Top,
        FaceTarget::Bottom,
        FaceTarget::Left,
        FaceTarget::Right,
        FaceTarget::Front,
        FaceTarget::Back,
    ] {
        let (n, mut d) = role.plane(b);
        d += b
            .face_extrusions
            .iter()
            .filter(|e| e.face == role)
            .map(|e| e.distance)
            .sum::<f64>();
        if b.extrusion > 0.
            && role.supported(b)
            && !points.is_empty()
            && points.iter().all(|p| (p.to_vec().dot(n) - d).abs() < 1e-6)
        {
            return Ok(role);
        }
    }
    Err("Select a planar base face or its extruded cap.".into())
}
pub fn apply(b: &Body, g: &mut Geometry) -> Result<(), String> {
    let mut prior = b.clone();
    prior.face_extrusions.clear();
    // Reversing the same cap retracts its previous sweep; it must not leave
    // coincident inner walls from an outward sweep followed by an inward one.
    let mut operations: Vec<Extrusion> = Vec::new();
    for operation in &b.face_extrusions {
        let mut combined = operation.clone();
        if operation.distance < 0.
            || operations
                .last()
                .is_some_and(|e| e.face == operation.face && e.distance < 0.)
        {
            while operations.last().is_some_and(|e| e.face == operation.face) {
                combined.distance += operations.pop().unwrap().distance;
            }
        }
        // Preserve existing all-outward histories and their component topology.
        operations.push(combined);
    }
    for operation in operations.iter().filter(|e| e.distance.abs() > 1e-8) {
        let previous = prior
            .face_extrusions
            .iter()
            .filter(|e| e.face == operation.face)
            .map(|e| e.distance)
            .sum::<f64>();
        if previous * operation.distance < 0. {
            return Err("Reversing a cap after editing another face requires undoing the intervening face operation first.".into());
        }
        if operation.distance < 0. {
            if !b
                .edge_treatments
                .iter()
                .any(|m| m.face == operation.face && m.edge.is_none())
            {
                return Err("Inward face extrusion currently requires a fully chamfered or beveled face perimeter; use Offset source to retract an untreated face.".into());
            }
            if operations
                .iter()
                .any(|e| e.face != operation.face && e.face != operation.face.opposite())
            {
                return Err("Intersecting face sweeps on different axes require a general trim/cut operation.".into());
            }
            let normal = operation.face.plane(b).0;
            let thickness = if normal.z.abs() > 0.5 {
                b.extrusion
            } else if normal.x.abs() > 0.5 {
                b.width
            } else {
                b.depth
            };
            let opposite = operation.face.opposite();
            let opposite_setback = b
                .edge_treatments
                .iter()
                .map(|m| {
                    let (a, c) = m.setbacks();
                    if m.face == opposite {
                        c
                    } else if m.edge == Some(opposite) {
                        a
                    } else {
                        0.
                    }
                })
                .fold(0., f64::max);
            let opposite_recess = prior
                .face_extrusions
                .iter()
                .filter(|e| e.face == opposite)
                .map(|e| e.distance)
                .sum::<f64>()
                .min(0.);
            if -operation.distance
                >= thickness + previous + opposite_recess - opposite_setback - 0.01
            {
                return Err("Inward face extrusion would reach the opposite wall or its treatment. Leave at least 0.01 local units; use a Borer operation for a through-cut.".into());
            }
        }
        let shell = g
            .shell
            .as_ref()
            .ok_or("Face extrusion requires an exact boundary.")?;
        let raw = crate::model::tessellate(shell)?;
        let indices: Vec<_> = (0..raw.face_count)
            .filter(|f| target(&prior, &raw, *f) == Ok(operation.face))
            .collect();
        if indices.is_empty() {
            return Err("The extrusion's planar face no longer exists.".into());
        }
        let selected: truck_modeling::Shell = indices.iter().map(|i| shell[*i].clone()).collect();
        let face = if indices.len() == 1 {
            selected[0].clone()
        } else {
            builder::try_attach_plane(&selected.extract_boundaries())
                .map_err(|e| format!("Cannot join the selected planar face: {e}"))?
        };
        let vector = crate::regions::transform(b)
            .transform_vector(operation.face.plane(b).0 * operation.distance);
        let sweep: Solid = builder::tsweep(&face, vector);
        let mut joined = shell.clone();
        let mut roles = if g.surface_roles.len() == shell.len() {
            g.surface_roles.clone()
        } else if g.triangle_faces.len() == shell.len() {
            g.triangle_faces.clone()
        } else {
            return Err("Missing evaluated surface mapping.".into());
        };
        let cap_role = roles[indices[0]];
        for index in indices.iter().rev() {
            joined.remove(*index);
            roles.remove(*index);
        }
        let side_role = roles.iter().copied().max().unwrap_or(0).max(cap_role) + 1;
        for patch in sweep.boundaries()[0].iter() {
            if patch.id() == face.id() {
                continue;
            }
            // The translated cap is last in Truck's translational face sweep.
            let cap = patch.id() == sweep.boundaries()[0].last().unwrap().id();
            joined.push(patch.clone());
            roles.push(if cap {
                cap_role
            } else if b.shape == crate::model::Shape::Circle {
                side_role
            } else {
                side_role + roles.len()
            });
        }
        Solid::try_new(vec![joined.clone()])
            .map_err(|e| format!("Face extrusion did not produce a closed solid: {e}"))?;
        let mut result = crate::model::tessellate(&joined)?;
        crate::edge_treatments::merge_surfaces(&mut result, &roles);
        result.is_solid = true;
        // Retain snap circles on the existing boundary and add the moved cap rim.
        result.circles = g.circles.clone();
        let (normal, mut plane) = operation.face.plane(&prior);
        plane += prior
            .face_extrusions
            .iter()
            .filter(|e| e.face == operation.face)
            .map(|e| e.distance)
            .sum::<f64>();
        let inverse = crate::regions::transform(b)
            .invert()
            .ok_or("Invalid placement.")?;
        for ring in &g.circles {
            if (inverse.transform_point(ring.center).to_vec().dot(normal) - plane).abs() < 1e-6 {
                let mut moved = ring.clone();
                moved.center += vector;
                result.circles.push(moved);
            }
        }
        *g = result;
        prior.face_extrusions.push(operation.clone());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        edge_treatments::{Kind, Modifier},
        model::{Shape, evaluate},
    };
    #[test]
    fn treated_cap_extrudes_without_moving_treatment() {
        for shape in [Shape::Circle, Shape::Rectangle] {
            for kind in [Kind::Chamfer, Kind::Bevel] {
                for face in [FaceTarget::Top, FaceTarget::Bottom] {
                    for distance in [1., -1.] {
                        let mut b = Body::new(shape);
                        b.id = 1;
                        b.width = 4.;
                        b.depth = 4.;
                        b.extrusion = 4.;
                        b.edge_treatments.push(Modifier::new(kind, face, &b));
                        b.position = [3., -2., 1.];
                        b.rotation = [15., 25., 35.];
                        b.scale = [1.5, 0.75, 2.];
                        let inverse = crate::regions::transform(&b).invert().unwrap();
                        let original = evaluate(&b).unwrap();
                        b.face_extrusions.push(Extrusion { face, distance });
                        let changed = evaluate(&b).unwrap();
                        assert!(changed.is_solid);
                        let volume = |g: &Geometry| {
                            g.triangles
                                .iter()
                                .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                                .sum::<f64>()
                        };
                        let cap_area = original
                            .triangles
                            .iter()
                            .zip(&original.triangle_faces)
                            .filter(|(_, f)| {
                                target(
                                    &Body {
                                        face_extrusions: vec![],
                                        ..b.clone()
                                    },
                                    &original,
                                    **f,
                                ) == Ok(face)
                            })
                            .map(|(t, _)| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
                            .sum::<f64>();
                        let expected = cap_area
                            * crate::regions::transform(&b)
                                .transform_vector(face.plane(&b).0 * distance)
                                .magnitude()
                            * distance.signum();
                        assert!(
                            (volume(&changed) - volume(&original) - expected).abs() < 0.1,
                            "The signed sweep must add/remove the cap's volume"
                        );
                        assert!(
                            (0..changed.face_count).any(|f| target(&b, &changed, f) == Ok(face)),
                            "Extruded cap must remain separately selectable"
                        );
                        assert!(changed.face_count <= original.face_count + 4);
                        assert!(changed.points.iter().any(|p| {
                            (inverse.transform_point(*p).z
                                - if face == FaceTarget::Top {
                                    4. + distance
                                } else {
                                    -distance
                                })
                            .abs()
                                < 1e-6
                        }));
                        for p in &original.points {
                            assert!(
                                changed.points.iter().any(|q| (*p - *q).magnitude() < 1e-6),
                                "Existing treatment moved"
                            );
                        }
                        b.face_extrusions.push(Extrusion {
                            face,
                            distance: 0.5,
                        });
                        evaluate(&b).unwrap();
                        let mut model = crate::model::Model::default();
                        model.commit(b.clone()).unwrap();
                        let mut next = model.bodies[0].clone();
                        next.face_extrusions.push(Extrusion {
                            face,
                            distance: 0.25,
                        });
                        model.edit(next).unwrap();
                        model.finish_edit();
                        model.undo();
                        assert_eq!(model.bodies[0].face_extrusions.len(), 2);
                        model.redo();
                        assert_eq!(model.bodies[0].face_extrusions.len(), 3);
                        let mut invalid = model.bodies[0].clone();
                        invalid.face_extrusions[0].distance = -10.;
                        assert!(model.edit(invalid).is_err());
                        assert_eq!(model.bodies[0].face_extrusions.len(), 3);
                        let saved = serde_json::to_vec(&b).unwrap();
                        assert_eq!(serde_json::from_slice::<Body>(&saved).unwrap(), b);
                        if shape == Shape::Circle
                            && kind == Kind::Chamfer
                            && face == FaceTarget::Top
                        {
                            let path = std::env::temp_dir()
                                .join(format!("osk-face-extrusion-{}.osk", std::process::id()));
                            crate::project::Project::new(
                                vec![b.clone()],
                                crate::viewport::Camera::default(),
                                true,
                                Some(1),
                            )
                            .save(&path)
                            .unwrap();
                            assert_eq!(
                                crate::project::Project::load(&path).unwrap().bodies,
                                vec![b.clone()]
                            );
                            std::fs::remove_file(&path).unwrap();
                            for format in [
                                crate::export::Format::Stl,
                                crate::export::Format::Glb,
                                crate::export::Format::Step,
                                crate::export::Format::Dxf,
                            ] {
                                let path = path.with_extension(format.extension());
                                crate::export::save(&[b.clone()], format, &path).unwrap();
                                assert!(std::fs::metadata(&path).unwrap().len() > 0);
                                std::fs::remove_file(path).unwrap();
                            }
                        }
                    }
                }
            }
        }
        let mut b = Body::new(Shape::Circle);
        b.id = 1;
        b.width = 4.;
        b.extrusion = 4.;
        for face in [FaceTarget::Top, FaceTarget::Bottom] {
            b.edge_treatments
                .push(Modifier::new(Kind::Chamfer, face, &b));
        }
        let base = evaluate(&b).unwrap();
        b.face_extrusions = vec![
            Extrusion {
                face: FaceTarget::Top,
                distance: 1.,
            },
            Extrusion {
                face: FaceTarget::Top,
                distance: 1.,
            },
        ];
        let extended = evaluate(&b).unwrap();
        assert_eq!(
            extended.face_count,
            base.face_count + 2,
            "Legacy outward sweeps retain their topology"
        );
        b.face_extrusions.push(Extrusion {
            face: FaceTarget::Top,
            distance: -2.,
        });
        assert_eq!(
            evaluate(&b).unwrap().topology,
            base.topology,
            "Full retraction restores the original boundary"
        );
        b.face_extrusions = vec![
            Extrusion {
                face: FaceTarget::Top,
                distance: -3.,
            },
            Extrusion {
                face: FaceTarget::Bottom,
                distance: -2.,
            },
        ];
        assert!(evaluate(&b).err().unwrap().contains("opposite wall"));
        b.face_extrusions = vec![Extrusion {
            face: FaceTarget::Top,
            distance: -4.,
        }];
        assert!(evaluate(&b).err().unwrap().contains("opposite wall"));
        b.edge_treatments.clear();
        b.face_extrusions[0].distance = -1.;
        assert!(
            evaluate(&b)
                .err()
                .unwrap()
                .contains("fully chamfered or beveled")
        );
    }
}
