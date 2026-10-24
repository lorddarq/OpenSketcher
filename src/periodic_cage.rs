//! One Cartesian polynomial field for an entire periodic cylinder, including kernel patch joins.
use super::*;
use truck_modeling::{BSplineCurve, Curve, Invertible, Matrix3};

pub(super) fn create(g: &Geometry, face: usize) -> Result<Cage, String> {
    let mut grouped = g.clone();
    grouped.group_cylinder_patches();
    let faces = grouped.face_elements(face);
    if faces.len() < 2
        || grouped
            .cylinder_supports
            .get(face)
            .is_none_or(Option::is_none)
    {
        return Err("A periodic cage requires a complete retained cylinder wall.".into());
    }
    let shell = g
        .shell
        .as_ref()
        .ok_or("Missing retained cylinder surfaces.")?;
    let mapping = g.surface_mapping()?;
    let indices = (0..shell.len())
        .filter(|&i| faces.contains(&mapping[i]))
        .collect::<Vec<_>>();
    if shell
        .iter()
        .enumerate()
        .any(|(i, f)| !indices.contains(&i) && !matches!(f.surface(), Surface::Plane(_)))
    {
        return Err("Create the periodic wall cage before cap cages; this input requires one cylinder with planar end caps.".into());
    }
    let wall: Shell = indices.iter().map(|&i| shell[i].clone()).collect();
    let boundaries = wall.extract_boundaries();
    if wall.connected_components().len() != 1 || boundaries.len() != 2 {
        return Err("The selected wall is not one complete periodic domain.".into());
    }
    let mut rings = boundaries
        .iter()
        .map(|w| {
            g.circles
                .iter()
                .find(|c| crate::face_extrusions::circle_on_wire(c, w))
                .cloned()
                .ok_or("A periodic wall needs two complete circular section boundaries.")
        })
        .collect::<Result<Vec<_>, _>>()?;
    let normal = rings[0].u.cross(rings[0].v).normalize();
    rings.sort_by(|a, b| {
        a.center
            .to_vec()
            .dot(normal)
            .total_cmp(&b.center.to_vec().dot(normal))
    });
    if (rings[0].u - rings[1].u).magnitude() > 1e-7 || (rings[0].v - rings[1].v).magnitude() > 1e-7
    {
        return Err("Periodic cages require matching translated circular sections.".into());
    }
    let origin = rings[0].center;
    let u = rings[0].u;
    let v = rings[0].v;
    let axis = rings[1].center - origin;
    if axis.dot(normal).abs() < 1e-7 {
        return Err("The cylinder height is degenerate.".into());
    }
    let controls = (0..8)
        .map(|i| {
            let angle = i as f64 * std::f64::consts::TAU / 8.;
            (0..5)
                .map(|j| {
                    (origin + u * angle.cos() + v * angle.sin() + axis * (j as f64 / 4.))
                        .to_vec()
                        .extend(1.)
                })
                .collect()
        })
        .collect();
    let net = NurbsSurface::new(BSplineSurface::new(
        (KnotVec::bezier_knot(7), KnotVec::bezier_knot(4)),
        controls,
    ));
    let mut binding = Binding::of(&net);
    binding.periodic = true;
    binding.patches = indices.clone();
    let patches = indices
        .into_iter()
        .map(|i| {
            let Surface::NurbsSurface(net) = shell[i].surface() else {
                return Err("Periodic cages require retained rational ruled patches.");
            };
            Ok((i, net))
        })
        .collect::<Result<Vec<_>, _>>()?;
    Ok(Cage {
        domain: 0,
        face,
        binding,
        reference: net.clone(),
        net,
        patches,
    })
}

fn product(a: &[f64], b: &[f64]) -> Vec<f64> {
    let n = a.len() - 1;
    let m = b.len() - 1;
    let mut out = vec![0.; n + m + 1];
    for (i, a) in a.iter().enumerate() {
        for (j, b) in b.iter().enumerate() {
            out[i + j] += a * b * choose(n, i) * choose(m, j) / choose(n + m, i + j);
        }
    }
    out
}
fn frame(cage: &Cage) -> Result<(Point3, Matrix3), String> {
    let p = positions(&cage.reference);
    let origin = p[0] + (p[20] - p[0]) / 2.;
    let u = p[0] - origin;
    let v = p[10] - origin;
    let axis = p[4] - p[0];
    let inverse = Matrix3::from_cols(u, v, axis)
        .invert()
        .ok_or("The periodic cage reference frame is singular.")?;
    Ok((origin, inverse))
}
fn vertical(cage: &Cage) -> Vec<Vec<Vector3>> {
    (0..8)
        .map(|i| {
            let delta = (0..5)
                .map(|j| {
                    cage.net.control_point(i, j).truncate()
                        - cage.reference.control_point(i, j).truncate()
                })
                .collect::<Vec<_>>();
            let components = (0..3)
                .map(|k| {
                    product(
                        &delta.iter().map(|p| p[k]).collect::<Vec<_>>(),
                        &[0., 0., 16. / 6., 0., 0.],
                    )
                })
                .collect::<Vec<_>>();
            (0..9)
                .map(|j| Vector3::new(components[0][j], components[1][j], components[2][j]))
                .collect()
        })
        .collect()
}
fn skin(
    source: &SurfaceNet,
    origin: Point3,
    inverse: Matrix3,
    vertical: &[Vec<Vector3>],
) -> Result<SurfaceNet, String> {
    let mut net = source.clone();
    let swap = net.udegree() == 1 && net.control_points().len() == 2;
    if swap {
        net.invert();
    }
    let p = net.control_points();
    let degree = net.udegree();
    if (2..=4).contains(&degree)
        && net.vdegree() == 1
        && p.iter().all(|row| row.len() == 2)
        && p.len() != degree + 1
    {
        // Reuse Truck's exact Bézier extraction, then stitch the calculated spans back
        // into one NURBS carrier for the same retained kernel face.
        let a = net.non_rationalized().row_curve(0).bezier_decomposition();
        let b = net.non_rationalized().row_curve(1).bezier_decomposition();
        if a.len() != b.len() || a.is_empty() {
            return Err("The ruled section knot spans no longer agree.".into());
        }
        let count = a.len();
        let mut controls = Vec::new();
        let mut knots = Vec::new();
        let out_degree = 3 * degree;
        for (span, (a, b)) in a.iter().zip(&b).enumerate() {
            let input = NurbsSurface::new(BSplineSurface::new(
                (a.knot_vec().clone(), KnotVec::bezier_knot(1)),
                a.control_points()
                    .iter()
                    .zip(b.control_points())
                    .map(|(a, b)| vec![*a, *b])
                    .collect(),
            ));
            let piece = skin(&input, origin, inverse, vertical)?;
            let (lo, hi) = a.range_tuple();
            if span == 0 {
                knots.extend(std::iter::repeat_n(lo, out_degree + 1));
            }
            controls.extend(
                piece
                    .control_points()
                    .iter()
                    .skip(usize::from(span > 0))
                    .cloned(),
            );
            knots.extend(std::iter::repeat_n(
                hi,
                if span + 1 == count {
                    out_degree + 1
                } else {
                    out_degree
                },
            ));
        }
        let mut result = NurbsSurface::new(BSplineSurface::new(
            (KnotVec::from(knots), KnotVec::bezier_knot(8)),
            controls,
        ));
        if swap {
            result.invert();
        }
        regularity(&result, false)?;
        return Ok(result);
    }
    if !(2..=4).contains(&degree)
        || p.len() != degree + 1
        || net.vdegree() != 1
        || p.iter().any(|row| row.len() != 2)
    {
        return Err(format!(
            "Periodic deformation requires rational Bézier cylinder patches with linear axial rows (degrees {:?}, lattice {}×{}).",
            net.degrees(),
            p.len(),
            p[0].len()
        ));
    }
    let coordinate = |p: Vector4| inverse * (p.truncate() - origin.to_vec() * p.w);
    let reverse = (coordinate(p[0][0]).z / p[0][0].w) > 0.5;
    let (a, b) = if reverse { (1, 0) } else { (0, 1) };
    let mut w = Vec::new();
    let mut x = Vec::new();
    let mut y = Vec::new();
    for row in p {
        let q = coordinate(row[a]);
        let r = coordinate(row[b]);
        if row[a].w <= 0.
            || (row[a].w - row[b].w).abs() > 1e-10
            || (q.z / row[a].w).abs() > 1e-7
            || (r.z / row[b].w - 1.).abs() > 1e-7
            || (q.x - r.x).abs() > 1e-7
            || (q.y - r.y).abs() > 1e-7
        {
            return Err(
                "The periodic cage input no longer has matching ruled cylinder rows.".into(),
            );
        }
        w.push(row[a].w);
        x.push(q.x);
        y.push(q.y);
    }
    let squared = product(&w, &w);
    let cubed = product(&squared, &w);
    let base = (0..2)
        .map(|j| {
            (0..3)
                .map(|k| {
                    product(
                        &p.iter()
                            .map(|row| row[if j == 0 { a } else { b }][k])
                            .collect::<Vec<_>>(),
                        &squared,
                    )
                })
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let angular = (0..8)
        .map(|i| {
            let angle = i as f64 * std::f64::consts::TAU / 8.;
            let linear = w
                .iter()
                .zip(&x)
                .zip(&y)
                .map(|((w, x), y)| w + x * angle.cos() + y * angle.sin())
                .collect::<Vec<_>>();
            product(&product(&linear, &linear), &linear)
                .into_iter()
                .map(|v| v / 20.)
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let controls = (0..cubed.len())
        .map(|i| {
            (0..9)
                .map(|j| {
                    let t = j as f64 / 8.;
                    let mut point = Vector3::new(0., 0., 0.);
                    for k in 0..3 {
                        point[k] = base[0][k][i] * (1. - t) + base[1][k][i] * t;
                    }
                    for n in 0..8 {
                        point += vertical[n][j] * angular[n][i];
                    }
                    point.extend(cubed[i])
                })
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    let controls = if reverse {
        controls
            .into_iter()
            .map(|row| row.into_iter().rev().collect())
            .collect()
    } else {
        controls
    };
    let mut result = NurbsSurface::new(BSplineSurface::new(
        (KnotVec::bezier_knot(3 * degree), KnotVec::bezier_knot(8)),
        controls,
    ));
    if swap {
        result.invert();
    }
    regularity(&result, false)?;
    Ok(result)
}

pub(super) fn apply(g: &Geometry, cage: &Cage) -> Result<Geometry, String> {
    let shell = g.shell.as_ref().ok_or("Missing periodic boundary.")?;
    let (origin, inverse) = frame(cage)?;
    let vertical = vertical(cage);
    let surfaces = cage
        .patches
        .iter()
        .map(|(index, source)| Ok((*index, skin(source, origin, inverse, &vertical)?)))
        .collect::<Result<Vec<_>, String>>()?;
    let rebuilt = shell.mapped(
        Clone::clone,
        |curve| {
            let (a, b) = curve.range_tuple();
            let first = curve.subs(a);
            let last = curve.subs(b);
            let start = inverse * (first - origin);
            let end = inverse * (last - origin);
            if (start.z - end.z).abs() < 0.5 {
                return curve.clone();
            }
            // One authoritative global field also defines every internal longitudinal trim curve.
            let controls = (0..9)
                .map(|j| {
                    let t = j as f64 / 8.;
                    let point = first + (last - first) * t;
                    let mut delta = Vector3::new(0., 0., 0.);
                    let index = if start.z < end.z { j } else { 8 - j };
                    for (i, row) in vertical.iter().enumerate() {
                        let angle = i as f64 * std::f64::consts::TAU / 8.;
                        let basis =
                            (1. + start.x * angle.cos() + start.y * angle.sin()).powi(3) / 20.;
                        delta += row[index] * basis;
                    }
                    (point + delta).to_vec().extend(1.)
                })
                .collect();
            Curve::NurbsCurve(truck_modeling::NurbsCurve::new(BSplineCurve::new(
                KnotVec::bezier_knot(8),
                controls,
            )))
        },
        Clone::clone,
    );
    for (index, surface) in surfaces {
        rebuilt[index].set_surface(Surface::NurbsSurface(surface));
    }
    let mut result = finish(g, &rebuilt)?;
    *result
        .cages
        .iter_mut()
        .find(|c| c.domain == cage.domain)
        .ok_or("Missing periodic cage.")? = cage.clone();
    Ok(result)
}

#[test]
fn periodic_cylinder_cages_share_seams_and_preserve_caps() {
    use crate::{
        model::{self, Body, Model, Shape},
        project::Project,
        viewport::Camera,
    };
    use truck_modeling::{BoundedSurface, ParametricSurface, ParametricSurface3D};
    for affine in [false, true] {
        let mut body = Body::new(Shape::Circle);
        body.id = 1;
        body.width = 4.;
        body.extrusion = 3.;
        if affine {
            body.position = [2., -3., 1.];
            body.rotation = [15., 25., 35.];
            body.scale = [1.4, 0.7, 1.2];
        }
        let base = model::evaluate(&body).unwrap();
        let bound = super::create(&body, &base, 1).unwrap();
        let start = model::evaluate(&bound).unwrap();
        assert_eq!(start.face_count, 3);
        assert_eq!(start.cages.len(), 1);
        let binding = &start.cages[0].binding;
        assert!(binding.periodic);
        let mut editor = Editor::new(&bound, &start, 1).unwrap();
        editor.sources = vec![[0, 2]];
        editor.radius = 4.;
        let before = positions(&super::net(&start, 1).unwrap());
        let weights = field(
            &start.cages[0].net,
            binding,
            Matrix4::identity(),
            &editor.sources,
            &[],
            true,
            4.,
        )
        .unwrap();
        assert_eq!(weights[2], 1.);
        assert!(
            weights[7 * 5 + 2] > 0. && weights[5 + 2] > 0.,
            "The control graph wraps at the actual periodic seam"
        );
        let delta = Vector3::new(0.4, 0.1, 0.05);
        let changed = editor.translated(&bound, &start, delta.into()).unwrap();
        let out = model::evaluate(&changed).unwrap();
        assert!(
            components::transformed(
                &changed,
                &out,
                SelectionMode::Face,
                Some(1),
                Matrix4::from_translation(Vector3::unit_z())
            )
            .is_ok_and(|candidate| model::evaluate(&candidate).is_err()),
            "Face movement must not silently translate an entire cylinder"
        );
        if !affine {
            if let Ok(path) = std::env::var("OSK_PERIODIC_REVIEW") {
                let mut camera = Camera::default();
                camera.fit(&model::evaluate_scene(std::slice::from_ref(&body)).unwrap());
                Project::new(vec![body.clone()], camera, false, None)
                    .save(std::path::Path::new(&path))
                    .unwrap();
            }
            let cap = super::create(&changed, &out, 2).unwrap();
            let cap_geometry = model::evaluate(&cap).unwrap();
            let mut cap_editor = Editor::new(&cap, &cap_geometry, 2).unwrap();
            cap_editor.sources = vec![[2, 2]];
            let domed = cap_editor
                .translated(&cap, &cap_geometry, [0., 0., 0.3])
                .unwrap();
            let domed_geometry = model::evaluate(&domed).unwrap();
            assert_eq!(domed_geometry.cages.len(), 2);
            let shifted = components::transformed(
                &domed,
                &domed_geometry,
                SelectionMode::Face,
                Some(2),
                Matrix4::from_translation(Vector3::new(0., 0., 0.2)),
            )
            .unwrap();
            let shifted_geometry = model::evaluate(&shifted).unwrap();
            assert_eq!(shifted_geometry.cages.len(), 2);
            assert!(
                shifted_geometry
                    .shell
                    .as_ref()
                    .unwrap()
                    .is_geometric_consistent()
            );
        }
        let after = positions(&super::net(&out, 1).unwrap());
        assert!((after[2] - before[2] - delta).magnitude() < 1e-8);
        for i in 0..8 {
            for j in [0, 4] {
                assert!((after[i * 5 + j] - before[i * 5 + j]).magnitude() < 1e-8);
            }
        }
        assert!(out.shell.as_ref().unwrap().is_geometric_consistent());
        assert_eq!(out.face_count, 3);
        let shell = out.shell.as_ref().unwrap();
        let mapping = out.surface_mapping().unwrap();
        let patches = (0..shell.len())
            .filter(|&i| mapping[i] == 1)
            .collect::<Vec<_>>();
        // Every artificial kernel seam is a shared curved edge with one consistent normal field.
        let mut comparisons = 0;
        for (p, &i) in patches.iter().enumerate() {
            for &j in patches.iter().skip(p + 1) {
                for a in shell[i].edge_iter() {
                    for b in shell[j].edge_iter() {
                        if a.id() == b.id() {
                            let curve = a.oriented_curve();
                            let (t0, t1) = curve.range_tuple();
                            for t in [0.2, 0.5, 0.8] {
                                let point = curve.subs(t0 + (t1 - t0) * t);
                                let s0 = shell[i].surface();
                                let s1 = shell[j].surface();
                                let (u0, v0) = s0.search_parameter(point, None, 128).unwrap();
                                let (u1, v1) = s1.search_parameter(point, None, 128).unwrap();
                                assert!((s0.subs(u0, v0) - s1.subs(u1, v1)).magnitude() < 1e-6);
                                assert!(
                                    s0.normal(u0, v0).dot(s1.normal(u1, v1)) > 1. - 1e-5,
                                    "The periodic deformation must not kink at a kernel patch seam"
                                );
                                comparisons += 1;
                            }
                        }
                    }
                }
            }
        }
        assert!(comparisons >= 9);
        for &(index, ref surface) in &start.cages[0].patches {
            let current = shell[index].surface();
            let (u, v) = surface.range_tuple();
            for t in [0., 0.25, 0.5, 0.75, 1.] {
                for edge in [v.0, v.1] {
                    let point = surface.subs(u.0 + (u.1 - u.0) * t, edge);
                    let (a, b) = current.search_parameter(point, None, 128).unwrap();
                    assert!(
                        (current.subs(a, b) - point).magnitude() < 1e-6,
                        "Both circular cap boundaries stay exact"
                    );
                }
            }
        }
        let mut model = Model::from_bodies(vec![bound.clone()]);
        model.commit(changed.clone()).unwrap();
        model.undo();
        assert_eq!(model.bodies, std::slice::from_ref(&bound));
        model.redo();
        assert_eq!(model.bodies, std::slice::from_ref(&changed));
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("periodic.osk");
        let project = Project::new(vec![changed.clone()], Camera::default(), false, None);
        project.save(&path).unwrap();
        assert_eq!(
            Project::load(&path).unwrap().bodies,
            std::slice::from_ref(&changed)
        );
        let mut old = serde_json::to_value(&project).unwrap();
        old["schema_version"] = 44.into();
        assert!(
            serde_json::from_value::<Project>(old)
                .unwrap()
                .validate()
                .is_err()
        );
        for format in crate::export::Format::ALL {
            assert!(
                !crate::export::encode(std::slice::from_ref(&changed), format)
                    .unwrap()
                    .is_empty()
            );
        }
    }
}
