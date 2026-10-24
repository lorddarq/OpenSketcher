//! Face replacement, deletion and closure on retained evaluated boundaries.
use crate::{
    components::ComponentEdit,
    model::{Body, Geometry},
    selection::{Pick, SelectionMode},
};
use serde::{Deserialize, Serialize};
use truck_modeling::{
    BoundedCurve, InnerSpace, Matrix4, ParameterDivision1D, ParametricCurve, ParametricSurface,
    Shell, SquareMatrix, Wire, builder,
};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FaceEdit {
    Open,
    Cap,
    Heal,
    CapOpenings,
    Shell(crate::shelling::EvaluatedShell),
    Draft(crate::drafting::EvaluatedDraft),
}
impl FaceEdit {
    pub fn label(self) -> &'static str {
        match self {
            Self::Open => "Delete face · Open",
            Self::Cap => "Delete face · Cap",
            Self::Heal => "Delete face · Heal",
            Self::CapOpenings => "Cap openings",
            Self::Shell(_) => "Shell",
            Self::Draft(_) => "Draft",
        }
    }
}

pub fn selected(
    bodies: &[Body],
    scene: &[(u64, Geometry)],
    picks: &[Pick],
    kind: FaceEdit,
) -> Result<Vec<Body>, String> {
    if picks.is_empty() {
        return Err("Select faces first.".into());
    }
    let mut changed = Vec::<Body>::new();
    for (id, element) in picks {
        let g = &scene
            .iter()
            .find(|(owner, _)| owner == id)
            .ok_or("Selected geometry no longer exists.")?
            .1;
        let body = bodies
            .iter()
            .find(|b| b.id == *id)
            .ok_or("Selected body no longer exists.")?;
        if g.shell.is_none() {
            return Err("Face operations require a retained kernel boundary.".into());
        }
        let face = if kind == FaceEdit::CapOpenings {
            if g.is_solid {
                return Err("This body is already closed.".into());
            }
            0
        } else {
            element.ok_or("Select faces in Face mode.")?
        };
        if face >= g.face_count {
            return Err("Selected face no longer exists.".into());
        }
        let faces = if kind == FaceEdit::CapOpenings {
            vec![face]
        } else {
            g.face_elements(face)
        };
        if let Some(body) = changed.iter_mut().find(|b| b.id == *id) {
            let edit = body.component_edits.last_mut().unwrap();
            for face in faces {
                if face != edit.element && !edit.elements.contains(&face) {
                    edit.elements.push(face);
                }
            }
        } else {
            let mut body = body.clone();
            body.component_edits.push(ComponentEdit {
                control: None,
                evaluated: true,
                retrim: false,
                convex_rebuild: matches!(kind, FaceEdit::Draft(_))
                    && g.face_count <= 64
                    && crate::components::planar_supports(g).is_ok(),
                section_transform: false,
                cap_rotation: false,
                sweep: None,
                tangent_walls: false,
                prismatic_cut: false,
                face_edit: Some(kind),
                edge_chamfer: None,
                prism_chamfer: false,
                prism_bevel: None,
                prism_curves: false,
                mode: SelectionMode::Face,
                element: face,
                elements: faces.into_iter().filter(|other| *other != face).collect(),
                topology: g.topology.clone(),
                matrix: Matrix4::identity().into(),
            });
            body.validate()?;
            changed.push(body);
        }
    }
    Ok(changed)
}

pub(crate) fn apply(g: &Geometry, selected: &[usize], kind: FaceEdit) -> Result<Geometry, String> {
    if let FaceEdit::Shell(operation) = kind {
        return crate::shelling::evaluated(g, selected, operation);
    }
    if let FaceEdit::Draft(operation) = kind {
        return crate::drafting::evaluated(g, selected, operation);
    }
    if kind == FaceEdit::Heal {
        return if crate::components::planar_supports(g).is_ok() {
            crate::edge_treatments::heal_planar_faces(g, selected)
        } else {
            heal_internal_walls(g, selected).or_else(|internal_error| {
                heal_planar_notch(g, selected).map_err(|notch_error| {
                    format!("{internal_error} Planar healing: {notch_error}")
                })
            })
        };
    }
    let source = g
        .shell
        .as_ref()
        .ok_or("Face operation requires a retained kernel boundary.")?;
    let mapping = g.surface_mapping()?;
    let mut roles = Vec::new();
    let mut shell: Shell = source
        .iter()
        .enumerate()
        .filter_map(|(i, f)| {
            if kind != FaceEdit::CapOpenings && selected.contains(&mapping[i]) {
                None
            } else {
                roles.push(mapping[i]);
                Some(f.clone())
            }
        })
        .collect();
    if shell.is_empty() {
        return if kind == FaceEdit::Open {
            Ok(Geometry::default())
        } else {
            Err("No neighboring geometry remains to close.".into())
        };
    }
    if kind != FaceEdit::Open {
        let caps = planar_caps(&shell)?;
        if caps.is_empty() {
            return Err("There are no open boundaries to cap.".into());
        }
        for cap in caps {
            roles.push(roles.iter().copied().max().unwrap_or(0) + 1);
            shell.push(cap);
        }
        crate::components::solid_parts(&shell)
            .map_err(|e| format!("Capping did not produce closed material: {e}"))?;
    }
    if !shell.is_geometric_consistent() {
        return Err("Face operation creates inconsistent boundaries.".into());
    }
    let mut result = crate::model::tessellate(&shell)?;
    crate::edge_treatments::merge_surfaces(&mut result, &roles);
    result.is_solid =
        shell.extract_boundaries().is_empty() && crate::components::solid_parts(&shell).is_ok();
    result.retain_circles(&g.circles);
    Ok(result)
}

/// Heal a complete internal pocket/bore by removing its trim loops from the
/// surviving supports. No approximation or new surface is needed.
fn heal_internal_walls(g: &Geometry, selected: &[usize]) -> Result<Geometry, String> {
    let source = g
        .shell
        .as_ref()
        .ok_or("Heal requires retained boundaries.")?;
    if !g.is_solid || source.connected_components().len() != 1 {
        return Err("Internal-wall healing requires one connected closed boundary.".into());
    }
    let mapping = g.surface_mapping()?;
    let removed: std::collections::HashSet<_> = source
        .iter()
        .zip(mapping)
        .filter(|(_, role)| selected.contains(role))
        .flat_map(|(face, _)| face.edge_iter().map(|e| e.id()))
        .collect();
    let mut shell = Shell::default();
    let mut roles = Vec::new();
    let mut healed = 0;
    for (face, role) in source.iter().zip(mapping) {
        if selected.contains(role) {
            continue;
        }
        let mut wires = Vec::new();
        for (index, wire) in face.boundaries().into_iter().enumerate() {
            let count = wire
                .iter()
                .filter(|edge| removed.contains(&edge.id()))
                .count();
            if count == 0 {
                wires.push(wire);
                continue;
            }
            if count != wire.len() {
                return Err(
                    "Select the complete pocket/bore wall set, including its floor when present."
                        .into(),
                );
            }
            if index == 0 {
                return Err(
                    "Heal cannot remove an outer support boundary; select complete internal walls."
                        .into(),
                );
            }
            healed += 1;
        }
        if wires.is_empty() {
            return Err("No support remains to heal.".into());
        }
        let rebuilt = truck_modeling::Face::try_new(wires, face.oriented_surface())
            .map_err(|e| format!("Cannot heal the retained support: {e}"))?;
        roles.push(*role);
        shell.push(rebuilt);
    }
    if healed == 0 || shell.is_empty() {
        return Err("Selected faces do not bound a complete internal pocket or bore.".into());
    }
    // Every retained curve is unchanged on its original support. Truck's
    // include-curve consistency check panics for IntersectionCurve, so do not
    // re-test that invariant via the unsupported API. Check closure and solids.
    if !shell.extract_boundaries().is_empty() {
        return Err("The selected walls do not heal to a closed consistent solid.".into());
    }
    crate::components::solid_parts(&shell)?;
    let mut result = crate::model::tessellate(&shell)?;
    crate::edge_treatments::merge_surfaces(&mut result, &roles);
    result.is_solid = true;
    result.retain_circles(&g.circles);
    Ok(result)
}

/// Extend surviving planar supports across a complete external notch. Missing
/// spans must have unambiguous endpoint pairs on two retained support planes.
fn heal_planar_notch(g: &Geometry, selected: &[usize]) -> Result<Geometry, String> {
    use std::collections::HashMap;
    use truck_modeling::{EuclideanSpace, Vertex, VertexID};
    let source = g.shell.as_ref().ok_or("Missing retained boundary.")?;
    if !g.is_solid || source.connected_components().len() != 1 {
        return Err("Requires one connected closed boundary.".into());
    }
    let mapping = g.surface_mapping()?;
    let removed: std::collections::HashSet<_> = source
        .iter()
        .zip(mapping)
        .filter(|(_, role)| selected.contains(role))
        .flat_map(|(face, _)| face.edge_iter().map(|e| e.id()))
        .collect();
    let mut untouched = Vec::new();
    let mut groups = Vec::<(crate::snapping::Plane, Shell, usize)>::new();
    for (face, role) in source
        .iter()
        .zip(mapping)
        .filter(|(_, role)| !selected.contains(role))
    {
        let truck_modeling::Surface::Plane(surface) = face.oriented_surface() else {
            if face.edge_iter().any(|edge| removed.contains(&edge.id())) {
                return Err("Supports adjacent to removed walls must be planar.".into());
            }
            untouched.push((face.clone(), *role));
            continue;
        };
        let origin = surface.subs(0., 0.);
        let normal = surface.normal().normalize();
        if let Some((_, shell, group_role)) = groups.iter_mut().find(|(plane, _, _)| {
            plane.normal.dot(normal) > 1. - 1e-8 && (plane.origin - origin).dot(normal).abs() < 1e-7
        }) {
            shell.push(face.clone());
            *group_role = (*group_role).min(*role);
        } else {
            let u = (surface.subs(1., 0.) - origin).normalize();
            groups.push((
                crate::snapping::Plane {
                    origin,
                    normal,
                    u,
                    v: normal.cross(u),
                },
                vec![face.clone()].into(),
                *role,
            ));
        }
    }
    if !(4..=256).contains(&groups.len()) {
        return Err("Requires 4–256 surviving planar supports.".into());
    }
    let mut boundaries = Vec::new();
    let mut endpoints = HashMap::<VertexID, Vec<(usize, Vertex, i32)>>::new();
    for (group, (_, shell, _)) in groups.iter().enumerate() {
        let edges = shell
            .extract_boundaries()
            .into_iter()
            .flat_map(|wire| wire.into_iter())
            .filter(|edge| !removed.contains(&edge.id()))
            .collect::<Vec<_>>();
        let mut balance = HashMap::<VertexID, (Vertex, i32)>::new();
        for edge in &edges {
            for (vertex, delta) in [(edge.front(), 1), (edge.back(), -1)] {
                balance
                    .entry(vertex.id())
                    .or_insert_with(|| (vertex.clone(), 0))
                    .1 += delta;
            }
        }
        for (id, (vertex, balance)) in balance {
            if balance != 0 {
                endpoints
                    .entry(id)
                    .or_default()
                    .push((group, vertex, balance));
            }
        }
        boundaries.push(edges);
    }
    let mut gaps = std::collections::BTreeMap::<(usize, usize), Vec<(Vertex, i32)>>::new();
    for mut owners in endpoints.into_values() {
        owners.sort_by_key(|(group, _, _)| *group);
        if owners.len() != 2 || owners[0].2.abs() != 1 || owners[0].2 != -owners[1].2 {
            return Err(
                "Incomplete or ambiguous notch boundary; select its complete wall/floor set."
                    .into(),
            );
        }
        gaps.entry((owners[0].0, owners[1].0))
            .or_default()
            .push((owners[0].1.clone(), owners[0].2));
    }
    if gaps.is_empty() {
        return Err("No external notch spans remain to heal.".into());
    }
    for ((a, b), mut ends) in gaps {
        let axis = groups[a].0.normal.cross(groups[b].0.normal);
        if axis.magnitude2() < 1e-12 || ends.len() % 2 != 0 {
            return Err("The remaining supports do not bound a finite healing span.".into());
        }
        ends.sort_by(|a, b| {
            a.0.point()
                .to_vec()
                .dot(axis)
                .total_cmp(&b.0.point().to_vec().dot(axis))
        });
        for pair in ends.chunks_exact(2) {
            if pair[0].1 == pair[1].1 || (pair[0].0.point() - pair[1].0.point()).magnitude() < 1e-7
            {
                return Err(
                    "The notch spans cannot be paired without changing boundary connectivity."
                        .into(),
                );
            }
            let edge = if pair[0].1 < 0 {
                builder::line(&pair[0].0, &pair[1].0)
            } else {
                builder::line(&pair[1].0, &pair[0].0)
            };
            boundaries[a].push(edge.clone());
            boundaries[b].push(edge.inverse());
        }
    }
    let mut shell = Shell::default();
    let mut roles = Vec::new();
    for ((plane, _, role), edges) in groups.into_iter().zip(boundaries) {
        for face in crate::curve_imprints::planar_arrangement(plane, edges)? {
            shell.push(face);
            roles.push(role);
        }
    }
    for (face, role) in untouched {
        shell.push(face);
        roles.push(role);
    }
    if shell.is_empty() || !shell.extract_boundaries().is_empty() {
        return Err("The remaining supports cannot heal a closed solid.".into());
    }
    crate::components::solid_parts(&shell)?;
    let mut result = crate::model::tessellate(&shell)?;
    crate::edge_treatments::merge_surfaces(&mut result, &roles);
    crate::components::validate_retrim_intersections(&result)?;
    result.is_solid = true;
    result.retain_circles(&g.circles);
    Ok(result)
}

/// Split only the explicitly selected planar face; the body's material is unchanged.
pub(crate) fn imprint(
    op: &crate::booleans::Boolean,
    inputs: &[&Geometry],
) -> Result<Geometry, String> {
    use crate::booleans::Kind;
    for (i, g) in inputs.iter().enumerate() {
        if g.topology != op.topology[i] {
            return Err("An Imprint input changed topology; reselect its face.".into());
        }
    }
    let source = inputs[0];
    let tool = inputs[1];
    if !source.is_solid || tool.is_solid {
        return Err(
            "Imprint requires a closed solid face first and an unextruded profile second.".into(),
        );
    }
    let element = op.operands[0].1.ok_or("Select the solid face first.")?;
    let plane = crate::snapping::Plane::face(source, element)
        .ok_or("Imprint currently requires a planar support face.")?;
    let selected_face = |g: &Geometry, index: usize| -> Result<truck_modeling::Face, String> {
        let shell = g
            .shell
            .as_ref()
            .ok_or("Imprint requires retained analytic boundaries.")?;
        let selected: Shell = shell
            .iter()
            .zip(g.surface_mapping()?)
            .filter(|(_, role)| **role == index)
            .map(|(f, _)| f.clone())
            .collect();
        if selected.len() == 1 {
            Ok(selected[0].clone())
        } else {
            builder::try_attach_plane(&selected.extract_boundaries()).map_err(|e| e.to_string())
        }
    };
    let support = selected_face(source, element)?;
    let selected = if let Some(index) = op.operands[1].1 {
        vec![index]
    } else {
        (0..tool.face_count).collect()
    };
    if selected.is_empty() {
        return Err("The profile has no closed face regions.".into());
    }
    let mut normals = std::collections::HashMap::new();
    for &face in &selected {
        let other = crate::snapping::Plane::face(tool, face)
            .ok_or("Every selected profile region must be planar.")?;
        if plane.normal.dot(other.normal).abs() < 1. - 1e-8
            || (plane.origin - other.origin).dot(plane.normal).abs() > 1e-6
        {
            return Err("The profile must lie on the selected face plane; Imprint does not project onto hidden or offset faces.".into());
        }
        normals.insert(face, plane.normal.dot(other.normal));
    }
    let shell = tool
        .shell
        .as_ref()
        .ok_or("The profile requires retained analytic boundaries.")?;
    let selected_shell: Shell = shell
        .iter()
        .zip(tool.surface_mapping()?)
        .filter_map(|(face, role)| {
            normals.get(role).map(|dot| {
                if *dot < 0. {
                    face.inverse()
                } else {
                    face.clone()
                }
            })
        })
        .collect();
    let profiles = planar_regions(selected_shell.extract_boundaries())?;
    if profiles.is_empty() {
        return Err("The profile has no closed face regions.".into());
    }
    if op.analytic_crossings
        && support
            .edge_iter()
            .chain(profiles.iter().flat_map(|f| f.edge_iter()))
            .any(|edge| !matches!(edge.curve(), truck_modeling::Curve::Line(_)))
    {
        return crate::profile_imprints::apply(source, element, &profiles, &tool.circles);
    }
    let mut view = Geometry {
        triangles: tool
            .triangles
            .iter()
            .zip(&tool.triangle_faces)
            .filter(|(_, face)| selected.contains(face))
            .map(|(t, _)| *t)
            .collect(),
        ..Default::default()
    };
    view.triangle_faces = vec![0; view.triangles.len()];
    let mut operation = op.clone();
    operation.operands[1].1 = Some(0);
    operation.topology[1] = view.topology.clone();
    operation.kind = Kind::Subtract;
    let remainder = crate::booleans::faces(&operation, &[source, &view])?;
    operation.kind = Kind::Intersect;
    let intersection = crate::booleans::faces(&operation, &[source, &view])?;
    if remainder.triangles.is_empty() || intersection.triangles.is_empty() {
        return Err(
            "The profile must divide the selected face into distinct nonempty regions.".into(),
        );
    }
    let touches_straight_boundary = support.edge_iter().any(|edge| {
        if !matches!(edge.curve(), truck_modeling::Curve::Line(_)) {
            return false;
        }
        let a = edge.front().point();
        let b = edge.back().point();
        let ab = b - a;
        profiles.iter().any(|profile| {
            profile.edge_iter().any(|e| {
                let p = e.front().point();
                let t = ((p - a).dot(ab) / ab.magnitude2()).clamp(0., 1.);
                (p - (a + ab * t)).magnitude2() < 1e-14
            })
        })
    });
    if !touches_straight_boundary && crate::booleans::face_coverage(&view, source, &[element])?.0 {
        let mut wires = support.boundaries();
        for profile in &profiles {
            wires.extend(profile.boundaries().iter().map(Wire::inverse));
        }
        let remainder = planar_regions(wires)?;
        let mut roles = Vec::new();
        let mut joined: Shell = source
            .shell
            .as_ref()
            .unwrap()
            .iter()
            .zip(source.surface_mapping()?)
            .filter_map(|(f, role)| {
                if *role == element {
                    None
                } else {
                    roles.push(*role);
                    Some(f.clone())
                }
            })
            .collect();
        let mut next = source.face_count;
        for (index, face) in remainder.iter().enumerate() {
            joined.push(face.clone());
            roles.push(if index == 0 {
                element
            } else {
                let role = next;
                next += 1;
                role
            });
        }
        for profile in &profiles {
            joined.push(profile.clone());
            roles.push(next);
            next += 1;
        }
        crate::components::solid_parts(&joined)?;
        let mut result = crate::model::tessellate(&joined)?;
        crate::edge_treatments::merge_surfaces(&mut result, &roles);
        result.is_solid = true;
        result.circles = source.circles.clone();
        result.circles.extend(
            tool.circles
                .iter()
                .filter(|ring| {
                    profiles.iter().any(|face| {
                        face.boundaries()
                            .iter()
                            .any(|wire| crate::face_extrusions::circle_on_wire(ring, wire))
                    })
                })
                .cloned(),
        );
        return Ok(result);
    }
    crate::edge_treatments::imprint_linear(source,element,&remainder,&intersection)?.ok_or_else(||"Analytic curved crossings remain unsupported; straight-edge crossings support holes and disconnected regions.".into())
}

/// Replace a selected support with a live referenced plane; keep the source
/// face orientation rather than importing the reference face's outward side.
pub(crate) fn replace(
    op: &crate::booleans::Boolean,
    inputs: &[&Geometry],
    bodies: &[Body],
) -> Result<Geometry, String> {
    use truck_modeling::{Point3, Rad, Transform};
    for (i, g) in inputs.iter().enumerate() {
        if g.topology != op.topology[i] {
            return Err(
                "A Replace Face input changed topology; reselect the intended faces.".into(),
            );
        }
    }
    let source = inputs[0];
    let face = op.operands[0].1.ok_or("Select a source face first.")?;
    let reference = op.operands[1]
        .1
        .ok_or("Select a replacement face second.")?;
    let plane = crate::snapping::Plane::face(source, face)
        .ok_or("Replace Face requires a planar source face.")?;
    let target = crate::snapping::Plane::face(inputs[1], reference)
        .ok_or("The replacement reference must be planar.")?;
    if !source.is_solid || source.shell.is_none() {
        return Err("Replace Face requires a closed source solid with a retained boundary.".into());
    }
    if crate::components::straight_planar_supports(source).is_ok() {
        let normal = if plane.normal.dot(target.normal) < 0. {
            -target.normal
        } else {
            target.normal
        };
        let axis = plane.normal.cross(normal);
        let rotation = if axis.magnitude2() < 1e-16 {
            Matrix4::identity()
        } else {
            Matrix4::from_axis_angle(
                axis.normalize(),
                Rad(plane.normal.dot(normal).clamp(-1., 1.).acos()),
            )
        };
        let moved = rotation.transform_point(plane.origin);
        let matrix =
            Matrix4::from_translation(normal * (target.origin - moved).dot(normal)) * rotation;
        return crate::components::retrim_faces(source, &[face], matrix, false);
    }
    // Reuse exact variable-height prism construction for analytic curved walls.
    let body = bodies
        .iter()
        .find(|b| b.id == op.operands[0].0)
        .ok_or("Replace Face source no longer exists.")?;
    if !matches!(
        body.shape,
        crate::model::Shape::Circle | crate::model::Shape::Semicircle
    ) || body.boolean.is_some()
        || body.bore.is_some()
        || body.taper.is_some()
        || body.hollow.is_some()
        || !body.component_edits.is_empty()
        || !body.edge_treatments.is_empty()
        || !body.face_extrusions.is_empty()
    {
        return Err("Replace Face currently retrims convex planar solids and original circular/semicircular prism caps; general curved/concave retrimming remains unsupported.".into());
    }
    let primitive = crate::model::evaluate(body)?;
    if source.topology != primitive.topology || source.triangles != primitive.triangles {
        return Err(
            "Replace Face cannot rebuild an altered curved prism from its source profile.".into(),
        );
    }
    let role = crate::edge_treatments::target(body, source, face)?;
    let direction = match role {
        crate::edge_treatments::FaceTarget::Top => 1.,
        crate::edge_treatments::FaceTarget::Bottom => -1.,
        _ => return Err("Select a circular prism's top or bottom cap.".into()),
    };
    let mut profile = body.clone();
    profile.extrusion = 0.;
    if direction < 0. {
        profile.position = crate::regions::transform(body)
            .transform_point(Point3::new(0., 0., body.extrusion))
            .into();
    }
    crate::operators::plane_travel(&profile, target, direction)?;
    let result = crate::operators::terminated(&profile, 0., target, direction)?;
    // Both constructions preserve analytic boundaries; no triangle coordinates
    // are used to fabricate the replacement surface.
    Ok(result)
}

/// Preserve analytic wires; sampling is used only to classify nested loops.
fn planar_caps(shell: &Shell) -> Result<Vec<truck_modeling::Face>, String> {
    planar_regions(
        shell
            .extract_boundaries()
            .into_iter()
            .map(|wire| wire.inverse())
            .collect(),
    )
}
pub(crate) fn wire_polygon(wire: &Wire, plane: &crate::snapping::Plane) -> Vec<[f64; 2]> {
    wire.iter()
        .flat_map(|edge| {
            let curve = edge.oriented_curve();
            let (a, b) = curve.range_tuple();
            let mut parameters = curve.parameter_division((a, b), 0.01).0;
            parameters.push((a + b) / 2.);
            parameters.sort_by(f64::total_cmp);
            parameters.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
            parameters.pop();
            parameters.into_iter().map(move |t| curve.subs(t))
        })
        .map(|p| {
            let d = p - plane.origin;
            [d.dot(plane.u), d.dot(plane.v)]
        })
        .collect()
}

pub(crate) fn planar_regions(wires: Vec<Wire>) -> Result<Vec<truck_modeling::Face>, String> {
    let mut groups = Vec::<(crate::snapping::Plane, Vec<Wire>)>::new();
    for wire in wires {
        let face = builder::try_attach_plane(std::slice::from_ref(&wire))
            .map_err(|_| "An opening is not planar; Cap cannot flatten its boundary.")?;
        let truck_modeling::Surface::Plane(surface) = face.oriented_surface() else {
            return Err("Cap requires planar openings.".into());
        };
        let origin = surface.subs(0., 0.);
        let normal = surface.normal().normalize();
        if let Some((_, wires)) = groups.iter_mut().find(|(plane, _)| {
            plane.normal.dot(normal).abs() > 1. - 1e-8
                && (plane.origin - origin).dot(normal).abs() < 1e-7
        }) {
            wires.push(wire);
        } else {
            let u = (surface.subs(1., 0.) - origin).normalize();
            groups.push((
                crate::snapping::Plane {
                    origin,
                    normal,
                    u,
                    v: normal.cross(u),
                },
                vec![wire],
            ));
        }
    }
    let mut caps = Vec::new();
    for (plane, wires) in groups {
        let polygons = wires
            .iter()
            .map(|wire| wire_polygon(wire, &plane))
            .collect::<Vec<_>>();
        if polygons
            .iter()
            .any(|p| p.len() < 3 || crate::regions::area(p).abs() < 1e-12)
        {
            return Err("An opening has a degenerate planar boundary.".into());
        }
        let depth = polygons
            .iter()
            .enumerate()
            .map(|(i, p)| {
                polygons
                    .iter()
                    .enumerate()
                    .filter(|(j, other)| i != *j && crate::regions::contains(p[0], other))
                    .count()
            })
            .collect::<Vec<_>>();
        for (i, outer) in polygons
            .iter()
            .enumerate()
            .filter(|(i, _)| depth[*i] % 2 == 0)
        {
            let mut boundaries = vec![wires[i].clone()];
            for (j, hole) in polygons.iter().enumerate() {
                if depth[j] == depth[i] + 1 && crate::regions::contains(hole[0], outer) {
                    boundaries.push(wires[j].clone());
                }
            }
            caps.push(
                builder::try_attach_plane(&boundaries)
                    .map_err(|e| format!("Cannot cap opening: {e}"))?,
            );
        }
    }
    Ok(caps)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        edge_treatments::{FaceTarget, Kind, Modifier},
        model::{Model, Shape, evaluate, evaluate_scene},
        project::Project,
        viewport::Camera,
    };
    use truck_modeling::{EuclideanSpace, Solid};
    fn volume(g: &Geometry) -> f64 {
        g.triangles
            .iter()
            .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
            .sum()
    }
    #[test]
    fn whole_profile_imprint_preserves_holes_islands_and_disconnected_regions() {
        use crate::booleans::{Boolean, Kind as Operation};
        for disconnected in [false, true] {
            let mut source = Body::new(Shape::Rectangle);
            source.id = 1;
            source.width = 8.;
            source.depth = 6.;
            source.extrusion = 3.;
            let mut outer = Body::new(Shape::Rectangle);
            outer.id = 2;
            outer.width = 4.;
            outer.depth = 4.;
            outer.position = [0., 0., 3.];
            let mut inner = outer.clone();
            inner.id = 3;
            inner.width = 2.;
            inner.depth = 2.;
            if disconnected {
                outer.width = 2.;
                outer.depth = 2.;
                outer.position[0] = -2.;
                inner.position[0] = 2.;
            }
            let mut bodies = vec![source.clone(), outer, inner];
            let scene = evaluate_scene(&bodies).unwrap();
            let mut profile = Body::new(Shape::Rectangle);
            profile.id = 4;
            profile.boolean = Some(Boolean::new(
                if disconnected {
                    Operation::Union
                } else {
                    Operation::Subtract
                },
                &[(2, Some(0)), (3, Some(0))],
                SelectionMode::Face,
                &scene,
            ));
            bodies.push(profile);
            let scene = evaluate_scene(&bodies).unwrap();
            let face = (0..scene[0].1.face_count)
                .find(|f| {
                    crate::edge_treatments::target(&source, &scene[0].1, *f) == Ok(FaceTarget::Top)
                })
                .unwrap();
            let mut feature = Body::new(Shape::Rectangle);
            feature.id = 5;
            feature.boolean = Some(Boolean::new(
                Operation::Imprint,
                &[(1, Some(face)), (4, None)],
                SelectionMode::Face,
                &scene,
            ));
            bodies.push(feature.clone());
            let scene = evaluate_scene(&bodies).unwrap();
            let g = &scene[4].1;
            assert!((volume(g) - 144.).abs() < 1e-6);
            assert_eq!(g.face_count, 8);
            assert!(scene[..4].iter().all(|(_, g)| g.triangles.is_empty()));
            let mut regions = (0..g.face_count)
                .filter(|f| {
                    crate::snapping::Plane::face(g, *f)
                        .is_some_and(|p| p.normal.z > 0.99 && (p.origin.z - 3.).abs() < 1e-7)
                })
                .map(|f| {
                    (
                        f,
                        g.triangles
                            .iter()
                            .zip(&g.triangle_faces)
                            .filter(|(_, face)| **face == f)
                            .map(|(t, _)| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
                            .sum::<f64>(),
                    )
                })
                .collect::<Vec<_>>();
            regions.sort_by(|a, b| a.1.total_cmp(&b.1));
            let expected = if disconnected {
                vec![4., 4., 40.]
            } else {
                vec![4., 12., 32.]
            };
            assert_eq!(regions.len(), 3);
            for ((_, area), expected) in regions.iter().zip(expected) {
                assert!((*area - expected).abs() < 1e-6);
            }
            let (face, area) = if disconnected { regions[0] } else { regions[1] };
            for distance in [0.5, -0.5, -4.] {
                let edited = crate::components::sweep_face(&feature, g, face, distance).unwrap();
                let mut changed = bodies.clone();
                changed[4] = edited;
                let out = evaluate_scene(&changed).unwrap();
                assert!((volume(&out[4].1) - (144. + area * distance.max(-3.))).abs() < 1e-6);
                let parts =
                    crate::components::solid_parts(out[4].1.shell.as_ref().unwrap()).unwrap();
                assert_eq!(
                    parts.len(),
                    if !disconnected && distance < -3. {
                        2
                    } else {
                        1
                    }
                );
                for format in crate::export::Format::ALL {
                    assert!(!crate::export::encode(&changed, format).unwrap().is_empty());
                }
            }
            let mut model = Model::from_bodies(bodies.clone());
            let mut grown = bodies[0].clone();
            grown.width = 9.;
            model.commit(grown).unwrap();
            assert!((volume(&evaluate_scene(&model.bodies).unwrap()[4].1) - 162.).abs() < 1e-6);
            model.undo();
            assert_eq!(model.bodies, bodies);
            model.redo();
            if !disconnected {
                let mut resized = bodies[2].clone();
                resized.width = 2.2;
                model.commit(resized).unwrap();
                let changed = evaluate_scene(&model.bodies).unwrap();
                let ring_area = (0..changed[4].1.face_count)
                    .map(|f| {
                        changed[4]
                            .1
                            .triangles
                            .iter()
                            .zip(&changed[4].1.triangle_faces)
                            .filter(|(_, face)| **face == f)
                            .map(|(t, _)| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
                            .sum::<f64>()
                    })
                    .any(|area| (area - 11.6).abs() < 1e-6);
                assert!(ring_area);
                model.undo();
                let mut crossing = bodies.clone();
                crossing[1].position[0] = 2.5;
                crossing[2].position[0] = 2.5;
                let crossed = evaluate_scene(&crossing).unwrap();
                let crossed = &crossed[4].1;
                assert!((volume(crossed) - 144.).abs() < 1e-6);
                assert_eq!(crossed.face_count, 8);
                let ring = (0..crossed.face_count)
                    .find(|f| {
                        (crossed
                            .triangles
                            .iter()
                            .zip(&crossed.triangle_faces)
                            .filter(|(_, face)| **face == *f)
                            .map(|(t, _)| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
                            .sum::<f64>()
                            - 10.)
                            .abs()
                            < 1e-6
                    })
                    .unwrap();
                crossing[4] =
                    crate::components::sweep_face(&crossing[4], crossed, ring, 0.5).unwrap();
                assert!((volume(&evaluate_scene(&crossing).unwrap()[4].1) - 149.).abs() < 1e-6);
                for format in crate::export::Format::ALL {
                    assert!(!crate::export::encode(&crossing, format).unwrap().is_empty());
                }
            }
            let accepted = model.bodies.clone();
            let mut invalid = bodies[2].clone();
            invalid.position[2] += 1.;
            assert!(model.commit(invalid).is_err());
            assert_eq!(model.bodies, accepted);
            let project = Project::new(accepted.clone(), Camera::default(), true, Some(5));
            let file = tempfile::NamedTempFile::new().unwrap().into_temp_path();
            project.save(file.as_ref()).unwrap();
            assert_eq!(Project::load(file.as_ref()).unwrap().bodies, accepted);
            let mut old = serde_json::to_value(project).unwrap();
            old["schema_version"] = 24.into();
            old["modeling_version"] = 24.into();
            assert!(
                serde_json::from_value::<Project>(old)
                    .unwrap()
                    .validate()
                    .unwrap_err()
                    .contains("version 25")
            );
            model.delete(5).unwrap();
            assert!(
                !evaluate_scene(&model.bodies).unwrap()[0]
                    .1
                    .triangles
                    .is_empty()
            );
        }
    }
    #[test]
    fn authored_analytic_ring_profile_regenerates_and_cuts_with_shared_boundaries() {
        use crate::booleans::{Boolean, Kind as Operation};
        let mut source = Body::new(Shape::Rectangle);
        source.id = 1;
        source.width = 8.;
        source.depth = 6.;
        source.extrusion = 3.;
        let mut tool = Body::new(Shape::Circle);
        tool.id = 2;
        tool.width = 2.;
        tool.extrusion = 1.;
        tool.position[2] = 2.;
        tool.hollow = Some(crate::shelling::Hollow::new(&tool, FaceTarget::Top).unwrap());
        tool.hollow.as_mut().unwrap().thickness = 0.2;
        let g = evaluate(&tool).unwrap();
        let cap = (0..g.face_count)
            .find(|f| crate::edge_treatments::target(&tool, &g, *f) == Ok(FaceTarget::Top))
            .unwrap();
        let remove = (0..g.face_count)
            .filter(|f| *f != cap)
            .map(|f| (2, Some(f)))
            .collect::<Vec<_>>();
        tool = selected(
            std::slice::from_ref(&tool),
            &[(2, g)],
            &remove,
            FaceEdit::Open,
        )
        .unwrap()
        .remove(0);
        let mut bodies = vec![source.clone(), tool];
        let scene = evaluate_scene(&bodies).unwrap();
        assert!(!scene[1].1.is_solid);
        let cap = (0..scene[0].1.face_count)
            .find(|f| {
                crate::edge_treatments::target(&source, &scene[0].1, *f) == Ok(FaceTarget::Top)
            })
            .unwrap();
        let mut feature = Body::new(Shape::Rectangle);
        feature.id = 3;
        feature.boolean = Some(Boolean::new(
            Operation::Imprint,
            &[(1, Some(cap)), (2, None)],
            SelectionMode::Face,
            &scene,
        ));
        bodies.push(feature.clone());
        let scene = evaluate_scene(&bodies).unwrap();
        let g = &scene[2].1;
        assert_eq!(g.face_count, 8);
        assert_eq!(g.circles.len(), 2);
        let ring = (0..g.face_count)
            .filter(|f| {
                crate::snapping::Plane::face(g, *f)
                    .is_some_and(|p| p.normal.z > 0.99 && (p.origin.z - 3.).abs() < 1e-7)
            })
            .find(|f| {
                g.shell
                    .as_ref()
                    .unwrap()
                    .iter()
                    .zip(g.surface_mapping().unwrap())
                    .any(|(face, role)| {
                        *role == *f
                            && face.boundaries().len() == 2
                            && face
                                .edge_iter()
                                .all(|e| !matches!(e.curve(), truck_modeling::Curve::Line(_)))
                    })
            })
            .unwrap();
        let area = std::f64::consts::PI * (1. - 0.8f64.powi(2));
        for distance in [0.5, -0.5, -4.] {
            let mut changed = bodies.clone();
            changed[2] = crate::components::sweep_face(&feature, g, ring, distance).unwrap();
            let scene = evaluate_scene(&changed).unwrap();
            let out = &scene[2].1;
            assert!((volume(out) - (144. + area * distance.max(-3.))).abs() < 0.15);
            assert_eq!(
                crate::components::solid_parts(out.shell.as_ref().unwrap())
                    .unwrap()
                    .len(),
                if distance < -3. { 2 } else { 1 }
            );
            for format in crate::export::Format::ALL {
                assert!(!crate::export::encode(&changed, format).unwrap().is_empty());
            }
        }
        let mut model = Model::from_bodies(bodies.clone());
        let mut resized = bodies[1].clone();
        resized.width = 2.2;
        model.commit(resized).unwrap();
        assert_eq!(evaluate_scene(&model.bodies).unwrap()[2].1.circles.len(), 2);
        model.undo();
        assert_eq!(model.bodies, bodies);
        model.redo();
        let project = Project::new(model.bodies.clone(), Camera::default(), true, Some(3));
        let file = tempfile::NamedTempFile::new().unwrap().into_temp_path();
        project.save(file.as_ref()).unwrap();
        assert_eq!(Project::load(file.as_ref()).unwrap().bodies, model.bodies);
    }
    #[test]
    fn nested_analytic_profile_loops_keep_original_voids_and_arc_boundaries() {
        use crate::booleans::{Boolean, Kind as Operation};
        let mut source = Body::new(Shape::Circle);
        source.id = 1;
        source.width = 4.;
        source.extrusion = 3.;
        source.hollow = Some(crate::shelling::Hollow::new(&source, FaceTarget::Top).unwrap());
        let g = evaluate(&source).unwrap();
        let cap = (0..g.face_count)
            .find(|f| crate::edge_treatments::target(&source, &g, *f) == Ok(FaceTarget::Top))
            .unwrap();
        for (x, outer, inner) in [(0., 3.9, 3.7), (1.9, 0.12, 0.06)] {
            let mut a = Body::new(Shape::Circle);
            a.width = outer;
            a.position = [x, 0., 3.];
            let mut b = a.clone();
            b.width = inner;
            let outer = builder::transformed(
                &crate::model::profile_wire(&a),
                crate::regions::transform(&a),
            );
            let inner = builder::transformed(
                &crate::model::profile_wire(&b),
                crate::regions::transform(&b),
            );
            let face = builder::try_attach_plane(&[outer, inner.inverse()]).unwrap();
            let mut tool = crate::model::tessellate(&std::iter::once(face).collect()).unwrap();
            tool.circles = evaluate(&a).unwrap().circles;
            tool.circles.extend(evaluate(&b).unwrap().circles);
            let scene = vec![(1, g.clone()), (2, tool.clone())];
            let op = Boolean::new(
                Operation::Imprint,
                &[(1, Some(cap)), (2, Some(0))],
                SelectionMode::Face,
                &scene,
            );
            let result = imprint(&op, &[&g, &tool]).unwrap();
            assert!((volume(&result) - volume(&g)).abs() < 0.1);
            assert_eq!(result.face_count, g.face_count + 2);
            let regions = (0..result.face_count)
                .filter(|f| {
                    crate::snapping::Plane::face(&result, *f)
                        .is_some_and(|p| p.normal.z > 0.99 && (p.origin.z - 3.).abs() < 1e-7)
                })
                .collect::<Vec<_>>();
            assert_eq!(regions.len(), 3);
            for (face, role) in result
                .shell
                .as_ref()
                .unwrap()
                .iter()
                .zip(result.surface_mapping().unwrap())
            {
                if regions.contains(role) {
                    assert!(
                        face.edge_iter()
                            .all(|e| !matches!(e.curve(), truck_modeling::Curve::Line(_)))
                    );
                }
            }
            crate::components::solid_parts(result.shell.as_ref().unwrap()).unwrap();
            let mut print = result.clone();
            crate::components::weld_geometry(&mut print);
            crate::components::boundary_shell(&print, true)
                .unwrap_or_else(|e| panic!("profile center {x}: {e}"));
            assert_eq!(result.circles.len(), g.circles.len() + 2);
        }
    }
    #[test]
    fn imprint_keeps_curves_material_and_editable_independent_regions() {
        use crate::booleans::{Boolean, Kind as Operation};
        use truck_modeling::{Point3, Transform};
        for host in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            for profile in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
                let mut source = Body::new(host);
                source.id = 1;
                source.width = 4.;
                source.depth = 4.;
                source.extrusion = 3.;
                source.position = [2., -3., 1.];
                source.rotation = [15., 25., 35.];
                source.scale = [1.5, 0.75, 2.];
                let mut tool = Body::new(profile);
                tool.id = 2;
                tool.width = 1.;
                tool.depth = 1.;
                tool.rotation = source.rotation;
                tool.scale = source.scale;
                tool.position = crate::regions::transform(&source)
                    .transform_point(Point3::new(
                        0.,
                        if host == Shape::Semicircle { 1. } else { 0. },
                        3.,
                    ))
                    .into();
                let mut bodies = vec![source.clone(), tool.clone()];
                let scene = evaluate_scene(&bodies).unwrap();
                let face = (0..scene[0].1.face_count)
                    .find(|f| {
                        crate::edge_treatments::target(&source, &scene[0].1, *f)
                            == Ok(FaceTarget::Top)
                    })
                    .unwrap();
                let original_volume = volume(&scene[0].1);
                let original_faces = scene[0].1.face_count;
                let mut feature = Body::new(Shape::Rectangle);
                feature.id = 3;
                feature.boolean = Some(Boolean::new(
                    Operation::Imprint,
                    &[(1, Some(face)), (2, Some(0))],
                    SelectionMode::Face,
                    &scene,
                ));
                bodies.push(feature.clone());
                let scene = evaluate_scene(&bodies).unwrap();
                let g = &scene[2].1;
                assert!(g.is_solid);
                assert_eq!(g.face_count, original_faces + 1);
                assert!((volume(g) - original_volume).abs() < 0.15);
                assert!(scene[0].1.triangles.is_empty() && scene[1].1.triangles.is_empty());
                let inner = g.face_count - 1;
                let area = g
                    .triangles
                    .iter()
                    .zip(&g.triangle_faces)
                    .filter(|(_, f)| **f == inner)
                    .map(|(t, _)| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
                    .sum::<f64>();
                if host == Shape::Rectangle {
                    for distance in [-0.5, -10.] {
                        let started = std::time::Instant::now();
                        let cut =
                            crate::components::sweep_face(&feature, g, inner, distance).unwrap();
                        let mut changed = bodies.clone();
                        changed[2] = cut;
                        let out = evaluate_scene(&changed).unwrap();
                        assert!(
                            (volume(&out[2].1)
                                - (original_volume
                                    - area * if distance < -6. { 6. } else { -distance }))
                            .abs()
                                < 0.15
                        );
                        crate::components::solid_parts(out[2].1.shell.as_ref().unwrap()).unwrap();
                        assert!(
                            started.elapsed() < std::time::Duration::from_secs(5),
                            "A contained pocket/through-cut must use the direct boundary path"
                        );
                        if profile == Shape::Circle {
                            let plane = crate::snapping::Plane::face(g, inner).unwrap();
                            let expected = truck_modeling::Point3::from(tool.position)
                                + plane.normal * distance.max(-6.);
                            assert!(
                                out[2]
                                    .1
                                    .circles
                                    .iter()
                                    .any(|ring| (ring.center - expected).magnitude() < 1e-6)
                            );
                            for format in crate::export::Format::ALL {
                                assert!(
                                    !crate::export::encode(&changed, format).unwrap().is_empty()
                                );
                            }
                        }
                    }
                }
                let swept = crate::components::sweep_face(&feature, g, inner, 0.5).unwrap();
                let mut changed = bodies.clone();
                changed[2] = swept.clone();
                let out = evaluate_scene(&changed).unwrap();
                assert!((volume(&out[2].1) - original_volume - area * 0.5).abs() < 0.15);
                let walls = match profile {
                    Shape::Rectangle => 4,
                    Shape::Circle => 1,
                    _ => 2,
                };
                assert_eq!(
                    out[2].1.face_count,
                    g.face_count + walls,
                    "Smooth curve segments must form one logical wall"
                );
                crate::components::solid_parts(out[2].1.shell.as_ref().unwrap()).unwrap();
                for format in crate::export::Format::ALL {
                    assert!(!crate::export::encode(&changed, format).unwrap().is_empty());
                }
                let mut model = Model::from_bodies(bodies.clone());
                model.commit(swept).unwrap();
                model.undo();
                assert_eq!(model.bodies, bodies);
                model.redo();
                assert_eq!(model.bodies, changed);
                let project = Project::new(changed.clone(), Camera::default(), true, Some(3));
                let file = tempfile::NamedTempFile::new().unwrap().into_temp_path();
                project.save(file.as_ref()).unwrap();
                assert_eq!(Project::load(file.as_ref()).unwrap().bodies, changed);
                let mut old = serde_json::to_value(&project).unwrap();
                old["schema_version"] = 23.into();
                old["modeling_version"] = 23.into();
                assert!(
                    serde_json::from_value::<Project>(old)
                        .unwrap()
                        .validate()
                        .unwrap_err()
                        .contains("version 24")
                );
                let accepted = model.bodies.clone();
                let mut offset = tool.clone();
                offset.position[2] += 2.;
                assert!(model.commit(offset).is_err());
                assert_eq!(model.bodies, accepted);
                assert!(model.delete(2).is_err());
                assert_eq!(model.bodies, accepted);
                model.delete(3).unwrap();
                let restored = evaluate_scene(&model.bodies).unwrap();
                assert_eq!(
                    restored[0].1.triangles,
                    evaluate(&source).unwrap().triangles
                );
            }
        }
    }
    #[test]
    fn linear_imprint_crossings_split_adjacent_edges_without_touching_other_faces() {
        use crate::booleans::{Boolean, Kind as Operation};
        use truck_modeling::{Point3, Transform};
        for affine in [false, true] {
            let mut source = Body::new(Shape::Rectangle);
            source.id = 1;
            source.width = 4.;
            source.depth = 4.;
            source.extrusion = 3.;
            if affine {
                source.position = [2., -3., 1.];
                source.rotation = [15., 25., 35.];
                source.scale = [1.5, 0.75, 2.];
            }
            let placement = crate::regions::transform(&source);
            let inverse = placement.invert().unwrap();
            let scale = if affine { 2.25 } else { 1. };
            for (width, depth, x, regions) in [(6., 1., 0., 8), (4., 1., 0., 8), (2., 2., 1.5, 7)] {
                let mut tool = Body::new(Shape::Rectangle);
                tool.id = 2;
                tool.width = width;
                tool.depth = depth;
                tool.position = placement.transform_point(Point3::new(x, 0., 3.)).into();
                tool.rotation = source.rotation;
                tool.scale = source.scale;
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
                    Operation::Imprint,
                    &[(1, Some(face)), (2, Some(0))],
                    SelectionMode::Face,
                    &scene,
                ));
                bodies.push(feature.clone());
                let scene = evaluate_scene(&bodies).unwrap();
                let g = &scene[2].1;
                assert!((volume(g) - 48. * scale).abs() < 1e-6);
                assert!(g.is_solid);
                assert_eq!(g.face_count, regions);
                crate::components::solid_parts(g.shell.as_ref().unwrap()).unwrap();
                let bottom = g
                    .triangles
                    .iter()
                    .filter(|t| t.iter().all(|p| inverse.transform_point(*p).z.abs() < 1e-7))
                    .map(|t| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
                    .sum::<f64>();
                assert!((bottom - 16. * source.scale[0] * source.scale[1]).abs() < 1e-6);
                assert!(
                    g.points
                        .iter()
                        .map(|p| inverse.transform_point(*p))
                        .any(|p| (p.z - 3.).abs() < 1e-7
                            && (p.y.abs() - depth / 2.).abs() < 1e-7
                            && (p.x.abs() - 2.).abs() < 1e-7)
                );
                let inner = g.face_count - 1;
                let area = g
                    .triangles
                    .iter()
                    .zip(&g.triangle_faces)
                    .filter(|(_, f)| **f == inner)
                    .map(|(t, _)| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
                    .sum::<f64>();
                for distance in [0.5, -0.5] {
                    let edit = crate::components::sweep_face(&feature, g, inner, distance).unwrap();
                    let mut changed = bodies.clone();
                    changed[2] = edit;
                    let out = evaluate_scene(&changed).unwrap();
                    assert!((volume(&out[2].1) - (48. * scale + area * distance)).abs() < 1e-6);
                }
                for format in crate::export::Format::ALL {
                    assert!(!crate::export::encode(&bodies, format).unwrap().is_empty());
                }
            }
        }
    }
    #[test]
    fn imprint_rejects_occluded_and_legacy_curved_crossings_and_preserves_holes() {
        use crate::booleans::{Boolean, Kind as Operation};
        let mut source = Body::new(Shape::Rectangle);
        source.id = 1;
        source.width = 4.;
        source.depth = 4.;
        source.extrusion = 3.;
        source.hollow = Some(crate::shelling::Hollow::new(&source, FaceTarget::Top).unwrap());
        let g = evaluate(&source).unwrap();
        let floor = (0..g.face_count)
            .find(|f| {
                crate::snapping::Plane::face(&g, *f)
                    .is_some_and(|p| p.normal.z > 0.99 && p.origin.z > 0. && p.origin.z < 1.)
            })
            .unwrap();
        let z = crate::snapping::Plane::face(&g, floor).unwrap().origin.z;
        let mut tool = Body::new(Shape::Circle);
        tool.id = 2;
        tool.width = 0.5;
        tool.position = [0., 0., z];
        let mut bodies = vec![source.clone(), tool.clone()];
        let scene = evaluate_scene(&bodies).unwrap();
        let mut feature = Body::new(Shape::Rectangle);
        feature.id = 3;
        feature.boolean = Some(Boolean::new(
            Operation::Imprint,
            &[(1, Some(floor)), (2, Some(0))],
            SelectionMode::Face,
            &scene,
        ));
        bodies.push(feature.clone());
        let g = evaluate_scene(&bodies).unwrap().remove(2).1;
        assert!(g.is_solid);
        assert!((volume(&g) - volume(&scene[0].1)).abs() < 0.02);
        // The cavity opening and the opposite exterior face are untouched.
        assert_eq!(
            g.points.iter().filter(|p| (p.z - 3.).abs() < 1e-7).count(),
            scene[0]
                .1
                .points
                .iter()
                .filter(|p| (p.z - 3.).abs() < 1e-7)
                .count()
        );
        let mut model = Model::from_bodies(bodies.clone());
        let mut kept = feature.clone();
        kept.boolean.as_mut().unwrap().keep_tools = true;
        model.commit(kept).unwrap();
        assert!(
            !evaluate_scene(&model.bodies).unwrap()[1]
                .1
                .triangles
                .is_empty()
        );
        model.undo();
        assert_eq!(model.bodies, bodies);
        for z in [0., 3.] {
            let mut hidden = tool.clone();
            hidden.position[2] = z;
            assert!(model.commit(hidden).is_err());
            assert_eq!(model.bodies, bodies);
        }
        let mut outside = tool;
        outside.position[0] = 10.;
        assert!(model.commit(outside).is_err());
        assert_eq!(model.bodies, bodies);
        let mut curved = Body::new(Shape::Circle);
        curved.id = 1;
        curved.width = 4.;
        curved.extrusion = 3.;
        let mut profile = Body::new(Shape::Rectangle);
        profile.id = 2;
        profile.width = 3.;
        profile.depth = 1.;
        profile.position = [1., 0., 3.];
        let scene = evaluate_scene(&[curved.clone(), profile.clone()]).unwrap();
        let top = (0..scene[0].1.face_count)
            .find(|f| {
                crate::edge_treatments::target(&curved, &scene[0].1, *f) == Ok(FaceTarget::Top)
            })
            .unwrap();
        let mut feature = Body::new(Shape::Rectangle);
        feature.id = 3;
        feature.boolean = Some(Boolean::new(
            Operation::Imprint,
            &[(1, Some(top)), (2, Some(0))],
            SelectionMode::Face,
            &scene,
        ));
        // Legacy histories retain the old crossing policy; newly authored
        // analytic-crossing Imprints are checked by profile_imprints tests.
        feature.boolean.as_mut().unwrap().analytic_crossings = false;
        assert!(
            evaluate_scene(&[curved, profile, feature])
                .err()
                .unwrap()
                .contains("curved crossings")
        );
        let mut ring = Body::new(Shape::Circle);
        ring.id = 1;
        ring.width = 4.;
        ring.extrusion = 3.;
        ring.hollow = Some(crate::shelling::Hollow::new(&ring, FaceTarget::Top).unwrap());
        let mut disk = Body::new(Shape::Circle);
        disk.id = 2;
        disk.width = 0.1;
        disk.position = [1.9, 0., 3.];
        let mut bodies = vec![ring.clone(), disk];
        let scene = evaluate_scene(&bodies).unwrap();
        let top = (0..scene[0].1.face_count)
            .find(|f| crate::edge_treatments::target(&ring, &scene[0].1, *f) == Ok(FaceTarget::Top))
            .unwrap();
        let mut feature = Body::new(Shape::Rectangle);
        feature.id = 3;
        feature.boolean = Some(Boolean::new(
            Operation::Imprint,
            &[(1, Some(top)), (2, Some(0))],
            SelectionMode::Face,
            &scene,
        ));
        bodies.push(feature);
        let result = evaluate_scene(&bodies).unwrap().remove(2).1;
        assert!((volume(&result) - volume(&scene[0].1)).abs() < 0.03);
        assert!(
            result
                .shell
                .as_ref()
                .unwrap()
                .iter()
                .any(|face| face.boundaries().len() == 3),
            "Existing inner loops remain holes when another region is imprinted"
        );
    }
    #[test]
    fn replace_planar_faces_and_curved_prism_caps_follow_live_references() {
        use crate::booleans::{Boolean, Kind as Operation};
        use truck_modeling::{Point3, Transform};
        for shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            for cap in [FaceTarget::Top, FaceTarget::Bottom] {
                let mut source = Body::new(shape);
                source.id = 1;
                source.width = 4.;
                source.depth = 3.;
                source.extrusion = 3.;
                source.position = [2., -3., 1.];
                source.rotation = [15., 25., 35.];
                source.scale = [1.5, 0.75, 2.];
                let mut reference = Body::new(Shape::Rectangle);
                reference.id = 2;
                reference.construction = true;
                reference.position = crate::regions::transform(&source)
                    .transform_point(Point3::new(
                        0.,
                        0.,
                        if cap == FaceTarget::Top { 2.5 } else { 0.5 },
                    ))
                    .into();
                reference.rotation = source.rotation;
                reference.rotation[0] += 10.;
                let originals = vec![source.clone(), reference.clone()];
                let scene = evaluate_scene(&originals).unwrap();
                let face = (0..scene[0].1.face_count)
                    .find(|f| crate::edge_treatments::target(&source, &scene[0].1, *f) == Ok(cap))
                    .unwrap();
                let plane = crate::snapping::Plane::face(&scene[1].1, 0).unwrap();
                let mut feature = Body::new(Shape::Rectangle);
                feature.id = 3;
                feature.boolean = Some(Boolean::new(
                    Operation::ReplaceFace,
                    &[(1, Some(face)), (2, Some(0))],
                    SelectionMode::Face,
                    &scene,
                ));
                let mut bodies = originals.clone();
                bodies.push(feature.clone());
                let scene = evaluate_scene(&bodies).unwrap();
                let result = &scene[2].1;
                assert!(scene[0].1.triangles.is_empty());
                assert_eq!(scene[1].1.points, evaluate(&reference).unwrap().points);
                assert!(result.is_solid);
                assert!(result.shell.as_ref().unwrap().is_geometric_consistent());
                let area = match shape {
                    Shape::Rectangle => 12.,
                    Shape::Circle => 4. * std::f64::consts::PI,
                    _ => 2. * std::f64::consts::PI,
                };
                let centroid = if shape == Shape::Semicircle {
                    8. / (3. * std::f64::consts::PI)
                } else {
                    0.
                };
                let shift = 0.75 / 2. * 10f64.to_radians().tan() * centroid;
                let expected = area
                    * (2.5
                        + if cap == FaceTarget::Top {
                            shift
                        } else {
                            -shift
                        })
                    * 2.25;
                assert!(
                    (volume(result) - expected).abs() < 0.3,
                    "{shape:?} {cap:?}: {} vs {expected}",
                    volume(result)
                );
                let fixed = crate::operators::support_frame(&source, cap.opposite()).unwrap();
                let normal = fixed.z.truncate();
                let origin = Point3::from_vec(fixed.w.truncate());
                assert!(
                    result
                        .points
                        .iter()
                        .all(|p| (*p - origin).dot(normal).abs() < 1e-6
                            || (*p - plane.origin).dot(plane.normal).abs() < 1e-6)
                );
                if shape == Shape::Circle {
                    assert_eq!(result.face_count, 3);
                    assert_eq!(result.circles.len(), 2);
                }
                for format in crate::export::Format::ALL {
                    assert!(!crate::export::encode(&bodies, format).unwrap().is_empty());
                }
                let mut model = Model::from_bodies(bodies.clone());
                let mut moved = reference.clone();
                moved.position = (Point3::from(moved.position) + fixed.z.truncate() * 0.25).into();
                model.commit(moved.clone()).unwrap();
                assert!(
                    (volume(&evaluate_scene(&model.bodies).unwrap()[2].1) - expected).abs()
                        > area * 0.1
                );
                model.undo();
                assert_eq!(model.bodies, bodies);
                model.redo();
                assert_eq!(model.bodies[1], moved);
                let accepted = model.bodies.clone();
                assert!(model.delete(2).is_err());
                assert_eq!(model.bodies, accepted);
                let project = Project::new(accepted.clone(), Camera::default(), true, Some(3));
                let file = tempfile::NamedTempFile::new().unwrap().into_temp_path();
                project.save(file.as_ref()).unwrap();
                assert_eq!(Project::load(file.as_ref()).unwrap().bodies, accepted);
                let mut old = serde_json::to_value(&project).unwrap();
                old["schema_version"] = 22.into();
                old["modeling_version"] = 22.into();
                assert!(
                    serde_json::from_value::<Project>(old)
                        .unwrap()
                        .validate()
                        .unwrap_err()
                        .contains("version 23")
                );
                let mut stale = feature.clone();
                stale.boolean.as_mut().unwrap().topology[0].faces.pop();
                assert!(model.commit(stale).is_err());
                assert_eq!(model.bodies, accepted);
                let mut invalid = feature.clone();
                invalid.boolean.as_mut().unwrap().keep_tools = true;
                assert!(invalid.validate().is_err());
                invalid = feature;
                invalid.boolean.as_mut().unwrap().operands[1].0 = 3;
                assert!(invalid.validate().is_err());
                model.delete(3).unwrap();
                let scene = evaluate_scene(&model.bodies).unwrap();
                assert!((volume(&scene[0].1) - volume(&evaluate(&source).unwrap())).abs() < 1e-6);
            }
        }
    }
    #[test]
    fn replacement_and_boolean_chains_share_ordered_reference_evaluation() {
        use crate::booleans::{Boolean, Kind as Operation};
        let mut source = Body::new(Shape::Rectangle);
        source.id = 1;
        source.width = 4.;
        source.depth = 4.;
        source.extrusion = 3.;
        let mut reference = Body::new(Shape::Rectangle);
        reference.id = 2;
        reference.construction = true;
        reference.position[2] = 2.5;
        let mut bodies = vec![source.clone(), reference];
        let scene = evaluate_scene(&bodies).unwrap();
        let face = (0..scene[0].1.face_count)
            .find(|f| {
                crate::edge_treatments::target(&source, &scene[0].1, *f) == Ok(FaceTarget::Top)
            })
            .unwrap();
        let mut feature = Body::new(Shape::Rectangle);
        feature.id = 3;
        feature.boolean = Some(Boolean::new(
            Operation::ReplaceFace,
            &[(1, Some(face)), (2, Some(0))],
            SelectionMode::Face,
            &scene,
        ));
        bodies.push(feature);
        let mut cutter = source;
        cutter.id = 4;
        cutter.width = 3.;
        cutter.depth = 3.;
        bodies.push(cutter);
        let scene = evaluate_scene(&bodies).unwrap();
        let mut intersection = Body::new(Shape::Rectangle);
        intersection.id = 5;
        intersection.boolean = Some(Boolean::new(
            Operation::Intersect,
            &[(3, None), (4, None)],
            SelectionMode::Volume,
            &scene,
        ));
        bodies.push(intersection);
        let mut reference = Body::new(Shape::Rectangle);
        reference.id = 6;
        reference.construction = true;
        reference.position[2] = 1.5;
        bodies.push(reference);
        let scene = evaluate_scene(&bodies).unwrap();
        let face = (0..scene[4].1.face_count)
            .find(|f| {
                crate::snapping::Plane::face(&scene[4].1, *f).is_some_and(|p| p.normal.z > 0.99)
            })
            .unwrap();
        let mut feature = Body::new(Shape::Rectangle);
        feature.id = 7;
        feature.boolean = Some(Boolean::new(
            Operation::ReplaceFace,
            &[(5, Some(face)), (6, Some(0))],
            SelectionMode::Face,
            &scene,
        ));
        bodies.push(feature);
        let scene = evaluate_scene(&bodies).unwrap();
        assert!((volume(&scene[6].1) - 13.5).abs() < 1e-6);
        for index in [0, 2, 3, 4] {
            assert!(scene[index].1.triangles.is_empty());
        }
        for format in crate::export::Format::ALL {
            assert!(!crate::export::encode(&bodies, format).unwrap().is_empty());
        }
        let mut model = Model::from_bodies(bodies.clone());
        let mut moved = bodies[1].clone();
        moved.position[2] = 2.75;
        model.commit(moved).unwrap();
        assert!((volume(&evaluate_scene(&model.bodies).unwrap()[6].1) - 13.5).abs() < 1e-6);
        model.undo();
        assert_eq!(model.bodies, bodies);
    }
    #[test]
    fn cap_preserves_annular_holes_and_rejects_nonplanar_openings_atomically() {
        let mut body = Body::new(Shape::Circle);
        body.id = 1;
        body.width = 4.;
        body.extrusion = 3.;
        body.hollow = Some(crate::shelling::Hollow::new(&body, FaceTarget::Top).unwrap());
        let g = evaluate(&body).unwrap();
        let original_volume = volume(&g);
        let top = (0..g.face_count)
            .find(|f| crate::edge_treatments::target(&body, &g, *f) == Ok(FaceTarget::Top))
            .unwrap();
        let mut changes = selected(
            &[body.clone()],
            &[(1, g.clone())],
            &[(1, Some(top))],
            FaceEdit::Open,
        )
        .unwrap();
        let opened = changes.remove(0);
        let open = evaluate(&opened).unwrap();
        assert_eq!(open.shell.as_ref().unwrap().extract_boundaries().len(), 2);
        let capped = selected(&[opened], &[(1, open)], &[(1, None)], FaceEdit::CapOpenings)
            .unwrap()
            .remove(0);
        let g = evaluate(&capped).unwrap();
        assert!(g.is_solid);
        assert!((volume(&g) - original_volume).abs() < 0.1);
        let cap = (0..g.face_count)
            .find(|f| {
                crate::snapping::Plane::face(&g, *f).is_some_and(|p| (p.origin.z - 3.).abs() < 1e-7)
            })
            .unwrap();
        assert_eq!(
            g.shell
                .as_ref()
                .unwrap()
                .iter()
                .zip(g.surface_mapping().unwrap())
                .find(|(_, role)| **role == cap)
                .unwrap()
                .0
                .boundaries()
                .len(),
            2,
            "Capping a rim must preserve its central hole"
        );
        assert!(
            !crate::export::encode(&[capped], crate::export::Format::Stl)
                .unwrap()
                .is_empty()
        );

        let mut a = Body::new(Shape::Rectangle);
        a.id = 1;
        a.extrusion = 3.;
        let mut b = a.clone();
        b.id = 2;
        b.position[0] = 5.;
        let scene = evaluate_scene(&[a.clone(), b.clone()]).unwrap();
        let picks = scene
            .iter()
            .flat_map(|(id, g)| {
                let body = if *id == 1 { &a } else { &b };
                (0..g.face_count)
                    .filter(|f| {
                        matches!(
                            crate::edge_treatments::target(body, g, *f),
                            Ok(FaceTarget::Top | FaceTarget::Right)
                        )
                    })
                    .map(|f| (*id, Some(f)))
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        let changes = selected(&[a.clone(), b.clone()], &scene, &picks, FaceEdit::Open).unwrap();
        assert_eq!(changes.len(), 2);
        assert!(
            changes
                .iter()
                .all(|b| b.component_edits[0].elements.len() == 1)
        );
        let mut model = Model::from_bodies(vec![a.clone(), b.clone()]);
        model.commit_changes(&changes).unwrap();
        let committed = model.bodies.clone();
        let scene = evaluate_scene(&committed).unwrap();
        let changes = selected(
            &committed,
            &scene,
            &[(1, None), (2, None)],
            FaceEdit::CapOpenings,
        )
        .unwrap();
        assert!(model.commit_changes(&changes).is_err());
        assert_eq!(model.bodies, committed);
        model.undo();
        assert_eq!(model.bodies, vec![a, b]);
        model.redo();
        assert_eq!(model.bodies, committed);
    }
    #[test]
    fn complete_external_notch_heals_across_planar_supports_with_history() {
        use crate::booleans::{Boolean, Kind as BooleanKind};
        for extra_hole in [false, true] {
            for blind in [false, true] {
                let mut base = Body::new(Shape::Rectangle);
                base.id = 1;
                base.width = 6.;
                base.depth = 6.;
                base.extrusion = 3.;
                let mut tool = Body::new(Shape::Rectangle);
                tool.id = 2;
                tool.width = 4.;
                tool.depth = 2.;
                tool.position = [2., 0., if blind { 1. } else { 0. }];
                tool.extrusion = 3.;
                let mut bodies = vec![base.clone(), tool];
                if extra_hole {
                    let mut hole = Body::new(Shape::Circle);
                    hole.id = 3;
                    hole.width = 1.;
                    hole.position = [-1.5, 0., 3.];
                    let mut bore = crate::operators::Bore::new(&base, FaceTarget::Top).unwrap();
                    bore.mode = crate::operators::BoreMode::Borer;
                    bore.travel = -4.;
                    hole.bore = Some(bore);
                    bodies.push(hole);
                }
                let result_index = bodies.len();
                let result_id = result_index as u64 + 1;

                let scene = evaluate_scene(&bodies).unwrap();
                if extra_hole {
                    assert_eq!(scene[0].1.circles.len(), 2, "Bore before notch");
                }
                let mut cut = Body::new(Shape::Rectangle);
                cut.id = result_id;
                cut.boolean = Some(Boolean::new(
                    BooleanKind::Subtract,
                    &[(1, None), (2, None)],
                    SelectionMode::Volume,
                    &scene,
                ));
                bodies.push(cut);
                let scene = evaluate_scene(&bodies).unwrap();
                let g = &scene[result_index].1;
                let walls = (0..g.face_count)
                    .filter(|face| {
                        let points =
                            crate::components::vertices(g, SelectionMode::Face, Some(*face));
                        !points.is_empty()
                            && points.iter().all(|i| {
                                let p = g.points[*i];
                                p.x >= -1e-6
                                    && p.y.abs() <= 1.000001
                                    && p.z >= if blind { 0.999999 } else { -1e-6 }
                            })
                    })
                    .map(|face| (result_id, Some(face)))
                    .collect::<Vec<_>>();
                assert!(!walls.is_empty());
                let healed = selected(&bodies, &scene, &walls, FaceEdit::Heal)
                    .unwrap()
                    .remove(0);
                let mut model = Model::from_bodies(bodies.clone());
                model.commit(healed.clone()).unwrap();
                let out = evaluate_scene(&model.bodies).unwrap();
                let out = &out[result_index].1;
                assert!(
                    out.is_solid && out.shell.as_ref().unwrap().extract_boundaries().is_empty()
                );
                let expected = 108.
                    - if extra_hole {
                        std::f64::consts::PI * 0.75
                    } else {
                        0.
                    };
                assert!((volume(out) - expected).abs() < 0.03);
                if extra_hole {
                    assert_eq!(g.circles.len(), 2);
                    assert_eq!(out.circles.len(), 2);
                    let direct = heal_planar_notch(
                        g,
                        &walls
                            .iter()
                            .map(|(_, face)| face.unwrap())
                            .collect::<Vec<_>>(),
                    )
                    .unwrap();
                    for face in g
                        .shell
                        .as_ref()
                        .unwrap()
                        .iter()
                        .filter(|f| !matches!(f.surface(), truck_modeling::Surface::Plane(_)))
                    {
                        assert!(
                            direct
                                .shell
                                .as_ref()
                                .unwrap()
                                .iter()
                                .any(|f| f.id() == face.id()),
                            "Unrelated curved walls keep their original surface and trim objects"
                        );
                    }
                } else {
                    assert!(out.shell.as_ref().unwrap().is_geometric_consistent());
                    assert_eq!(out.face_count, 6);
                }
                for format in crate::export::Format::ALL {
                    assert!(
                        !crate::export::encode(&model.bodies, format)
                            .unwrap()
                            .is_empty()
                    );
                }
                let file = tempfile::NamedTempFile::new().unwrap().into_temp_path();
                Project::new(
                    model.bodies.clone(),
                    Camera::default(),
                    true,
                    Some(result_id),
                )
                .save(file.as_ref())
                .unwrap();
                assert_eq!(Project::load(file.as_ref()).unwrap().bodies, model.bodies);
                model.undo();
                assert_eq!(model.bodies, bodies);
                model.redo();
                assert_eq!(model.bodies[result_index], healed);
                if extra_hole && !blind {
                    if let Some(path) = std::env::var_os("OSK_NOTCH_REVIEW_PROJECT") {
                        let mut review =
                            Project::new(bodies.clone(), Camera::default(), true, Some(result_id));
                        review.selection_mode = SelectionMode::Face;
                        review.selection = walls.clone();
                        review.selected_element = walls.last().unwrap().1;
                        review.save(std::path::Path::new(&path)).unwrap();
                    }
                    let project =
                        Project::new(bodies.clone(), Camera::default(), true, Some(result_id));
                    let mut json = serde_json::to_value(&project).unwrap();
                    json["schema_version"] = 30.into();
                    json["modeling_version"] = 30.into();
                    assert!(
                        serde_json::from_value::<Project>(json.clone())
                            .unwrap()
                            .validate()
                            .unwrap_err()
                            .contains("version 31")
                    );
                    json["bodies"][result_index]["boolean"]
                        .as_object_mut()
                        .unwrap()
                        .remove("analytic_prisms");
                    std::fs::write(
                        file.as_ref() as &std::path::Path,
                        serde_json::to_vec(&json).unwrap(),
                    )
                    .unwrap();
                    let legacy = Project::load(file.as_ref()).unwrap();
                    assert!(
                        !legacy.bodies[result_index]
                            .boolean
                            .as_ref()
                            .unwrap()
                            .analytic_prisms
                    );
                    let old = evaluate_scene(&legacy.bodies).unwrap();
                    assert!(
                        old[result_index]
                            .1
                            .shell
                            .as_ref()
                            .unwrap()
                            .iter()
                            .all(|face| matches!(
                                face.surface(),
                                truck_modeling::Surface::Plane(_)
                            )),
                        "Missing intent preserves the historical faceted prism result"
                    );
                }
                let partial = selected(&bodies, &scene, &walls[..1], FaceEdit::Heal)
                    .unwrap()
                    .remove(0);
                let mut unchanged = Model::from_bodies(bodies.clone());
                assert!(unchanged.commit(partial).is_err());
                assert_eq!(unchanged.bodies, bodies);
                assert!(!unchanged.can_undo());
            }
        }
    }

    #[test]
    fn complete_pocket_and_bore_walls_heal_without_flattening_supports() {
        use crate::operators::{Bore, BoreMode};
        for shape in [Shape::Rectangle, Shape::Circle] {
            for through in [false, true] {
                let mut base = Body::new(shape);
                base.id = 1;
                base.width = 6.;
                base.depth = 6.;
                base.extrusion = 3.;
                let mut tool = Body::new(shape);
                tool.id = 2;
                tool.width = 2.;
                tool.depth = 2.;
                tool.position[2] = 3.;
                let mut bore = Bore::new(&base, FaceTarget::Top).unwrap();
                bore.mode = BoreMode::Borer;
                bore.travel = if through { -4. } else { -2. };
                tool.bore = Some(bore);
                let bodies = vec![base.clone(), tool];
                let scene = evaluate_scene(&bodies).unwrap();
                let g = &scene[0].1;
                let inner = (0..g.face_count)
                    .filter(|face| {
                        let vertices =
                            crate::components::vertices(g, SelectionMode::Face, Some(*face));
                        !vertices.is_empty()
                            && vertices.iter().all(|i| {
                                g.points[*i].x.abs() <= 1.000001 && g.points[*i].y.abs() <= 1.000001
                            })
                    })
                    .map(|face| (1, Some(g.face_group(face))))
                    .collect::<Vec<_>>();
                assert!(!inner.is_empty());
                let healed = selected(&bodies, &scene, &inner, FaceEdit::Heal)
                    .unwrap()
                    .remove(0);
                let mut model = Model::from_bodies(bodies.clone());
                model.commit(healed.clone()).unwrap();
                let out = evaluate_scene(&model.bodies).unwrap();
                let out = &out[0].1;
                assert!(out.is_solid && out.shell.as_ref().unwrap().is_geometric_consistent());
                let original = evaluate(&base).unwrap();
                assert!((volume(out) - volume(&original)).abs() < 0.03);
                assert_eq!(out.face_count, original.face_count);
                assert_eq!(out.circles.len(), original.circles.len());
                for format in crate::export::Format::ALL {
                    assert!(
                        !crate::export::encode(&model.bodies, format)
                            .unwrap()
                            .is_empty()
                    );
                }
                let file = tempfile::NamedTempFile::new().unwrap().into_temp_path();
                Project::new(model.bodies.clone(), Camera::default(), true, Some(1))
                    .save(file.as_ref())
                    .unwrap();
                assert_eq!(Project::load(file.as_ref()).unwrap().bodies, model.bodies);
                model.undo();
                assert_eq!(model.bodies, bodies);
                model.redo();
                assert_eq!(model.bodies[0], healed);
                if shape == Shape::Rectangle || !through {
                    let partial = selected(&bodies, &scene, &inner[..1], FaceEdit::Heal)
                        .unwrap()
                        .remove(0);
                    let mut unchanged = Model::from_bodies(bodies.clone());
                    assert!(unchanged.commit(partial).is_err());
                    assert_eq!(unchanged.bodies, bodies);
                    assert!(!unchanged.can_undo());
                }
            }
        }
        // Healing one of two holes must preserve the other hole and its trims.
        let mut base = Body::new(Shape::Rectangle);
        base.id = 1;
        base.width = 6.;
        base.depth = 6.;
        base.extrusion = 3.;
        let mut bodies = vec![base];
        for (id, x) in [(2, -1.5), (3, 1.5)] {
            let mut tool = Body::new(Shape::Circle);
            tool.id = id;
            tool.width = 1.;
            tool.position = [x, 0., 3.];
            let mut bore = Bore::new(&bodies[0], FaceTarget::Top).unwrap();
            bore.mode = BoreMode::Borer;
            bore.travel = -4.;
            tool.bore = Some(bore);
            bodies.push(tool);
        }
        let scene = evaluate_scene(&bodies).unwrap();
        let g = &scene[0].1;
        assert_eq!(
            g.circles.len(),
            4,
            "Both bores retain entry and exit circle references"
        );
        let inner = (0..g.face_count)
            .filter(|face| {
                let vertices = crate::components::vertices(g, SelectionMode::Face, Some(*face));
                !vertices.is_empty()
                    && vertices.iter().all(|i| {
                        (g.points[*i].x + 1.5).abs() <= 0.500001 && g.points[*i].y.abs() <= 0.500001
                    })
            })
            .map(|face| (1, Some(g.face_group(face))))
            .collect::<Vec<_>>();
        assert!(!inner.is_empty());
        let changed = selected(&bodies, &scene, &inner, FaceEdit::Heal)
            .unwrap()
            .remove(0);
        bodies[0] = changed;
        let out = evaluate_scene(&bodies).unwrap();
        let healed = &out[0].1;
        assert!((volume(healed) - (108. - std::f64::consts::PI * 0.75)).abs() < 0.03);
        assert_eq!(
            healed
                .shell
                .as_ref()
                .unwrap()
                .iter()
                .map(|face| face.boundaries().len() - 1)
                .sum::<usize>(),
            2,
            "The other through-hole keeps its top and bottom inner loops"
        );
        assert_eq!(healed.circles.len(), 2);
        assert!(
            healed
                .circles
                .iter()
                .all(|ring| (ring.center.x - 1.5).abs() < 1e-7)
        );
    }

    #[test]
    fn delete_open_cap_and_heal_keep_ordered_history_and_exact_boundaries() {
        for shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            let mut body = Body::new(shape);
            body.id = 1;
            body.width = 4.;
            body.depth = 3.;
            body.extrusion = 3.;
            body.position = [2., -3., 1.];
            body.rotation = [15., 25., 35.];
            body.scale = [1.5, 0.75, 2.];
            let scene = evaluate_scene(&[body.clone()]).unwrap();
            let cap = (0..scene[0].1.face_count)
                .find(|f| {
                    crate::edge_treatments::target(&body, &scene[0].1, *f) == Ok(FaceTarget::Top)
                })
                .unwrap();
            let source_volume = volume(&scene[0].1);
            let opened = selected(&[body.clone()], &scene, &[(1, Some(cap))], FaceEdit::Open)
                .unwrap()
                .remove(0);
            let open = evaluate(&opened).unwrap();
            if shape == Shape::Circle {
                assert_eq!(
                    open.circles.len(),
                    scene[0].1.circles.len(),
                    "An exposed open rim remains a circular inference target"
                );
            }
            assert!(!open.is_solid);
            assert_eq!(open.face_count, scene[0].1.face_count - 1);
            assert!(!open.shell.as_ref().unwrap().extract_boundaries().is_empty());
            assert!(
                open.topology.faces.iter().all(|edges| !edges.is_empty()),
                "Open sheet boundaries must remain selectable"
            );
            assert!(
                crate::export::encode(std::slice::from_ref(&opened), crate::export::Format::Stl)
                    .is_err()
            );
            for format in [
                crate::export::Format::Glb,
                crate::export::Format::Step,
                crate::export::Format::Dxf,
            ] {
                assert!(
                    !crate::export::encode(std::slice::from_ref(&opened), format)
                        .unwrap()
                        .is_empty()
                );
            }
            let capped = selected(
                std::slice::from_ref(&opened),
                &[(1, open.clone())],
                &[(1, None)],
                FaceEdit::CapOpenings,
            )
            .unwrap()
            .remove(0);
            let closed = evaluate(&capped).unwrap();
            assert!(closed.is_solid);
            assert!((volume(&closed) - source_volume).abs() < 0.06);
            Solid::try_new(closed.shell.as_ref().unwrap().connected_components()).unwrap();
            let end_face = (0..closed.face_count)
                .find(|f| {
                    crate::edge_treatments::target(&capped, &closed, *f) == Ok(FaceTarget::Top)
                })
                .unwrap();
            let extended = crate::components::sweep_face(&capped, &closed, end_face, 0.5).unwrap();
            let extended = evaluate(&extended).unwrap();
            assert!(extended.is_solid);
            assert!(volume(&extended) > volume(&closed));
            let direct = selected(&[body.clone()], &scene, &[(1, Some(cap))], FaceEdit::Cap)
                .unwrap()
                .remove(0);
            assert!((volume(&evaluate(&direct).unwrap()) - source_volume).abs() < 0.06);
            let mut model = Model::from_bodies(vec![body.clone()]);
            model.commit(opened.clone()).unwrap();
            model.commit(capped.clone()).unwrap();
            model.undo();
            assert_eq!(model.bodies, vec![opened.clone()]);
            model.redo();
            assert_eq!(model.bodies, vec![capped.clone()]);
            let project = Project::new(model.bodies.clone(), Camera::default(), true, Some(1));
            let file = tempfile::NamedTempFile::new().unwrap().into_temp_path();
            project.save(file.as_ref()).unwrap();
            assert_eq!(Project::load(file.as_ref()).unwrap().bodies, model.bodies);
            let mut old = serde_json::to_value(&project).unwrap();
            old["schema_version"] = 21.into();
            old["modeling_version"] = 21.into();
            assert!(
                serde_json::from_value::<Project>(old)
                    .unwrap()
                    .validate()
                    .unwrap_err()
                    .contains("version 22")
            );
            for format in crate::export::Format::ALL {
                assert!(
                    !crate::export::encode(std::slice::from_ref(&capped), format)
                        .unwrap()
                        .is_empty()
                );
            }
            let mut bad = opened.clone();
            bad.component_edits[0].matrix[3][0] = 1.;
            assert!(bad.validate().is_err());
            let mut changed_source = capped.clone();
            changed_source.width = 4.5;
            assert!(evaluate(&changed_source).unwrap().is_solid);
        }
        let mut body = Body::new(Shape::Rectangle);
        body.id = 1;
        body.width = 4.;
        body.depth = 3.;
        body.extrusion = 3.;
        body.edge_treatments
            .push(Modifier::new(Kind::Chamfer, FaceTarget::Top, &body));
        let g = evaluate(&body).unwrap();
        let bands = (0..g.face_count)
            .filter(|f| crate::edge_treatments::target(&body, &g, *f).is_err())
            .map(|f| (1, Some(f)))
            .collect::<Vec<_>>();
        assert!(!bands.is_empty());
        let healed = selected(&[body.clone()], &[(1, g)], &bands, FaceEdit::Heal)
            .unwrap()
            .remove(0);
        let g = evaluate(&healed).unwrap();
        assert!(g.is_solid);
        assert_eq!(g.face_count, 6);
        assert!((volume(&g) - 36.).abs() < 1e-6);
        for format in crate::export::Format::ALL {
            assert!(
                !crate::export::encode(std::slice::from_ref(&healed), format)
                    .unwrap()
                    .is_empty()
            );
        }
        let mut plain = body.clone();
        plain.edge_treatments.clear();
        let g = evaluate(&plain).unwrap();
        let top = (0..g.face_count)
            .find(|f| crate::edge_treatments::target(&plain, &g, *f) == Ok(FaceTarget::Top))
            .unwrap();
        let invalid = selected(
            &[plain.clone()],
            &[(1, g)],
            &[(1, Some(top))],
            FaceEdit::Heal,
        )
        .unwrap()
        .remove(0);
        let mut model = Model::from_bodies(vec![plain.clone()]);
        assert!(model.commit(invalid).is_err());
        assert_eq!(model.bodies, vec![plain]);
    }
}
