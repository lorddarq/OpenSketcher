//! Authored component transforms. Base profiles remain editable; output is regenerated.
use crate::{
    model::{Body, Geometry, Shape},
    selection::SelectionMode,
};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use truck_modeling::{
    Edge, EuclideanSpace, InnerSpace, Matrix3, Matrix4, Point3, Shell, Solid, SquareMatrix,
    Transform, Vector3, Vector4, Vertex, Wire, builder,
};

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Topology {
    pub edges: Vec<[usize; 2]>,
    pub faces: Vec<Vec<usize>>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ComponentEdit {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub control: Option<crate::soft_selection::Operation>,
    /// False preserves legacy edits on the source; true edits the evaluated result.
    #[serde(default)]
    pub evaluated: bool,
    #[serde(default)]
    pub retrim: bool,
    #[serde(default)]
    pub convex_rebuild: bool,
    #[serde(default)]
    pub section_transform: bool,
    #[serde(default)]
    pub cap_rotation: bool,
    #[serde(default)]
    pub sweep: Option<[f64; 3]>,
    #[serde(default)]
    pub tangent_walls: bool,
    #[serde(default)]
    pub prismatic_cut: bool,
    #[serde(default)]
    pub face_edit: Option<crate::face_operations::FaceEdit>,
    /// Source-local setbacks on the two adjacent evaluated support faces.
    #[serde(default)]
    pub edge_chamfer: Option<[f64; 2]>,
    #[serde(default)]
    pub prism_chamfer: bool,
    #[serde(default)]
    pub prism_bevel: Option<crate::edge_treatments::PrismBevel>,
    #[serde(default)]
    pub prism_curves: bool,
    pub mode: SelectionMode,
    pub element: usize,
    #[serde(default)]
    pub elements: Vec<usize>,
    pub topology: Topology,
    pub matrix: [[f64; 4]; 4],
}

pub fn vertices(g: &Geometry, mode: SelectionMode, element: Option<usize>) -> Vec<usize> {
    match (mode, element) {
        (SelectionMode::Volume, _) => (0..g.points.len()).collect(),
        (SelectionMode::Point, Some(i)) if i < g.points.len() => vec![i],
        (SelectionMode::Edge, Some(i)) => g
            .topology
            .edges
            .get(i)
            .map(|v| v.to_vec())
            .unwrap_or_default(),
        (SelectionMode::Face, Some(i)) => {
            let mut vertices: Vec<_> = g
                .topology
                .faces
                .get(i)
                .into_iter()
                .flatten()
                .flat_map(|edge| g.topology.edges[*edge])
                .collect();
            vertices.sort_unstable();
            vertices.dedup();
            vertices
        }
        _ => Vec::new(),
    }
}

pub fn pivot(g: &Geometry, mode: SelectionMode, element: Option<usize>) -> Option<Point3> {
    let points: Vec<_> = match mode {
        SelectionMode::Face => g
            .triangles
            .iter()
            .zip(&g.triangle_faces)
            .filter(|(_, face)| Some(g.face_group(**face)) == element.map(|f| g.face_group(f)))
            .flat_map(|(t, _)| t.iter().copied())
            .collect(),
        SelectionMode::Edge => g
            .edges
            .iter()
            .zip(&g.edge_groups)
            .filter(|(_, edge)| Some(g.edge_group(**edge)) == element.map(|e| g.edge_group(e)))
            .flat_map(|(e, _)| e.iter().copied())
            .collect(),
        _ => vertices(g, mode, element)
            .iter()
            .map(|i| g.points[*i])
            .collect(),
    };
    if points.is_empty() {
        return None;
    }
    Some(Point3::from(std::array::from_fn::<_, 3, _>(|axis| {
        let low = points.iter().map(|p| p[axis]).fold(f64::INFINITY, f64::min);
        let high = points
            .iter()
            .map(|p| p[axis])
            .fold(f64::NEG_INFINITY, f64::max);
        (low + high) / 2.
    })))
}

pub fn transformed(
    body: &Body,
    g: &Geometry,
    mode: SelectionMode,
    element: Option<usize>,
    matrix: Matrix4,
) -> Result<Body, String> {
    transformed_many(
        body,
        g,
        mode,
        &element.into_iter().collect::<Vec<_>>(),
        matrix,
    )
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OffsetTarget {
    Plane(crate::edge_treatments::FaceTarget),
    CylinderWall,
    EvaluatedPlane(usize),
}
impl OffsetTarget {
    pub fn label(self) -> &'static str {
        match self {
            Self::Plane(face) => face.label(),
            Self::CylinderWall => "Cylinder wall",
            Self::EvaluatedPlane(_) => "Evaluated planar face",
        }
    }
}
pub fn offset_target(body: &Body, g: &Geometry, face: usize) -> Result<OffsetTarget, String> {
    if let Ok(role) = crate::edge_treatments::target(body, g, face) {
        return Ok(OffsetTarget::Plane(role));
    }
    if body.shape == Shape::Circle
        && body.extrusion > 0.
        && body.component_edits.is_empty()
        && body.bore.is_none()
    {
        let inverse = crate::regions::transform(body)
            .invert()
            .ok_or("Invalid source placement.")?;
        let triangles: Vec<_> = g
            .triangles
            .iter()
            .zip(&g.triangle_faces)
            .filter(|(_, f)| **f == face)
            .map(|(t, _)| t.map(|p| inverse.transform_point(p)))
            .collect();
        if !triangles.is_empty()
            && triangles.iter().all(|t| {
                let n = (t[1] - t[0]).cross(t[2] - t[0]);
                n.z.abs() <= n.magnitude() * 1e-6
                    && t.iter()
                        .all(|p| (p.x.hypot(p.y) - body.width / 2.).abs() < 1e-5)
            })
        {
            return Ok(OffsetTarget::CylinderWall);
        }
    }
    Err("Select an original planar source face or cylindrical wall; transition/deformed faces require general surface retrimming.".into())
}
pub fn offset_source(body: &Body, target: OffsetTarget, distance: f64) -> Result<Body, String> {
    match target {
        OffsetTarget::Plane(face) => offset_source_face(body, face, distance),
        OffsetTarget::EvaluatedPlane(_)=>Err("Use Extrude face for this evaluated boundary; source offset does not apply.".into()),
        OffsetTarget::CylinderWall => {
            (|| {
            body.validate()?;
            if body.shape != Shape::Circle
                || body.extrusion <= 0.
                || body.bore.is_some()
                || !body.component_edits.is_empty()
                || !distance.is_finite()
            {
                return Err("Cylinder wall offset requires an undeformed circular solid and finite distance.".into());
            }
            if (body.scale[0] - body.scale[1]).abs() > 1e-9 * body.scale[0].max(body.scale[1]) {
                return Err("Cylinder wall offset requires equal X/Y scale. An ellipse's parallel offset is not another ellipse.".into());
            }
            let mut changed = body.clone();
            changed.width += 2. * distance / body.scale[0];
            changed.validate()?;
            Ok(changed)
            })().map_err(|reason:String|body.error(format!("Offset cylinder wall: {reason}")))
        }
    }
}

/// Edit an existing primitive's support plane without deforming its render mesh.
/// Distance is signed along the outward world normal; the opposite plane stays fixed.
pub fn offset_source_face(
    body: &Body,
    face: crate::edge_treatments::FaceTarget,
    distance: f64,
) -> Result<Body, String> {
    (|| {
    use crate::edge_treatments::FaceTarget;
    body.validate()?;
    if !distance.is_finite() {
        return Err("Face offset must be a finite distance.".into());
    }
    if !body.component_edits.is_empty() {
        return Err("Offset face currently requires an undeformed source solid; existing component edits cannot be retrimmed analytically yet.".into());
    }
    if body.shape == Shape::Line || body.extrusion <= 0. || !face.supported(body) {
        return Err(
            "Offset face supports original box faces and circular/semicircular solid caps.".into(),
        );
    }
    let placement = crate::regions::transform(body);
    let (normal, _) = face.plane(body);
    // Primitive placement is rotation * diagonal scale: this axis remains normal
    // to its support plane, even with nonuniform scale.
    let delta = distance / placement.transform_vector(normal).magnitude();
    let mut result = body.clone();
    let shift = match face {
        FaceTarget::Top | FaceTarget::Bottom => {
            result.extrusion += delta;
            if result.extrusion < 0.01 {
                return Err("Face offset would collapse or invert the solid. Leave at least 0.01 local units of thickness; passing through requires a Borer operation.".into());
            }
            if face == FaceTarget::Bottom {
                normal * delta
            } else {
                Vector3::new(0., 0., 0.)
            }
        }
        FaceTarget::Left | FaceTarget::Right => {
            result.width += delta;
            normal * (delta / 2.)
        }
        FaceTarget::Front | FaceTarget::Back => {
            result.depth += delta;
            normal * (delta / 2.)
        }
    };
    result.position = (Point3::from(body.position) + placement.transform_vector(shift)).into();
    result.validate()?;
    Ok(result)
    })().map_err(|reason:String|body.error(format!("Offset {}: {reason}",face.label())))
}
fn transformed_many(
    body: &Body,
    g: &Geometry,
    mode: SelectionMode,
    elements: &[usize],
    matrix: Matrix4,
) -> Result<Body, String> {
    if mode == SelectionMode::Edge && elements.iter().any(|edge| g.internal_edges.contains(edge)) {
        return Err("Internal cylinder junctions are not editable edges.".into());
    }
    let mut expanded = Vec::new();
    for element in elements {
        let group = match mode {
            SelectionMode::Face => g.face_elements(*element),
            SelectionMode::Edge => g.edge_elements(*element),
            _ => vec![*element],
        };
        for member in group {
            if !expanded.contains(&member) {
                expanded.push(member);
            }
        }
    }
    let elements = expanded.as_slice();
    let element = elements.first().copied();
    let mut result = body.clone();
    if matrix == Matrix4::identity() {
        return Ok(result);
    }
    if mode != SelectionMode::Volume && body.component_edits.iter().any(|e| e.control.is_some()) {
        if mode == SelectionMode::Face
            && elements.len() == 1
            && let Some(domain) = crate::soft_selection::domain(body, elements[0])
        {
            if matrix.x != Vector4::unit_x()
                || matrix.y != Vector4::unit_y()
                || matrix.z != Vector4::unit_z()
            {
                return Err("Control-face editing currently supports translation; rotate the volume to preserve the control representation.".into());
            }
            let mut operation = domain.clone();
            operation.action = crate::soft_selection::Action::MoveFace {
                delta: crate::regions::transform(body)
                    .invert()
                    .ok_or("Invalid object placement.")?
                    .transform_vector(matrix.w.truncate())
                    .into(),
            };
            return crate::soft_selection::append(body, g, elements[0], operation);
        }
        return Err("This body has an authored control net. Edit its nodes, move its control face, or transform the volume to preserve editability.".into());
    }
    if mode == SelectionMode::Volume {
        result.position = matrix.transform_point(Point3::from(body.position)).into();
        let delta = Matrix3::from_cols(
            matrix.x.truncate(),
            matrix.y.truncate(),
            matrix.z.truncate(),
        );
        if delta != Matrix3::identity() {
            let rotation = Matrix3::from_cols(
                matrix.x.truncate(),
                matrix.y.truncate(),
                matrix.z.truncate(),
            ) * Matrix3::from_angle_z(truck_modeling::Deg(body.rotation[2]))
                * Matrix3::from_angle_y(truck_modeling::Deg(body.rotation[1]))
                * Matrix3::from_angle_x(truck_modeling::Deg(body.rotation[0]));
            result.rotation = crate::regions::rotation_angles(rotation);
        }
    } else {
        let element = element.ok_or("Select a component first.")?;
        if vertices(g, mode, Some(element)).is_empty() {
            return Err("Selected component no longer exists.".into());
        }
        if mode == SelectionMode::Point
            && elements.len() == 1
            && (matrix.transform_point(g.points[element]) - g.points[element]).magnitude2() < 1e-18
        {
            return Ok(result);
        }
        let placement = crate::regions::transform(body);
        let retrim = mode == SelectionMode::Face && planar_supports(g).is_ok();
        let convex_rebuild = retrim && g.face_count <= 64;
        let mut selected = elements
            .iter()
            .flat_map(|element| vertices(g, mode, Some(*element)))
            .collect::<Vec<_>>();
        selected.sort_unstable();
        selected.dedup();
        let section_transform = !retrim
            && matches!(mode, SelectionMode::Face | SelectionMode::Edge)
            && crate::section_edits::translation(g, &selected, matrix)?.is_some();
        let retrim = retrim
            || (!section_transform
                && mode == SelectionMode::Face
                && straight_planar_supports(g).is_ok());
        result.component_edits.push(ComponentEdit {
            control: None,
            evaluated: true,
            retrim,
            convex_rebuild,
            section_transform,
            cap_rotation: !retrim && !section_transform && mode == SelectionMode::Face,
            sweep: None,
            tangent_walls: false,
            prismatic_cut: false,
            face_edit: None,
            edge_chamfer: None,
            prism_chamfer: false,
            prism_bevel: None,
            prism_curves: false,
            mode,
            element,
            elements: elements.iter().skip(1).copied().collect(),
            topology: g.topology.clone(),
            matrix: (placement.invert().ok_or("Invalid object transform.")? * matrix * placement)
                .into(),
        });
    }
    result.validate()?;
    Ok(result)
}

pub fn sweep_face(body: &Body, g: &Geometry, face: usize, distance: f64) -> Result<Body, String> {
    if !distance.is_finite() || distance.abs() > 100. {
        return Err("Face extrusion distance must be finite and within ±100 mm.".into());
    }
    let mut changed = body.clone();
    if distance == 0. {
        return Ok(changed);
    }
    let plane = crate::snapping::Plane::face(g, face).ok_or("Select a planar face to extrude.")?;
    if g.shell.is_none() {
        return Err(
            "This component deformation does not retain an exact boundary for extrusion.".into(),
        );
    }
    let vector = crate::regions::transform(body)
        .invert()
        .ok_or("Invalid source placement.")?
        .transform_vector(plane.normal * distance);
    changed.component_edits.push(ComponentEdit {
        control: None,
        evaluated: true,
        retrim: false,
        convex_rebuild: false,
        section_transform: false,
        cap_rotation: false,
        sweep: Some(vector.into()),
        tangent_walls: true,
        prismatic_cut: true,
        face_edit: None,
        edge_chamfer: None,
        prism_chamfer: false,
        prism_bevel: None,
        prism_curves: false,
        mode: SelectionMode::Face,
        element: face,
        elements: Vec::new(),
        topology: g.topology.clone(),
        matrix: Matrix4::identity().into(),
    });
    changed.validate()?;
    Ok(changed)
}

pub fn transform_selection(
    bodies: &[Body],
    scene: &[(u64, Geometry)],
    mode: SelectionMode,
    picks: &[crate::selection::Pick],
    matrix: Matrix4,
) -> Result<Vec<Body>, String> {
    if let Some((id, element)) = picks
        .iter()
        .find(|p| !crate::selection::valid(**p, scene, mode))
    {
        return Err(format!(
            "Transform · body {id} · {} {} no longer exists.",
            mode.label(),
            element.map_or(String::new(), |e| e.saturating_add(1).to_string())
        ));
    }
    let mut result = bodies.to_vec();
    for body in &mut result {
        let selected: Vec<_> = picks.iter().filter(|p| p.0 == body.id).collect();
        if selected.is_empty() {
            continue;
        }
        if mode == SelectionMode::Volume
            && let Some(bore) = &body.bore
        {
            // The attached profile follows its selected parent once, rather than
            // receiving the same group movement a second time through its frame.
            if picks.iter().any(|p| p.0 == bore.target) {
                continue;
            }
            let target = bodies
                .iter()
                .find(|b| b.id == bore.target)
                .ok_or("Attached profile target no longer exists.")?;
            let resolved = crate::operators::resolve_profile(body, target, bore)?;
            let moved = transformed(&resolved, &Geometry::default(), mode, None, matrix)?;
            let delta = crate::operators::attachment_delta(target, bore)?;
            let mut source = transformed(
                &moved,
                &Geometry::default(),
                mode,
                None,
                delta.invert().ok_or("Invalid support placement.")?,
            )?;
            source.bore = body.bore.clone();
            *body = source;
            continue;
        }
        let geometry = &scene
            .iter()
            .find(|(id, _)| *id == body.id)
            .ok_or("Selected body no longer exists.")?
            .1;
        let elements: Vec<_> = selected.iter().filter_map(|p| p.1).collect();
        *body = if selected.len() == 1 {
            transformed(body, geometry, mode, selected[0].1, matrix)?
        } else {
            transformed_many(body, geometry, mode, &elements, matrix)?
        };
    }
    Ok(result)
}

pub fn validate(edits: &[ComponentEdit]) -> Result<(), String> {
    if edits.len() > 100 {
        return Err("A body supports up to 100 component edits.".into());
    }
    let mut control_domain = false;
    for (index, edit) in edits.iter().enumerate() {
        if control_domain && edit.control.is_none() {
            return Err("Control domains currently require subsequent edits to use node/rigid-face tools; remove the control features before other component operators.".into());
        }
        control_domain |= edit.control.is_some();
        if let Some(control) = &edit.control {
            control.validate()?;
            if !edit.evaluated
                || edit.mode != SelectionMode::Face
                || !edit.elements.is_empty()
                || edit.retrim
                || edit.convex_rebuild
                || edit.section_transform
                || edit.cap_rotation
                || edit.sweep.is_some()
                || edit.face_edit.is_some()
                || edit.edge_chamfer.is_some()
                || edit.prism_chamfer
                || edit.prism_bevel.is_some()
                || edit.prism_curves
                || edit.tangent_walls
                || edit.prismatic_cut
                || Matrix4::from(edit.matrix) != Matrix4::identity()
            {
                return Err("Control edits cannot combine with other component operators.".into());
            }
        }
        let m = Matrix4::from(edit.matrix);
        if (edit.cap_rotation && (!edit.evaluated || edit.mode!=SelectionMode::Face || edit.retrim || edit.section_transform || edit.sweep.is_some() || edit.face_edit.is_some() || edit.edge_chamfer.is_some() || edit.prism_bevel.is_some()))
            || (edit.convex_rebuild && (!edit.evaluated || edit.mode != SelectionMode::Face || !edit.retrim && !matches!(edit.face_edit,Some(crate::face_operations::FaceEdit::Draft(_)))))
            || (edit.section_transform
            && (!edit.evaluated
                || !matches!(edit.mode, SelectionMode::Face | SelectionMode::Edge)
                || edit.retrim
                || edit.sweep.is_some()
                || edit.face_edit.is_some()))
            || (edit.tangent_walls && edit.sweep.is_none())
            || (edit.prismatic_cut && (!edit.tangent_walls || edit.sweep.is_none()))
            || edit.face_edit.is_some_and(|kind| {
                !edit.evaluated
                    || edit.mode != SelectionMode::Face
                    || edit.retrim
                    || edit.sweep.is_some()
                    || m != Matrix4::identity()
                    || matches!(kind,crate::face_operations::FaceEdit::Shell(operation) if operation.validate().is_err())
                    || matches!(kind,crate::face_operations::FaceEdit::Draft(operation) if operation.validate().is_err() || operation.neutral_plane.is_none() && (operation.neutral >= edit.topology.faces.len() || operation.neutral == edit.element || edit.elements.contains(&operation.neutral)))
            })
            || edit.edge_chamfer.is_some_and(|distances| {
                !edit.evaluated
                    || edit.mode != SelectionMode::Edge
                    || edit.retrim
                    || edit.section_transform
                    || edit.face_edit.is_some()
                    || edit.sweep.is_some()
                    || m != Matrix4::identity()
                    || distances
                        .iter()
                        .any(|d| !d.is_finite() || *d <= 1e-5 || *d > 10000.)
            })
            || (edit.prism_chamfer && edit.edge_chamfer.is_none())
            || (edit.prism_curves && !edit.prism_chamfer && edit.prism_bevel.is_none())
            || edit.prism_bevel.is_some_and(|bevel| {
                !edit.evaluated
                    || edit.mode != SelectionMode::Edge
                    || edit.edge_chamfer.is_some()
                    || edit.prism_chamfer
                    || edit.retrim
                    || edit.section_transform
                    || edit.sweep.is_some()
                    || edit.face_edit.is_some()
                    || m != Matrix4::identity()
                    || !bevel.amount.is_finite()
                    || bevel.amount <= 1e-5
                    || bevel.amount > 10000.
            })
            || edit.sweep.is_some_and(|v| {
                edit.mode != SelectionMode::Face
                    || !edit.evaluated
                    || m != Matrix4::identity()
                    || !edit.elements.is_empty()
                    || edit.retrim
                    || v.iter().any(|x| !x.is_finite() || x.abs() > 10000.)
                    || Vector3::from(v).magnitude2() < 1e-16
            })
            || (edit.retrim && edit.mode != SelectionMode::Face)
            || edit.mode == SelectionMode::Volume
            || edit.elements.len() > 8192
            || edit.topology.edges.len() > 8192
            || edit.topology.faces.len() > 8192
            || edit.topology.faces.iter().map(Vec::len).sum::<usize>() > 32768
            || edit
                .matrix
                .iter()
                .flatten()
                .any(|v| !v.is_finite() || v.abs() > 1e6)
            || [m.x.w, m.y.w, m.z.w].iter().any(|v| v.abs() > 1e-10)
            || (m.w.w - 1.).abs() > 1e-10
            || m.invert().is_none()
        {
            return Err(format!(
                "Component operation {} · {} {}: invalid transform or topology reference.",
                index + 1,
                edit.mode.label(),
                edit.element.saturating_add(1)
            ));
        }
    }
    Ok(())
}

pub fn has_source_edits(body: &Body) -> bool {
    body.component_edits.iter().any(|edit| !edit.evaluated)
}
pub fn apply(body: &Body, g: &mut Geometry) -> Result<(), String> {
    apply_stage(body, g, false, &[])
}
pub fn apply_evaluated(body: &Body, g: &mut Geometry, bodies: &[Body]) -> Result<(), String> {
    apply_stage(body, g, true, bodies)
}
fn apply_stage(
    body: &Body,
    g: &mut Geometry,
    evaluated: bool,
    bodies: &[Body],
) -> Result<(), String> {
    if !body
        .component_edits
        .iter()
        .any(|e| e.evaluated == evaluated)
    {
        return Ok(());
    }
    let placement = crate::regions::transform(body);
    map_geometry(g, placement.invert().ok_or("Invalid object transform.")?);
    weld_geometry(g);
    for (index, edit) in body
        .component_edits
        .iter()
        .enumerate()
        .filter(|(_, e)| e.evaluated == evaluated)
    {
        (|| {
        if edit.topology != g.topology {
            return Err("An upstream edit changed component topology. Undo it or remove component edits before changing topology.".into());
        }
        if let Some(control)=&edit.control {
            *g=crate::soft_selection::apply(body,g,edit,control,placement)?;
            return Ok(());
        }
        if let Some(bevel)=edit.prism_bevel {
            let edges=std::iter::once(edit.element).chain(edit.elements.iter().copied()).collect::<Vec<_>>();
            *g=crate::edge_treatments::bevel_prism(g,&edges,bevel,edit.prism_curves)?;
            return Ok(());
        }
        if let Some(distances)=edit.edge_chamfer {
            let edges=std::iter::once(edit.element).chain(edit.elements.iter().copied()).collect::<Vec<_>>();
            *g=if edit.prism_chamfer {
                crate::edge_treatments::chamfer_prism(g,&edges,distances,edit.prism_curves)?
            }else{crate::edge_treatments::chamfer_evaluated(g,&edges,distances)?};
            return Ok(());
        }
        if let Some(kind)=edit.face_edit {
            let faces=std::iter::once(edit.element).chain(edit.elements.iter().copied()).collect::<Vec<_>>();
            if faces.iter().any(|f|*f>=g.face_count) {return Err("Selected face no longer exists.".into());}
            *g=if let crate::face_operations::FaceEdit::Draft(operation)=kind {
                let reference=operation.neutral_plane.map(|id|crate::drafting::reference_plane(body,bodies,id)).transpose()?;
                crate::drafting::evaluated_with_plane(g,&faces,operation,reference,edit.convex_rebuild)
            }else{crate::face_operations::apply(g,&faces,kind)}.map_err(|e|format!("{}: {e}",kind.label()))?;
            return Ok(());
        }
        let mut selected = Vec::new();
        for element in std::iter::once(&edit.element).chain(&edit.elements) {
            let vertices = vertices(g, edit.mode, Some(*element));
            if vertices.is_empty() {
                return Err(format!("Missing {} {}.",edit.mode.label(),element.saturating_add(1)));
            }
            selected.extend(vertices);
        }
        selected.sort_unstable();
        selected.dedup();
        if selected.is_empty() {
            return Err("A component transform references a missing element.".into());
        }
        if let Some(vector)=edit.sweep {
            *g=crate::face_extrusions::evaluated_sweep(g,edit.element,vector.into(),body.shape==Shape::Circle,edit.tangent_walls,edit.prismatic_cut)?;
            return Ok(());
        }
        let matrix = Matrix4::from(edit.matrix);
        if edit.section_transform {
            let exact=crate::section_edits::translation(g,&selected,matrix)?.ok_or("The referenced complete cap or opposite section changed; reselect the cap.")?;
            map_geometry(g,exact);
            return Ok(());
        }
        if edit.retrim {
            let faces=std::iter::once(edit.element).chain(edit.elements.iter().copied()).collect::<Vec<_>>();
            *g=retrim_faces(g,&faces,matrix,edit.convex_rebuild)?;
            return Ok(());
        }
        if edit.cap_rotation {
            *g=crate::section_edits::rotate_cap(g,&selected,matrix)?.ok_or("This curved-face rotation needs retained cap/neighbor surface reconstruction. Use an editable control domain; the display mesh was not modified.")?;
            return Ok(());
        }
        // A continuous cylinder wall can own every boundary vertex. This is
        // a rigid transform of the entire retained shape, not mesh deformation.
        if selected.len()==g.points.len() {
            map_geometry(g,matrix);
            return Ok(());
        }
        let old = g.clone();
        let weight = |i: usize| if selected.contains(&i) { 1. } else { 0. };
        let mut lengths = vec![0.; g.edge_count];
        for (edge, group) in old.edges.iter().zip(&old.edge_groups) {
            lengths[*group] += (edge[1] - edge[0]).magnitude();
        }
        let mut traveled = vec![0.; g.edge_count];
        let mut edge_weights = Vec::new();
        for (edge, group) in old.edges.iter().zip(&old.edge_groups) {
            let [a, b] = old.topology.edges[*group];
            let length = (edge[1] - edge[0]).magnitude();
            let fraction = |distance: f64| {
                if lengths[*group] > 1e-12 {
                    distance / lengths[*group]
                } else {
                    0.
                }
            };
            let blend = |t: f64| weight(a) * (1. - t) + weight(b) * t;
            edge_weights.push([
                blend(fraction(traveled[*group])),
                blend(fraction(traveled[*group] + length)),
            ]);
            traveled[*group] += length;
        }
        let moved = |p: Point3, w: f64| p + (matrix.transform_point(p) - p) * w;
        for (i, p) in g.points.iter_mut().enumerate() {
            *p = moved(*p, weight(i));
        }
        for (edge, weights) in g.edges.iter_mut().zip(&edge_weights) {
            for i in 0..2 {
                edge[i] = moved(edge[i], weights[i]);
            }
        }
        for (triangle, face) in g.triangles.iter_mut().zip(&old.triangle_faces) {
            for p in triangle {
                let mut sum = 0.;
                let mut influence = 0.;
                let mut boundary = None;
                for ((edge, group), weights) in
                    old.edges.iter().zip(&old.edge_groups).zip(&edge_weights)
                {
                    if !old.topology.faces[*face].contains(group) {
                        continue;
                    }
                    let ab = edge[1] - edge[0];
                    let t = ((*p - edge[0]).dot(ab) / ab.magnitude2().max(1e-20)).clamp(0., 1.);
                    let distance = (*p - (edge[0] + ab * t)).magnitude2();
                    let w = weights[0] * (1. - t) + weights[1] * t;
                    if distance < 1e-16 {
                        boundary = Some(w);
                        break;
                    }
                    sum += 1. / distance;
                    influence += w / distance;
                }
                *p = moved(
                    *p,
                    boundary.unwrap_or_else(|| if sum > 0. { influence / sum } else { 0. }),
                );
            }
        }
        weld_geometry(g);
        validate_boundary(g, if evaluated {old.is_solid} else {body.extrusion > 0. && body.shape != Shape::Line})?;
        g.shell=None;g.circles.clear();g.triangle_normals.clear();
        Ok(())
        })().map_err(|reason:String|format!("Component operation {} · {} {}: {reason}",index+1,edit.mode.label(),edit.element.saturating_add(1)))?;
    }
    map_geometry(g, placement);
    Ok(())
}
// Extract planar supports without approximating curved/deformed faces.
fn face_supports(g: &Geometry) -> Result<Vec<(Vector3, f64)>, String> {
    if !g.is_solid || g.face_count > 256 || g.face_count < 4 {
        return Err("Exact face retrim requires a planar solid (4–256 faces).".into());
    }
    let mut planes = Vec::new();
    for face in 0..g.face_count {
        let triangles = g
            .triangles
            .iter()
            .zip(&g.triangle_faces)
            .filter(|(_, f)| **f == face)
            .map(|(t, _)| t)
            .collect::<Vec<_>>();
        let t = triangles.first().ok_or("Missing planar face.")?;
        let n = (t[1] - t[0]).cross(t[2] - t[0]).normalize();
        let d = n.dot(t[0].to_vec());
        if triangles
            .iter()
            .flat_map(|t| t.iter())
            .any(|p| (n.dot(p.to_vec()) - d).abs() > 1e-6)
        {
            return Err("Exact face retrim requires planar faces.".into());
        }
        planes.push((n, d));
    }
    Ok(planes)
}
pub(crate) fn convex_supports(g: &Geometry) -> Result<Vec<(Vector3, f64)>, String> {
    let planes = face_supports(g)?;
    if planes
        .iter()
        .any(|(n, d)| g.points.iter().any(|p| n.dot(p.to_vec()) > d + 1e-6))
    {
        return Err("Operation requires a convex planar solid.".into());
    }
    Ok(planes)
}
pub(crate) fn planar_supports(g: &Geometry) -> Result<Vec<(Vector3, f64)>, String> {
    convex_supports(g)?;
    straight_planar_supports(g)
}
pub(crate) fn straight_planar_supports(g: &Geometry) -> Result<Vec<(Vector3, f64)>, String> {
    let planes = face_supports(g)?;
    for (segment, group) in g.edges.iter().zip(&g.edge_groups) {
        let [a, b] = g.topology.edges[*group];
        let ab = g.points[b] - g.points[a];
        if ab.magnitude2() < 1e-16
            || segment
                .iter()
                .any(|p| (*p - g.points[a]).cross(ab).magnitude() > 1e-6 * ab.magnitude())
        {
            return Err("Exact planar face retrim requires straight boundary edges.".into());
        }
    }
    Ok(planes)
}
pub(crate) fn retrim_faces(
    g: &Geometry,
    faces: &[usize],
    matrix: Matrix4,
    convex_rebuild: bool,
) -> Result<Geometry, String> {
    let mut planes = straight_planar_supports(g)?;
    for face in faces {
        let t = g
            .triangles
            .iter()
            .zip(&g.triangle_faces)
            .find(|(_, f)| **f == *face)
            .ok_or("Missing face reference.")?
            .0;
        let [a, b, c] = t.map(|p| matrix.transform_point(p));
        let n = (b - a).cross(c - a).normalize();
        planes[*face] = (n, n.dot(a.to_vec()));
    }
    match retrim_planes(g, &planes) {
        Ok(result) => Ok(result),
        Err(error) if convex_rebuild && planes.len() <= 64 && convex_supports(g).is_ok() => {
            let indexed = planes.into_iter().enumerate().collect::<Vec<_>>();
            let result = crate::edge_treatments::convex_from_planes(&indexed)
                .and_then(crate::edge_treatments::planar_regions)
                .map_err(|reason| format!("{error} Convex rebuild: {reason}"))?;
            crate::operators::geometry(&crate::operators::solid(&result)?)
        }
        Err(error) => Err(error),
    }
}

/// Rebuild shared vertices and ordered outer/inner trims from replacement supports.
/// Callers derive these planes from the existing straight planar input.
pub(crate) fn retrim_planes(g: &Geometry, planes: &[(Vector3, f64)]) -> Result<Geometry, String> {
    if planes.len() != g.face_count {
        return Err("Missing replacement support planes.".into());
    }
    let convex = convex_supports(g).is_ok();
    let mut points = Vec::new();
    for vertex in 0..g.points.len() {
        let incident = (0..g.face_count)
            .filter(|face| {
                g.topology.faces[*face]
                    .iter()
                    .any(|e| g.topology.edges[*e].contains(&vertex))
            })
            .collect::<Vec<_>>();
        let mut solution = None;
        let mut determinant = 1e-10;
        for (i, &a) in incident.iter().enumerate() {
            for (j, &b) in incident.iter().enumerate().skip(i + 1) {
                for &c in incident.iter().skip(j + 1) {
                    let (na, da) = planes[a];
                    let (nb, db) = planes[b];
                    let (nc, dc) = planes[c];
                    let det = na.dot(nb.cross(nc));
                    if det.abs() > determinant {
                        determinant = det.abs();
                        solution = Some(Point3::from_vec(
                            (nb.cross(nc) * da + nc.cross(na) * db + na.cross(nb) * dc) / det,
                        ));
                    }
                }
            }
        }
        // A sweep can retain a seam vertex in the middle of a straight edge.
        // Such a vertex has two independent supports; project onto their exact
        // intersection instead of requiring a third, nonexistent corner plane.
        if solution.is_none()
            && let Some(&first) = incident.first()
        {
            let (na, da) = planes[first];
            let original = g.points[vertex];
            solution = Some(original + na * (da - na.dot(original.to_vec())));
            for &other in &incident {
                let (nb, db) = planes[other];
                let cosine = na.dot(nb);
                let determinant = 1. - cosine * cosine;
                if determinant > 1e-10 {
                    let a = da - na.dot(original.to_vec());
                    let b = db - nb.dot(original.to_vec());
                    solution = Some(
                        original + (na * (a - cosine * b) + nb * (b - cosine * a)) / determinant,
                    );
                    break;
                }
            }
        }
        let p = solution.ok_or("Face retrim references a missing corner.")?;
        if incident
            .iter()
            .any(|f| (planes[*f].0.dot(p.to_vec()) - planes[*f].1).abs() > 1e-6)
            || (convex && planes.iter().any(|(n, d)| n.dot(p.to_vec()) > *d + 1e-6))
        {
            return Err("Face retrim consumes a face or changes corner connectivity; reduce the movement/rotation.".into());
        }
        points.push(p);
    }
    let vertices = points
        .iter()
        .copied()
        .map(builder::vertex)
        .collect::<Vec<_>>();
    let edges = g
        .topology
        .edges
        .iter()
        .map(|[a, b]| {
            if (points[*a] - points[*b]).magnitude2() < 1e-16 {
                return Err("Face retrim collapses an edge.");
            }
            Ok(builder::line(&vertices[*a], &vertices[*b]))
        })
        .collect::<Result<Vec<_>, _>>()?;
    let mut shell = Shell::default();
    if !convex {
        // Preserve concave outlines and hole winding from the retained BRep.
        // Sorting all vertices around a centroid loses both of these.
        let source = g
            .shell
            .as_ref()
            .ok_or("Concave face retrim requires a retained boundary.")?;
        let roles = g.surface_mapping()?;
        if source.len() != g.face_count
            || roles
                .iter()
                .copied()
                .collect::<std::collections::HashSet<_>>()
                .len()
                != g.face_count
        {
            return Err(
                "Concave face retrim requires one planar support per retained face.".into(),
            );
        }
        for face in 0..g.face_count {
            let source = &source[roles
                .iter()
                .position(|role| *role == face)
                .ok_or("Missing support face.")?];
            let wires = source
                .boundaries()
                .iter()
                .map(|wire| {
                    wire.iter()
                        .map(|old| {
                            let vertex = |p: Point3| {
                                g.points
                                    .iter()
                                    .position(|v| (*v - p).magnitude() < 1e-7)
                                    .ok_or("Missing boundary vertex.")
                            };
                            let a = vertex(old.front().point())?;
                            let b = vertex(old.back().point())?;
                            let index = g.topology.faces[face]
                                .iter()
                                .find(|e| {
                                    let pair = g.topology.edges[**e];
                                    pair == [a, b] || pair == [b, a]
                                })
                                .ok_or("Missing boundary edge.")?;
                            Ok(if g.topology.edges[*index] == [a, b] {
                                edges[*index].clone()
                            } else {
                                edges[*index].inverse()
                            })
                        })
                        .collect::<Result<Wire, &str>>()
                })
                .collect::<Result<Vec<_>, _>>()?;
            shell.push(
                builder::try_attach_plane(&wires)
                    .map_err(|e| format!("Cannot retrim concave face: {e}"))?,
            );
        }
    } else {
        for (face, (normal, _)) in planes.iter().enumerate() {
            let mut indices = crate::components::vertices(g, SelectionMode::Face, Some(face));
            let center =
                indices.iter().map(|i| points[*i].to_vec()).sum::<Vector3>() / indices.len() as f64;
            let u = (points[indices[0]].to_vec() - center).normalize();
            let v = normal.cross(u);
            indices.sort_by(|a, b| {
                let a = points[*a].to_vec() - center;
                let b = points[*b].to_vec() - center;
                a.dot(v)
                    .atan2(a.dot(u))
                    .total_cmp(&b.dot(v).atan2(b.dot(u)))
            });
            let wire: Wire = (0..indices.len())
                .map(|i| {
                    let a = indices[i];
                    let b = indices[(i + 1) % indices.len()];
                    let edge = g.topology.faces[face]
                        .iter()
                        .find(|e| {
                            let pair = g.topology.edges[**e];
                            pair == [a, b] || pair == [b, a]
                        })
                        .ok_or("Face retrim changes boundary connectivity.")?;
                    Ok(if g.topology.edges[*edge] == [a, b] {
                        edges[*edge].clone()
                    } else {
                        edges[*edge].inverse()
                    })
                })
                .collect::<Result<Wire, &str>>()?;
            shell.push(
                builder::try_attach_plane(&[wire])
                    .map_err(|e| format!("Cannot retrim planar face: {e}"))?,
            );
        }
    }
    solid_parts(&shell)?;
    let mut result = crate::model::tessellate(&shell)?;
    // Keep guarded logical references stable; the retained BRep face order matches.
    result.points = points;
    result.topology = g.topology.clone();
    result.edge_count = g.edge_count;
    result.edges = result
        .topology
        .edges
        .iter()
        .map(|[a, b]| [result.points[*a], result.points[*b]])
        .collect();
    result.edge_groups = (0..result.edge_count).collect();
    result.is_solid = true;
    if !convex {
        for (triangle, face) in result.triangles.iter().zip(&result.triangle_faces) {
            if (triangle[1] - triangle[0])
                .cross(triangle[2] - triangle[0])
                .dot(planes[*face].0)
                <= 0.
            {
                return Err(
                    "Face retrim reverses a boundary; reduce the movement/rotation.".into(),
                );
            }
        }
        validate_retrim_intersections(&result)?;
    }
    Ok(result)
}

// Connectivity alone cannot detect a concave wall crossing another wall.
// Check straight boundaries and their intersections with triangulated planar faces.
pub(crate) fn validate_retrim_intersections(g: &Geometry) -> Result<(), String> {
    const EPS: f64 = 1e-7;
    let invalid = "Face retrim crosses or touches another boundary; reduce the movement/rotation.";
    if g.edges.len().saturating_mul(g.triangles.len()) > 2_000_000 {
        return Err("Planar retrim exceeds its boundary-intersection budget.".into());
    }
    for (i, a) in g.points.iter().enumerate() {
        if g.points
            .iter()
            .skip(i + 1)
            .any(|b| (*a - *b).magnitude() < EPS)
        {
            return Err(invalid.into());
        }
    }
    for (i, [a, b]) in g.topology.edges.iter().copied().enumerate() {
        let p = g.points[a];
        let ab = g.points[b] - p;
        for [c, d] in g.topology.edges.iter().copied().skip(i + 1) {
            let q = g.points[c];
            let cd = g.points[d] - q;
            let cross = ab.cross(cd);
            let det = cross.magnitude2();
            let shared = [a, b].iter().any(|v| [c, d].contains(v));
            if det > 1e-20 * ab.magnitude2() * cd.magnitude2() {
                let t = (q - p).cross(cd).dot(cross) / det;
                let u = (q - p).cross(ab).dot(cross) / det;
                if (-EPS..=1. + EPS).contains(&t)
                    && (-EPS..=1. + EPS).contains(&u)
                    && (p + ab * t - q - cd * u).magnitude() < EPS
                    && !shared
                {
                    return Err(invalid.into());
                }
            } else if (q - p).cross(ab).magnitude() < EPS * ab.magnitude() {
                let first = (q - p).dot(ab) / ab.magnitude2();
                let last = (q + cd - p).dot(ab) / ab.magnitude2();
                let overlap = first.max(last).min(1.) - first.min(last).max(0.);
                if overlap * ab.magnitude() > EPS || (overlap >= -EPS && !shared) {
                    return Err(invalid.into());
                }
            }
        }
        for (triangle, face) in g.triangles.iter().zip(&g.triangle_faces) {
            if g.topology.faces[*face].contains(&i) {
                continue;
            }
            let [x, y, z] = *triangle;
            let u = y - x;
            let v = z - x;
            let normal = u.cross(v);
            let normal_length = normal.magnitude();
            if normal_length < 1e-16 {
                continue;
            }
            let denominator = ab.dot(normal);
            let inside = |point: Point3, strict: bool| {
                let delta = point - x;
                let alpha = delta.cross(v).dot(normal) / normal.magnitude2();
                let beta = u.cross(delta).dot(normal) / normal.magnitude2();
                let margin = if strict { 1e-8 } else { -1e-8 };
                alpha > margin && beta > margin && alpha + beta < 1. - margin
            };
            if denominator.abs() > 1e-12 * normal_length * ab.magnitude() {
                let t = (x - p).dot(normal) / denominator;
                if (-EPS..=1. + EPS).contains(&t) {
                    let hit = p + ab * t;
                    let shared = g.topology.faces[*face].iter().any(|edge| {
                        g.topology.edges[*edge]
                            .iter()
                            .any(|vertex| (g.points[*vertex] - hit).magnitude() < EPS)
                    });
                    if !shared && inside(hit, false) {
                        return Err(invalid.into());
                    }
                }
            } else if (p - x).dot(normal).abs() < EPS * normal_length
                && [p, p + ab * 0.5, p + ab]
                    .iter()
                    .any(|point| inside(*point, true))
            {
                return Err(invalid.into());
            }
        }
    }
    Ok(())
}

pub fn map_geometry(g: &mut Geometry, matrix: Matrix4) {
    use truck_modeling::Matrix;
    if let Some(inverse) = matrix.invert() {
        let normal_matrix = inverse.transpose();
        for n in g.triangle_normals.iter_mut().flatten() {
            *n = normal_matrix.transform_vector(*n).normalize();
        }
    } else {
        g.triangle_normals.clear();
    }
    if let Some(shell) = &mut g.shell {
        *shell = builder::transformed(shell, matrix);
    }
    for circle in &mut g.circles {
        circle.center = matrix.transform_point(circle.center);
        circle.u = matrix.transform_vector(circle.u);
        circle.v = matrix.transform_vector(circle.v);
    }

    for p in g
        .points
        .iter_mut()
        .chain(g.edges.iter_mut().flatten())
        .chain(g.triangles.iter_mut().flatten())
    {
        *p = matrix.transform_point(*p);
    }
}

// Truck tessellates neighboring analytic surfaces independently. Reconcile tiny local-space
// projection differences against shared boundary samples before and after deformation.
pub fn weld_geometry(g: &mut Geometry) {
    // Two independently projected surfaces may each differ from their shared
    // kernel boundary by TOLERANCE, so their mutual separation can be twice that.
    let tolerance = 2. * truck_modeling::TOLERANCE;
    let mut buckets: HashMap<[i64; 3], Vec<Point3>> = HashMap::new();
    for p in g
        .points
        .iter_mut()
        .chain(g.edges.iter_mut().flatten())
        .chain(g.triangles.iter_mut().flatten())
    {
        let key = [p.x, p.y, p.z].map(|v| (v / tolerance).floor() as i64);
        let mut matched = None;
        'search: for x in -1..=1 {
            for y in -1..=1 {
                for z in -1..=1 {
                    if let Some(points) = buckets.get(&[key[0] + x, key[1] + y, key[2] + z]) {
                        for candidate in points {
                            if (*p - *candidate).magnitude2() < tolerance * tolerance {
                                matched = Some(*candidate);
                                break 'search;
                            }
                        }
                    }
                }
            }
        }
        if let Some(point) = matched {
            *p = point;
        } else {
            buckets.entry(key).or_default().push(*p);
        }
    }
}

fn validate_boundary(g: &Geometry, closed: bool) -> Result<(), String> {
    boundary_shell(g, closed).map(|_| ())
}

// STEP represents enclosed cavities as BREP_WITH_VOIDS, while disconnected
// positive-volume islands remain independent solids. Do not export a void as material.
pub(crate) fn solid_parts(shell: &truck_modeling::Shell) -> Result<Vec<Solid>, String> {
    let parts = shell.connected_components();
    let meshes = parts
        .iter()
        .map(crate::model::tessellate)
        .collect::<Result<Vec<_>, _>>()?;
    let volumes: Vec<f64> = meshes
        .iter()
        .map(|g| {
            let origin = g.triangles[0][0];
            g.triangles
                .iter()
                .map(|t| {
                    let [a, b, c] = t.map(|p| p - origin);
                    a.dot(b.cross(c)) / 6.
                })
                .sum()
        })
        .collect();
    let mut groups: Vec<_> = volumes
        .iter()
        .enumerate()
        .filter(|(_, v)| **v > 0.)
        .map(|(i, _)| (i, vec![parts[i].clone()]))
        .collect();
    for (index, volume) in volumes.iter().enumerate() {
        if volume.abs() <= 1e-10 || !volume.is_finite() {
            return Err("Cannot export a zero-volume solid boundary.".into());
        }
        if *volume > 0. {
            continue;
        }
        let point = meshes[index].triangles[0][0];
        let owner = groups
            .iter()
            .enumerate()
            .filter(|(_, (i, _))| {
                let winding: f64 = meshes[*i]
                    .triangles
                    .iter()
                    .map(|t| {
                        let [a, b, c] = t.map(|p| p - point);
                        2. * a.dot(b.cross(c)).atan2(
                            a.magnitude() * b.magnitude() * c.magnitude()
                                + a.dot(b) * c.magnitude()
                                + b.dot(c) * a.magnitude()
                                + c.dot(a) * b.magnitude(),
                        )
                    })
                    .sum();
                winding.abs() > std::f64::consts::TAU
            })
            .min_by(|(_, (a, _)), (_, (b, _))| volumes[*a].total_cmp(&volumes[*b]))
            .map(|(i, _)| i)
            .ok_or("An inward solid boundary has no enclosing outer boundary.")?;
        groups[owner].1.push(parts[index].clone());
    }
    groups
        .into_iter()
        .map(|(_, boundaries)| Solid::try_new(boundaries).map_err(|e| e.to_string()))
        .collect()
}

pub fn boundary_shell(g: &Geometry, closed: bool) -> Result<Shell, String> {
    if g.points
        .iter()
        .chain(g.edges.iter().flatten())
        .chain(g.triangles.iter().flatten())
        .any(|p| {
            [p.x, p.y, p.z]
                .iter()
                .any(|v| !v.is_finite() || v.abs() > 1e6)
        })
    {
        return Err("Component transform exceeds supported coordinates.".into());
    }
    if g.edges.iter().any(|e| (e[1] - e[0]).magnitude2() < 1e-16) {
        return Err("Component transform collapses an edge.".into());
    }
    let mut vertices: HashMap<[i64; 3], Vertex> = HashMap::new();
    let mut edges: HashMap<([i64; 3], [i64; 3]), Edge> = HashMap::new();
    let mut shell = Shell::default();
    for t in &g.triangles {
        if (t[1] - t[0]).cross(t[2] - t[0]).magnitude2() < 1e-20 {
            return Err("Component transform collapses a face.".into());
        }
        let keys = t.map(|p| [p.x, p.y, p.z].map(|v| (v * 1e7).round() as i64));
        for i in 0..3 {
            vertices
                .entry(keys[i])
                .or_insert_with(|| builder::vertex(t[i]));
        }
        let wire: Wire = (0..3)
            .map(|i| {
                let a = keys[i];
                let b = keys[(i + 1) % 3];
                let key = if a < b { (a, b) } else { (b, a) };
                let edge = edges
                    .entry(key)
                    .or_insert_with(|| builder::line(&vertices[&key.0], &vertices[&key.1]));
                if a < b { edge.clone() } else { edge.inverse() }
            })
            .collect();
        shell.push(
            builder::try_attach_plane(&[wire])
                .map_err(|e| format!("Cannot rebuild adjacent faces: {e}"))?,
        );
    }
    if !shell.is_geometric_consistent() {
        return Err("Component transform creates inconsistent adjacent faces.".into());
    }
    if closed && !shell.is_empty() {
        solid_parts(&shell)
            .map_err(|e| format!("Component transform would invalidate the body: {e}"))?;
    }
    Ok(shell)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        model::{Model, evaluate, evaluate_scene},
        project::Project,
        viewport::Camera,
    };
    use truck_modeling::{Deg, Vector3};
    #[test]
    fn evaluated_pipe_edges_move_after_boring() {
        use crate::{
            edge_treatments::FaceTarget,
            model::evaluate_scene,
            operators::{Bore, BoreMode},
        };
        let mut base = Body::new(Shape::Circle);
        base.id = 1;
        base.width = 4.;
        base.extrusion = 2.;
        let mut cutter = Body::new(Shape::Circle);
        cutter.id = 2;
        cutter.width = 2.;
        cutter.position = [0., 0., 2.];
        let mut bore = Bore::new(&base, FaceTarget::Top).unwrap();
        bore.mode = BoreMode::Borer;
        bore.travel = -2.;
        cutter.bore = Some(bore);
        let bodies = vec![base, cutter];
        let scene = evaluate_scene(&bodies).unwrap();
        let g = &scene[0].1;
        let edges: Vec<_> = g
            .topology
            .edges
            .iter()
            .enumerate()
            .filter(|(_, e)| {
                let [a, b] = e.map(|i| g.points[i]);
                (a.x.hypot(a.y) - 1.).abs() < 1e-6
                    && (b.x.hypot(b.y) - 1.).abs() < 1e-6
                    && (a.z - 2.).abs() < 1e-6
                    && (b.z - 2.).abs() < 1e-6
            })
            .take(2)
            .map(|(i, _)| (1, Some(i)))
            .collect();
        assert_eq!(edges.len(), 2);
        let matrix = Matrix4::from_translation(Vector3::new(0.2, 0., 0.));
        let changed =
            transform_selection(&bodies, &scene, SelectionMode::Edge, &edges, matrix).unwrap();
        let moved =
            evaluate_scene(&changed).expect("Selected edges of the evaluated pipe must move");
        let selected: Vec<_> = edges
            .iter()
            .flat_map(|p| vertices(g, SelectionMode::Edge, p.1))
            .collect();
        for i in selected {
            assert!(
                (moved[0].1.points[i] - matrix.transform_point(g.points[i])).magnitude() < 1e-6
            );
        }
        validate_boundary(&moved[0].1, true).unwrap();
        let mut model = Model::from_bodies(bodies.clone());
        model.commit_changes(&changed).unwrap();
        model.undo();
        assert_eq!(model.bodies, bodies);
        model.redo();
        assert_eq!(model.bodies, changed);
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("moved-pipe.osk");
        Project::new(changed.clone(), Camera::default(), true, Some(1))
            .save(&path)
            .unwrap();
        assert_eq!(Project::load(&path).unwrap().bodies, changed);
        let mut stale = changed[0].clone();
        stale.component_edits[0].topology.edges.pop();
        assert!(model.edit(stale).is_err());
        assert_eq!(model.bodies, changed);

        // The same shared edit path must run after an explicit Boolean result.
        let mut a = Body::new(Shape::Rectangle);
        a.id = 1;
        a.extrusion = 2.;
        let mut b = a.clone();
        b.id = 2;
        b.position[0] = 1.;
        let mut sources = vec![a, b];
        let scene = evaluate_scene(&sources).unwrap();
        let mut result = Body::new(Shape::Rectangle);
        result.id = 3;
        result.boolean = Some(crate::booleans::Boolean::new(
            crate::booleans::Kind::Subtract,
            &[(1, None), (2, None)],
            SelectionMode::Volume,
            &scene,
        ));
        sources.push(result);
        let scene = evaluate_scene(&sources).unwrap();
        let changed = transform_selection(
            &sources,
            &scene,
            SelectionMode::Edge,
            &[(3, Some(0))],
            matrix,
        )
        .unwrap();
        let out = evaluate_scene(&changed).unwrap();
        for i in vertices(&scene[2].1, SelectionMode::Edge, Some(0)) {
            assert!(
                (out[2].1.points[i] - matrix.transform_point(scene[2].1.points[i])).magnitude()
                    < 1e-6
            );
        }
        validate_boundary(&out[2].1, true).unwrap();
        // Old .osk edits omit the new stage flag and keep their original replay order.
        let mut legacy = changed[2].component_edits[0].clone();
        legacy.evaluated = false;
        let mut json = serde_json::to_value(&legacy).unwrap();
        json.as_object_mut().unwrap().remove("evaluated");
        assert_eq!(
            serde_json::from_value::<ComponentEdit>(json).unwrap(),
            legacy
        );
    }
    #[test]
    fn cylindrical_offset_changes_radius_and_preserves_axis() {
        let mut body = Body::new(Shape::Circle);
        body.id = 1;
        body.width = 4.;
        body.extrusion = 3.;
        body.position = [2., 3., 4.];
        body.rotation = [15., 25., 35.];
        body.scale = [2., 2., 0.5];
        body.edge_treatments
            .push(crate::edge_treatments::Modifier::new(
                crate::edge_treatments::Kind::Bevel,
                crate::edge_treatments::FaceTarget::Top,
                &body,
            ));
        let g = evaluate(&body).unwrap();
        assert!(
            (0..g.face_count)
                .any(|f| offset_target(&body, &g, f) == Ok(OffsetTarget::CylinderWall))
        );
        for distance in [-0.25, 0.25] {
            let changed = offset_source(&body, OffsetTarget::CylinderWall, distance).unwrap();
            assert_eq!(changed.position, body.position);
            assert_eq!(changed.rotation, body.rotation);
            assert_eq!(changed.extrusion, body.extrusion);
            assert_eq!(changed.edge_treatments, body.edge_treatments);
            let out = evaluate(&changed).unwrap();
            assert!(
                (out.circles[0].u.magnitude() - g.circles[0].u.magnitude() - distance).abs() < 1e-7
            );
            assert!(out.shell.is_some());
        }
        assert!(offset_source(&body, OffsetTarget::CylinderWall, -5.).is_err());
        body.scale[1] = 1.;
        assert!(offset_source(&body, OffsetTarget::CylinderWall, 0.25).is_err());
    }
    #[test]
    fn source_face_offsets_preserve_opposite_planes_and_analytic_geometry() {
        use crate::edge_treatments::{FaceTarget, Kind, Modifier};
        for shape in [Shape::Rectangle, Shape::Circle, Shape::Semicircle] {
            let mut body = Body::new(shape);
            body.id = 1;
            body.width = 4.;
            body.depth = 3.;
            body.extrusion = 2.;
            body.position = [3., -2., 4.];
            body.rotation = [17., 31., 43.];
            body.scale = [1.5, 0.75, 2.];
            body.edge_treatments
                .push(Modifier::new(Kind::Bevel, FaceTarget::Top, &body));
            let source = evaluate(&body).unwrap();
            for face in [
                FaceTarget::Top,
                FaceTarget::Bottom,
                FaceTarget::Left,
                FaceTarget::Right,
                FaceTarget::Front,
                FaceTarget::Back,
            ] {
                if !face.supported(&body) {
                    continue;
                }
                let (normal, _) = face.plane(&body);
                let normal = crate::regions::transform(&body)
                    .transform_vector(normal)
                    .normalize();
                let limits = |g: &Geometry| {
                    g.triangles
                        .iter()
                        .flatten()
                        .map(|p| p.to_vec().dot(normal))
                        .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), d| {
                            (min.min(d), max.max(d))
                        })
                };
                let original = limits(&source);
                for distance in [-0.3, 0.3] {
                    let changed = offset_source_face(&body, face, distance).unwrap();
                    assert!(changed.component_edits.is_empty());
                    assert_eq!(changed.edge_treatments, body.edge_treatments);
                    let g = evaluate(&changed).unwrap();
                    assert!(g.shell.is_some());
                    let mut closed = g.clone();
                    weld_geometry(&mut closed);
                    boundary_shell(&closed, true)
                        .unwrap_or_else(|e| panic!("{shape:?} {face:?} {distance}: {e}"));
                    let bounds = limits(&g);
                    assert!(
                        (bounds.0 - original.0).abs() < 1e-6,
                        "{shape:?} {face:?}: opposite support moved"
                    );
                    assert!(
                        (bounds.1 - original.1 - distance).abs() < 1e-6,
                        "{shape:?} {face:?}: incorrect world distance"
                    );
                    assert!(
                        (0..g.face_count)
                            .any(|f| crate::edge_treatments::target(&changed, &g, f) == Ok(face))
                    );
                    if shape == Shape::Circle {
                        assert_eq!(changed.width, body.width);
                        assert_eq!(g.circles.len(), source.circles.len());
                        for (a, b) in g.circles.iter().zip(&source.circles) {
                            assert!((a.u - b.u).magnitude() < 1e-8);
                            assert!((a.v - b.v).magnitude() < 1e-8);
                        }
                    }
                }
            }
            let mut model = Model::from_bodies(vec![body.clone()]);
            for d in [0.1, 0.2] {
                model
                    .edit(offset_source_face(&body, FaceTarget::Top, d).unwrap())
                    .unwrap();
            }
            model.finish_edit();
            let edited = model.bodies.clone();
            let dir = tempfile::tempdir().unwrap();
            let path = dir.path().join("offset.osk");
            Project::new(edited.clone(), Camera::default(), true, Some(1))
                .save(&path)
                .unwrap();
            assert_eq!(Project::load(&path).unwrap().bodies, edited);
            model.undo();
            assert_eq!(model.bodies, vec![body.clone()]);
            assert!(!model.can_undo(), "one continuous edit is one undo step");
            model.redo();
            assert_eq!(model.bodies, edited);
            for distance in [f64::NAN, f64::INFINITY, -4., -5.] {
                assert!(offset_source_face(&body, FaceTarget::Top, distance).is_err());
                assert_eq!(model.bodies, edited);
            }
            // A valid primitive resize can still invalidate a treatment; commit
            // must reject it without consuming history or replacing geometry.
            let mut oversized = body.clone();
            oversized.edge_treatments[0].amount = 0.9;
            assert!(offset_source_face(&oversized, FaceTarget::Top, -1.).is_err());
            let deformed = transformed(
                &body,
                &source,
                SelectionMode::Point,
                Some(0),
                Matrix4::from_translation(Vector3::unit_z() * 0.1),
            )
            .unwrap();
            assert!(offset_source_face(&deformed, FaceTarget::Top, 0.1).is_err());
        }
    }
    #[test]
    fn all_selection_modes_transform_with_history_persistence_and_shared_boundaries() {
        for shape in [
            Shape::Rectangle,
            Shape::Circle,
            Shape::Semicircle,
            Shape::Line,
        ] {
            let mut body = Body::new(shape);
            body.id = 1;
            body.extrusion = 2.;
            let g = evaluate(&body).unwrap();
            for mode in [
                SelectionMode::Volume,
                SelectionMode::Face,
                SelectionMode::Edge,
                SelectionMode::Point,
            ] {
                let element = if mode == SelectionMode::Volume {
                    None
                } else {
                    Some(0)
                };
                for rotate in [false, true] {
                    let center = pivot(&g, mode, element).unwrap() + Vector3::new(0.3, 0.2, 0.1);
                    let matrix = if rotate {
                        Matrix4::from_translation(center.to_vec())
                            * Matrix4::from_angle_z(Deg(8.))
                            * Matrix4::from_translation(-center.to_vec())
                    } else {
                        Matrix4::from_translation(Vector3::new(0.15, 0.1, 0.2))
                    };
                    let edited = transformed(&body, &g, mode, element, matrix).unwrap();
                    let mut model = Model::from_bodies(vec![body.clone()]);
                    model
                        .commit(edited.clone())
                        .unwrap_or_else(|e| panic!("{shape:?} {mode:?} rotate={rotate}: {e}"));
                    let output = evaluate_scene(&model.bodies).unwrap();
                    let selected = match (mode, element) {
                        (SelectionMode::Face, Some(face)) => g.face_elements(face),
                        (SelectionMode::Edge, Some(edge)) => g.edge_elements(edge),
                        _ => element.into_iter().collect(),
                    };
                    let chosen = if mode == SelectionMode::Volume {
                        (0..g.points.len()).collect::<Vec<_>>()
                    } else {
                        selected
                            .into_iter()
                            .flat_map(|element| vertices(&g, mode, Some(element)))
                            .collect()
                    };
                    for i in 0..g.points.len() {
                        if edited.component_edits.last().is_some_and(|e| e.retrim) {
                            let face = element.unwrap();
                            let support = crate::snapping::Plane::face(&g, face).unwrap();
                            let origin = matrix.transform_point(support.origin);
                            let normal = matrix.transform_vector(support.normal).normalize();
                            if chosen.contains(&i) {
                                assert!((output[0].1.points[i] - origin).dot(normal).abs() < 1e-6);
                            }
                            continue;
                        }
                        let expected = if chosen.contains(&i) {
                            matrix.transform_point(g.points[i])
                        } else {
                            g.points[i]
                        };
                        assert!(
                            (output[0].1.points[i] - expected).magnitude() < 1e-6,
                            "{shape:?} {mode:?}"
                        );
                    }
                    assert_eq!(output[0].1.topology, g.topology);
                    let directory = tempfile::tempdir().unwrap();
                    let path = directory.path().join("gizmo.osk");
                    Project::new(model.bodies.clone(), Camera::default(), true, Some(1))
                        .save(&path)
                        .unwrap();
                    let loaded = Project::load(&path).unwrap();
                    assert_eq!(loaded.bodies, model.bodies);
                    let regenerated = evaluate_scene(&loaded.bodies).unwrap();
                    assert_eq!(regenerated[0].1.points, output[0].1.points);
                    model.undo();
                    assert_eq!(model.bodies[0], body);
                    model.redo();
                    assert_eq!(model.bodies[0], edited);
                }
            }
        }
        let mut body = Body::new(Shape::Rectangle);
        body.id = 1;
        body.extrusion = 2.;
        let geometry = evaluate(&body).unwrap();
        let mut invalid = transformed(
            &body,
            &geometry,
            SelectionMode::Point,
            Some(0),
            Matrix4::from_translation(Vector3::unit_x() * 0.1),
        )
        .unwrap();
        invalid.component_edits[0].topology.edges.clear();
        let mut model = Model::from_bodies(vec![body.clone()]);
        assert!(model.commit(invalid).is_err());
        assert_eq!(model.bodies[0], body);
        assert!(!model.can_undo());
        let neighbor = geometry
            .topology
            .edges
            .iter()
            .find(|edge| edge.contains(&0))
            .unwrap();
        let other = if neighbor[0] == 0 {
            neighbor[1]
        } else {
            neighbor[0]
        };
        let collapsed = transformed(
            &body,
            &geometry,
            SelectionMode::Point,
            Some(0),
            Matrix4::from_translation(geometry.points[other] - geometry.points[0]),
        )
        .unwrap();
        assert!(model.commit(collapsed).is_err());
        assert_eq!(model.bodies[0], body);
        assert!(!model.can_undo());
        // World-axis rotation composes with an existing, nonuniformly scaled pose.
        let mut posed = body.clone();
        posed.position = [2., 3., 4.];
        posed.rotation = [20., 35., 70.];
        posed.scale = [2., 0.5, 1.5];
        let source = evaluate(&posed).unwrap();
        let center = Vector3::new(1., -2., 0.);
        let matrix = Matrix4::from_translation(center)
            * Matrix4::from_angle_x(Deg(30.))
            * Matrix4::from_translation(-center);
        let rotated = transformed(&posed, &source, SelectionMode::Volume, None, matrix).unwrap();
        let result = evaluate(&rotated).unwrap();
        for (a, b) in source.points.iter().zip(&result.points) {
            assert!((matrix.transform_point(*a) - *b).magnitude() < 1e-6);
        }
        // An outer region with a notch also supports face editing, while deleting
        // its cutting profile must not invalidate saved component references.
        let mut outer = Body::new(Shape::Rectangle);
        outer.id = 1;
        outer.width = 6.;
        outer.depth = 4.;
        outer.extrusion = 2.;
        let mut inner = Body::new(Shape::Rectangle);
        inner.id = 2;
        inner.cuts_profiles = true;
        inner.width = 1.;
        inner.position = [-2.5, 0., 0.];
        let mut split = Model::from_bodies(vec![outer.clone(), inner]);
        let scene = evaluate_scene(&split.bodies).unwrap();
        let source = &scene[0].1;
        let top = source
            .triangles
            .iter()
            .zip(&source.triangle_faces)
            .find(|(t, _)| t.iter().all(|p| (p.z - 2.).abs() < 1e-7))
            .map(|(_, face)| *face)
            .unwrap();
        let edited = transformed(
            &outer,
            source,
            SelectionMode::Face,
            Some(top),
            Matrix4::from_translation(Vector3::unit_z() * 0.5),
        )
        .unwrap();
        split.commit(edited).unwrap();
        let before = split.bodies.clone();
        assert!(split.delete(2).is_err());
        assert_eq!(split.bodies, before);
        // A single point rotated about itself is a no-op, not an owner-body rotation.
        let p = geometry.points[0];
        let matrix = Matrix4::from_translation(p.to_vec())
            * Matrix4::from_angle_z(Deg(30.))
            * Matrix4::from_translation(-p.to_vec());
        assert_eq!(
            transformed(&body, &geometry, SelectionMode::Point, Some(0), matrix).unwrap(),
            body
        );
    }
    #[test]
    fn concave_planar_faces_retrim_with_loops_history_and_collision_rejection() {
        use crate::booleans::{Boolean, Kind};
        for hole in [false, true] {
            let mut a = Body::new(Shape::Rectangle);
            a.id = 1;
            a.width = 6.;
            a.depth = 6.;
            a.extrusion = 3.;
            let mut b = Body::new(Shape::Rectangle);
            b.id = 2;
            b.width = if hole { 2. } else { 4. };
            b.depth = 2.;
            b.extrusion = 3.;
            b.position[0] = if hole { 0. } else { 2. };
            let mut bodies = vec![a, b];
            let scene = evaluate_scene(&bodies).unwrap();
            let mut difference = Body::new(Shape::Rectangle);
            difference.id = 3;
            difference.boolean = Some(Boolean::new(
                Kind::Subtract,
                &[(1, None), (2, None)],
                SelectionMode::Volume,
                &scene,
            ));
            bodies.push(difference.clone());
            let scene = evaluate_scene(&bodies).unwrap();
            let g = &scene[2].1;
            assert!(convex_supports(g).is_err());
            let face = (0..g.face_count)
                .find(|face| {
                    crate::snapping::Plane::face(g, *face).is_some_and(|p| {
                        p.normal.x > 0.99 && (p.origin.x - if hole { -1. } else { 0. }).abs() < 1e-6
                    })
                })
                .unwrap();
            let pivot = pivot(g, SelectionMode::Face, Some(face)).unwrap();
            for matrix in [
                Matrix4::from_translation(Vector3::unit_x() * 0.3),
                Matrix4::from_translation(pivot.to_vec())
                    * Matrix4::from_angle_z(Deg(7.))
                    * Matrix4::from_translation(-pivot.to_vec()),
            ] {
                let changed =
                    transformed(&difference, g, SelectionMode::Face, Some(face), matrix).unwrap();
                assert!(changed.component_edits.last().unwrap().retrim);
                let mut model = Model::from_bodies(bodies.clone());
                model.commit(changed.clone()).unwrap();
                let out = evaluate_scene(&model.bodies).unwrap();
                let out = &out[2].1;
                assert!(out.shell.as_ref().unwrap().is_geometric_consistent());
                assert_eq!(out.topology, g.topology);
                let before = straight_planar_supports(g).unwrap();
                let after = straight_planar_supports(out).unwrap();
                for (i, ((n, d), (new_n, new_d))) in before.iter().zip(&after).enumerate() {
                    if i != face {
                        assert!((*n - *new_n).magnitude() < 1e-6 && (*d - *new_d).abs() < 1e-6);
                    }
                }
                for format in crate::export::Format::ALL {
                    assert!(
                        !crate::export::encode(&model.bodies, format)
                            .unwrap()
                            .is_empty()
                    );
                }
                let directory = tempfile::tempdir().unwrap();
                let path = directory.path().join("concave.osk");
                Project::new(model.bodies.clone(), Camera::default(), true, Some(3))
                    .save(&path)
                    .unwrap();
                assert_eq!(Project::load(&path).unwrap().bodies, model.bodies);
                model.undo();
                assert_eq!(model.bodies, bodies);
                model.redo();
                assert_eq!(model.bodies[2], changed);
            }
            // Replace Face follows the same retained-loop reconstruction with
            // a live support reference, rather than flattening a Boolean result.
            let mut reference = Body::new(Shape::Rectangle);
            reference.id = 4;
            reference.construction = true;
            reference.rotation = [0., 90., 7.];
            reference.position[0] = if hole { -0.7 } else { 0.3 };
            let mut replaced = bodies.clone();
            replaced.push(reference);
            let scene = evaluate_scene(&replaced).unwrap();
            let mut feature = Body::new(Shape::Rectangle);
            feature.id = 5;
            feature.boolean = Some(Boolean::new(
                Kind::ReplaceFace,
                &[(3, Some(face)), (4, Some(0))],
                SelectionMode::Face,
                &scene,
            ));
            replaced.push(feature);
            let mut model = Model::from_bodies(replaced);
            for shift in [0., 0.2] {
                if shift != 0. {
                    let mut reference = model.bodies[3].clone();
                    reference.position[0] += shift;
                    model.commit(reference).unwrap();
                }
                let out = evaluate_scene(&model.bodies).unwrap();
                let target = crate::snapping::Plane::face(&out[3].1, 0).unwrap();
                let cap = crate::snapping::Plane::face(&out[4].1, face).unwrap();
                assert!((cap.origin - target.origin).dot(target.normal).abs() < 1e-6);
                assert!(cap.normal.dot(target.normal).abs() > 1. - 1e-7);
                assert_eq!(out[4].1.topology, g.topology);
                assert!(out[4].1.shell.as_ref().unwrap().is_geometric_consistent());
                for format in crate::export::Format::ALL {
                    assert!(
                        !crate::export::encode(&model.bodies, format)
                            .unwrap()
                            .is_empty()
                    );
                }
                let directory = tempfile::tempdir().unwrap();
                let path = directory.path().join("concave-replace.osk");
                Project::new(model.bodies.clone(), Camera::default(), true, Some(5))
                    .save(&path)
                    .unwrap();
                assert_eq!(Project::load(&path).unwrap().bodies, model.bodies);
            }
            model.undo();
            assert_eq!(model.bodies[3].position[0], if hole { -0.7 } else { 0.3 });
            model.redo();
            assert!(
                (model.bodies[3].position[0] - (if hole { -0.7 } else { 0.3 }) - 0.2).abs() < 1e-8
            );
            let before = model.bodies.clone();
            let mut invalid_reference = model.bodies[3].clone();
            invalid_reference.position[0] = -5.;
            assert!(model.commit(invalid_reference).is_err());
            assert_eq!(model.bodies, before);
            let crossing = transformed(
                &difference,
                g,
                SelectionMode::Face,
                Some(face),
                Matrix4::from_translation(-Vector3::unit_x() * 4.),
            )
            .unwrap();
            let mut model = Model::from_bodies(bodies.clone());
            assert!(model.commit(crossing).is_err());
            assert_eq!(model.bodies, bodies);
            assert!(!model.can_undo());
        }
    }

    #[test]
    fn convex_face_rebuild_splits_corners_consumes_supports_and_replays() {
        use crate::{
            booleans,
            edge_treatments::{self, FaceTarget, Kind, Modifier},
            selection,
        };
        let volume = |g: &Geometry| {
            g.triangles
                .iter()
                .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                .sum::<f64>()
        };
        for boolean in [false, true] {
            for affine in [false, true] {
                if boolean && affine {
                    continue;
                }
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
                if !boolean {
                    let mut modifier = Modifier::new(Kind::Chamfer, FaceTarget::Top, &body);
                    modifier.amount = 0.3;
                    body.edge_treatments.push(modifier);
                }
                let mut bodies = vec![body.clone()];
                if boolean {
                    let mut tool = body.clone();
                    tool.id = 2;
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
                    let scene = evaluate_scene(&bodies).unwrap();
                    let g = &scene[2].1;
                    let top = planar_supports(g)
                        .unwrap()
                        .iter()
                        .position(|(n, _)| n.z > 0.99)
                        .unwrap();
                    let changes = edge_treatments::selected_operations(
                        &bodies,
                        &scene,
                        &[(3, Some(top))],
                        SelectionMode::Face,
                        Kind::Chamfer,
                    )
                    .unwrap();
                    bodies[2] = changes[0].clone();
                }
                let owner = bodies.last().unwrap();
                let id = owner.id;
                let placement = crate::regions::transform(owner);
                let inverse = placement.invert().unwrap();
                let scene = evaluate_scene(&bodies).unwrap();
                let g = &scene.last().unwrap().1;
                let mut local = g.clone();
                map_geometry(&mut local, inverse);
                let supports = planar_supports(&local).unwrap();
                let band = supports
                    .iter()
                    .position(|(n, _)| n.x > 0.3 && n.z > 0.3 && n.y.abs() < 1e-6)
                    .unwrap();
                let (normal, distance) = supports[band];
                let center = pivot(&local, SelectionMode::Face, Some(band)).unwrap();
                for operation in 0..4 {
                    let matrix = if operation == 3 {
                        Matrix4::from_translation(center.to_vec())
                            * Matrix4::from_angle_z(Deg(5.))
                            * Matrix4::from_translation(-center.to_vec())
                    } else {
                        Matrix4::from_translation(normal * [-0.08, 0.08, 1.][operation])
                    };
                    let world = placement * matrix * inverse;
                    let changed =
                        transformed(owner, g, SelectionMode::Face, Some(band), world).unwrap();
                    assert!(changed.component_edits.last().unwrap().convex_rebuild);
                    let mut legacy = changed.clone();
                    legacy.component_edits.last_mut().unwrap().convex_rebuild = false;
                    let mut old = bodies.clone();
                    *old.last_mut().unwrap() = legacy;
                    assert!(
                        evaluate_scene(&old).is_err(),
                        "The legacy corner solver cannot rebuild this edit"
                    );
                    let mut model = Model::from_bodies(bodies.clone());
                    model.commit(changed.clone()).unwrap();
                    let output = evaluate_scene(&model.bodies).unwrap();
                    let result = &output.last().unwrap().1;
                    assert!(result.is_solid);
                    assert!(result.shell.as_ref().unwrap().is_geometric_consistent());
                    solid_parts(result.shell.as_ref().unwrap()).unwrap();
                    assert_eq!(result.shell.as_ref().unwrap().len(), result.face_count);
                    assert_ne!(result.topology, g.topology);
                    assert!(selection::topology_changed(
                        &[(id, Some(band))],
                        &scene,
                        &output
                    ));
                    assert!(!selection::topology_changed(&[(id, None)], &scene, &output));
                    assert!(!selection::topology_changed(
                        &[(id, Some(band))],
                        &scene,
                        &scene
                    ));
                    assert!(selection::topology_changed(
                        &[(id, Some(band))],
                        &scene,
                        &[]
                    ));
                    let mut local_out = result.clone();
                    map_geometry(&mut local_out, inverse);
                    let after = planar_supports(&local_out).unwrap();
                    for (face, (n, d)) in supports.iter().enumerate().filter(|(f, _)| *f != band) {
                        assert!(
                            after.iter().any(
                                |(m, e)| (*m - *n).magnitude() < 1e-6 && (*e - *d).abs() < 1e-6
                            ),
                            "Unchanged support {face}"
                        );
                    }
                    let selected_triangle = local
                        .triangles
                        .iter()
                        .zip(&local.triangle_faces)
                        .find(|(_, f)| **f == band)
                        .unwrap()
                        .0
                        .map(|p| matrix.transform_point(p));
                    let n = (selected_triangle[1] - selected_triangle[0])
                        .cross(selected_triangle[2] - selected_triangle[0])
                        .normalize();
                    let d = n.dot(selected_triangle[0].to_vec());
                    if operation == 2 {
                        assert_eq!(result.face_count, g.face_count - 1);
                        assert!(
                            !after
                                .iter()
                                .any(|(m, e)| (*m - n).magnitude() < 1e-6 && (*e - d).abs() < 1e-6),
                            "The redundant blend support disappears"
                        );
                    } else {
                        assert!(
                            after
                                .iter()
                                .any(|(m, e)| (*m - n).magnitude() < 1e-6 && (*e - d).abs() < 1e-6)
                        );
                        assert_eq!(result.face_count, g.face_count);
                    }
                    if operation < 3 {
                        assert!(
                            (volume(&local_out) - volume(&local))
                                * if operation == 0 { -1. } else { 1. }
                                > 1e-6
                        );
                        assert!((d - distance - [-0.08, 0.08, 1.][operation]).abs() < 1e-6);
                    }
                    for format in crate::export::Format::ALL {
                        assert!(
                            !crate::export::encode(&model.bodies, format)
                                .unwrap()
                                .is_empty()
                        );
                    }
                    let dir = tempfile::tempdir().unwrap();
                    let path = dir.path().join("convex-rebuild.osk");
                    let project =
                        Project::new(model.bodies.clone(), Camera::default(), true, Some(id));
                    project.save(&path).unwrap();
                    assert_eq!(Project::load(&path).unwrap().bodies, model.bodies);
                    let mut old = serde_json::to_value(&project).unwrap();
                    old["schema_version"] = 39.into();
                    old["modeling_version"] = 39.into();
                    assert!(
                        serde_json::from_value::<Project>(old)
                            .unwrap()
                            .validate()
                            .unwrap_err()
                            .contains("version 40")
                    );
                    let accepted = model.bodies.clone();
                    model.undo();
                    assert_eq!(model.bodies, bodies);
                    model.redo();
                    assert_eq!(model.bodies, accepted);
                    let top = after.iter().position(|(n, _)| n.z > 0.99).unwrap();
                    let next = transformed(
                        &changed,
                        result,
                        SelectionMode::Face,
                        Some(top),
                        placement * Matrix4::from_translation(Vector3::unit_z() * 0.05) * inverse,
                    )
                    .unwrap();
                    model.commit(next).unwrap();
                    Project::new(model.bodies.clone(), Camera::default(), true, Some(id))
                        .save(&path)
                        .unwrap();
                    assert_eq!(Project::load(&path).unwrap().bodies, model.bodies);
                    if !boolean
                        && !affine
                        && operation == 1
                        && let Some(path) = std::env::var_os("OSK_CONVEX_REBUILD_REVIEW_PROJECT")
                    {
                        let mut review =
                            Project::new(bodies.clone(), Camera::default(), true, Some(id));
                        review.selection_mode = SelectionMode::Face;
                        review.selected_element = Some(band);
                        review.selection = vec![(id, Some(band))];
                        review.save(std::path::Path::new(&path)).unwrap();
                    }
                }
                let top = supports.iter().position(|(n, _)| n.z > 0.99).unwrap();
                let invalid = transformed(
                    owner,
                    g,
                    SelectionMode::Face,
                    Some(top),
                    placement * Matrix4::from_translation(-Vector3::unit_z() * 10.) * inverse,
                )
                .unwrap();
                let mut model = Model::from_bodies(bodies.clone());
                assert!(model.commit(invalid).is_err());
                assert_eq!(model.bodies, bodies);
                assert!(!model.can_undo());
            }
        }
    }

    #[test]
    fn convex_planar_face_moves_retrim_neighbors_and_preserve_exact_history() {
        use crate::edge_treatments::{FaceTarget, Kind, Modifier};
        for chamfer in [false, true] {
            let mut body = Body::new(Shape::Rectangle);
            body.id = 1;
            body.width = 4.;
            body.depth = 3.;
            body.extrusion = 5.;
            body.position = [2., -3., 1.];
            body.rotation = [15., 25., 35.];
            body.scale = [1.5, 0.75, 2.];
            if chamfer {
                body.edge_treatments
                    .push(Modifier::new(Kind::Chamfer, FaceTarget::Top, &body));
            }
            let g = evaluate(&body).unwrap();
            let face = (0..g.face_count)
                .find(|f| crate::edge_treatments::target(&body, &g, *f) == Ok(FaceTarget::Top))
                .unwrap();
            let plane = crate::snapping::Plane::face(&g, face).unwrap();
            let center = pivot(&g, SelectionMode::Face, Some(face)).unwrap();
            let originals = planar_supports(&g).unwrap();
            for matrix in [
                Matrix4::from_translation(plane.normal * 0.4),
                Matrix4::from_translation(center.to_vec())
                    * Matrix4::from_axis_angle(plane.u, Deg(7.))
                    * Matrix4::from_translation(-center.to_vec()),
            ] {
                let edited =
                    transformed(&body, &g, SelectionMode::Face, Some(face), matrix).unwrap();
                assert!(edited.component_edits.last().unwrap().retrim);
                let out = evaluate(&edited).unwrap();
                assert_eq!(out.topology, g.topology);
                let supports = planar_supports(&out).unwrap();
                for (i, (n, d)) in supports.iter().enumerate() {
                    let (expected_n, expected_d) = if i == face {
                        let n = matrix.transform_vector(plane.normal).normalize();
                        (n, n.dot(matrix.transform_point(plane.origin).to_vec()))
                    } else {
                        originals[i]
                    };
                    assert!(
                        (*n - expected_n).magnitude() < 1e-7 && (*d - expected_d).abs() < 1e-7,
                        "Neighbor planes must extend/retrim without deforming"
                    );
                }
                let shell = out.shell.as_ref().unwrap();
                assert_eq!(shell.len(), g.face_count);
                assert!(
                    shell
                        .iter()
                        .all(|f| matches!(f.oriented_surface(), truck_modeling::Surface::Plane(_)))
                );
                solid_parts(shell).unwrap();
                let mut model = Model::from_bodies(vec![body.clone()]);
                model.commit(edited.clone()).unwrap();
                model.undo();
                assert_eq!(model.bodies, vec![body.clone()]);
                model.redo();
                assert_eq!(model.bodies, vec![edited.clone()]);
                let dir = tempfile::tempdir().unwrap();
                let path = dir.path().join("retrim.osk");
                let project = Project::new(vec![edited.clone()], Camera::default(), true, Some(1));
                project.save(&path).unwrap();
                assert_eq!(Project::load(&path).unwrap().bodies, vec![edited.clone()]);
                let mut legacy = serde_json::to_value(project).unwrap();
                legacy["schema_version"] = 17.into();
                legacy["modeling_version"] = 17.into();
                std::fs::write(&path, serde_json::to_vec(&legacy).unwrap()).unwrap();
                assert!(
                    Project::load(&path)
                        .unwrap_err()
                        .contains("Exact planar face retrim requires .osk version 18")
                );
                legacy["bodies"][0]["component_edits"][0]
                    .as_object_mut()
                    .unwrap()
                    .remove("retrim");
                legacy["bodies"][0]["component_edits"][0]
                    .as_object_mut()
                    .unwrap()
                    .remove("convex_rebuild");
                std::fs::write(&path, serde_json::to_vec(&legacy).unwrap()).unwrap();
                assert!(!Project::load(&path).unwrap().bodies[0].component_edits[0].retrim);
                for format in [
                    crate::export::Format::Stl,
                    crate::export::Format::Glb,
                    crate::export::Format::Step,
                    crate::export::Format::Dxf,
                ] {
                    let bytes =
                        crate::export::encode(std::slice::from_ref(&edited), format).unwrap();
                    assert!(!bytes.is_empty());
                    if format == crate::export::Format::Step {
                        assert_eq!(
                            String::from_utf8(bytes)
                                .unwrap()
                                .matches(" = PLANE(")
                                .count(),
                            g.face_count
                        );
                    }
                }
                let bad = Matrix4::from_translation(plane.normal * -30.);
                let invalid =
                    transformed(&edited, &out, SelectionMode::Face, Some(face), bad).unwrap();
                assert!(model.commit(invalid).is_err());
                assert_eq!(model.bodies, vec![edited]);
            }
        }
    }
    #[test]
    fn ordered_evaluated_face_sweeps_follow_retrim_and_boolean_results() {
        let volume = |g: &Geometry| {
            g.triangles
                .iter()
                .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                .sum::<f64>()
        };
        for rotated in [false, true] {
            let mut body = Body::new(Shape::Rectangle);
            body.id = 1;
            body.width = 4.;
            body.depth = 3.;
            body.extrusion = 5.;
            body.position = [2., -3., 1.];
            body.rotation = [15., 25., 35.];
            body.scale = [1.5, 0.75, 2.];
            let original = evaluate(&body).unwrap();
            let face = (0..original.face_count)
                .find(|f| {
                    crate::edge_treatments::target(&body, &original, *f)
                        == Ok(crate::edge_treatments::FaceTarget::Top)
                })
                .unwrap();
            let plane = crate::snapping::Plane::face(&original, face).unwrap();
            body = transformed(
                &body,
                &original,
                SelectionMode::Face,
                Some(face),
                Matrix4::from_translation(plane.normal * 0.5),
            )
            .unwrap();
            if rotated {
                let g = evaluate(&body).unwrap();
                let center = pivot(&g, SelectionMode::Face, Some(face)).unwrap();
                body = transformed(
                    &body,
                    &g,
                    SelectionMode::Face,
                    Some(face),
                    Matrix4::from_translation(center.to_vec())
                        * Matrix4::from_axis_angle(plane.u, Deg(7.))
                        * Matrix4::from_translation(-center.to_vec()),
                )
                .unwrap();
            }
            let g = evaluate(&body).unwrap();
            let before = volume(&g);
            let area = g
                .triangles
                .iter()
                .zip(&g.triangle_faces)
                .filter(|(_, f)| **f == face)
                .map(|(t, _)| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
                .sum::<f64>();
            for distance in if rotated {
                vec![0.4]
            } else {
                vec![0.4, -0.4, -30.]
            } {
                let changed = sweep_face(&body, &g, face, distance).unwrap();
                assert!(changed.component_edits.last().unwrap().sweep.is_some());
                let out = evaluate(&changed).unwrap();
                let expected = (before + area * distance).max(0.);
                assert!(
                    (volume(&out) - expected).abs() < 1e-6,
                    "World normal displacement after source scaling/rotation: {} vs {expected}",
                    volume(&out)
                );
                if expected > 0. {
                    assert!(out.shell.is_some());
                    solid_parts(out.shell.as_ref().unwrap()).unwrap();
                }
                let mut model = Model::from_bodies(vec![body.clone()]);
                model.commit(changed.clone()).unwrap();
                model.undo();
                assert_eq!(model.bodies, vec![body.clone()]);
                model.redo();
                assert_eq!(model.bodies, vec![changed.clone()]);
                let dir = tempfile::tempdir().unwrap();
                let path = dir.path().join("sweep.osk");
                let project = Project::new(vec![changed.clone()], Camera::default(), true, Some(1));
                project.save(&path).unwrap();
                assert_eq!(Project::load(&path).unwrap().bodies, vec![changed.clone()]);
                let mut old = serde_json::to_value(project).unwrap();
                // Exercise the original v19 payload without later opt-ins.
                for edit in old["bodies"][0]["component_edits"].as_array_mut().unwrap() {
                    edit.as_object_mut().unwrap().remove("tangent_walls");
                    edit.as_object_mut().unwrap().remove("prismatic_cut");
                    edit.as_object_mut().unwrap().remove("convex_rebuild");
                }
                old["schema_version"] = 18.into();
                old["modeling_version"] = 18.into();
                std::fs::write(&path, serde_json::to_vec(&old).unwrap()).unwrap();
                assert!(
                    Project::load(&path)
                        .unwrap_err()
                        .contains("Evaluated planar face sweeps require .osk version 19")
                );
                old["schema_version"] = 19.into();
                old["modeling_version"] = 19.into();
                std::fs::write(&path, serde_json::to_vec(&old).unwrap()).unwrap();
                let legacy = Project::load(&path).unwrap();
                assert!(
                    !legacy.bodies[0]
                        .component_edits
                        .last()
                        .unwrap()
                        .tangent_walls
                );
                assert!((volume(&evaluate(&legacy.bodies[0]).unwrap()) - expected).abs() < 1e-6);
                if expected > 0. {
                    for format in crate::export::Format::ALL {
                        assert!(
                            !crate::export::encode(std::slice::from_ref(&changed), format)
                                .unwrap()
                                .is_empty()
                        );
                    }
                    // A later move uses the swept topology, not the old source face slots.
                    let plane = crate::snapping::Plane::face(&g, face).unwrap();
                    let cap = (0..out.face_count)
                        .find(|f| {
                            crate::snapping::Plane::face(&out, *f).is_some_and(|p| {
                                p.normal.dot(plane.normal) > 1. - 1e-6
                                    && (p.origin - plane.origin - plane.normal * distance)
                                        .dot(plane.normal)
                                        .abs()
                                        < 1e-6
                            })
                        })
                        .unwrap();
                    let continued = transformed(
                        &changed,
                        &out,
                        SelectionMode::Face,
                        Some(cap),
                        Matrix4::from_translation(plane.normal * 0.1),
                    )
                    .unwrap();
                    assert!(
                        (volume(&evaluate(&continued).unwrap()) - (expected + area * 0.1)).abs()
                            < 1e-6
                    );
                }
                let mut invalid = changed.clone();
                invalid.component_edits.last_mut().unwrap().sweep = Some([f64::NAN, 0., 0.]);
                assert!(model.commit(invalid).is_err());
                assert_eq!(model.bodies, vec![changed]);
            }
        }
        let mut a = Body::new(Shape::Rectangle);
        a.id = 1;
        a.width = 4.;
        a.depth = 4.;
        a.extrusion = 3.;
        let mut b = a.clone();
        b.id = 2;
        b.position = [1., 0., 0.];
        let mut bodies = vec![a, b];
        let input = crate::model::evaluate_scene(&bodies).unwrap();
        let mut result = Body::new(Shape::Rectangle);
        result.id = 3;
        result.boolean = Some(crate::booleans::Boolean::new(
            crate::booleans::Kind::Intersect,
            &[(1, None), (2, None)],
            SelectionMode::Volume,
            &input,
        ));
        bodies.push(result);
        let scene = crate::model::evaluate_scene(&bodies).unwrap();
        let g = &scene[2].1;
        let face = (0..g.face_count)
            .find(|f| crate::snapping::Plane::face(g, *f).is_some_and(|p| p.normal.z > 0.99))
            .unwrap();
        let before = volume(g);
        let area = g
            .triangles
            .iter()
            .zip(&g.triangle_faces)
            .filter(|(_, f)| **f == face)
            .map(|(t, _)| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
            .sum::<f64>();
        bodies[2] = sweep_face(&bodies[2], g, face, 0.5).unwrap();
        let out = crate::model::evaluate_scene(&bodies).unwrap();
        assert!((volume(&out[2].1) - before - area * 0.5).abs() < 1e-6);
        assert!(out[0].1.triangles.is_empty() && out[1].1.triangles.is_empty());
        for format in crate::export::Format::ALL {
            assert!(!crate::export::encode(&bodies, format).unwrap().is_empty());
        }
        let mut treated = Body::new(Shape::Rectangle);
        treated.id = 1;
        treated.extrusion = 4.;
        treated
            .edge_treatments
            .push(crate::edge_treatments::Modifier::new(
                crate::edge_treatments::Kind::Chamfer,
                crate::edge_treatments::FaceTarget::Top,
                &treated,
            ));
        let g = evaluate(&treated).unwrap();
        let band = (0..g.face_count)
            .find(|f| {
                crate::edge_treatments::target(&treated, &g, *f).is_err()
                    && crate::snapping::Plane::face(&g, *f).is_some()
            })
            .unwrap();
        let area = g
            .triangles
            .iter()
            .zip(&g.triangle_faces)
            .filter(|(_, f)| **f == band)
            .map(|(t, _)| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
            .sum::<f64>();
        let expanded = sweep_face(&treated, &g, band, 0.2).unwrap();
        let out = evaluate(&expanded).unwrap();
        assert!(
            (volume(&out) - volume(&g) - area * 0.2).abs() < 1e-6,
            "Planar transition regions use their evaluated boundary"
        );
    }
}
