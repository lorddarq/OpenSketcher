//! Authored control-net operations; topology vertices and display samples are never controls.
use crate::{
    components::{self, ComponentEdit},
    model::{Body, Geometry},
    selection::SelectionMode,
};
use serde::{Deserialize, Serialize};
use truck_modeling::{
    BSplineSurface, BoundedCurve, BoundedSurface, EuclideanSpace, Face, InnerSpace, KnotVec,
    Matrix4, NurbsSurface, ParametricCurve, Point3, SearchParameter, Shell, SquareMatrix, Surface,
    Transform, Vector3, Vector4, builder,
};

#[path = "periodic_cage.rs"]
mod periodic;
type SurfaceNet = NurbsSurface<Vector4>;

pub const MAX_RADIUS: f64 = 1e6;

/// Parameter-lattice coordinates are node identities within a guarded authored domain.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Binding {
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub disk: bool,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub periodic: bool,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub patches: Vec<usize>,
    pub rows: usize,
    pub columns: usize,
    pub uknots: Vec<f64>,
    pub vknots: Vec<f64>,
}
impl Binding {
    fn of(net: &SurfaceNet) -> Self {
        Self {
            disk: false,
            periodic: false,
            patches: Vec::new(),
            rows: net.control_points().len(),
            columns: net.control_points()[0].len(),
            uknots: net.uknot_vec().iter().copied().collect(),
            vknots: net.vknot_vec().iter().copied().collect(),
        }
    }
    fn validate(&self) -> Result<(), String> {
        if self.periodic
            && (self.disk
                || self.rows != 8
                || self.columns != 5
                || self.uknots != KnotVec::bezier_knot(7).to_vec()
                || self.vknots != KnotVec::bezier_knot(4).to_vec()
                || self.patches.is_empty()
                || self.patches.len() > 16
                || self.patches.iter().any(|&i| i >= 8192)
                || self.patches.windows(2).any(|p| p[0] >= p[1]))
            || !self.periodic && !self.patches.is_empty()
        {
            return Err("Invalid periodic cage definition or patch references.".into());
        }
        if self.disk
            && (self.rows != 5
                || self.columns != 5
                || self.uknots != KnotVec::bezier_knot(4).to_vec()
                || self.vknots != self.uknots)
        {
            return Err(
                "Circular cage identities require the authored 5×5 degree-four lattice.".into(),
            );
        }
        if !(3..=16).contains(&self.rows)
            || !(3..=16).contains(&self.columns)
            || self.uknots.len() > 33
            || self.vknots.len() > 33
            || self.uknots.len() <= self.rows + 1
            || self.vknots.len() <= self.columns + 1
            || self
                .uknots
                .iter()
                .chain(&self.vknots)
                .any(|v| !v.is_finite())
            || self
                .uknots
                .windows(2)
                .chain(self.vknots.windows(2))
                .any(|v| v[0] > v[1])
        {
            return Err("Unsupported control-net definition (3–16 rows/columns, finite clamped knots required).".into());
        }
        Ok(())
    }
    fn interior(&self, [u, v]: [usize; 2]) -> bool {
        v > 0
            && v < self.columns - 1
            && u < self.rows
            && (self.periodic || u > 0 && u < self.rows - 1)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Action {
    Create {
        planar: bool,
    },
    Translate {
        sources: Vec<[usize; 2]>,
        locked: Vec<[usize; 2]>,
        enabled: bool,
        radius: f64,
        delta: [f64; 3],
    },
    MoveFace {
        delta: [f64; 3],
    },
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Operation {
    pub domain: u64,
    pub binding: Binding,
    /// v1 = shortest world-edge paths + nearest-source smoothstep + owner-local translation; interactive input is world-space.
    pub version: u32,
    pub action: Action,
}
impl Operation {
    pub fn validate(&self) -> Result<(), String> {
        self.binding.validate()?;
        if self.version
            != if self.binding.periodic {
                3
            } else if self.binding.disk {
                2
            } else {
                1
            }
            || self.domain == 0
        {
            return Err("Unsupported control-domain or falloff version.".into());
        }
        match &self.action {
            Action::Create { .. } => (),
            Action::Translate {
                sources,
                locked,
                radius,
                delta,
                ..
            } => {
                if sources.is_empty()
                    || sources.len() > 256
                    || locked.len() > 256
                    || sources
                        .iter()
                        .chain(locked)
                        .any(|&id| !self.binding.interior(id))
                    || sources.iter().any(|id| locked.contains(id))
                    || !radius.is_finite()
                    || !(0. ..=MAX_RADIUS).contains(radius)
                    || !delta.iter().all(|v| v.is_finite() && v.abs() <= 10000.)
                {
                    return Err("Invalid node reference, lock, radius or displacement.".into());
                }
            }
            Action::MoveFace { delta } => {
                if !delta.iter().all(|v| v.is_finite() && v.abs() <= 10000.) {
                    return Err("Face displacement must be finite and within ±10000 mm.".into());
                }
            }
        }
        Ok(())
    }
}

fn patch(g: &Geometry, face: usize) -> Result<Face, String> {
    let shell = g
        .shell
        .as_ref()
        .ok_or("Node editing requires an exact retained surface.")?;
    let indices: Vec<_> = (0..shell.len())
        .filter(|&i| g.surface_roles.get(i).copied().unwrap_or(i) == face)
        .collect();
    if indices.len() != 1 {
        return Err("Select one unpartitioned editable surface patch.".into());
    }
    Ok(shell[indices[0]].clone())
}

/// An authored cage and its undeformed planar binding, independent of display samples.
#[derive(Clone)]
pub struct Cage {
    pub domain: u64,
    pub face: usize,
    pub binding: Binding,
    pub patches: Vec<(usize, SurfaceNet)>,
    pub net: SurfaceNet,
    pub reference: SurfaceNet,
}
impl Cage {
    pub fn transform(&mut self, matrix: Matrix4) {
        for (_, patch) in &mut self.patches {
            patch.transform_control_points(|p| *p = matrix * *p);
        }
        self.net.transform_control_points(|p| *p = matrix * *p);
        self.reference
            .transform_control_points(|p| *p = matrix * *p);
    }
}

fn disk_net(g: &Geometry, face: &Face) -> Result<SurfaceNet, String> {
    let Surface::Plane(plane) = face.surface() else {
        return Err("Circular cages require a planar circular input face.".into());
    };
    let bounds = face.boundaries();
    if bounds.len() != 1 {
        return Err(
            "Circular cages currently require a complete disk without interior trims.".into(),
        );
    }
    let ring = g
        .circles
        .iter()
        .find(|c| crate::face_extrusions::circle_on_wire(c, &bounds[0]))
        .ok_or("No retained complete circular boundary is available for this cage.")?;
    let (u, v) = if ring.u.cross(ring.v).dot(plane.normal()) >= 0. {
        (ring.u, ring.v)
    } else {
        (ring.v, ring.u)
    };
    let controls = (0..5)
        .map(|i| {
            (0..5)
                .map(|j| {
                    (ring.center + u * (i as f64 / 2. - 1.) + v * (j as f64 / 2. - 1.))
                        .to_vec()
                        .extend(1.)
                })
                .collect()
        })
        .collect();
    let knots = KnotVec::bezier_knot(4);
    Ok(NurbsSurface::new(BSplineSurface::new(
        (knots.clone(), knots),
        controls,
    )))
}

fn choose(n: usize, k: usize) -> f64 {
    if k > n {
        return 0.;
    }
    (0..k).fold(1., |p, i| p * (n - i) as f64 / (i + 1) as f64)
}

/// Compose cage displacement with (1-x²-y²)² exactly in Bernstein form.
/// On the circular trim the field and its first derivatives vanish, keeping the shared rim fixed.
fn disk_surface(cage: &Cage) -> SurfaceNet {
    let mut radial = [[0.; 3]; 3];
    radial[0][0] = -1.;
    radial[1][0] = 4.;
    radial[0][1] = 4.;
    radial[2][0] = -4.;
    radial[0][2] = -4.;
    let mut power = [[0.; 5]; 5];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                for l in 0..3 {
                    power[i + k][j + l] += radial[i][j] * radial[k][l];
                }
            }
        }
    }
    let mut mask = [[0.; 5]; 5];
    for (i, row) in mask.iter_mut().enumerate() {
        for (j, value) in row.iter_mut().enumerate() {
            for (k, coefficients) in power.iter().enumerate().take(i + 1) {
                for (l, p) in coefficients.iter().enumerate().take(j + 1) {
                    *value += p * choose(i, k) / choose(4, k) * choose(j, l) / choose(4, l);
                }
            }
        }
    }
    let mut displacement = [[Vector3::new(0., 0., 0.); 9]; 9];
    for i in 0..5 {
        for j in 0..5 {
            let d = cage.net.control_point(i, j).truncate()
                - cage.reference.control_point(i, j).truncate();
            for (k, row) in mask.iter().enumerate() {
                for (l, m) in row.iter().enumerate() {
                    displacement[i + k][j + l] += d
                        * (*m * choose(4, i) * choose(4, k) / choose(8, i + k)
                            * choose(4, j)
                            * choose(4, l)
                            / choose(8, j + l));
                }
            }
        }
    }
    let controls = displacement
        .iter()
        .enumerate()
        .map(|(i, row)| {
            row.iter()
                .enumerate()
                .map(|(j, d)| {
                    (cage.reference.subs(i as f64 / 8., j as f64 / 8.) + *d)
                        .to_vec()
                        .extend(1.)
                })
                .collect()
        })
        .collect();
    let knots = KnotVec::bezier_knot(8);
    NurbsSurface::new(BSplineSurface::new((knots.clone(), knots), controls))
}

/// Evaluate analytic derivatives over the actual supported domain before generating a display mesh.
/// Sampling detects local rank loss; it is not a proof of global injectivity.
fn regularity(surface: &SurfaceNet, disk: bool) -> Result<(), String> {
    let (u, v) = surface.range_tuple();
    for i in 0..=32 {
        for j in 0..=32 {
            let a = i as f64 / 32.;
            let b = j as f64 / 32.;
            if disk && (2. * a - 1.).powi(2) + (2. * b - 1.).powi(2) > 1. + 1e-12 {
                continue;
            }
            let x = u.0 + (u.1 - u.0) * a;
            let y = v.0 + (v.1 - v.0) * b;
            let du = surface.uder(x, y);
            let dv = surface.vder(x, y);
            let area = du.cross(dv).magnitude2();
            let scale = du.magnitude2() * dv.magnitude2();
            if !area.is_finite() || !scale.is_finite() || scale <= 0. || area <= scale * 1e-12 {
                return Err("Control edit produces a singular or numerically degenerate surface tangent plane.".into());
            }
        }
    }
    Ok(())
}

fn rectangle(face: &Face) -> Result<SurfaceNet, String> {
    let boundaries = face.boundaries();
    if boundaries.len() != 1 || boundaries[0].len() != 4 {
        return Err(
            "Create control net requires an untrimmed rectangular/parallelogram planar face."
                .into(),
        );
    }
    let points: Vec<_> = boundaries[0].iter().map(|e| e.front().point()).collect();
    if (points[0] + (points[2] - points[1]) - points[3]).magnitude() > 1e-7 {
        return Err("Create control net requires a parallelogram boundary.".into());
    }
    for edge in &boundaries[0] {
        if !matches!(edge.curve(), truck_modeling::Curve::Line(_)) {
            return Err("Curved trims require an existing supported NURBS patch.".into());
        }
    }
    let controls = (0..5)
        .map(|u| {
            (0..5)
                .map(|v| {
                    let p = points[0]
                        + (points[1] - points[0]) * (u as f64 / 4.)
                        + (points[3] - points[0]) * (v as f64 / 4.);
                    p.to_vec().extend(1.)
                })
                .collect()
        })
        .collect();
    let knots = KnotVec::bezier_knot(4);
    let mut net = NurbsSurface::new(BSplineSurface::new((knots.clone(), knots), controls));
    // The surface must follow the underlying face orientation, not its oriented wire.
    if !face.orientation() {
        net = NurbsSurface::new(BSplineSurface::new(
            (net.uknot_vec().clone(), net.vknot_vec().clone()),
            net.control_points().iter().rev().cloned().collect(),
        ));
    }
    Ok(net)
}

fn supported(net: &SurfaceNet, face: &Face) -> Result<(), String> {
    let binding = Binding::of(net);
    binding.validate()?;
    if net
        .control_points()
        .iter()
        .flatten()
        .any(|p| !p.w.is_finite() || p.w <= 0. || ![p.x, p.y, p.z].iter().all(|v| v.is_finite()))
    {
        return Err("Control nets require finite positions and positive rational weights.".into());
    }
    // Only clamped full patches: locking outer controls then preserves the exact trims.
    let (ud, vd) = net.degrees();
    for (knots, degree) in [(net.uknot_vec(), ud), (net.vknot_vec(), vd)] {
        if degree < 2
            || knots.iter().take(degree + 1).any(|v| *v != knots[0])
            || knots
                .iter()
                .rev()
                .take(degree + 1)
                .any(|v| *v != knots[knots.len() - 1])
        {
            return Err(
                "Periodic and unclamped surfaces need explicit seam bindings before node editing."
                    .into(),
            );
        }
    }
    let bounds = face.boundaries();
    if bounds.len() != 1 || bounds[0].len() != 4 {
        return Err(
            "Trimmed/holey patches are not eligible for fixed-boundary node editing.".into(),
        );
    }
    let (u, v) = net.range_tuple();
    let corners = [
        net.subs(u.0, v.0),
        net.subs(u.1, v.0),
        net.subs(u.1, v.1),
        net.subs(u.0, v.1),
    ];
    if bounds[0].iter().any(|e| {
        !corners
            .iter()
            .any(|p| (*p - e.front().point()).magnitude() < 1e-6)
    }) {
        return Err("The face trim does not match the complete NURBS domain.".into());
    }
    for edge in &bounds[0] {
        let curve = edge.oriented_curve();
        let (a, b) = curve.range_tuple();
        for i in 0..=2 {
            let p = curve.subs(a + (b - a) * i as f64 / 2.);
            let (x, y) = net
                .search_parameter(p, None, 64)
                .ok_or("Trim could not be resolved on the control surface.")?;
            if (x - u.0).abs().min((x - u.1).abs()) > 1e-7
                && (y - v.0).abs().min((y - v.1).abs()) > 1e-7
            {
                return Err(
                    "Interior trims need explicit cage/trim bindings before node editing.".into(),
                );
            }
        }
    }
    Ok(())
}

pub fn net(g: &Geometry, face: usize) -> Result<SurfaceNet, String> {
    if let Some(cage) = g.cages.iter().find(|c| c.face == face) {
        return Ok(cage.net.clone());
    }
    let f = patch(g, face)?;
    if let Surface::NurbsSurface(net) = f.surface() {
        Ok(net)
    } else {
        Err("Create an editable control net before entering node mode.".into())
    }
}

pub fn create(body: &Body, g: &Geometry, face: usize) -> Result<Body, String> {
    if body.construction {
        return Err("Construction references have no editable control net.".into());
    }
    if domain(body, face).is_some() {
        return Err("This face already has a control net; choose Edit nodes.".into());
    }
    if g.face_elements(face).len() > 1 {
        if body.component_edits.iter().any(|e| e.control.is_some()) {
            return Err(
                "Create a periodic wall cage before other control domains on this body.".into(),
            );
        }
        let cage = periodic::create(g, face)?;
        return append(
            body,
            g,
            face,
            Operation {
                domain: 1,
                binding: cage.binding,
                version: 3,
                action: Action::Create { planar: false },
            },
        );
    }
    let f = patch(g, face)?;
    let (net,planar,disk)=match f.surface() {
        Surface::Plane(_)=>match rectangle(&f) {Ok(net)=>(net,true,false),Err(_)=>(disk_net(g,&f)?,true,true)},
        Surface::NurbsSurface(net)=>(net,false,false),
        _=>return Err("Create control net supports planar parallelograms, circular disks and clamped full NURBS patches.".into()),
    };
    if !disk {
        supported(&net, &f)?;
    }
    let mut binding = Binding::of(&net);
    binding.disk = disk;
    let op = Operation {
        domain: body
            .component_edits
            .iter()
            .filter_map(|e| e.control.as_ref().map(|o| o.domain))
            .max()
            .unwrap_or(0)
            .checked_add(1)
            .ok_or("Control domain IDs are exhausted.")?,
        binding,
        version: if disk { 2 } else { 1 },
        action: Action::Create { planar },
    };
    append(body, g, face, op)
}

pub fn append(
    body: &Body,
    g: &Geometry,
    face: usize,
    operation: Operation,
) -> Result<Body, String> {
    operation.validate()?;
    let mut result = body.clone();
    result.component_edits.push(ComponentEdit {
        control: Some(operation),
        evaluated: true,
        retrim: false,
        convex_rebuild: false,
        section_transform: false,
        cap_rotation: false,
        sweep: None,
        tangent_walls: false,
        prismatic_cut: false,
        face_edit: None,
        edge_chamfer: None,
        prism_chamfer: false,
        prism_bevel: None,
        prism_curves: false,
        mode: SelectionMode::Face,
        element: face,
        elements: vec![],
        topology: g.topology.clone(),
        matrix: Matrix4::identity().into(),
    });
    result.validate()?;
    Ok(result)
}

pub fn domain(body: &Body, face: usize) -> Option<&Operation> {
    body.component_edits
        .iter()
        .find(|e| {
            e.element == face
                && matches!(
                    e.control.as_ref().map(|o| &o.action),
                    Some(Action::Create { .. })
                )
        })
        .and_then(|e| e.control.as_ref())
}

pub fn positions(net: &SurfaceNet) -> Vec<Point3> {
    net.control_points()
        .iter()
        .flatten()
        .map(|p| Point3::new(p.x / p.w, p.y / p.w, p.z / p.w))
        .collect()
}
pub fn edges(binding: &Binding) -> Vec<[usize; 2]> {
    let mut edges = Vec::new();
    for u in 0..binding.rows {
        for v in 0..binding.columns {
            let i = u * binding.columns + v;
            if u + 1 < binding.rows {
                edges.push([i, i + binding.columns]);
            }
            if binding.periodic && u + 1 == binding.rows {
                edges.push([i, v]);
            }
            if v + 1 < binding.columns {
                edges.push([i, i + 1]);
            }
        }
    }
    edges
}
pub fn locks(binding: &Binding, extra: &[[usize; 2]]) -> Vec<bool> {
    (0..binding.rows)
        .flat_map(|u| {
            (0..binding.columns).map(move |v| !binding.interior([u, v]) || extra.contains(&[u, v]))
        })
        .collect()
}
pub fn field(
    net: &SurfaceNet,
    binding: &Binding,
    placement: Matrix4,
    sources: &[[usize; 2]],
    locked: &[[usize; 2]],
    enabled: bool,
    radius: f64,
) -> Result<Vec<f64>, String> {
    if !radius.is_finite() || !(0. ..=MAX_RADIUS).contains(&radius) {
        return Err("Radius must be finite and within 0–1000000 mm.".into());
    }
    if sources
        .iter()
        .chain(locked)
        .any(|&id| !binding.interior(id))
    {
        return Err("The selected control node no longer exists or is boundary-locked.".into());
    }
    let rest = positions(net)
        .into_iter()
        .map(|p| placement.transform_point(p))
        .collect::<Vec<_>>();
    let indices = sources
        .iter()
        .map(|&[u, v]| u * binding.columns + v)
        .collect::<Vec<_>>();
    influences(
        &rest,
        &edges(binding),
        &indices,
        &locks(binding, locked),
        if enabled { radius } else { 0. },
    )
    .map_err(str::to_owned)
}
pub fn default_radius(net: &SurfaceNet, binding: &Binding) -> f64 {
    let p = positions(net);
    let mut lengths = edges(binding)
        .iter()
        .map(|&[a, b]| (p[a] - p[b]).magnitude())
        .filter(|d| *d > 0.)
        .collect::<Vec<_>>();
    lengths.sort_by(f64::total_cmp);
    match lengths.len() {
        0 => 0.,
        n if n % 2 == 0 => lengths[n / 2 - 1] + lengths[n / 2],
        n => lengths[n / 2] * 2.,
    }
}

pub fn apply(
    body: &Body,
    g: &Geometry,
    edit: &ComponentEdit,
    operation: &Operation,
    placement: Matrix4,
) -> Result<Geometry, String> {
    let mut created =
        if operation.binding.periodic && matches!(operation.action, Action::Create { .. }) {
            Some(periodic::create(g, edit.element)?)
        } else {
            None
        };
    let f = if operation.binding.periodic {
        let index = *operation
            .binding
            .patches
            .first()
            .ok_or("Missing periodic patches.")?;
        g.shell
            .as_ref()
            .and_then(|s| s.get(index))
            .ok_or("A periodic source patch no longer exists.")?
            .clone()
    } else {
        patch(g, edit.element)?
    };
    let mut surface = if let Some(cage) = &created {
        cage.net.clone()
    } else {
        match &operation.action {
            Action::Create { planar: true } => {
                if operation.binding.disk {
                    disk_net(g, &f)?
                } else {
                    rectangle(&f)?
                }
            }
            _ => net(g, edit.element)?,
        }
    };
    if !operation.binding.disk && !operation.binding.periodic {
        supported(&surface, &f)?;
    }
    let mut binding = created
        .as_ref()
        .map(|c| c.binding.clone())
        .or_else(|| {
            g.cages
                .iter()
                .find(|c| c.domain == operation.domain && c.face == edit.element)
                .map(|c| c.binding.clone())
        })
        .unwrap_or_else(|| Binding::of(&surface));
    binding.disk = operation.binding.disk;
    if binding != operation.binding {
        return Err("An upstream edit changed authored control-node identities, knots or periodic patch bindings.".into());
    }
    if (operation.binding.disk || operation.binding.periodic)
        && !matches!(operation.action, Action::Create { .. })
        && !g
            .cages
            .iter()
            .any(|c| c.domain == operation.domain && c.face == edit.element)
    {
        return Err("The authored cage binding is missing.".into());
    }
    if matches!(operation.action, Action::Create { .. })
        && body
            .component_edits
            .iter()
            .take_while(|e| !std::ptr::eq(*e, edit))
            .any(|e| {
                e.element == edit.element
                    && matches!(
                        e.control.as_ref().map(|o| &o.action),
                        Some(Action::Create { .. })
                    )
            })
    {
        return Err("This face already belongs to an authored control domain.".into());
    }
    if matches!(operation.action, Action::Create { .. })
        && body
            .component_edits
            .iter()
            .take_while(|e| !std::ptr::eq(*e, edit))
            .any(|e| {
                e.control
                    .as_ref()
                    .is_some_and(|o| o.domain == operation.domain)
            })
    {
        return Err("Duplicate authored control-domain identifier.".into());
    }
    if !matches!(operation.action, Action::Create { .. }) {
        let previous = body
            .component_edits
            .iter()
            .take_while(|e| !std::ptr::eq(*e, edit))
            .find(|e| {
                e.control.as_ref().is_some_and(|o| {
                    o.domain == operation.domain && matches!(o.action, Action::Create { .. })
                })
            });
        if previous.is_none_or(|e| {
            e.element != edit.element || e.control.as_ref().unwrap().binding != operation.binding
        }) {
            return Err("The authored control domain is missing; nodes cannot be rebound by proximity or output slot.".into());
        }
    }
    if let Some(mut cage) = created.take() {
        cage.domain = operation.domain;
        let mut result = g.clone();
        let mut roles = g.surface_mapping()?.to_vec();
        for &i in &operation.binding.patches {
            roles[i] = edit.element;
        }
        crate::edge_treatments::merge_surfaces(&mut result, &roles);
        let actual = result.surface_roles[operation.binding.patches[0]];
        if actual != edit.element {
            return Err(
                "This cylinder's logical face order cannot retain the authored domain reference."
                    .into(),
            );
        }
        result.cages.push(cage);
        return Ok(result);
    }
    if operation.binding.disk && matches!(operation.action, Action::Create { .. }) {
        let mut result = g.clone();
        result.cages.push(Cage {
            domain: operation.domain,
            face: edit.element,
            binding: operation.binding.clone(),
            patches: Vec::new(),
            reference: surface.clone(),
            net: surface,
        });
        return Ok(result);
    }
    match &operation.action {
        Action::Translate {
            sources,
            locked,
            enabled,
            radius,
            delta,
        } => {
            let weights = field(
                &surface,
                &operation.binding,
                placement,
                sources,
                locked,
                *enabled,
                *radius,
            )?;
            let local = Vector3::from(*delta);
            for (p, w) in surface.control_points_mut().zip(weights) {
                let d = local * w * p.w;
                p.x += d.x;
                p.y += d.y;
                p.z += d.z;
            }
        }
        Action::MoveFace { delta } => {
            let local = Vector3::from(*delta);
            if operation.binding.periodic {
                return Err("Moving the complete periodic wall would also move both caps. Use Volume movement, or move an end cap to rebuild its neighbors.".into());
            }
            if !g.is_solid && g.face_count == 1 {
                let mut result = g.clone();
                components::map_geometry(&mut result, Matrix4::from_translation(local));
                return Ok(result);
            }
            let boundary = f.boundaries();
            let ring = g
                .circles
                .iter()
                .find(|r| crate::face_extrusions::circle_on_wire(r, &boundary[0]));
            let plane = if let Some(ring) = ring {
                let u = ring.u.normalize();
                let normal = ring.u.cross(ring.v).normalize();
                crate::snapping::Plane {
                    origin: ring.center,
                    u,
                    v: normal.cross(u),
                    normal,
                }
            } else {
                let corners = boundary[0]
                    .iter()
                    .map(|e| e.front().point())
                    .collect::<Vec<_>>();
                if corners.len() < 3 {
                    return Err(
                        "Rigid control-face movement needs a planar boundary section.".into(),
                    );
                }
                let u = (corners[1] - corners[0]).normalize();
                let normal = (corners[1] - corners[0])
                    .cross(corners[corners.len() - 1] - corners[0])
                    .normalize();
                if corners
                    .iter()
                    .any(|p| (*p - corners[0]).dot(normal).abs() > 1e-7)
                {
                    return Err("Rigid control-face movement needs coplanar boundary nodes.".into());
                }
                crate::snapping::Plane {
                    origin: corners[0],
                    u,
                    v: normal.cross(u),
                    normal,
                }
            };
            let mapping = g.surface_mapping()?;
            let selected = components::vertices(g, SelectionMode::Face, Some(edit.element));
            let matrix = crate::section_edits::translation_from_plane(
                g,
                &selected,
                Matrix4::from_translation(local),
                plane,
            )?
            .ok_or(
                "Rigid control-face movement needs a complete cap and fixed opposite section.",
            )?;
            let shell: Shell = builder::transformed(g.shell.as_ref().unwrap(), matrix);
            use truck_modeling::Transformed;
            let translation = Matrix4::from_translation(local);
            let original = g.shell.as_ref().unwrap();
            let mut fixed = Vec::new();
            for (i, (old, new)) in original.iter().zip(&shell).enumerate() {
                if mapping[i] == edit.element {
                    new.set_surface(old.surface().transformed(translation));
                } else if old.edge_iter().all(|e| {
                    [e.front().point(), e.back().point()].iter().all(|p| {
                        !selected
                            .iter()
                            .any(|&j| (g.points[j] - *p).magnitude() < 1e-7)
                    })
                }) {
                    new.set_surface(old.surface());
                    fixed.push(mapping[i]);
                }
            }
            let mut result = finish(g, &shell)?;
            for cage in &mut result.cages {
                if cage.face == edit.element {
                    cage.transform(translation);
                } else if !fixed.contains(&cage.face) {
                    cage.transform(matrix);
                }
            }
            let rings = g
                .circles
                .iter()
                .map(|r| {
                    let mut r = r.clone();
                    if (r.center - plane.origin).dot(plane.normal).abs() < 1e-7 {
                        r.center += local;
                    }
                    r
                })
                .collect::<Vec<_>>();
            result.retain_circles(&rings);
            return Ok(result);
        }
        Action::Create { .. } => (),
    }
    if operation.binding.periodic {
        let mut cage = g
            .cages
            .iter()
            .find(|c| c.domain == operation.domain)
            .ok_or("Missing periodic cage.")?
            .clone();
        cage.net = surface;
        return periodic::apply(g, &cage);
    }
    // Fresh topology prevents surface mutation from corrupting the source/cache on rejection.
    let shell: Shell = builder::transformed(g.shell.as_ref().unwrap(), Matrix4::identity());
    let idx = (0..shell.len())
        .find(|&i| g.surface_roles.get(i).copied().unwrap_or(i) == edit.element)
        .unwrap();
    let mut cage = g
        .cages
        .iter()
        .find(|c| c.domain == operation.domain && c.face == edit.element)
        .cloned();
    if operation.binding.disk {
        let c = cage.as_mut().ok_or("Missing circular cage.")?;
        c.net = surface;
        let skin = disk_surface(c);
        regularity(&skin, true)?;
        shell[idx].set_surface(Surface::NurbsSurface(skin));
    } else {
        regularity(&surface, false)?;
        shell[idx].set_surface(Surface::NurbsSurface(surface));
    }
    let mut result = finish(g, &shell)?;
    if let Some(cage) = cage {
        *result
            .cages
            .iter_mut()
            .find(|c| c.domain == operation.domain)
            .ok_or("Missing circular cage.")? = cage;
    }
    Ok(result)
}
fn finish(g: &Geometry, shell: &Shell) -> Result<Geometry, String> {
    if !shell.is_geometric_consistent() {
        return Err("Control edit would detach a shared trim or create an invalid surface.".into());
    }
    let mut result = crate::model::tessellate(shell)?;
    if g.surface_roles.iter().copied().ne(0..shell.len()) {
        crate::edge_treatments::merge_surfaces(&mut result, &g.surface_roles);
    }
    result.cages = g.cages.clone();
    result.is_solid = g.is_solid;
    result.retain_circles(&g.circles);
    components::weld_geometry(&mut result);
    components::validate_retrim_intersections(&result)?;
    validate_intersections(&result)?;
    if g.is_solid {
        components::solid_parts(shell)?;
    }
    Ok(result)
}

/// Bound the tessellated validation workload; retain the analytic shell as the model.
pub(crate) fn validate_intersections(g: &Geometry) -> Result<(), String> {
    let tris = &g.triangles;
    let error =
        "Control edit folds, collapses or intersects a surface; reduce displacement or radius.";
    let eps = 1e-7;
    let bounds = tris
        .iter()
        .map(|t| {
            std::array::from_fn::<_, 3, _>(|k| {
                [
                    t.iter().map(|p| p[k]).fold(f64::INFINITY, f64::min),
                    t.iter().map(|p| p[k]).fold(f64::NEG_INFINITY, f64::max),
                ]
            })
        })
        .collect::<Vec<_>>();
    let axis = (0..3)
        .max_by(|&a, &b| {
            let extent = |k: usize| {
                bounds
                    .iter()
                    .map(|p| p[k][1])
                    .fold(f64::NEG_INFINITY, f64::max)
                    - bounds.iter().map(|p| p[k][0]).fold(f64::INFINITY, f64::min)
            };
            extent(a).total_cmp(&extent(b))
        })
        .unwrap();
    let mut order = (0..tris.len()).collect::<Vec<_>>();
    order.sort_unstable_by(|&a, &b| bounds[a][axis][0].total_cmp(&bounds[b][axis][0]));
    let mut pairs = 0usize;
    for (slot, &i) in order.iter().enumerate() {
        let a = &tris[i];
        let normal = (a[1] - a[0]).cross(a[2] - a[0]);
        if normal.magnitude2() < 1e-20 {
            return Err(error.into());
        }
        for &j in order.iter().skip(slot + 1) {
            if bounds[j][axis][0] > bounds[i][axis][1] + eps {
                break;
            }
            let b = &tris[j];
            if (0..3).any(|k| {
                bounds[i][k][1] < bounds[j][k][0] - eps || bounds[j][k][1] < bounds[i][k][0] - eps
            }) {
                continue;
            }
            pairs += 1;
            if pairs > 2_000_000 {
                return Err(
                    "Control edit exceeds the overlapping-surface intersection budget.".into(),
                );
            }
            let shared = a
                .iter()
                .filter(|p| b.iter().any(|q| (**p - *q).magnitude() < eps))
                .copied()
                .collect::<Vec<_>>();
            let on_shared = |p: Point3| {
                shared.iter().any(|q| (p - *q).magnitude() < eps)
                    || shared.len() == 2 && {
                        let d = shared[1] - shared[0];
                        let t = (p - shared[0]).dot(d) / d.magnitude2();
                        (-eps..=1. + eps).contains(&t) && (p - shared[0] - d * t).magnitude() < eps
                    }
            };
            for (x, y) in [(a, b), (b, a)] {
                for k in 0..3 {
                    let d = x[(k + 1) % 3] - x[k];
                    if let Some(t) = crate::selection::triangle_hit(x[k], d, y)
                        && t <= 1. + eps
                        && !on_shared(x[k] + d * t)
                    {
                        return Err(error.into());
                    }
                }
            }
            // Ray tests are parallel for coplanar overlap. Test strict interior probes.
            if b.iter()
                .all(|p| (*p - a[0]).dot(normal).abs() < eps * normal.magnitude())
            {
                let inside = |p: Point3, t: &[Point3; 3]| {
                    let u = t[1] - t[0];
                    let v = t[2] - t[0];
                    let n = u.cross(v);
                    let q = p - t[0];
                    let alpha = q.cross(v).dot(n) / n.magnitude2();
                    let beta = u.cross(q).dot(n) / n.magnitude2();
                    alpha > eps && beta > eps && alpha + beta < 1. - eps
                };
                for k in 0..3 {
                    for l in 0..3 {
                        let u = a[(k + 1) % 3] - a[k];
                        let v = b[(l + 1) % 3] - b[l];
                        let q = b[l] - a[k];
                        let den = u.cross(v).dot(normal);
                        if den.abs() > 1e-14 * u.magnitude() * v.magnitude() * normal.magnitude() {
                            let s = q.cross(v).dot(normal) / den;
                            let t = q.cross(u).dot(normal) / den;
                            if s > eps && s < 1. - eps && t > eps && t < 1. - eps {
                                return Err(error.into());
                            }
                        }
                    }
                }
                if inside(a[0] + ((a[1] - a[0]) + (a[2] - a[0])) / 3., b)
                    || inside(b[0] + ((b[1] - b[0]) + (b[2] - b[0])) / 3., a)
                {
                    return Err(error.into());
                }
            }
        }
    }
    Ok(())
}

/// Rest positions must already be in world coordinates. Locks fix positions, not paths.
fn influences(
    rest: &[Point3],
    edges: &[[usize; 2]],
    sources: &[usize],
    locked: &[bool],
    radius: f64,
) -> Result<Vec<f64>, &'static str> {
    if !radius.is_finite()
        || !(0. ..=MAX_RADIUS).contains(&radius)
        || locked.len() != rest.len()
        || rest
            .iter()
            .any(|p| !p.x.is_finite() || !p.y.is_finite() || !p.z.is_finite())
        || edges.iter().flatten().any(|&i| i >= rest.len())
        || sources.iter().any(|&i| i >= rest.len() || locked[i])
    {
        return Err("Invalid control domain, source or radius.");
    }
    let mut adjacency = vec![Vec::new(); rest.len()];
    for &[a, b] in edges {
        let length = (rest[a] - rest[b]).magnitude();
        if !length.is_finite() {
            return Err("Control-edge length is unrepresentable.");
        }
        adjacency[a].push((b, length));
        adjacency[b].push((a, length));
    }
    let mut distance = vec![f64::INFINITY; rest.len()];
    let mut visited = vec![false; rest.len()];
    for &source in sources {
        distance[source] = 0.;
    }
    // Bounded proof uses stdlib Dijkstra without a custom floating-point heap key.
    for _ in rest {
        let next = (0..rest.len())
            .filter(|&i| !visited[i])
            .min_by(|&a, &b| distance[a].total_cmp(&distance[b]));
        let Some(a) = next else { break };
        if !distance[a].is_finite() || distance[a] > radius {
            break;
        }
        visited[a] = true;
        for &(b, length) in &adjacency[a] {
            distance[b] = distance[b].min(distance[a] + length);
        }
    }
    Ok(distance
        .iter()
        .enumerate()
        .map(|(i, &d)| {
            if locked[i] {
                0.
            } else if sources.contains(&i) {
                1.
            } else if radius == 0. || d >= radius {
                0.
            } else {
                let t = d / radius;
                1. - t * t * (3. - 2. * t)
            }
        })
        .collect())
}

#[test]
fn soft_selection_control_net_proof() {
    use truck_modeling::{BSplineSurface, KnotVec, NurbsSurface, Vector3, Vector4};
    let rest: Vec<_> = (0..6).map(|i| Point3::new(i as f64, 0., 0.)).collect();
    let edges = [[0, 1], [1, 2], [2, 3], [3, 4]]; // node 5 is disconnected
    let mut locks = vec![false; 6];
    let w = influences(&rest, &edges, &[0], &locks, 4.).unwrap();
    assert_eq!(w, [1., 0.84375, 0.5, 0.15625, 0., 0.]);
    assert_eq!(
        influences(&rest, &edges, &[0, 4], &locks, 4.).unwrap(),
        [1., 0.84375, 0.5, 0.84375, 1., 0.]
    );
    assert_eq!(
        influences(&rest, &edges, &[0], &locks, 0.).unwrap(),
        [1., 0., 0., 0., 0., 0.]
    );
    locks[1] = true;
    assert_eq!(
        influences(&rest, &edges, &[0], &locks, 4.).unwrap()[1..3],
        [0., 0.5]
    );
    assert!(influences(&rest, &edges, &[1], &locks, 4.).is_err());
    for radius in [-1., f64::NAN, f64::INFINITY] {
        assert!(influences(&rest, &edges, &[0], &locks, radius).is_err());
    }
    assert!(influences(&rest, &[[0, 6]], &[0], &locks, 4.).is_err());
    let mut folded = rest.clone();
    folded[4] = Point3::new(0.01, 0., 0.);
    locks[1] = false;
    assert_eq!(
        influences(&folded, &edges, &[0], &locks, 1.).unwrap()[4],
        0.
    );
    let scaled = rest
        .iter()
        .map(|p| Point3::new(p.x * 2., p.y, p.z))
        .collect::<Vec<_>>();
    assert_eq!(
        influences(&scaled, &edges, &[0], &locks, 4.).unwrap()[1],
        0.5
    );
    let mut dense = Geometry::default();
    for x in 0..50 {
        for y in 0..50 {
            let a = Point3::new(x as f64, y as f64, 0.);
            let b = a + Vector3::unit_x();
            let c = a + Vector3::unit_y();
            let d = b + Vector3::unit_y();
            dense.triangles.extend([[a, b, c], [b, d, c]]);
        }
    }
    validate_intersections(&dense).unwrap();
    dense.triangles = vec![
        [
            Point3::new(0., 0., 0.),
            Point3::new(10., 0., 0.),
            Point3::new(0., 1., 0.),
        ],
        [
            Point3::new(1., -10., 0.),
            Point3::new(2., -10., 0.),
            Point3::new(1., 10., 0.),
        ],
    ];
    assert!(
        validate_intersections(&dense).is_err(),
        "Coplanar crossings must reject even when neither centroid is inside the other triangle"
    );
    let knots = KnotVec::bezier_knot(2);
    let controls = (0..3)
        .map(|u| {
            (0..3)
                .map(|v| {
                    let weight = if u == 1 && v == 1 { 2. } else { 1. };
                    Vector4::new(u as f64 * weight, v as f64 * weight, 0., weight)
                })
                .collect()
        })
        .collect();
    let surface = NurbsSurface::new(BSplineSurface::new((knots.clone(), knots), controls));
    let preview = |delta: Vector3| {
        let mut result = surface.clone();
        // Homogeneous coordinates: multiply positional displacement by rational weight.
        let p = result.control_point_mut(1, 1);
        p.x += delta.x * p.w;
        p.y += delta.y * p.w;
        p.z += delta.z * p.w;
        result
    };
    let moved = preview(Vector3::unit_z());
    assert!(moved.subs(0.5, 0.5).z > surface.subs(0.5, 0.5).z);
    assert_eq!(moved.degrees(), surface.degrees());
    for i in 0..6 {
        assert_eq!(moved.uknot(i), surface.uknot(i));
        assert_eq!(moved.vknot(i), surface.vknot(i));
    }
    assert_eq!(moved.control_point(1, 1).w, 2.);
    for _ in 0..100 {
        assert_eq!(preview(Vector3::unit_z()), moved);
        assert_eq!(preview(Vector3::new(0., 0., 0.)), surface);
    }
    // Clamped patch boundary remains fixed; no claim about general solid trims.
    for i in 0..=10 {
        let t = i as f64 / 10.;
        for (u, v) in [(0., t), (1., t), (t, 0.), (t, 1.)] {
            assert!((moved.subs(u, v) - surface.subs(u, v)).magnitude() < 1e-12);
        }
    }
}

#[test]
fn soft_selection_authored_history() {
    use crate::{
        model::{self, Model, Shape},
        project::Project,
        viewport::Camera,
    };
    for solid in [false, true] {
        let mut body = Body::new(Shape::Rectangle);
        body.id = 1;
        body.width = 4.;
        body.depth = 4.;
        if solid {
            body.extrusion = 3.;
        }
        let base = model::evaluate(&body).unwrap();
        let face = if solid {
            (0..base.face_count)
                .find(|&f| {
                    crate::edge_treatments::target(&body, &base, f)
                        == Ok(crate::edge_treatments::FaceTarget::Top)
                })
                .unwrap()
        } else {
            0
        };
        if solid {
            let volume = |g: &Geometry| {
                g.triangles
                    .iter()
                    .map(|t| t[0].to_vec().dot(t[1].to_vec().cross(t[2].to_vec())) / 6.)
                    .sum::<f64>()
            };
            for f in 0..base.face_count {
                let created = create(&body, &base, f).unwrap();
                let out = model::evaluate(&created).unwrap();
                assert!((volume(&base) - volume(&out)).abs() < 1e-7);
                assert_eq!(out.topology, base.topology);
            }
        }
        let cage = create(&body, &base, face).unwrap();
        let geometry = model::evaluate(&cage).unwrap();
        if solid {
            let other = (face + 1) % base.face_count;
            let both = create(&cage, &geometry, other).unwrap();
            let both_geometry = model::evaluate(&both).unwrap();
            let other_before = net(&both_geometry, other).unwrap();
            let mut editor = Editor::new(&both, &both_geometry, face).unwrap();
            editor.sources = vec![[2, 2]];
            let edited = editor
                .translated(&both, &both_geometry, [0., 0., 0.2])
                .unwrap();
            assert_eq!(
                net(&model::evaluate(&edited).unwrap(), other).unwrap(),
                other_before
            );
        }
        assert_eq!(geometry.topology, base.topology);
        let domain = domain(&cage, face).unwrap();
        let operation = Operation {
            action: Action::Translate {
                sources: vec![[2, 2]],
                locked: vec![],
                enabled: true,
                radius: 3.,
                delta: [0., 0., 0.5],
            },
            ..domain.clone()
        };
        let deformed = append(&cage, &geometry, face, operation).unwrap();
        let result = model::evaluate(&deformed).unwrap();
        assert_eq!(result.is_solid, solid);
        assert!(result.shell.as_ref().unwrap().is_geometric_consistent());
        assert_eq!(
            net(&result, face).unwrap().control_point(2, 2).z,
            net(&geometry, face).unwrap().control_point(2, 2).z + 0.5
        );
        let mut history = Model::from_bodies(vec![cage.clone()]);
        history
            .commit_changes(std::slice::from_ref(&deformed))
            .unwrap();
        history.undo();
        assert_eq!(history.bodies, std::slice::from_ref(&cage));
        history.redo();
        assert_eq!(history.bodies, std::slice::from_ref(&deformed));
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("control.osk");
        let project = Project::new(vec![deformed.clone()], Camera::default(), true, None);
        project.save(&path).unwrap();
        assert_eq!(Project::load(&path).unwrap().bodies, [deformed]);
        let mut old = serde_json::to_value(&project).unwrap();
        old["schema_version"] = 41.into();
        assert!(
            serde_json::from_value::<Project>(old)
                .unwrap()
                .validate()
                .is_err()
        );
        for format in crate::export::Format::ALL {
            if solid || format != crate::export::Format::Stl {
                assert!(
                    !crate::export::encode(&project.bodies, format)
                        .unwrap()
                        .is_empty()
                );
            }
        }
    }
}

/// Transient editor preferences; each authored operation owns a separate settings snapshot.
#[derive(Clone)]
pub struct Editor {
    pub binding: Binding,
    pub body: u64,
    pub face: usize,
    pub sources: Vec<[usize; 2]>,
    pub locked: Vec<[usize; 2]>,
    pub enabled: bool,
    pub radius: f64,
    pub delta: [f64; 3],
}
impl Editor {
    pub fn new(body: &Body, geometry: &Geometry, face: usize) -> Result<Self, String> {
        let binding = domain(body, face)
            .ok_or("Create a control net first.")?
            .binding
            .clone();
        let surface = net(geometry, face)?;
        Ok(Self {
            body: body.id,
            face,
            sources: vec![],
            locked: vec![],
            enabled: true,
            radius: default_radius(&surface, &binding),
            binding,
            delta: [0.; 3],
        })
    }
    pub fn translated(
        &self,
        body: &Body,
        geometry: &Geometry,
        delta: [f64; 3],
    ) -> Result<Body, String> {
        if delta == [0.; 3] {
            return Ok(body.clone());
        }
        let mut operation = domain(body, self.face)
            .ok_or("The authored control domain no longer exists.")?
            .clone();
        operation.action = Action::Translate {
            sources: self.sources.clone(),
            locked: self.locked.clone(),
            enabled: self.enabled,
            radius: self.radius,
            delta: crate::regions::transform(body)
                .invert()
                .ok_or("Invalid object placement.")?
                .transform_vector(Vector3::from(delta))
                .into(),
        };
        append(body, geometry, self.face, operation)
    }
    pub fn pick(
        &mut self,
        geometry: &Geometry,
        query: crate::selection::Query<'_>,
        camera: &crate::viewport::Camera,
        rect: eframe::egui::Rect,
        pointer: eframe::egui::Pos2,
        extend: bool,
    ) {
        let Ok(surface) = net(geometry, self.face) else {
            return;
        };
        let binding = &self.binding;
        let locked = locks(binding, &self.locked);
        let hit = positions(&surface)
            .iter()
            .enumerate()
            .filter(|(i, p)| !locked[*i] && query.visible(camera, **p, rect))
            .filter_map(|(i, p)| {
                let (screen, depth) = camera.project(*p, rect);
                let d = screen.distance(pointer);
                (d <= 9.).then_some((i, d, depth))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1).then_with(|| b.2.total_cmp(&a.2)))
            .map(|(i, _, _)| [i / binding.columns, i % binding.columns]);
        if !extend {
            self.sources.clear();
        }
        if let Some(id) = hit {
            if extend && self.sources.contains(&id) {
                self.sources.retain(|p| *p != id);
            } else {
                self.sources.push(id);
            }
        }
    }
    pub fn pivot(&self, geometry: &Geometry) -> Option<Point3> {
        if self.sources.is_empty() {
            return None;
        }
        let surface = net(geometry, self.face).ok()?;
        let columns = surface.control_points()[0].len();
        let p = positions(&surface);
        let sum = self
            .sources
            .iter()
            .try_fold(Vector3::new(0., 0., 0.), |sum, &[u, v]| {
                p.get(u * columns + v).map(|p| sum + p.to_vec())
            })?;
        Some(Point3::from_vec(sum / self.sources.len() as f64))
    }
    pub fn draw(
        &self,
        geometry: &Geometry,
        query: crate::selection::Query<'_>,
        camera: &crate::viewport::Camera,
        rect: eframe::egui::Rect,
        ui: &mut eframe::egui::Ui,
        frozen: Option<&[f64]>,
    ) {
        use eframe::egui::{self, Color32, Stroke};
        let Ok(surface) = net(geometry, self.face) else {
            return;
        };
        let binding = &self.binding;
        let points = positions(&surface);
        let field = field(
            &surface,
            &self.binding,
            Matrix4::identity(),
            &self.sources,
            &self.locked,
            self.enabled,
            self.radius,
        )
        .unwrap_or_else(|_| vec![0.; points.len()]);
        let weights = frozen.unwrap_or(&field);
        let locked = locks(binding, &self.locked);
        let painter = ui.painter().with_clip_rect(rect);
        for [a, b] in edges(binding) {
            for i in 0..8 {
                let p = points[a] + (points[b] - points[a]) * (i as f64 / 8.);
                let q = points[a] + (points[b] - points[a]) * ((i + 1) as f64 / 8.);
                if query.visible(camera, p + (q - p) * 0.5, rect) {
                    painter.line_segment(
                        [camera.project(p, rect).0, camera.project(q, rect).0],
                        Stroke::new(1., Color32::from_gray(145)),
                    );
                }
            }
        }
        for (i, p) in points.iter().enumerate() {
            if !query.visible(camera, *p, rect) {
                continue;
            }
            let screen = camera.project(*p, rect).0;
            let id = [i / binding.columns, i % binding.columns];
            let selected = self.sources.contains(&id);
            let w = weights.get(i).copied().unwrap_or(0.);
            let color = if locked[i] {
                Color32::from_gray(100)
            } else if w == 0. {
                Color32::from_rgb(65, 90, 145)
            } else {
                heat(w)
            };
            painter.circle_filled(screen, if selected { 5. } else { 3.5 }, color);
            if selected {
                painter.rect_stroke(
                    egui::Rect::from_center_size(screen, egui::vec2(13., 13.)),
                    1.,
                    Stroke::new(1.5, Color32::WHITE),
                    egui::StrokeKind::Outside,
                );
            }
            if locked[i] {
                painter.line_segment(
                    [screen - egui::vec2(2., 2.), screen + egui::vec2(2., 2.)],
                    Stroke::new(1., Color32::WHITE),
                );
            }
            if ui.rect_contains_pointer(egui::Rect::from_center_size(screen, egui::vec2(14., 14.)))
            {
                painter.text(
                    screen + egui::vec2(10., -12.),
                    egui::Align2::LEFT_BOTTOM,
                    format!(
                        "Node {},{} · {:.1}%{}",
                        id[0] + 1,
                        id[1] + 1,
                        w * 100.,
                        if locked[i] { " · locked" } else { "" }
                    ),
                    egui::FontId::proportional(12.),
                    Color32::WHITE,
                );
            }
            if selected && self.enabled && self.radius > 0. {
                painter.circle_stroke(
                    screen,
                    (self.radius * rect.height() as f64 / camera.span as f64) as f32,
                    Stroke::new(0.5, Color32::from_gray(110)),
                );
            }
        }
    }
}
fn heat(weight: f64) -> eframe::egui::Color32 {
    let stops = [
        [40., 90., 240.],
        [0., 220., 240.],
        [35., 205., 75.],
        [250., 220., 30.],
        [245., 65., 45.],
    ];
    let t = weight.clamp(0., 1.) * 4.;
    let i = (t as usize).min(3);
    let f = t - i as f64;
    let c = std::array::from_fn::<_, 3, _>(|axis| {
        (stops[i][axis] * (1. - f) + stops[i + 1][axis] * f) as u8
    });
    eframe::egui::Color32::from_rgb(c[0], c[1], c[2])
}

#[test]
fn soft_selection_rigid_face_and_invalid_rollback() {
    use crate::model::{self, Model, Shape};
    for affine in [false, true] {
        let mut body = Body::new(Shape::Rectangle);
        body.id = 1;
        body.width = 4.;
        body.depth = 4.;
        body.extrusion = 3.;
        if affine {
            body.position = [2., -3., 1.];
            body.rotation = [15., 25., 35.];
            body.scale = [1.4, 0.7, 1.2];
        }
        let g = model::evaluate(&body).unwrap();
        let face = (0..g.face_count)
            .find(|&f| {
                crate::edge_treatments::target(&body, &g, f)
                    == Ok(crate::edge_treatments::FaceTarget::Top)
            })
            .unwrap();
        let cage = create(&body, &g, face).unwrap();
        let g = model::evaluate(&cage).unwrap();
        let mut editor = Editor::new(&cage, &g, face).unwrap();
        editor.sources = vec![[2, 2]];
        editor.radius = 3.;
        let deformed = editor.translated(&cage, &g, [0., 0., 0.4]).unwrap();
        let g = model::evaluate(&deformed).unwrap();
        let controls = positions(&net(&g, face).unwrap());
        let before = positions(&net(&model::evaluate(&cage).unwrap(), face).unwrap());
        assert!((controls[12] - before[12] - Vector3::new(0., 0., 0.4)).magnitude() < 1e-8);
        // Volume rotation carries the authored deformation; radius is invariant under rigid placement.
        let rotation = Matrix4::from_angle_z(truck_modeling::Rad(0.4));
        let rotated =
            components::transformed(&deformed, &g, SelectionMode::Volume, None, rotation).unwrap();
        let rotated_controls = positions(&net(&model::evaluate(&rotated).unwrap(), face).unwrap());
        for (a, b) in controls.iter().zip(rotated_controls) {
            assert!((rotation.transform_point(*a) - b).magnitude() < 1e-8);
        }

        let delta = Vector3::new(0.2, -0.1, 0.3);
        let moved = components::transformed(
            &deformed,
            &g,
            SelectionMode::Face,
            Some(face),
            Matrix4::from_translation(delta),
        )
        .unwrap();
        let result = model::evaluate(&moved).unwrap();
        let next = positions(&net(&result, face).unwrap());
        for (a, b) in controls.iter().zip(&next) {
            assert!((*b - *a - delta).magnitude() < 1e-8);
        }
        assert!(result.shell.as_ref().unwrap().is_geometric_consistent());
        let mut model = Model::from_bodies(vec![deformed.clone()]);
        let mut invalid = deformed.clone();
        if let Action::Translate { delta, .. } = &mut invalid
            .component_edits
            .last_mut()
            .unwrap()
            .control
            .as_mut()
            .unwrap()
            .action
        {
            *delta = [0., 0., -100.];
        }
        assert!(model.commit_changes(&[invalid]).is_err());
        assert_eq!(model.bodies, std::slice::from_ref(&deformed));
        // Removing the creation feature never rebinds its selected node coordinates.
        let mut missing = deformed.clone();
        missing.component_edits.remove(0);
        assert!(model.commit_changes(&[missing]).is_err());
        assert_eq!(model.bodies, std::slice::from_ref(&deformed));
        // No-op numeric/gesture preview creates no authored operation.
        assert_eq!(editor.translated(&deformed, &g, [0.; 3]).unwrap(), deformed);
    }
}

#[test]
fn circular_cages_keep_exact_rims_and_editable_history() {
    use crate::{
        model::{self, Model, Shape},
        project::Project,
        viewport::Camera,
    };
    use truck_modeling::ParametricSurface3D;
    for affine in [false, true] {
        let mut source = Body::new(Shape::Circle);
        source.id = 1;
        source.width = 4.;
        source.extrusion = 3.;
        if affine {
            source.position = [2., -3., 1.];
            source.rotation = [15., 25., 35.];
            source.scale = [1.4, 0.7, 1.2];
        }
        let base = model::evaluate(&source).unwrap();
        let face = (0..base.face_count)
            .find(|&f| {
                crate::edge_treatments::target(&source, &base, f)
                    == Ok(crate::edge_treatments::FaceTarget::Top)
            })
            .unwrap();
        let bound = create(&source, &base, face).unwrap();
        let start = model::evaluate(&bound).unwrap();
        assert!(domain(&bound, face).unwrap().binding.disk);
        assert_eq!(start.cages.len(), 1);
        let mut editor = Editor::new(&bound, &start, face).unwrap();
        editor.sources = vec![[2, 2]];
        editor.radius = 2.5;
        let before = positions(&net(&start, face).unwrap());
        let delta = Vector3::new(0.1, -0.05, 0.6);
        let changed = editor.translated(&bound, &start, delta.into()).unwrap();
        let out = model::evaluate(&changed).unwrap();
        let after = positions(&net(&out, face).unwrap());
        assert!((after[12] - before[12] - delta).magnitude() < 1e-8);
        assert_eq!(out.topology, start.topology);
        assert!(out.shell.as_ref().unwrap().is_geometric_consistent());
        let Surface::NurbsSurface(skin) = patch(&out, face).unwrap().surface() else {
            panic!("Deformation must create a calculated NURBS skin")
        };
        assert_eq!(skin.degrees(), (8, 8));
        let cage = &out.cages[0];
        for i in 0..64 {
            let angle = i as f64 * std::f64::consts::TAU / 64.;
            let u = 0.5 + 0.5 * angle.cos();
            let v = 0.5 + 0.5 * angle.sin();
            assert!(
                (skin.subs(u, v) - cage.reference.subs(u, v)).magnitude() < 1e-8,
                "The exact circular rim stays shared"
            );
            assert!(
                skin.normal(u, v).dot(cage.reference.normal(u, v)) > 1. - 1e-8,
                "The circular binding has no normal discontinuity at its rim"
            );
        }
        assert!((skin.subs(0.5, 0.5) - cage.reference.subs(0.5, 0.5)).magnitude() > 0.05);
        let shift = Vector3::new(0.1, 0., 0.2);
        let moved = components::transformed(
            &changed,
            &out,
            SelectionMode::Face,
            Some(face),
            Matrix4::from_translation(shift),
        )
        .unwrap();
        let moved_geometry = model::evaluate(&moved).unwrap();
        let Surface::NurbsSurface(moved_skin) = patch(&moved_geometry, face).unwrap().surface()
        else {
            panic!("Moved skin remains NURBS")
        };
        for (u, v) in [(0.2, 0.4), (0.5, 0.5), (0.7, 0.7)] {
            assert!((moved_skin.subs(u, v) - skin.subs(u, v) - shift).magnitude() < 1e-8);
        }
        for (a, b) in out.cages[0].net.control_points().iter().flatten().zip(
            moved_geometry.cages[0]
                .net
                .control_points()
                .iter()
                .flatten(),
        ) {
            assert!((b.truncate() - a.truncate() - shift).magnitude() < 1e-8);
        }
        assert_eq!(moved_skin.knot_vecs(), skin.knot_vecs());
        assert!(
            moved_geometry
                .shell
                .as_ref()
                .unwrap()
                .is_geometric_consistent()
        );
        let mut model = Model::from_bodies(vec![bound.clone()]);
        model.commit(changed.clone()).unwrap();
        model.undo();
        assert_eq!(model.bodies, std::slice::from_ref(&bound));
        model.redo();
        assert_eq!(model.bodies, std::slice::from_ref(&changed));
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("disk.osk");
        let project = Project::new(vec![changed.clone()], Camera::default(), false, None);
        project.save(&path).unwrap();
        assert_eq!(
            Project::load(&path).unwrap().bodies,
            std::slice::from_ref(&changed)
        );
        let mut old = serde_json::to_value(&project).unwrap();
        old["modeling_version"] = 43.into();
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
