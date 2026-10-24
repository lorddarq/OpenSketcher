use serde::{Deserialize, Serialize};
use truck_meshalgo::tessellation::MeshableShape;
use truck_modeling::{
    BoundedCurve, EuclideanSpace, InnerSpace, ParametricCurve, Point3, Rad, Shell, Solid,
    Transform, Vector3, Wire, builder,
};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Shape {
    Rectangle,
    Circle,
    Line,
    Semicircle,
}
impl Shape {
    pub fn label(self) -> &'static str {
        match self {
            Self::Rectangle => "Rectangle",
            Self::Circle => "Circle",
            Self::Line => "Line",
            Self::Semicircle => "Semicircle",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Body {
    pub id: u64,
    pub shape: Shape,
    pub width: f64,
    pub depth: f64,
    pub extrusion: f64,
    #[serde(default)]
    pub cuts_profiles: bool,
    #[serde(default)]
    pub construction: bool,
    pub position: [f64; 3],
    pub rotation: [f64; 3],
    pub scale: [f64; 3],
    #[serde(default)]
    pub face_extrusions: Vec<crate::face_extrusions::Extrusion>,
    #[serde(default)]
    pub component_edits: Vec<crate::components::ComponentEdit>,
    #[serde(default, alias = "modifiers")]
    pub edge_treatments: Vec<crate::edge_treatments::Modifier>,
    #[serde(default)]
    pub taper: Option<crate::drafting::Draft>,
    #[serde(default)]
    pub hollow: Option<crate::shelling::Hollow>,
    #[serde(default)]
    pub bore: Option<crate::operators::Bore>,
    #[serde(default)]
    pub boolean: Option<crate::booleans::Boolean>,
}
impl Body {
    pub fn new(shape: Shape) -> Self {
        Self {
            id: 0,
            shape,
            width: 2.,
            depth: 2.,
            extrusion: 0.,
            cuts_profiles: false,
            construction: false,
            position: [0.; 3],
            rotation: [0.; 3],
            scale: [1.; 3],
            face_extrusions: Vec::new(),
            component_edits: Vec::new(),
            edge_treatments: Vec::new(),
            taper: None,
            hollow: None,
            bore: None,
            boolean: None,
        }
    }
    pub fn from_points(shape: Shape, a: [f64; 3], b: [f64; 3]) -> Result<Self, String> {
        let dx = b[0] - a[0];
        let dy = b[1] - a[1];
        let mut body = Self::new(shape);
        body.position = [(a[0] + b[0]) / 2., (a[1] + b[1]) / 2., 0.];
        match shape {
            Shape::Rectangle => {
                body.width = dx.abs();
                body.depth = dy.abs();
            }
            Shape::Circle => {
                body.position = a;
                body.width = 2. * dx.hypot(dy);
            }
            Shape::Line | Shape::Semicircle => {
                body.width = dx.hypot(dy);
                body.rotation[2] = dy.atan2(dx).to_degrees();
            }
        }
        body.validate()?;
        Ok(body)
    }
    pub fn label(&self) -> String {
        let name = self
            .boolean
            .as_ref()
            .map_or(self.shape.label(), |op| op.kind.label());
        let name = if self.construction {
            "Construction plane"
        } else {
            name
        };
        if self.id == 0 {
            format!("New {name}")
        } else {
            format!("{name} {}", self.id)
        }
    }
    pub(crate) fn error(&self, reason: impl std::fmt::Display) -> String {
        format!("{}: {reason}", self.label())
    }
    pub fn validate(&self) -> Result<(), String> {
        (|| {
            if self.construction && (self.shape!=Shape::Rectangle || self.extrusion!=0. || self.cuts_profiles
                || self.boolean.is_some() || self.bore.is_some() || self.hollow.is_some() || self.taper.is_some()
                || !self.component_edits.is_empty() || !self.edge_treatments.is_empty() || !self.face_extrusions.is_empty()) {
                return Err("Construction planes are rectangular references; move/rotate the whole reference instead of applying geometry operations.".into());
            }
            if let Some(operation) = &self.boolean {
                operation
                    .validate(self)
                    .map_err(|e| format!("{}: {e}", operation.references()))?;
            }
            if let Some(bore) = &self.bore {
                bore.validate(self).map_err(|e| {
                    format!(
                        "{} · target body {} / {}: {e}",
                        bore.mode.label(),
                        bore.target,
                        bore.face.label()
                    )
                })?;
            }
            crate::drafting::validate(self)?;
            crate::shelling::validate(self)?;
            crate::face_extrusions::validate(self)?;
            crate::components::validate(&self.component_edits)?;
            crate::edge_treatments::validate(self)?;
            if ![self.width, self.depth, self.extrusion]
                .into_iter()
                .chain(self.position)
                .chain(self.rotation)
                .chain(self.scale)
                .all(f64::is_finite)
            {
                return Err("Parameters must be finite numbers.".into());
            }
            if !(0.01..=100.).contains(&self.width)
                || !(0.01..=100.).contains(&self.depth)
                || !(0.0..=100.).contains(&self.extrusion)
                || (self.extrusion > 0. && self.extrusion < 0.01)
            {
                return Err(
                    "Dimensions must be 0.01–100 units; extrusion may also be zero.".into(),
                );
            }
            if self.scale.iter().any(|v| !(0.01..=100.).contains(v))
                || self.position.iter().any(|v| v.abs() > 1000.)
                || self.rotation.iter().any(|v| v.abs() > 36000.)
            {
                return Err("Scale must be 0.01–100; position ±1000; rotation ±36000°.".into());
            }
            Ok(())
        })()
        .map_err(|reason: String| self.error(reason))
    }
}

/// Exact circular source boundary after placement (an ellipse under nonuniform scale).
#[derive(Clone, Debug)]
pub struct CircularEdge {
    pub center: Point3,
    pub u: Vector3,
    pub v: Vector3,
}

#[derive(Clone, Default)]
pub struct Geometry {
    pub triangles: Vec<[Point3; 3]>,
    pub edges: Vec<[Point3; 2]>,
    pub triangle_faces: Vec<usize>,
    pub edge_groups: Vec<usize>,
    pub points: Vec<Point3>,
    pub face_count: usize,
    pub edge_count: usize,
    pub topology: crate::components::Topology,
    pub circles: Vec<CircularEdge>,
    pub shell: Option<Shell>,
    pub surface_roles: Vec<usize>,
    pub is_solid: bool,
    // Modeling aliases for internal patches, independent of persisted kernel guards.
    pub face_groups: Vec<usize>,
    pub cylinder_supports: Vec<Option<(usize, Vector3)>>,
    pub edge_aliases: Vec<usize>,
    pub internal_edges: std::collections::HashSet<usize>,
}

impl Geometry {
    /// Disposable inference metadata: require a complete retained trim loop,
    /// never just proximity to the tessellated display edges.
    pub(crate) fn retain_circles(&mut self, candidates: &[CircularEdge]) {
        let Some(shell) = &self.shell else {
            self.circles.clear();
            return;
        };
        let loops: Vec<_> = shell
            .iter()
            .flat_map(|face| face.boundaries())
            .chain(shell.extract_boundaries())
            .collect();
        let mut retained = Vec::<CircularEdge>::new();
        for ring in self.circles.iter().chain(candidates) {
            let uu = ring.u.dot(ring.u);
            let uv = ring.u.dot(ring.v);
            let vv = ring.v.dot(ring.v);
            let det = uu * vv - uv * uv;
            if det <= 1e-20 {
                continue;
            }
            let duplicate = retained.iter().any(|old| {
                (old.center - ring.center).magnitude() < 1e-6
                    && (0..3).all(|i| {
                        (0..3).all(|j| {
                            let a = old.u[i] * old.u[j] + old.v[i] * old.v[j];
                            let b = ring.u[i] * ring.u[j] + ring.v[i] * ring.v[j];
                            (a - b).abs() < 1e-8 * (1. + a.abs().max(b.abs()))
                        })
                    })
            });
            if duplicate {
                continue;
            }
            let matches = loops.iter().any(|wire| {
                if !wire.is_closed() {
                    return false;
                }
                let mut previous: Option<[f64; 2]> = None;
                let mut winding = 0.;
                for edge in wire {
                    let curve = edge.oriented_curve();
                    let (lo, hi) = curve.range_tuple();
                    for n in 0..=16 {
                        let p = curve.subs(lo + (hi - lo) * n as f64 / 16.);
                        let delta = p - ring.center;
                        let a = (delta.dot(ring.u) * vv - delta.dot(ring.v) * uv) / det;
                        let b = (delta.dot(ring.v) * uu - delta.dot(ring.u) * uv) / det;
                        let radius = a.hypot(b);
                        if radius < 1e-12
                            || (delta - ring.u * (a / radius) - ring.v * (b / radius)).magnitude()
                                > 1e-6
                        {
                            return false;
                        }
                        let current = [a / radius, b / radius];
                        if let Some(last) = previous {
                            winding += (last[0] * current[1] - last[1] * current[0])
                                .atan2(last[0] * current[0] + last[1] * current[1]);
                        }
                        previous = Some(current);
                    }
                }
                (winding.abs() - std::f64::consts::TAU).abs() < 1e-5
            });
            if matches {
                retained.push(ring.clone());
            }
        }
        self.circles = retained;
    }

    pub fn face_group(&self, face: usize) -> usize {
        self.face_groups.get(face).copied().unwrap_or(face)
    }
    pub fn face_elements(&self, face: usize) -> Vec<usize> {
        (0..self.face_count)
            .filter(|other| self.face_group(*other) == self.face_group(face))
            .collect()
    }
    pub fn edge_group(&self, edge: usize) -> usize {
        self.edge_aliases.get(edge).copied().unwrap_or(edge)
    }
    pub fn edge_elements(&self, edge: usize) -> Vec<usize> {
        (0..self.edge_count)
            .filter(|other| {
                !self.internal_edges.contains(other)
                    && self.edge_group(*other) == self.edge_group(edge)
            })
            .collect()
    }
    fn group_cylinder_patches(&mut self) {
        self.face_groups = (0..self.face_count).collect();
        self.edge_aliases = (0..self.edge_count).collect();
        self.internal_edges.clear();
        self.cylinder_supports.clear();
        let Some(shell) = &self.shell else {
            return;
        };
        let Ok(mapping) = self.surface_mapping() else {
            return;
        };
        let mut curved = vec![true; self.face_count];
        for (face, role) in shell.iter().zip(mapping) {
            if matches!(face.surface(), truck_modeling::Surface::Plane(_)) {
                curved[*role] = false;
            }
        }
        let mut cylinder = vec![None; self.face_count];
        for (face, is_curved) in curved.iter().enumerate() {
            if !is_curved {
                continue;
            }
            let triangles: Vec<_> = self
                .triangles
                .iter()
                .zip(&self.triangle_faces)
                .filter(|(_, f)| **f == face)
                .map(|(t, _)| t)
                .collect();
            if triangles.is_empty() {
                continue;
            }
            cylinder[face] = self
                .circles
                .iter()
                .enumerate()
                .find_map(|(ring_index, ring)| {
                    let uu = ring.u.dot(ring.u);
                    let uv = ring.u.dot(ring.v);
                    let vv = ring.v.dot(ring.v);
                    let det = uu * vv - uv * uv;
                    if det <= 1e-20 {
                        return None;
                    }
                    let normal = ring.u.cross(ring.v).normalize();
                    // Parallel translated caps can form an oblique cylinder.
                    // Prove the support against all samples rather than assuming
                    // the extrusion axis is perpendicular to the rim plane.
                    let mut axes = vec![normal];
                    for other in &self.circles {
                        if (other.u - ring.u).magnitude() > 1e-7
                            || (other.v - ring.v).magnitude() > 1e-7
                        {
                            continue;
                        }
                        let delta = other.center - ring.center;
                        let height = delta.dot(normal);
                        if height.abs() < 1e-7 {
                            continue;
                        }
                        let axis = delta.normalize() * height.signum();
                        if !axes.iter().any(|other| (*other - axis).magnitude() < 1e-10) {
                            axes.push(axis);
                        }
                    }
                    // A display triangle may connect three angular samples at
                    // different heights. Its chord normal need not be radial.
                    axes.iter()
                        .position(|axis| {
                            let heights = triangles
                                .iter()
                                .flat_map(|t| t.iter())
                                .map(|p| (*p - ring.center).dot(normal))
                                .fold([f64::INFINITY, f64::NEG_INFINITY], |[lo, hi], v| {
                                    [lo.min(v), hi.max(v)]
                                });
                            heights[1] - heights[0] > 1e-7
                                && triangles.iter().flat_map(|t| t.iter()).all(|p| {
                                    let d = *p - ring.center;
                                    let radial = d - *axis * (d.dot(normal) / axis.dot(normal));
                                    let a =
                                        (radial.dot(ring.u) * vv - radial.dot(ring.v) * uv) / det;
                                    let b =
                                        (radial.dot(ring.v) * uu - radial.dot(ring.u) * uv) / det;
                                    (a * a + b * b - 1.).abs() < 1e-7
                                })
                        })
                        .map(|axis| (ring_index, axes[axis]))
                });
        }
        self.cylinder_supports = cylinder.clone();
        let mut owners = vec![Vec::new(); self.edge_count];
        for (face, edges) in self.topology.faces.iter().enumerate() {
            for edge in edges {
                owners[*edge].push(face);
            }
        }
        for (edge, faces) in owners.iter().enumerate() {
            if faces.len() != 2
                || cylinder[faces[0]].is_none()
                || cylinder[faces[0]] != cylinder[faces[1]]
            {
                continue;
            }
            let a = self.face_groups[faces[0]];
            let b = self.face_groups[faces[1]];
            for group in &mut self.face_groups {
                if *group == a || *group == b {
                    *group = a.min(b);
                }
            }
            self.internal_edges.insert(edge);
        }
        // Circular rim segments are one modeling edge, not independent arcs.
        for ring in &self.circles {
            let mut valid = vec![true; self.edge_count];
            let mut counts = vec![0; self.edge_count];
            let uu = ring.u.dot(ring.u);
            let uv = ring.u.dot(ring.v);
            let vv = ring.v.dot(ring.v);
            let det = uu * vv - uv * uv;
            if det <= 1e-20 {
                continue;
            }
            for (edge, group) in self.edges.iter().zip(&self.edge_groups) {
                if self.internal_edges.contains(group) {
                    continue;
                }
                counts[*group] += 1;
                valid[*group] &= edge.iter().all(|point| {
                    let d = *point - ring.center;
                    let a = (d.dot(ring.u) * vv - d.dot(ring.v) * uv) / det;
                    let b = (d.dot(ring.v) * uu - d.dot(ring.u) * uv) / det;
                    (a * a + b * b - 1.).abs() < 1e-7
                        && (d - ring.u * a - ring.v * b).magnitude() < 1e-7
                });
            }
            let mut vertices = std::collections::HashMap::<usize, Vec<usize>>::new();
            for edge in (0..self.edge_count).filter(|edge| valid[*edge] && counts[*edge] > 1) {
                for vertex in self.topology.edges[edge] {
                    vertices.entry(vertex).or_default().push(edge);
                }
            }
            for edges in vertices.into_values() {
                for pair in edges.windows(2) {
                    let a = self.edge_aliases[pair[0]];
                    let b = self.edge_aliases[pair[1]];
                    for alias in &mut self.edge_aliases {
                        if *alias == a || *alias == b {
                            *alias = a.min(b);
                        }
                    }
                }
            }
        }
    }

    /// Conservative analytic support interval; no viewport samples determine a cut depth.
    pub(crate) fn projection_range(&self, axis: Vector3) -> Result<[f64; 2], String> {
        use truck_modeling::{Curve, Surface};
        fn range(values: impl Iterator<Item = f64>) -> Result<[f64; 2], String> {
            let mut bounds = [f64::INFINITY, f64::NEG_INFINITY];
            for value in values {
                if !value.is_finite() {
                    return Err("Cannot bound nonfinite geometry.".into());
                }
                bounds = [bounds[0].min(value), bounds[1].max(value)];
            }
            if bounds.iter().any(|v| !v.is_finite()) {
                Err("Cannot bound nonfinite or empty geometry.".into())
            } else {
                Ok(bounds)
            }
        }
        fn rational(
            points: impl Iterator<Item = truck_modeling::Vector4>,
            axis: Vector3,
        ) -> Result<[f64; 2], String> {
            let mut values = Vec::new();
            for p in points {
                if !p.w.is_finite() || p.w <= 0. {
                    return Err("Through-all bounds require positive rational weights.".into());
                }
                values.push((p.truncate() / p.w).dot(axis));
            }
            range(values.into_iter())
        }
        fn surface(surface: &Surface, axis: Vector3) -> Result<Option<[f64; 2]>, String> {
            match surface {
                Surface::Plane(_) => Ok(None),
                Surface::BSplineSurface(s) => range(
                    s.control_points()
                        .iter()
                        .flatten()
                        .map(|p| p.to_vec().dot(axis)),
                )
                .map(Some),
                Surface::NurbsSurface(s) => {
                    rational(s.control_points().iter().flatten().copied(), axis).map(Some)
                }
                Surface::RevolutedCurve(rev) => {
                    let source = rev.entity();
                    if matches!(source.entity_curve(), Curve::IntersectionCurve(_)) {
                        return Err(
                            "Cannot bound an intersection-generated revolution for Through all."
                                .into(),
                        );
                    }
                    let generator = source.entity_curve().clone().lift_up();
                    let transform = *rev.transform();
                    let normal = source.axis().normalize();
                    let projected = Vector3::new(
                        axis.dot(transform.x.truncate()),
                        axis.dot(transform.y.truncate()),
                        axis.dot(transform.z.truncate()),
                    );
                    let radial_projection =
                        (projected - normal * projected.dot(normal)).magnitude();
                    let mut values = Vec::new();
                    for p in generator.control_points() {
                        if !p.w.is_finite() || p.w <= 0. {
                            return Err(
                                "Through-all bounds require positive rational weights.".into()
                            );
                        }
                        let point = Point3::from_vec(p.truncate() / p.w);
                        let axial =
                            source.origin() + normal * (point - source.origin()).dot(normal);
                        let center = transform.transform_point(axial).to_vec().dot(axis);
                        let radius = (point - axial).magnitude() * radial_projection;
                        values.extend([center - radius, center + radius]);
                    }
                    range(values.into_iter()).map(Some)
                }
            }
        }
        fn curve(curve: &Curve, ends: [Point3; 2], axis: Vector3) -> Result<[f64; 2], String> {
            match curve {
                Curve::Line(_) => range(ends.into_iter().map(|p| p.to_vec().dot(axis))),
                Curve::BSplineCurve(c) => {
                    range(c.control_points().iter().map(|p| p.to_vec().dot(axis)))
                }
                Curve::NurbsCurve(c) => rational(c.control_points().iter().copied(), axis),
                Curve::IntersectionCurve(c) => {
                    match (surface(c.surface0(), axis)?, surface(c.surface1(), axis)?) {
                        (Some(a), Some(b)) => Ok([a[0].max(b[0]), a[1].min(b[1])]),
                        (Some(a), None) | (None, Some(a)) => Ok(a),
                        (None, None) => {
                            let (Surface::Plane(a), Surface::Plane(b)) =
                                (c.surface0(), c.surface1())
                            else {
                                unreachable!()
                            };
                            if a.normal().cross(b.normal()).magnitude2() < 1e-16 {
                                return Err(
                                    "Cannot bound coincident-plane intersection curves.".into()
                                );
                            }
                            range(ends.into_iter().map(|p| p.to_vec().dot(axis)))
                        }
                    }
                }
            }
        }
        let shell = self
            .shell
            .as_ref()
            .ok_or("Through-all scope requires retained solid boundaries.")?;
        let mut bounds = Vec::new();
        for face in shell {
            if let Some(b) = surface(&face.oriented_surface(), axis)? {
                bounds.extend(b);
            } else {
                for edge in face.edge_iter() {
                    bounds.extend(curve(
                        &edge.oriented_curve(),
                        [edge.front().point(), edge.back().point()],
                        axis,
                    )?);
                }
            }
        }
        range(bounds.into_iter())
    }

    pub(crate) fn surface_mapping(&self) -> Result<&[usize], String> {
        let count = self
            .shell
            .as_ref()
            .ok_or("Operation requires a retained kernel boundary.")?
            .len();
        if self.surface_roles.len() == count {
            Ok(&self.surface_roles)
        } else if self.triangle_faces.len() == count {
            Ok(&self.triangle_faces)
        } else {
            Err("Missing evaluated surface mapping.".into())
        }
    }
}

pub fn evaluate(body: &Body) -> Result<Geometry, String> {
    body.validate()?;
    let mut geometry = build_checked(body, None)?;
    crate::components::apply_evaluated(body, &mut geometry).map_err(|e| body.error(e))?;
    geometry.group_cylinder_patches();
    Ok(geometry)
}
fn build_checked(body: &Body, loops: Option<Vec<Vec<[f64; 2]>>>) -> Result<Geometry, String> {
    // One error boundary for standalone, preview, save/load, and scene evaluation.
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| build(body, loops)))
        .map_err(|_| body.error("The geometry kernel could not evaluate these parameters."))?
        .map_err(|reason| body.error(reason))
}
// Validation and viewport upload request the same scene consecutively. Keep only the
// latest successful result; exact parameter equality also covers edits and undo.
#[derive(Default)]
struct SceneCache {
    bodies: Vec<Body>,
    geometry: Vec<(u64, Geometry)>,
}
struct BaseGeometry {
    body: Body,
    loops: Option<Vec<Vec<[f64; 2]>>>,
    geometry: Geometry,
}
thread_local! {
    // Only the latest successful scene's bases; never accumulate edit revisions.
    static BASE_CACHE: std::cell::RefCell<Vec<BaseGeometry>> = const { std::cell::RefCell::new(Vec::new()) };
    static SCENE_CACHE: std::cell::RefCell<Option<SceneCache>> = const { std::cell::RefCell::new(None) };
}
pub(crate) fn cache_scene(bodies: &[Body], geometry: &[(u64, Geometry)]) {
    if bodies.is_empty() {
        BASE_CACHE.with(|cache| cache.borrow_mut().clear());
    }
    SCENE_CACHE.with(|cache| {
        *cache.borrow_mut() = Some(SceneCache {
            bodies: bodies.to_vec(),
            geometry: geometry.to_vec(),
        })
    });
}
pub fn evaluate_scene(bodies: &[Body]) -> Result<Vec<(u64, Geometry)>, String> {
    if let Some(geometry) = SCENE_CACHE.with(|cache| {
        cache
            .borrow()
            .as_ref()
            .filter(|c| c.bodies == bodies)
            .map(|c| c.geometry.clone())
    }) {
        return Ok(geometry);
    }
    let geometry = build_scene(bodies)?;
    SCENE_CACHE.with(|cache| {
        *cache.borrow_mut() = Some(SceneCache {
            bodies: bodies.to_vec(),
            geometry: geometry.clone(),
        })
    });
    Ok(geometry)
}
fn build_scene(bodies: &[Body]) -> Result<Vec<(u64, Geometry)>, String> {
    for body in bodies {
        body.validate()?;
    }
    for cutter in bodies.iter().filter(|body| {
        !body.component_edits.is_empty()
            || !body.edge_treatments.is_empty()
            || body.hollow.is_some()
            || body.taper.is_some()
            || !body.face_extrusions.is_empty()
    }) {
        for target in bodies
            .iter()
            .filter(|body| body.id < cutter.id && body.shape != Shape::Line)
        {
            if crate::regions::boundaries(target, &[target.clone(), cutter.clone()])
                .map_err(|e| cutter.error(format!("Partition against body {}: {e}", target.id)))?
                .is_some()
            {
                return Err(cutter.error(format!("Profile partition against body {} cannot replay solid/component operations. Disable its cutting flag or use an explicit Boolean operation.",target.id)));
            }
        }
    }
    let mut bases = Vec::new();
    let mut scene: Vec<_> = bodies
        .iter()
        .map(|body| {
            if body.boolean.is_some() {
                return Ok((body.id, Geometry::default()));
            }
            let loops = crate::regions::boundaries(body, bodies).map_err(|e| body.error(e))?;
            let cached = BASE_CACHE.with(|cache| {
                cache
                    .borrow()
                    .iter()
                    .find(|base| base.body == *body && base.loops == loops)
                    .map(|base| base.geometry.clone())
            });
            let geometry = match cached {
                Some(geometry) => geometry,
                None => build_checked(body, loops.clone())?,
            };
            bases.push(BaseGeometry {
                body: body.clone(),
                loops,
                geometry: geometry.clone(),
            });
            Ok((body.id, geometry))
        })
        .collect::<Result<_, String>>()?;
    crate::operators::apply_bores(bodies, &mut scene)?;
    for (body, (_, geometry)) in bodies.iter().zip(&mut scene) {
        if body.boolean.is_none() {
            crate::components::apply_evaluated(body, geometry).map_err(|e| body.error(e))?;
        }
    }
    crate::booleans::apply(bodies, &mut scene)?;
    for (_, geometry) in &mut scene {
        geometry.group_cylinder_patches();
    }
    BASE_CACHE.with(|cache| *cache.borrow_mut() = bases);
    Ok(scene)
}
pub(crate) fn build(b: &Body, loops: Option<Vec<Vec<[f64; 2]>>>) -> Result<Geometry, String> {
    let intact_circle = b.shape == Shape::Circle && loops.is_none();
    let mut g = build_geometry(b, loops)?;
    if intact_circle && b.taper.is_none() && b.hollow.is_none() {
        let transform = crate::regions::transform(b);
        let radius = b.width / 2.;
        let mut rings = vec![(radius, 0.)];
        if b.extrusion > 0. {
            rings.push((radius, b.extrusion));
        }
        for modifier in &b.edge_treatments {
            let (radial, height) = modifier.setbacks();
            if modifier.face == crate::edge_treatments::FaceTarget::Top {
                rings.retain(|(_, z)| *z != b.extrusion);
                rings.extend([
                    (radius - radial, b.extrusion),
                    (radius, b.extrusion - height),
                ]);
            } else if modifier.face == crate::edge_treatments::FaceTarget::Bottom {
                rings.retain(|(_, z)| *z != 0.);
                rings.extend([(radius - radial, 0.), (radius, height)]);
            }
        }
        g.circles.extend(
            rings
                .into_iter()
                .map(|(r, z)| CircularEdge {
                    center: transform.transform_point(Point3::new(0., 0., z)),
                    u: transform.transform_vector(Vector3::unit_x() * r),
                    v: transform.transform_vector(Vector3::unit_y() * r),
                })
                .collect::<Vec<_>>(),
        );
    }
    if b.construction {
        g.triangles.clear();
        g.triangle_faces.clear();
    }
    crate::face_extrusions::apply(b, &mut g)?;
    crate::components::apply(b, &mut g)?;
    g.is_solid = b.extrusion > 0. && b.shape != Shape::Line;
    Ok(g)
}
fn build_geometry(b: &Body, loops: Option<Vec<Vec<[f64; 2]>>>) -> Result<Geometry, String> {
    if b.taper.is_some() {
        return crate::drafting::evaluate(b, loops.is_some());
    }
    if b.hollow.is_some() {
        return crate::shelling::evaluate(b, loops.is_some());
    }
    if !b.edge_treatments.is_empty() {
        return crate::edge_treatments::evaluate(b, loops.is_some());
    }
    if b.shape != Shape::Line && b.extrusion > 0. && loops.is_none() {
        return tessellate(&primitive_shell(b)?);
    }
    let wire = profile_wire(b);
    let v = |x, y| builder::vertex(Point3::new(x, y, 0.));
    let transform = crate::regions::transform(b);
    let mut geometry = Geometry::default();
    let shell: Shell = if b.shape == Shape::Line {
        if b.extrusion == 0. {
            let edge = builder::transformed(&wire[0], transform);
            geometry
                .edges
                .push([edge.front().point(), edge.back().point()]);
            geometry.edge_groups.push(0);
            geometry.edge_count = 1;
            geometry.topology.edges.push([0, 1]);
            geometry
                .points
                .extend([edge.front().point(), edge.back().point()]);
            return Ok(geometry);
        }
        vec![builder::tsweep(&wire[0], Vector3::unit_z() * b.extrusion)].into()
    } else {
        let faces = if let Some(loops) = loops {
            if loops.is_empty() {
                return Ok(geometry);
            }
            let wires: Vec<Wire> = loops
                .iter()
                .map(|polygon| {
                    let vertices: Vec<_> = polygon.iter().map(|p| v(p[0], p[1])).collect();
                    (0..vertices.len())
                        .map(|i| builder::line(&vertices[i], &vertices[(i + 1) % vertices.len()]))
                        .collect()
                })
                .collect();
            let mut faces = Vec::new();
            for (i, polygon) in loops
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
                    if crate::regions::contains(hole[0], polygon) {
                        boundaries.push(wires[j].clone());
                    }
                }
                faces.push(builder::try_attach_plane(&boundaries).map_err(|e| e.to_string())?);
            }
            faces
        } else {
            vec![builder::try_attach_plane(&[wire]).map_err(|e| e.to_string())?]
        };
        let mut shell = Shell::default();
        for face in faces {
            if b.extrusion == 0. {
                shell.push(face);
            } else {
                shell.extend(extrude_face(&face, b.extrusion)?);
            }
        }
        shell
    };
    let shell = builder::transformed(&shell, transform);
    let geometry = tessellate(&shell)?;
    Ok(geometry)
}

pub(crate) fn profile_wire(b: &Body) -> Wire {
    let r = b.width / 2.;
    let v = |x, y| builder::vertex(Point3::new(x, y, 0.));
    match b.shape {
        Shape::Rectangle => {
            let points = [
                v(-r, -b.depth / 2.),
                v(r, -b.depth / 2.),
                v(r, b.depth / 2.),
                v(-r, b.depth / 2.),
            ];
            (0..4)
                .map(|i| builder::line(&points[i], &points[(i + 1) % 4]))
                .collect()
        }
        Shape::Line => vec![builder::line(&v(-r, 0.), &v(r, 0.))].into(),
        Shape::Circle => builder::rsweep(
            &v(r, 0.),
            Point3::new(0., 0., 0.),
            Vector3::unit_z(),
            Rad(std::f64::consts::TAU),
        ),
        Shape::Semicircle => {
            let a = v(r, 0.);
            let z = v(-r, 0.);
            vec![
                builder::circle_arc(&a, &z, Point3::new(0., r, 0.)),
                builder::line(&z, &a),
            ]
            .into()
        }
    }
}
pub(crate) fn extrude_face(face: &truck_modeling::Face, depth: f64) -> Result<Shell, String> {
    let solid: Solid = builder::tsweep(face, Vector3::unit_z() * depth);
    if !solid.is_geometric_consistent() {
        return Err("Extrusion is not geometrically consistent.".into());
    }
    Ok(solid.boundaries()[0].clone())
}
pub(crate) fn primitive_shell(b: &Body) -> Result<Shell, String> {
    let face = builder::try_attach_plane(&[profile_wire(b)]).map_err(|e| e.to_string())?;
    Ok(builder::transformed(
        &extrude_face(&face, b.extrusion)?,
        crate::regions::transform(b),
    ))
}

pub fn tessellate(shell: &Shell) -> Result<Geometry, String> {
    let mut geometry = Geometry {
        shell: Some(shell.clone()),
        ..Default::default()
    };
    let meshed = shell.triangulation(0.01);
    // Preserve face-level output until all faces have successfully tessellated.
    let mut edges = std::collections::HashMap::new();
    let mut vertices = std::collections::HashMap::new();
    for (face_index, face) in meshed.face_iter().enumerate() {
        geometry.face_count += 1;
        geometry.surface_roles.push(face_index);
        let mesh = face.surface();
        let mesh = mesh.as_ref().ok_or("A face could not be tessellated.")?;
        geometry.topology.faces.push(Vec::new());
        for tri in mesh.tri_faces() {
            let mut triangle = tri.map(|v| mesh.positions()[v.pos]);
            if !face.orientation() {
                triangle.swap(1, 2);
            }
            geometry.triangles.push(triangle);
            geometry.triangle_faces.push(face_index);
        }
        for edge in face.edge_iter() {
            for vertex in [edge.front(), edge.back()] {
                let index = vertices.len();
                if let std::collections::hash_map::Entry::Vacant(entry) =
                    vertices.entry(vertex.id())
                {
                    entry.insert(index);
                    geometry.points.push(vertex.point());
                }
            }
            if let Some(index) = edges.get(&edge.id()) {
                geometry.topology.faces[face_index].push(*index);
                continue;
            }
            let index = edges.len();
            edges.insert(edge.id(), index);
            geometry.topology.faces[face_index].push(index);
            geometry
                .topology
                .edges
                .push([vertices[&edge.front().id()], vertices[&edge.back().id()]]);
            for pair in edge.oriented_curve().windows(2) {
                geometry.edges.push([pair[0], pair[1]]);
                geometry.edge_groups.push(index);
            }
        }
    }
    geometry.edge_count = edges.len();
    if geometry.triangles.is_empty() {
        return Err("No displayable faces were generated.".into());
    }
    Ok(geometry)
}

#[derive(Default, Clone)]
pub struct Model {
    pub bodies: Vec<Body>,
    undo: Vec<Vec<Body>>,
    redo: Vec<Vec<Body>>,
    next_id: u64,
    editing: bool,
}
impl Model {
    pub fn from_bodies(bodies: Vec<Body>) -> Self {
        let next_id = bodies.iter().map(|b| b.id).max().unwrap_or(0);
        Self {
            bodies,
            next_id,
            ..Self::default()
        }
    }
    pub fn commit(&mut self, body: Body) -> Result<u64, String> {
        self.finish_edit();
        self.store(body, false)
    }
    pub fn edit(&mut self, body: Body) -> Result<u64, String> {
        self.store(body, true)
    }
    pub fn finish_edit(&mut self) {
        self.editing = false;
    }
    fn store(&mut self, mut body: Body, live: bool) -> Result<u64, String> {
        body.validate()?;
        if body.id == 0 {
            if self.bodies.len() >= 1000 {
                return Err("The current project limit is 1,000 objects.".into());
            }
            self.next_id = self
                .next_id
                .checked_add(1)
                .filter(|id| *id < u64::MAX)
                .ok_or("Object identifiers are exhausted.")?;
            body.id = self.next_id;
        }
        let id = body.id;
        if self.bodies.iter().any(|b| b == &body) {
            return Ok(id);
        }
        let mut candidate = self.bodies.clone();
        if let Some(old) = candidate.iter_mut().find(|b| b.id == id) {
            *old = body.clone();
        } else {
            candidate.push(body.clone());
        }
        self.replace_scene(candidate, live)?;
        Ok(id)
    }
    fn replace_scene(&mut self, candidate: Vec<Body>, live: bool) -> Result<(), String> {
        if candidate == self.bodies {
            return Ok(());
        }
        evaluate_scene(&candidate)?;
        if !live || !self.editing {
            self.checkpoint();
        }
        self.editing = live;
        self.bodies = candidate;
        Ok(())
    }
    pub fn commit_changes(&mut self, changes: &[Body]) -> Result<(), String> {
        let mut candidate = self.bodies.clone();
        for change in changes {
            *candidate
                .iter_mut()
                .find(|b| b.id == change.id)
                .ok_or("Changed body no longer exists.")? = change.clone();
        }
        self.replace_scene(candidate, false)
    }
    pub fn delete_many(&mut self, ids: &[u64]) -> Result<(), String> {
        let mut candidate = self.bodies.clone();
        candidate.retain(|b| !ids.contains(&b.id));
        self.replace_scene(candidate, false)
    }
    fn checkpoint(&mut self) {
        self.undo.push(self.bodies.clone());
        if self.undo.len() > 100 {
            self.undo.remove(0);
        }
        self.redo.clear();
    }
    pub fn delete(&mut self, id: u64) -> Result<(), String> {
        self.delete_many(&[id])
    }
    pub fn undo(&mut self) {
        self.finish_edit();
        if let Some(previous) = self.undo.pop() {
            self.redo
                .push(std::mem::replace(&mut self.bodies, previous));
        }
    }
    pub fn redo(&mut self) {
        self.finish_edit();
        if let Some(next) = self.redo.pop() {
            self.undo.push(std::mem::replace(&mut self.bodies, next));
        }
    }
    pub fn can_undo(&self) -> bool {
        !self.undo.is_empty()
    }
    pub fn can_redo(&self) -> bool {
        !self.redo.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn split_profiles_extrude_holes_notches_and_disconnected_regions() {
        let mut outer = Body::new(Shape::Rectangle);
        outer.id = 1;
        outer.width = 6.;
        outer.depth = 4.;
        outer.extrusion = 3.;
        let mut inner = Body::new(Shape::Rectangle);
        inner.id = 2;
        inner.cuts_profiles = true;
        inner.width = 1.;
        inner.depth = 2.;
        let cap_area = |g: &Geometry| -> f64 {
            g.triangles
                .iter()
                .filter(|t| t.iter().all(|p| (p.z - 3.).abs() < 1e-7))
                .map(|t| ((t[1] - t[0]).cross(t[2] - t[0])).z.abs() / 2.)
                .sum()
        };
        for (position, width, depth, expected) in [
            ([0., 0., 0.], 1., 2., 22.),   // Interior hole.
            ([-2.5, 0., 0.], 1., 2., 22.), // Flush with an outer edge: notch.
            ([-3., 0., 0.], 2., 2., 22.),  // Crossing an outer edge.
            ([0., 0., 0.], 1., 6., 20.),   // Crossing two edges: separate remaining regions.
        ] {
            inner.position = position;
            inner.width = width;
            inner.depth = depth;
            let scene = evaluate_scene(&[outer.clone(), inner.clone()]).unwrap();
            assert!(
                (cap_area(&scene[0].1) - expected).abs() < 1e-6,
                "cap area {}",
                cap_area(&scene[0].1)
            );
            assert!(scene[0].1.points.iter().any(
                |p| (p.z - 3.).abs() < 1e-7 && (p.x - (position[0] + width / 2.)).abs() < 1e-7
            ));
            assert!(
                scene[1]
                    .1
                    .triangles
                    .iter()
                    .flatten()
                    .all(|p| p.z.abs() < 1e-7)
            );
        }
        inner.position = [0., 0., 0.];
        inner.width = 2.;
        inner.depth = 2.;
        let mut second = inner.clone();
        second.id = 3;
        second.position[0] = 1.;
        let scene = evaluate_scene(&[outer.clone(), inner.clone(), second.clone()]).unwrap();
        assert!((cap_area(&scene[0].1) - 18.).abs() < 1e-6); // Union, not double subtraction.
        let project = crate::project::Project::new(
            vec![outer.clone(), inner.clone(), second],
            crate::viewport::Camera::default(),
            true,
            Some(1),
        );
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("split.osk");
        project.save(&path).unwrap();
        let restored = crate::project::Project::load(&path).unwrap();
        assert!((cap_area(&evaluate_scene(&restored.bodies).unwrap()[0].1) - 18.).abs() < 1e-6);
        inner.position = [0., 0., 0.];
        inner.shape = Shape::Circle;
        inner.width = 1.;
        let scene = evaluate_scene(&[outer.clone(), inner.clone()]).unwrap();
        assert!((cap_area(&scene[0].1) - (24. - std::f64::consts::PI / 4.)).abs() < 0.02);
        inner.position[2] = 1.;
        assert_eq!(
            cap_area(&evaluate_scene(&[outer.clone(), inner.clone()]).unwrap()[0].1),
            24.
        );
        // Live edits and history regenerate the same partition from persisted profiles.
        inner.position[2] = 0.;
        inner.shape = Shape::Rectangle;
        inner.width = 1.;
        inner.depth = 2.;
        let mut model = Model::from_bodies(vec![outer, inner]);
        let mut edit = model.bodies[1].clone();
        edit.position[0] = 1.;
        model.edit(edit).unwrap();
        model.undo();
        assert!((cap_area(&evaluate_scene(&model.bodies).unwrap()[0].1) - 22.).abs() < 1e-6);
    }
    #[test]
    fn creation_extrusion_transforms_and_history() {
        let rectangle = Body::from_points(Shape::Rectangle, [4., 3., 0.], [-2., -1., 0.]).unwrap();
        assert_eq!(
            (rectangle.width, rectangle.depth, rectangle.position),
            (6., 4., [1., 1., 0.])
        );
        assert!(Body::from_points(Shape::Rectangle, [0.; 3], [2., 0., 0.]).is_err());
        let circle = Body::from_points(Shape::Circle, [1., 2., 0.], [4., 6., 0.]).unwrap();
        assert_eq!((circle.width, circle.position), (10., [1., 2., 0.]));
        let line = Body::from_points(Shape::Line, [0.; 3], [0., 4., 0.]).unwrap();
        assert_eq!((line.width, line.rotation[2]), (4., 90.));
        for shape in [
            Shape::Rectangle,
            Shape::Circle,
            Shape::Line,
            Shape::Semicircle,
        ] {
            let mut model = Model::default();
            let mut b = Body::new(shape);
            let flat = evaluate(&b).unwrap();
            assert!(!flat.edges.is_empty());
            if shape == Shape::Line {
                assert!(flat.triangles.is_empty());
            } else {
                assert!(!flat.triangles.is_empty());
            }
            b.extrusion = 3.;
            let id = model.commit(b.clone()).unwrap();
            b.id = id;
            b.scale = [2., 1., 1.];
            b.rotation = [0., 0., 90.];
            b.position = [4., 5., 6.];
            let g = evaluate(&b).unwrap();
            assert_eq!(g.triangle_faces.len(), g.triangles.len());
            assert_eq!(g.edge_groups.len(), g.edges.len());
            if shape == Shape::Rectangle {
                assert_eq!((g.face_count, g.edge_count, g.points.len()), (6, 12, 8));
            }
            let low = g
                .triangles
                .iter()
                .flatten()
                .map(|p| p.z)
                .fold(f64::INFINITY, f64::min);
            let high = g
                .triangles
                .iter()
                .flatten()
                .map(|p| p.z)
                .fold(f64::NEG_INFINITY, f64::max);
            assert!((low - 6.).abs() < 1e-6 && (high - 9.).abs() < 1e-6);
            model.edit(b.clone()).unwrap();
            b.position[0] = 7.;
            model.edit(b.clone()).unwrap();
            model.finish_edit();
            model.undo();
            assert_eq!(model.bodies[0].position, [0.; 3]);
            model.redo();
            assert_eq!(model.bodies[0], b);
            let before = model.bodies.clone();
            b.width = f64::NAN;
            assert!(model.commit(b).is_err());
            assert_eq!(model.bodies, before);
            model.delete(id).unwrap();
            assert!(model.bodies.is_empty());
            model.undo();
            assert_eq!(model.bodies, before);
        }
    }
    #[test]
    fn circle_metadata_requires_complete_retained_loops_and_deduplicates_bases() {
        let mut body = Body::new(Shape::Circle);
        body.width = 4.;
        body.extrusion = 3.;
        body.rotation = [17., 23., 31.];
        body.position = [2., -3., 1.];
        body.scale = [2., 0.5, 1.2];
        let mut g = evaluate(&body).unwrap();
        let rings = g.circles.clone();
        assert_eq!(rings.len(), 2);
        let mut candidates = rings.clone();
        for ring in &rings {
            candidates.push(CircularEdge {
                center: ring.center,
                u: (ring.u + ring.v) * std::f64::consts::FRAC_1_SQRT_2,
                v: (ring.v - ring.u) * std::f64::consts::FRAC_1_SQRT_2,
            });
            candidates.push(CircularEdge {
                center: ring.center + ring.u.normalize() * 0.001,
                ..ring.clone()
            });
        }
        g.circles.clear();
        g.retain_circles(&candidates);
        assert_eq!(g.circles.len(), 2);
        for (a, b) in g.circles.iter().zip(&rings) {
            assert!((a.center - b.center).magnitude() < 1e-8);
        }
        body.shape = Shape::Semicircle;
        let mut half = evaluate(&body).unwrap();
        half.retain_circles(&rings);
        assert!(
            half.circles.is_empty(),
            "A partial arc and its diameter cannot provide a complete circle snap reference"
        );
    }

    #[test]
    fn base_cache_reuses_unchanged_bodies_and_invalidates_profile_dependencies() {
        cache_scene(&[], &[]);
        let mut a = Body::new(Shape::Rectangle);
        a.id = 1;
        a.width = 8.;
        a.depth = 8.;
        a.extrusion = 3.;
        let mut b = Body::new(Shape::Circle);
        b.id = 2;
        b.position[0] = 20.;
        b.extrusion = 2.;
        let first = evaluate_scene(&[a.clone(), b.clone()]).unwrap();
        b.extrusion = 4.;
        let edited = evaluate_scene(&[a.clone(), b.clone()]).unwrap();
        assert_eq!(
            first[0].1.shell.as_ref().unwrap()[0].id(),
            edited[0].1.shell.as_ref().unwrap()[0].id()
        );
        assert_ne!(
            first[1].1.shell.as_ref().unwrap()[0].id(),
            edited[1].1.shell.as_ref().unwrap()[0].id()
        );
        // The host parameters stay identical, but the resolved partition changes.
        b.position[0] = 0.;
        b.cuts_profiles = true;
        let partitioned = evaluate_scene(&[a.clone(), b.clone()]).unwrap();
        assert_ne!(
            edited[0].1.shell.as_ref().unwrap()[0].id(),
            partitioned[0].1.shell.as_ref().unwrap()[0].id()
        );
        BASE_CACHE.with(|cache| cache.borrow_mut().clear());
        SCENE_CACHE.with(|cache| *cache.borrow_mut() = None);
        let cold = evaluate_scene(&[a, b]).unwrap();
        for ((_, warm), (_, cold)) in partitioned.iter().zip(&cold) {
            assert_eq!(warm.triangles, cold.triangles);
            assert_eq!(warm.topology, cold.topology);
        }
        let mut invalid = Body::new(Shape::Circle);
        invalid.width = f64::NAN;
        assert!(evaluate_scene(&[invalid]).is_err());
        assert!(evaluate_scene(&[]).unwrap().is_empty());
        BASE_CACHE.with(|cache| assert!(cache.borrow().is_empty()));
    }

    #[test]
    fn scene_cache_tracks_parameters_and_preserves_validation() {
        let mut b = Body::new(Shape::Circle);
        b.id = 1;
        b.extrusion = 2.;
        let first = evaluate_scene(&[b.clone()]).unwrap();
        let cached = evaluate_scene(&[b.clone()]).unwrap();
        assert_eq!(first[0].1.triangles, cached[0].1.triangles);
        // A hit must not replace the cached kernel shape with a rebuilt one.
        SCENE_CACHE.with(|c| {
            assert_eq!(
                c.borrow().as_ref().unwrap().geometry[0]
                    .1
                    .shell
                    .as_ref()
                    .unwrap()[0]
                    .id(),
                cached[0].1.shell.as_ref().unwrap()[0].id()
            )
        });
        b.extrusion = 5.;
        let changed = evaluate_scene(&[b.clone()]).unwrap();
        assert!(
            changed[0]
                .1
                .triangles
                .iter()
                .flatten()
                .any(|p| (p.z - 5.).abs() < 1e-6)
        );
        b.width = f64::NAN;
        assert!(evaluate_scene(&[b]).is_err());
        assert!(evaluate_scene(&[]).unwrap().is_empty());
    }

    #[test]
    #[ignore = "manual geometry timing: cargo test circle_update_timings -- --ignored --nocapture"]
    fn circle_update_timings() {
        for treated in [false, true] {
            for width in [2., 4., 20., 100.] {
                let mut body = Body::new(Shape::Circle);
                body.id = 1;
                body.width = width;
                body.extrusion = 3.;
                if treated {
                    body.edge_treatments
                        .push(crate::edge_treatments::Modifier::new(
                            crate::edge_treatments::Kind::Bevel,
                            crate::edge_treatments::FaceTarget::Top,
                            &body,
                        ));
                }
                let start = std::time::Instant::now();
                let g = evaluate_scene(&[body.clone()]).unwrap().remove(0).1;
                let build_time = start.elapsed();
                let start = std::time::Instant::now();
                evaluate_scene(&[body]).unwrap();
                eprintln!(
                    "circle width={width} treated={treated}: build {build_time:?}, reuse {:?}, {} triangles, {} boundary samples",
                    start.elapsed(),
                    g.triangles.len(),
                    g.edges.len()
                );
            }
        }
    }
}
