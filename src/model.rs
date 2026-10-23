use serde::{Deserialize, Serialize};
use truck_meshalgo::tessellation::MeshableShape;
use truck_modeling::{Point3, Rad, Shell, Solid, Vector3, Wire, builder};

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
    pub position: [f64; 3],
    pub rotation: [f64; 3],
    pub scale: [f64; 3],
    #[serde(default)]
    pub component_edits: Vec<crate::components::ComponentEdit>,
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
            position: [0.; 3],
            rotation: [0.; 3],
            scale: [1.; 3],
            component_edits: Vec::new(),
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
    pub fn validate(&self) -> Result<(), String> {
        crate::components::validate(&self.component_edits)?;
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
            return Err("Dimensions must be 0.01–100 units; extrusion may also be zero.".into());
        }
        if self.scale.iter().any(|v| !(0.01..=100.).contains(v))
            || self.position.iter().any(|v| v.abs() > 1000.)
            || self.rotation.iter().any(|v| v.abs() > 36000.)
        {
            return Err("Scale must be 0.01–100; position ±1000; rotation ±36000°.".into());
        }
        Ok(())
    }
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
}

pub fn evaluate(body: &Body) -> Result<Geometry, String> {
    body.validate()?;
    // The kernel is never called with unchecked UI values. Its panic must not discard the document.
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| build(body, None)))
        .map_err(|_| "The geometry kernel could not evaluate these parameters.".to_owned())?
}
pub fn evaluate_scene(bodies: &[Body]) -> Result<Vec<(u64, Geometry)>, String> {
    for body in bodies {
        body.validate()?;
    }
    for cutter in bodies
        .iter()
        .filter(|body| !body.component_edits.is_empty())
    {
        for target in bodies
            .iter()
            .filter(|body| body.id < cutter.id && body.shape != Shape::Line)
        {
            if crate::regions::boundaries(target, &[target.clone(), cutter.clone()])?.is_some() {
                return Err("Component edits on a profile cutting another region are not supported yet. Use Volume transforms for that profile.".into());
            }
        }
    }
    bodies
        .iter()
        .map(|body| {
            let loops = crate::regions::boundaries(body, bodies)?;
            let geometry =
                std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| build(body, loops)))
                    .map_err(|_| {
                        "The geometry kernel could not build a split face.".to_owned()
                    })??;
            Ok((body.id, geometry))
        })
        .collect()
}
fn build(b: &Body, loops: Option<Vec<Vec<[f64; 2]>>>) -> Result<Geometry, String> {
    let r = b.width / 2.;
    let v = |x, y| builder::vertex(Point3::new(x, y, 0.));
    let wire: Wire = match b.shape {
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
    };
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
            crate::components::apply(b, &mut geometry)?;
            return Ok(geometry);
        }
        vec![builder::tsweep(&wire[0], Vector3::unit_z() * b.extrusion)].into()
    } else {
        let faces = if let Some(loops) = loops {
            if loops.is_empty() {
                crate::components::apply(b, &mut geometry)?;
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
                let solid: Solid = builder::tsweep(&face, Vector3::unit_z() * b.extrusion);
                if !solid.is_geometric_consistent() {
                    return Err("Extrusion is not geometrically consistent.".into());
                }
                shell.extend(solid.boundaries()[0].clone());
            }
        }
        shell
    };
    let shell = builder::transformed(&shell, transform);
    let meshed = shell.triangulation(0.01);
    // Preserve face-level output until all faces have successfully tessellated.
    let mut edges = std::collections::HashMap::new();
    let mut vertices = std::collections::HashMap::new();
    for (face_index, face) in meshed.face_iter().enumerate() {
        geometry.face_count += 1;
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
    crate::components::apply(b, &mut geometry)?;
    Ok(geometry)
}

#[derive(Default)]
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
        evaluate_scene(&candidate)?;
        if !live || !self.editing {
            self.checkpoint();
        }
        self.editing = live;
        if let Some(old) = self.bodies.iter_mut().find(|b| b.id == id) {
            *old = body;
        } else {
            self.bodies.push(body);
        }
        Ok(id)
    }
    fn checkpoint(&mut self) {
        self.undo.push(self.bodies.clone());
        if self.undo.len() > 100 {
            self.undo.remove(0);
        }
        self.redo.clear();
    }
    pub fn delete(&mut self, id: u64) -> Result<(), String> {
        self.finish_edit();
        if self.bodies.iter().any(|b| b.id == id) {
            let mut candidate = self.bodies.clone();
            candidate.retain(|body| body.id != id);
            evaluate_scene(&candidate)?;
            self.checkpoint();
            self.bodies = candidate;
        }
        Ok(())
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
}
