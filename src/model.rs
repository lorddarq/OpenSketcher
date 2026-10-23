use serde::{Deserialize, Serialize};
use truck_meshalgo::tessellation::MeshableShape;
use truck_modeling::{Deg, Matrix4, Point3, Rad, Shell, Solid, Vector3, Wire, builder};

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
    pub position: [f64; 3],
    pub rotation: [f64; 3],
    pub scale: [f64; 3],
}
impl Body {
    pub fn new(shape: Shape) -> Self {
        Self {
            id: 0,
            shape,
            width: 2.,
            depth: 2.,
            extrusion: 0.,
            position: [0.; 3],
            rotation: [0.; 3],
            scale: [1.; 3],
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

#[derive(Default)]
pub struct Geometry {
    pub triangles: Vec<[Point3; 3]>,
    pub edges: Vec<[Point3; 2]>,
}

pub fn evaluate(body: &Body) -> Result<Geometry, String> {
    body.validate()?;
    // The kernel is never called with unchecked UI values. Its panic must not discard the document.
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| build(body)))
        .map_err(|_| "The geometry kernel could not evaluate these parameters.".to_owned())?
}
fn build(b: &Body) -> Result<Geometry, String> {
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
    let rotation = Matrix4::from_angle_z(Deg(b.rotation[2]))
        * Matrix4::from_angle_y(Deg(b.rotation[1]))
        * Matrix4::from_angle_x(Deg(b.rotation[0]));
    let transform = Matrix4::from_translation(Vector3::from(b.position))
        * rotation
        * Matrix4::from_nonuniform_scale(b.scale[0], b.scale[1], b.scale[2]);
    let mut geometry = Geometry::default();
    let shell: Shell = if b.shape == Shape::Line {
        if b.extrusion == 0. {
            let edge = builder::transformed(&wire[0], transform);
            geometry
                .edges
                .push([edge.front().point(), edge.back().point()]);
            return Ok(geometry);
        }
        vec![builder::tsweep(&wire[0], Vector3::unit_z() * b.extrusion)].into()
    } else {
        let face = builder::try_attach_plane(&[wire]).map_err(|e| e.to_string())?;
        if b.extrusion == 0. {
            vec![face].into()
        } else {
            let solid: Solid = builder::tsweep(&face, Vector3::unit_z() * b.extrusion);
            if !solid.is_geometric_consistent() {
                return Err("Extrusion is not geometrically consistent.".into());
            }
            solid.boundaries()[0].clone()
        }
    };
    let shell = builder::transformed(&shell, transform);
    let meshed = shell.triangulation(0.01);
    // Preserve face-level output until all faces have successfully tessellated.
    for face in meshed.face_iter() {
        let mesh = face.surface();
        let mesh = mesh.as_ref().ok_or("A face could not be tessellated.")?;
        for tri in mesh.tri_faces() {
            geometry
                .triangles
                .push(tri.map(|v| mesh.positions()[v.pos]));
        }
        // The meshed boundary curves are polylines; they retain curved face edges without drawing triangulation diagonals.
        for edge in face.edge_iter() {
            let curve = edge.oriented_curve();
            for pair in curve.windows(2) {
                geometry.edges.push([pair[0], pair[1]]);
            }
        }
    }
    if geometry.triangles.is_empty() {
        return Err("No displayable faces were generated.".into());
    }
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
        evaluate(&body)?;
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
    pub fn delete(&mut self, id: u64) {
        self.finish_edit();
        if self.bodies.iter().any(|b| b.id == id) {
            self.checkpoint();
            self.bodies.retain(|b| b.id != id);
        }
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
            model.delete(id);
            assert!(model.bodies.is_empty());
            model.undo();
            assert_eq!(model.bodies, before);
        }
    }
}
