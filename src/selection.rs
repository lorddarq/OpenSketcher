use crate::{model::Geometry, viewport::Camera};
use eframe::egui;
use serde::{Deserialize, Serialize};
use truck_modeling::{InnerSpace, Point3, Vector3};

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SelectionMode {
    #[default]
    Volume,
    Face,
    Edge,
    Point,
}
impl SelectionMode {
    pub fn label(self) -> &'static str {
        match self {
            Self::Volume => "Volume",
            Self::Face => "Face",
            Self::Edge => "Edge",
            Self::Point => "Point",
        }
    }
    pub fn count(self, g: &Geometry) -> usize {
        match self {
            Self::Volume => 1,
            Self::Face => g.face_count,
            Self::Edge => g.edge_count,
            Self::Point => g.points.len(),
        }
    }
}

pub type Pick = (u64, Option<usize>);

/// Normal click replaces; Shift-click toggles. Shift-clicking empty space preserves selection.
pub fn extend(input: &egui::InputState) -> bool {
    // Shift may be released before this frame is painted; use the click event's modifiers.
    input
        .events
        .iter()
        .rev()
        .find_map(|event| match event {
            egui::Event::PointerButton {
                button: egui::PointerButton::Primary,
                modifiers,
                ..
            } => Some(modifiers.shift),
            _ => None,
        })
        .unwrap_or(input.modifiers.shift)
}
pub fn click(picks: &mut Vec<Pick>, picked: Option<Pick>, extend: bool) {
    if !extend {
        picks.clear();
    }
    if let Some(pick) = picked {
        if let Some(index) = picks.iter().position(|p| *p == pick) {
            picks.remove(index);
        } else {
            picks.push(pick);
        }
    }
}
pub fn valid(pick: Pick, scene: &[(u64, Geometry)], mode: SelectionMode) -> bool {
    scene
        .iter()
        .find(|(id, _)| *id == pick.0)
        .is_some_and(|(_, g)| {
            if mode == SelectionMode::Volume {
                pick.1.is_none()
            } else {
                pick.1.is_some_and(|i| {
                    i < mode.count(g)
                        && (mode != SelectionMode::Edge || !g.internal_edges.contains(&i))
                })
            }
        })
}

fn triangle_hit(o: Point3, d: Vector3, tri: &[Point3; 3]) -> Option<f64> {
    let e1 = tri[1] - tri[0];
    let e2 = tri[2] - tri[0];
    let h = d.cross(e2);
    let det = e1.dot(h);
    if det.abs() < 1e-12 {
        return None;
    }
    let s = o - tri[0];
    let u = s.dot(h) / det;
    let q = s.cross(e1);
    let v = d.dot(q) / det;
    let t = e2.dot(q) / det;
    // Camera projection uses f32 screen coordinates. Keep silhouette ray tests
    // tolerant of that rounding so an exposed top corner occludes its base.
    (u >= -1e-6 && v >= -1e-6 && u + v <= 1. + 1e-6 && t >= 0.).then_some(t)
}
#[derive(Default)]
pub struct SpatialIndex {
    triangles: Vec<(usize, usize)>,
    nodes: Vec<Node>,
}
struct Node {
    bounds: [[f64; 3]; 2],
    start: usize,
    end: usize,
    children: Option<[usize; 2]>,
}
impl SpatialIndex {
    pub fn new(scene: &[(u64, Geometry)]) -> Self {
        let mut index = Self {
            triangles: scene
                .iter()
                .enumerate()
                .flat_map(|(body, (_, g))| {
                    (0..g.triangles.len()).map(move |triangle| (body, triangle))
                })
                .collect(),
            nodes: Vec::new(),
        };
        if !index.triangles.is_empty() {
            // Each triangle box is invariant during construction. Computing it at
            // every tree level repeats expensive geometry work for dense scenes.
            let bounds: Vec<Vec<_>> = scene
                .iter()
                .enumerate()
                .map(|(body, (_, g))| {
                    (0..g.triangles.len())
                        .map(|triangle| Self::triangle_bounds(scene, (body, triangle)))
                        .collect()
                })
                .collect();
            index.build(scene, &bounds, 0, index.triangles.len());
        }
        index
    }
    #[cfg(test)]
    pub fn allocated_bytes(&self) -> usize {
        self.triangles.capacity() * std::mem::size_of::<(usize, usize)>()
            + self.nodes.capacity() * std::mem::size_of::<Node>()
    }
    fn triangle_bounds(
        scene: &[(u64, Geometry)],
        (body, triangle): (usize, usize),
    ) -> [[f64; 3]; 2] {
        let t = scene[body].1.triangles[triangle];
        let mut bounds = [[f64::INFINITY; 3], [f64::NEG_INFINITY; 3]];
        for axis in 0..3 {
            // Match triangle_hit's barycentric tolerance, including silhouette
            // rays just outside a triangle's exact box after f32 projection.
            let pad =
                2e-6 * ((t[1][axis] - t[0][axis]).abs() + (t[2][axis] - t[0][axis]).abs()) + 1e-9;
            for p in t {
                bounds[0][axis] = bounds[0][axis].min(p[axis] - pad);
                bounds[1][axis] = bounds[1][axis].max(p[axis] + pad);
            }
        }
        bounds
    }
    fn build(
        &mut self,
        scene: &[(u64, Geometry)],
        triangle_bounds: &[Vec<[[f64; 3]; 2]>],
        start: usize,
        end: usize,
    ) -> usize {
        let mut bounds = [[f64::INFINITY; 3], [f64::NEG_INFINITY; 3]];
        for key in &self.triangles[start..end] {
            let b = triangle_bounds[key.0][key.1];
            for axis in 0..3 {
                bounds[0][axis] = bounds[0][axis].min(b[0][axis]);
                bounds[1][axis] = bounds[1][axis].max(b[1][axis]);
            }
        }
        let id = self.nodes.len();
        self.nodes.push(Node {
            bounds,
            start,
            end,
            children: None,
        });
        if end - start > 8 {
            let axis = (0..3)
                .max_by(|a, b| {
                    (bounds[1][*a] - bounds[0][*a]).total_cmp(&(bounds[1][*b] - bounds[0][*b]))
                })
                .unwrap();
            let middle = (start + end) / 2;
            self.triangles[start..end].select_nth_unstable_by(middle - start, |a, b| {
                let center = |(body, triangle): (usize, usize)| {
                    scene[body].1.triangles[triangle]
                        .iter()
                        .map(|p| p[axis])
                        .sum::<f64>()
                };
                center(*a).total_cmp(&center(*b))
            });
            let left = self.build(scene, triangle_bounds, start, middle);
            let right = self.build(scene, triangle_bounds, middle, end);
            self.nodes[id].children = Some([left, right]);
        }
        id
    }
    fn visit(&self, o: Point3, d: Vector3, limit: f64, mut triangle: impl FnMut(usize, usize)) {
        if self.nodes.is_empty() {
            return;
        }
        let mut stack = vec![0];
        while let Some(id) = stack.pop() {
            let node = &self.nodes[id];
            let mut lo: f64 = 0.;
            let mut hi = limit;
            for axis in 0..3 {
                if d[axis] == 0. {
                    if o[axis] < node.bounds[0][axis] || o[axis] > node.bounds[1][axis] {
                        hi = -1.;
                        break;
                    }
                } else {
                    let a = (node.bounds[0][axis] - o[axis]) / d[axis];
                    let b = (node.bounds[1][axis] - o[axis]) / d[axis];
                    lo = lo.max(a.min(b));
                    hi = hi.min(a.max(b));
                }
            }
            if hi < lo {
                continue;
            }
            if let Some(children) = node.children {
                stack.extend(children);
            } else {
                for (body, index) in &self.triangles[node.start..node.end] {
                    triangle(*body, *index);
                }
            }
        }
    }
}

#[derive(Clone, Copy)]
pub struct Query<'a> {
    pub scene: &'a [(u64, Geometry)],
    index: Option<&'a SpatialIndex>,
}
impl<'a> Query<'a> {
    pub fn indexed(scene: &'a [(u64, Geometry)], index: &'a SpatialIndex) -> Self {
        Self {
            scene,
            index: Some(index),
        }
    }
    #[cfg(test)]
    pub fn brute(scene: &'a [(u64, Geometry)]) -> Self {
        Self { scene, index: None }
    }
    pub fn surface_hit(
        self,
        camera: &Camera,
        p: egui::Pos2,
        rect: egui::Rect,
    ) -> Option<(f64, u64, usize)> {
        let (o, d) = camera.ray(p, rect);
        let limit = camera.span as f64 * 100.;
        let mut hits = Vec::new();
        let mut visit = |body: usize, index: usize| {
            let (id, g) = &self.scene[body];
            if let Some(depth) =
                triangle_hit(o, d, &g.triangles[index]).filter(|depth| *depth <= limit)
            {
                hits.push((
                    depth,
                    *id,
                    g.face_group(g.triangle_faces[index]),
                    body,
                    index,
                ));
            }
        };
        if let Some(index) = self.index {
            index.visit(o, d, limit, &mut visit);
        } else {
            for (body, (_, g)) in self.scene.iter().enumerate() {
                for i in 0..g.triangles.len() {
                    visit(body, i);
                }
            }
        }
        let nearest = hits.iter().map(|hit| hit.0).min_by(f64::total_cmp)?;
        hits.into_iter()
            .filter(|hit| hit.0 <= nearest + 1e-7)
            .max_by_key(|hit| (hit.1, hit.2, hit.3, hit.4))
            .map(|hit| (hit.0, hit.1, hit.2))
    }

    pub fn visible(self, camera: &Camera, p: Point3, rect: egui::Rect) -> bool {
        let (screen, z) = camera.project(p, rect);
        let depth = camera.span as f64 * 50. - z as f64;
        let tolerance = (camera.span as f64 * 1e-5).max(1e-5);
        depth >= 0.
            && depth <= camera.span as f64 * 100.
            && self
                .surface_hit(camera, screen, rect)
                .is_none_or(|hit| depth <= hit.0 + tolerance)
    }

    pub fn pick(
        self,
        camera: &Camera,
        p: egui::Pos2,
        rect: egui::Rect,
        mode: SelectionMode,
    ) -> Option<(u64, Option<usize>)> {
        let scene = self.scene;
        let surface = self.surface_hit(camera, p, rect);
        if mode == SelectionMode::Face {
            return surface.map(|(_, id, face)| (id, Some(face)));
        }
        let mut candidates = Vec::new();
        if mode == SelectionMode::Volume
            && let Some((depth, id, _)) = surface
        {
            // A visible standalone line needs its screen-space acquisition radius
            // even when it lies on a filled face; otherwise the face always wins.
            candidates.push((6.5, depth, id, None));
        }
        for (id, g) in scene {
            if mode == SelectionMode::Point {
                for (index, point) in g.points.iter().enumerate() {
                    let (screen, z) = camera.project(*point, rect);
                    let distance = screen.distance(p);
                    if distance <= 8. && self.visible(camera, *point, rect) {
                        candidates.push((
                            distance,
                            camera.span as f64 * 50. - z as f64,
                            *id,
                            Some(index),
                        ));
                    }
                }
            } else if mode == SelectionMode::Edge
                || (mode == SelectionMode::Volume && g.triangles.is_empty())
            {
                for (index, [a, b]) in g.edges.iter().enumerate() {
                    if mode == SelectionMode::Edge
                        && g.internal_edges.contains(&g.edge_groups[index])
                    {
                        continue;
                    }
                    let (sa, _) = camera.project(*a, rect);
                    let (sb, _) = camera.project(*b, rect);
                    let ab = sb - sa;
                    let t = ((p - sa).dot(ab) / ab.length_sq().max(1e-8)).clamp(0., 1.);
                    let distance = p.distance(sa + ab * t);
                    let world = *a + (*b - *a) * t as f64;
                    if distance <= 6. && self.visible(camera, world, rect) {
                        let z = camera.project(world, rect).1;
                        candidates.push((
                            distance,
                            camera.span as f64 * 50. - z as f64,
                            *id,
                            if mode == SelectionMode::Edge {
                                Some(g.edge_group(g.edge_groups[index]))
                            } else {
                                None
                            },
                        ));
                    }
                }
            }
        }
        candidates
            .into_iter()
            .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)))
            .map(|(_, _, id, element)| (id, element))
    }
}
// Keep the independent exhaustive path for regression comparisons.
#[cfg(test)]
pub fn visible(scene: &[(u64, Geometry)], camera: &Camera, p: Point3, rect: egui::Rect) -> bool {
    Query::brute(scene).visible(camera, p, rect)
}
#[cfg(test)]
pub fn pick(
    scene: &[(u64, Geometry)],
    camera: &Camera,
    p: egui::Pos2,
    rect: egui::Rect,
    mode: SelectionMode,
) -> Option<Pick> {
    Query::brute(scene).pick(camera, p, rect, mode)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        model::{Body, Shape, evaluate},
        viewport::View,
    };
    #[test]
    fn indexed_queries_preserve_hits_visibility_and_prune_triangle_work() {
        let bodies = (0..8)
            .map(|i| {
                let mut b = Body::new(if i % 3 == 0 {
                    Shape::Rectangle
                } else {
                    Shape::Circle
                });
                b.id = i + 1;
                b.extrusion = 2.;
                b.width = 4.;
                b.position = [(i % 4) as f64 * 6., (i / 4) as f64 * 6., 0.];
                b
            })
            .collect::<Vec<_>>();
        let mut scene = crate::model::evaluate_scene(&bodies).unwrap();
        // Coincident picks must prefer the latest owner even after BVH reorders triangles.
        scene.push((100, scene[0].1.clone()));
        let index = SpatialIndex::new(&scene);
        let fast = Query::indexed(&scene, &index);
        let slow = Query::brute(&scene);
        let mut camera = Camera::default();
        camera.fit(&scene);
        let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(1280., 850.));
        for view in [View::Isometric, View::Top, View::Side] {
            camera.preset(view);
            let mut pixels = Vec::new();
            for (_, g) in &scene {
                for p in &g.points {
                    pixels.push(camera.project(*p, rect).0);
                    assert_eq!(
                        fast.visible(&camera, *p, rect),
                        slow.visible(&camera, *p, rect)
                    );
                }
                for edge in &g.edges {
                    pixels.push(camera.project(edge[0] + (edge[1] - edge[0]) / 2., rect).0);
                }
            }
            pixels.extend(
                (0..9).flat_map(|x| {
                    (0..7).map(move |y| egui::pos2(x as f32 * 160., y as f32 * 140.))
                }),
            );
            for p in pixels {
                assert_eq!(
                    fast.surface_hit(&camera, p, rect),
                    slow.surface_hit(&camera, p, rect)
                );
            }
            for mode in [
                SelectionMode::Volume,
                SelectionMode::Face,
                SelectionMode::Edge,
                SelectionMode::Point,
            ] {
                for (_, g) in &scene {
                    for p in &g.points {
                        let pixel = camera.project(*p, rect).0;
                        assert_eq!(
                            fast.pick(&camera, pixel, rect, mode),
                            slow.pick(&camera, pixel, rect, mode)
                        );
                    }
                }
            }
        }
        camera.preset(View::Top);
        let pixel = camera.project(Point3::new(0., 0., 2.), rect).0;
        let (o, d) = camera.ray(pixel, rect);
        let mut tested = 0;
        index.visit(o, d, camera.span as f64 * 100., |_, _| tested += 1);
        let total = scene.iter().map(|(_, g)| g.triangles.len()).sum::<usize>();
        assert!(
            tested < total / 10,
            "Broad phase tested {tested} of {total} triangles"
        );
        assert_eq!(fast.surface_hit(&camera, pixel, rect).unwrap().1, 100);
        assert!(
            Query::indexed(&[], &SpatialIndex::default())
                .surface_hit(&camera, pixel, rect)
                .is_none()
        );
    }

    #[test]
    fn modes_use_topology_and_hide_occluded_components() {
        let mut camera = Camera::default();
        camera.preset(View::Top);
        let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(800., 600.));
        let mut body = Body::new(Shape::Rectangle);
        body.extrusion = 2.;
        let scene = vec![(1, evaluate(&body).unwrap())];
        let center = camera.project(Point3::new(0., 0., 2.), rect).0;
        assert_eq!(
            pick(&scene, &camera, center, rect, SelectionMode::Volume),
            Some((1, None))
        );
        assert!(
            pick(&scene, &camera, center, rect, SelectionMode::Face)
                .unwrap()
                .1
                .is_some()
        );
        assert!(pick(&scene, &camera, center, rect, SelectionMode::Edge).is_none());
        assert!(pick(&scene, &camera, center, rect, SelectionMode::Point).is_none());
        let edge = camera.project(Point3::new(1., 0., 2.), rect).0;
        assert!(
            pick(&scene, &camera, edge, rect, SelectionMode::Edge)
                .unwrap()
                .1
                .is_some()
        );
        let corner = camera.project(Point3::new(1., 1., 2.), rect).0;
        let (_, Some(index)) = pick(&scene, &camera, corner, rect, SelectionMode::Point).unwrap()
        else {
            panic!("expected point")
        };
        assert!((scene[0].1.points[index].z - 2.).abs() < 1e-6);
        assert!(!visible(&scene, &camera, Point3::new(0., 0., 0.), rect));
        let mut profile = Body::new(Shape::Rectangle);
        profile.width = 1.;
        profile.depth = 1.;
        profile.position[2] = 2.;
        let overlaid = vec![
            (1, evaluate(&body).unwrap()),
            (2, evaluate(&profile).unwrap()),
        ];
        assert_eq!(
            pick(&overlaid, &camera, center, rect, SelectionMode::Volume),
            Some((2, None))
        );
        let mut line_on_face = Body::new(Shape::Line);
        line_on_face.position[2] = 2.;
        let mut with_line = overlaid;
        with_line.push((3, evaluate(&line_on_face).unwrap()));
        assert_eq!(
            pick(&with_line, &camera, center, rect, SelectionMode::Volume),
            Some((3, None))
        );
        let line = Body::new(Shape::Line);
        let scene = vec![(2, evaluate(&line).unwrap())];
        assert!(pick(&scene, &camera, center, rect, SelectionMode::Face).is_none());
        assert_eq!(
            pick(&scene, &camera, center, rect, SelectionMode::Edge),
            Some((2, Some(0)))
        );
    }
    #[test]
    fn shift_click_and_group_transforms_keep_membership_and_atomic_history() {
        use crate::{
            components,
            model::{Body, Model, Shape, evaluate, evaluate_scene},
            project::Project,
            viewport::Camera,
        };
        use truck_modeling::{InnerSpace, Matrix4, Transform, Vector3};
        let mut a = Body::new(Shape::Rectangle);
        a.id = 1;
        a.extrusion = 2.;
        let mut b = a.clone();
        b.id = 2;
        b.position[0] = 4.;
        let bodies = vec![a.clone(), b];
        let scene = evaluate_scene(&bodies).unwrap();
        let mut input = egui::InputState::default();
        input.events.push(egui::Event::PointerButton {
            pos: egui::Pos2::ZERO,
            button: egui::PointerButton::Primary,
            pressed: false,
            modifiers: egui::Modifiers::SHIFT,
        });
        assert!(
            extend(&input),
            "click modifiers survive Shift release before paint"
        );
        let mut picks = Vec::new();
        click(&mut picks, Some((1, None)), false);
        click(&mut picks, Some((2, None)), true);
        assert_eq!(picks, vec![(1, None), (2, None)]);
        click(&mut picks, None, true);
        assert_eq!(picks.len(), 2);
        click(&mut picks, Some((1, None)), true);
        assert_eq!(picks, vec![(2, None)]);
        click(&mut picks, Some((1, None)), true);
        let matrix = Matrix4::from_translation(Vector3::new(0., 0., 0.25));
        let changed =
            components::transform_selection(&bodies, &scene, SelectionMode::Volume, &picks, matrix)
                .unwrap();
        assert!(changed.iter().all(|b| b.position[2] == 0.25));
        let mut model = Model::from_bodies(bodies.clone());
        model.commit_changes(&changed).unwrap();
        model.undo();
        assert_eq!(model.bodies, bodies);
        model.redo();
        assert_eq!(model.bodies, changed);
        let mut invalid = changed.clone();
        invalid[1].scale[0] = 0.;
        assert!(model.commit_changes(&invalid).is_err());
        assert_eq!(model.bodies, changed);
        model.delete_many(&[1, 2]).unwrap();
        assert!(model.bodies.is_empty());
        model.undo();
        assert_eq!(model.bodies, changed);
        let mut project = Project::new(bodies.clone(), Camera::default(), true, Some(1));
        project.selection = picks.clone();
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("selection.osk");
        project.save(&path).unwrap();
        assert_eq!(Project::load(&path).unwrap().picks(), picks);
        let geometry = evaluate(&a).unwrap();
        for mode in [
            SelectionMode::Face,
            SelectionMode::Edge,
            SelectionMode::Point,
        ] {
            let picks = vec![(1, Some(0)), (1, Some(1))];
            let changed = components::transform_selection(
                std::slice::from_ref(&a),
                &[(1, geometry.clone())],
                mode,
                &picks,
                matrix,
            )
            .unwrap();
            assert_eq!(changed[0].component_edits.len(), 1);
            assert_eq!(changed[0].component_edits[0].elements, vec![1]);
            let g = evaluate(&changed[0]).unwrap();
            let mut vertices = components::vertices(&geometry, mode, Some(0));
            vertices.extend(components::vertices(&geometry, mode, Some(1)));
            vertices.sort_unstable();
            vertices.dedup();
            if mode == SelectionMode::Face {
                assert!(changed[0].component_edits[0].retrim);
                for face in 0..geometry.face_count {
                    let before = crate::snapping::Plane::face(&geometry, face).unwrap();
                    let after = crate::snapping::Plane::face(&g, face).unwrap();
                    let origin = if face < 2 {
                        matrix.transform_point(before.origin)
                    } else {
                        before.origin
                    };
                    assert!((after.normal - before.normal).magnitude() < 1e-6);
                    assert!(
                        (after.origin - origin).dot(before.normal).abs() < 1e-6,
                        "Each selected plane must move once; unselected planes stay fixed"
                    );
                }
            }
            for (index, (before, after)) in geometry.points.iter().zip(&g.points).enumerate() {
                if mode == SelectionMode::Face {
                    continue;
                }
                let expected = *before
                    + if vertices.contains(&index) {
                        Vector3::new(0., 0., 0.25)
                    } else {
                        Vector3::new(0., 0., 0.)
                    };
                assert!(
                    (*after - expected).magnitude() < 1e-6,
                    "shared vertices must move once"
                );
            }
            let mut project = Project::new(changed, Camera::default(), true, Some(1));
            project.selection_mode = mode;
            project.selected_element = Some(1);
            project.selection = picks;
            project.save(&path).unwrap();
            assert_eq!(Project::load(&path).unwrap(), project);
        }
    }
}
