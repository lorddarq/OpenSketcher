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
    click_many(picks, picked.as_slice(), extend);
}
/// The last item is the clicked primary. Shift toggles the whole chain together.
pub fn click_many(picks: &mut Vec<Pick>, picked: &[Pick], extend: bool) {
    let remove = extend && picked.last().is_some_and(|primary| picks.contains(primary));
    if !extend {
        picks.clear();
    }
    if remove {
        picks.retain(|p| !picked.contains(p));
    } else {
        for pick in picked {
            if !picks.contains(pick) {
                picks.push(*pick);
            }
        }
    }
}

/// Expand a logical edge along analytic tangent runs, stopping at ambiguous joins.
/// Display samples only disambiguate retained-edge identity; curve derivatives
/// determine tangency through the same helper used for swept-wall grouping.
pub fn tangent_chain(g: &Geometry, seed: usize) -> Vec<usize> {
    use std::collections::{BTreeSet, HashMap, HashSet};
    use truck_modeling::{ParametricCurve, SearchParameter, VertexID};
    if seed >= g.edge_count || g.internal_edges.contains(&seed) {
        return Vec::new();
    }
    let seed = g.edge_group(seed);
    let Some(shell) = &g.shell else {
        return vec![seed];
    };
    let key = |p: Point3| [p.x, p.y, p.z].map(|v| (v * 1e6).round() as i64);
    let pair = |a: Point3, b: Point3| {
        let (a, b) = (key(a), key(b));
        if a < b { (a, b) } else { (b, a) }
    };
    let mut candidates = HashMap::<_, Vec<usize>>::new();
    let mut probes = g
        .topology
        .edges
        .iter()
        .map(|[a, b]| g.points[*a] + (g.points[*b] - g.points[*a]) * 0.5)
        .collect::<Vec<_>>();
    let mut distances = vec![0.; probes.len()];
    for (i, [a, b]) in g.topology.edges.iter().enumerate() {
        candidates
            .entry(pair(g.points[*a], g.points[*b]))
            .or_default()
            .push(i);
    }
    for (edge, group) in g.edges.iter().zip(&g.edge_groups) {
        let Some([a, b]) = g.topology.edges.get(*group) else {
            continue;
        };
        let ab = g.points[*b] - g.points[*a];
        for p in edge {
            let delta = *p - g.points[*a];
            let distance = if ab.magnitude2() > 1e-20 {
                delta.cross(ab).magnitude2() / ab.magnitude2()
            } else {
                delta.magnitude2()
            };
            if distance > distances[*group] + 1e-20 {
                probes[*group] = *p;
                distances[*group] = distance;
            }
        }
    }
    let mut edges = HashMap::new();
    let mut seen = HashSet::new();
    for edge in shell.edge_iter() {
        if !seen.insert(edge.id()) {
            continue;
        }
        let Some(raw) = candidates.get(&pair(edge.front().point(), edge.back().point())) else {
            continue;
        };
        let curve = edge.oriented_curve();
        let matches = raw
            .iter()
            .filter(|index| !g.internal_edges.contains(index))
            .filter(|index| {
                curve
                    .search_parameter(probes[**index], None, 64)
                    .is_some_and(|t| (curve.subs(t) - probes[**index]).magnitude() < 1e-7)
            })
            .collect::<Vec<_>>();
        if matches.len() == 1 {
            edges.insert(edge.id(), g.edge_group(*matches[0]));
        }
    }
    let mut joins = HashMap::<(usize, VertexID), BTreeSet<usize>>::new();
    for face in shell {
        for wire in face.boundaries() {
            let groups = crate::model::tangent_groups(&wire);
            for i in 0..wire.len() {
                let next = (i + 1) % wire.len();
                if groups[i] != groups[next] {
                    continue;
                }
                if let (Some(&a), Some(&b)) =
                    (edges.get(&wire[i].id()), edges.get(&wire[next].id()))
                {
                    let vertex = wire[i].back().id();
                    joins.entry((a, vertex)).or_default().insert(b);
                    joins.entry((b, vertex)).or_default().insert(a);
                }
            }
        }
    }
    let mut adjacent = vec![BTreeSet::new(); g.edge_count];
    for ((edge, vertex), neighbors) in &joins {
        if neighbors.len() != 1 {
            continue;
        }
        let other = *neighbors.first().unwrap();
        if other != *edge
            && joins
                .get(&(other, *vertex))
                .is_some_and(|back| back.len() == 1 && back.contains(edge))
        {
            adjacent[*edge].insert(other);
        }
    }
    let mut chain = BTreeSet::new();
    let mut pending = vec![seed];
    while let Some(edge) = pending.pop() {
        if !chain.insert(edge) {
            continue;
        }
        pending.extend(adjacent[edge].iter().filter(|e| !chain.contains(e)));
    }
    chain.remove(&seed);
    let mut result = chain.into_iter().collect::<Vec<_>>();
    result.push(seed);
    result
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
    fn tangent_chains_follow_analytic_curves_and_toggle_as_one_selection() {
        use truck_modeling::{Deg, Matrix4, Vector3, builder};
        let points = [
            [0., 0., 0.],
            [2., 0., 0.],
            [3., 1., 0.],
            [3., 3., 0.],
            [0., 3., 0.],
        ];
        let vertices = points.map(|p| builder::vertex(Point3::from(p)));
        let arc = builder::circle_arc(
            &vertices[1],
            &vertices[2],
            Point3::new(
                2. + std::f64::consts::FRAC_1_SQRT_2,
                1. - std::f64::consts::FRAC_1_SQRT_2,
                0.,
            ),
        );
        let wire: truck_modeling::Wire = vec![
            builder::line(&vertices[0], &vertices[1]),
            arc,
            builder::line(&vertices[2], &vertices[3]),
            builder::line(&vertices[3], &vertices[4]),
            builder::line(&vertices[4], &vertices[0]),
        ]
        .into();
        let face = builder::try_attach_plane(&[wire]).unwrap();
        for affine in [false, true] {
            let mut shell: truck_modeling::Shell = vec![face.clone()].into();
            if affine {
                shell = builder::transformed(
                    &shell,
                    Matrix4::from_translation(Vector3::new(2., -1., 3.))
                        * Matrix4::from_angle_x(Deg(23.))
                        * Matrix4::from_nonuniform_scale(1.7, 0.6, 1.2),
                );
            }
            let g = crate::model::tessellate(&shell).unwrap();
            for seed in 0..3 {
                let chain = tangent_chain(&g, seed);
                assert_eq!(chain.last(), Some(&seed));
                assert_eq!(
                    chain
                        .iter()
                        .copied()
                        .collect::<std::collections::BTreeSet<_>>(),
                    [0, 1, 2].into()
                );
                let expanded = chain.iter().map(|e| (7, Some(*e))).collect::<Vec<_>>();
                let mut picks = vec![(9, Some(1))];
                click_many(&mut picks, &expanded, true);
                assert_eq!(picks.last(), Some(&(7, Some(seed))));
                assert_eq!(picks.len(), 4);
                click_many(&mut picks, &expanded, true);
                assert_eq!(picks, vec![(9, Some(1))]);
                click_many(&mut picks, &expanded, false);
                assert_eq!(picks, expanded);
                click_many(&mut picks, &[], true);
                assert_eq!(picks, expanded);
                click_many(&mut picks, &[], false);
                assert!(picks.is_empty());
            }
            assert_eq!(tangent_chain(&g, 3), vec![3]);
            assert_eq!(tangent_chain(&g, 4), vec![4]);
            assert!(tangent_chain(&g, 99).is_empty());
        }
        let half = evaluate(&Body::new(Shape::Semicircle)).unwrap();
        // Arc and diameter share endpoints but must not be misidentified.
        for edge in 0..half.edge_count {
            assert_eq!(tangent_chain(&half, edge), vec![edge]);
        }
        // Two tangent continuations at the same endpoint are ambiguous, even
        // though the shared edge has only two neighbors in the edge graph.
        let [a, b, c, p, d] = [
            [0., 0., 0.],
            [1., 0., 0.],
            [2., 0., 0.],
            [2., 2., 0.],
            [2., -1., 0.],
        ]
        .map(|p| builder::vertex(Point3::from(p)));
        let shared = builder::line(&a, &b);
        let branch = truck_modeling::Edge::new(
            &b,
            &d,
            truck_modeling::Curve::BSplineCurve(truck_modeling::BSplineCurve::new(
                truck_modeling::KnotVec::bezier_knot(2),
                vec![b.point(), Point3::new(1.5, 0., 0.), d.point()],
            )),
        );
        let wires: [truck_modeling::Wire; 2] = [
            vec![
                shared.clone(),
                builder::line(&b, &c),
                builder::line(&c, &p),
                builder::line(&p, &a),
            ]
            .into(),
            vec![shared.inverse(), builder::line(&a, &d), branch.inverse()].into(),
        ];
        let shell = wires
            .map(|wire| builder::try_attach_plane(&[wire]).unwrap())
            .into_iter()
            .collect::<truck_modeling::Shell>();
        let g = crate::model::tessellate(&shell).unwrap();
        assert_eq!(tangent_chain(&g, 0), vec![0]);
        assert_eq!(tangent_chain(&g, 1), vec![1]);
        let mut body = Body::new(Shape::Circle);
        body.extrusion = 3.;
        let circle = evaluate(&body).unwrap();
        for edge in 0..circle.edge_count {
            if circle.internal_edges.contains(&edge) {
                assert!(tangent_chain(&circle, edge).is_empty());
            } else {
                assert_eq!(tangent_chain(&circle, edge), vec![circle.edge_group(edge)]);
            }
        }
    }

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
