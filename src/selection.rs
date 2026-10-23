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
    (u >= -1e-8 && v >= -1e-8 && u + v <= 1. + 1e-8 && t >= 0.).then_some(t)
}
fn surface_hit(
    scene: &[(u64, Geometry)],
    camera: &Camera,
    p: egui::Pos2,
    rect: egui::Rect,
) -> Option<(f64, u64, usize)> {
    let (o, d) = camera.ray(p, rect);
    scene
        .iter()
        .flat_map(|(id, g)| {
            g.triangles.iter().enumerate().filter_map(move |(i, t)| {
                triangle_hit(o, d, t)
                    .filter(|depth| *depth <= camera.span as f64 * 100.)
                    .map(|depth| (depth, *id, g.triangle_faces[i]))
            })
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
}

pub fn visible(scene: &[(u64, Geometry)], camera: &Camera, p: Point3, rect: egui::Rect) -> bool {
    let (screen, z) = camera.project(p, rect);
    let depth = camera.span as f64 * 50. - z as f64;
    let tolerance = (camera.span as f64 * 1e-5).max(1e-5);
    depth >= 0.
        && depth <= camera.span as f64 * 100.
        && surface_hit(scene, camera, screen, rect).is_none_or(|hit| depth <= hit.0 + tolerance)
}

pub fn pick(
    scene: &[(u64, Geometry)],
    camera: &Camera,
    p: egui::Pos2,
    rect: egui::Rect,
    mode: SelectionMode,
) -> Option<(u64, Option<usize>)> {
    let surface = surface_hit(scene, camera, p, rect);
    if mode == SelectionMode::Face {
        return surface.map(|(_, id, face)| (id, Some(face)));
    }
    let mut candidates = Vec::new();
    if mode == SelectionMode::Volume
        && let Some((depth, id, _)) = surface
    {
        candidates.push((0., depth, id, None));
    }
    for (id, g) in scene {
        if mode == SelectionMode::Point {
            for (index, point) in g.points.iter().enumerate() {
                let (screen, z) = camera.project(*point, rect);
                let distance = screen.distance(p);
                if distance <= 8. && visible(scene, camera, *point, rect) {
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
                let (sa, _) = camera.project(*a, rect);
                let (sb, _) = camera.project(*b, rect);
                let ab = sb - sa;
                let t = ((p - sa).dot(ab) / ab.length_sq().max(1e-8)).clamp(0., 1.);
                let distance = p.distance(sa + ab * t);
                let world = *a + (*b - *a) * t as f64;
                if distance <= 6. && visible(scene, camera, world, rect) {
                    let z = camera.project(world, rect).1;
                    candidates.push((
                        distance,
                        camera.span as f64 * 50. - z as f64,
                        *id,
                        if mode == SelectionMode::Edge {
                            Some(g.edge_groups[index])
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        model::{Body, Shape, evaluate},
        viewport::View,
    };
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
        let line = Body::new(Shape::Line);
        let scene = vec![(2, evaluate(&line).unwrap())];
        assert!(pick(&scene, &camera, center, rect, SelectionMode::Face).is_none());
        assert_eq!(
            pick(&scene, &camera, center, rect, SelectionMode::Edge),
            Some((2, Some(0)))
        );
    }
}
