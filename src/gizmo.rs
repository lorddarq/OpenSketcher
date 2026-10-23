//! Screen-sized world-space transform handles and their constrained drag mathematics.
use crate::viewport::Camera;
use eframe::egui::{self, Color32, Pos2, Rect, Stroke};
use truck_modeling::{EuclideanSpace, InnerSpace, Matrix4, Point3, Rad, SquareMatrix, Vector3};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Handle {
    Axis(usize),
    Plane(usize),
    Ring(usize),
}

// Press/release may arrive together. PointerState then has no press_origin and
// interact_pos is already the release position; retain the actual event origin.
pub fn press_origin(events: &[egui::Event]) -> Option<Pos2> {
    events.iter().find_map(|event| match event {
        egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed: true,
            ..
        } => Some(*pos),
        _ => None,
    })
}

pub struct Drag {
    pub handle: Handle,
    pub start: Pos2,
    pub pivot: Point3,
    pub matrix: Matrix4,
    pub value: String,
}
impl Drag {
    pub fn new(handle: Handle, start: Pos2, pivot: Point3) -> Self {
        Self {
            handle,
            start,
            pivot,
            matrix: Matrix4::identity(),
            value: String::new(),
        }
    }
    pub fn update(&mut self, camera: &Camera, point: Pos2, rect: Rect, snap: bool) {
        let scale = camera.span as f64 / rect.height().max(1.) as f64;
        let quantize = |v: f64, step: f64| if snap { (v / step).round() * step } else { v };
        match self.handle {
            Handle::Axis(axis) => {
                let mut delta = Vector3::new(0., 0., 0.);
                let parameter = |p| {
                    let (o, d) = camera.ray(p, rect);
                    let u = unit(axis);
                    let w = o - self.pivot;
                    let dot = d.dot(u);
                    let den = 1. - dot * dot;
                    (den > 1e-6).then(|| (w.dot(u) - dot * w.dot(d)) / den)
                };
                let distance = match (parameter(point), parameter(self.start)) {
                    (Some(a), Some(b)) => a - b,
                    _ => (self.start.y - point.y) as f64 * scale,
                };
                delta[axis] = quantize(distance, 0.5);
                self.matrix = Matrix4::from_translation(delta);
                self.value = format!("{} · {:.3} units", LABELS[axis], delta[axis]);
            }
            Handle::Plane(axis) => {
                if let (Some(a), Some(b)) = (
                    camera.plane(self.start, rect, self.pivot.into(), axis),
                    camera.plane(point, rect, self.pivot.into(), axis),
                ) {
                    let delta = Vector3::from(std::array::from_fn::<_, 3, _>(|i| {
                        if i == axis {
                            0.
                        } else {
                            quantize(b[i] - a[i], 0.5)
                        }
                    }));
                    self.matrix = Matrix4::from_translation(delta);
                    self.value = format!(
                        "{}{} · {:.3}, {:.3} units",
                        LABELS[(axis + 1) % 3],
                        LABELS[(axis + 2) % 3],
                        delta[(axis + 1) % 3],
                        delta[(axis + 2) % 3]
                    );
                }
            }
            Handle::Ring(axis) => {
                let angle = |p| {
                    camera
                        .plane(p, rect, self.pivot.into(), axis)
                        .and_then(|p| {
                            let x = p[(axis + 1) % 3] - self.pivot[(axis + 1) % 3];
                            let y = p[(axis + 2) % 3] - self.pivot[(axis + 2) % 3];
                            (x.hypot(y) > 1e-8).then(|| y.atan2(x))
                        })
                };
                let delta = match (angle(point), angle(self.start)) {
                    (Some(a), Some(b)) => (a - b).sin().atan2((a - b).cos()).to_degrees(),
                    _ => (point.x - self.start.x - self.start.y + point.y) as f64 * 0.5,
                };
                let degrees = quantize(delta, 15.);
                self.matrix = Matrix4::from_translation(self.pivot.to_vec())
                    * Matrix4::from_axis_angle(unit(axis), Rad(degrees.to_radians()))
                    * Matrix4::from_translation(-self.pivot.to_vec());
                self.value = format!("{} · {:.1}°", LABELS[axis], degrees);
            }
        }
    }
}
const LABELS: [&str; 3] = ["X", "Y", "Z"];
const COLORS: [Color32; 3] = [
    Color32::from_rgb(245, 85, 85),
    Color32::from_rgb(90, 220, 120),
    Color32::from_rgb(90, 155, 255),
];
fn unit(axis: usize) -> Vector3 {
    let mut v = Vector3::new(0., 0., 0.);
    v[axis] = 1.;
    v
}
fn distance(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_sq().max(1e-8)).clamp(0., 1.);
    p.distance(a + ab * t)
}
fn inside(p: Pos2, polygon: &[Pos2]) -> bool {
    let mut signs = [false; 2];
    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        let c = (b - a).rot90().dot(p - a);
        signs[usize::from(c >= 0.)] = true;
    }
    !(signs[0] && signs[1])
}
fn handles(camera: &Camera, rect: Rect, pivot: Point3, rotate: bool) -> Vec<(Handle, Vec<Pos2>)> {
    let scale = camera.span as f64 / rect.height().max(1.) as f64;
    let mut handles = Vec::new();
    for axis in 0..3 {
        if rotate {
            let points = (0..=96)
                .map(|i| {
                    let angle = std::f64::consts::TAU * i as f64 / 96.;
                    camera
                        .project(
                            pivot
                                + (unit((axis + 1) % 3) * angle.cos()
                                    + unit((axis + 2) % 3) * angle.sin())
                                    * scale
                                    * 72.,
                            rect,
                        )
                        .0
                })
                .collect();
            handles.push((Handle::Ring(axis), points));
        } else {
            let center = camera.project(pivot, rect).0;
            let tip = camera.project(pivot + unit(axis) * scale * 100., rect).0;
            let tip = if center.distance(tip) < 12. {
                center + egui::vec2(0., -32.)
            } else {
                tip
            };
            let direction = (tip - center).normalized();
            handles.push((Handle::Axis(axis), vec![center + direction * 14., tip]));
            let a = unit((axis + 1) % 3) * scale * 100.;
            let b = unit((axis + 2) % 3) * scale * 100.;
            let points: Vec<_> = [(0.22, 0.22), (0.44, 0.22), (0.44, 0.44), (0.22, 0.44)]
                .into_iter()
                .map(|(x, y)| camera.project(pivot + a * x + b * y, rect).0)
                .collect();
            let area = (points[1] - points[0])
                .rot90()
                .dot(points[3] - points[0])
                .abs();
            if area > 24. {
                handles.push((Handle::Plane(axis), points));
            }
        }
    }
    handles
}
pub fn hit(
    camera: &Camera,
    rect: Rect,
    pivot: Point3,
    rotate: bool,
    point: Pos2,
) -> Option<Handle> {
    handles(camera, rect, pivot, rotate)
        .iter()
        .filter_map(|(handle, points)| {
            let distance = if matches!(handle, Handle::Plane(_)) {
                if !inside(point, points) {
                    return None;
                }
                0.
            } else {
                points
                    .windows(2)
                    .map(|p| distance(point, p[0], p[1]))
                    .fold(f32::INFINITY, f32::min)
            };
            (distance <= 8.).then_some((*handle, distance))
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|v| v.0)
}
pub fn draw(
    painter: &egui::Painter,
    camera: &Camera,
    rect: Rect,
    pivot: Point3,
    rotate: bool,
    active: Option<Handle>,
    pointer: Option<Pos2>,
) {
    let painter = painter.with_clip_rect(rect);
    let hovered = pointer.and_then(|p| hit(camera, rect, pivot, rotate, p));
    for (handle, points) in handles(camera, rect, pivot, rotate) {
        let axis = match handle {
            Handle::Axis(i) | Handle::Plane(i) | Handle::Ring(i) => i,
        };
        let color = if active == Some(handle) || hovered == Some(handle) {
            Color32::YELLOW
        } else {
            COLORS[axis]
        };
        match handle {
            Handle::Plane(_) => {
                painter.add(egui::Shape::convex_polygon(
                    points,
                    color.gamma_multiply(0.22),
                    Stroke::new(1.5, color),
                ));
            }
            Handle::Axis(_) => {
                painter.line_segment([points[0], points[1]], Stroke::new(5., Color32::BLACK));
                painter.arrow(points[0], points[1] - points[0], Stroke::new(2.5, color));
                painter.text(
                    points[1] + (points[1] - points[0]).normalized() * 13.,
                    egui::Align2::CENTER_CENTER,
                    LABELS[axis],
                    egui::FontId::proportional(14.),
                    color,
                );
            }
            Handle::Ring(_) => {
                painter.add(egui::Shape::line(
                    points.clone(),
                    Stroke::new(4.5, Color32::BLACK),
                ));
                painter.add(egui::Shape::line(points.clone(), Stroke::new(2., color)));
                let p = points[axis * 19 + 10];
                painter.text(
                    p,
                    egui::Align2::CENTER_CENTER,
                    LABELS[axis],
                    egui::FontId::proportional(14.),
                    color,
                );
            }
        }
    }
    let center = camera.project(pivot, rect).0;
    painter.circle_filled(center, 4., Color32::WHITE);
    painter.circle_stroke(center, 5., Stroke::new(1.5, Color32::BLACK));
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::viewport::View;
    use truck_modeling::Transform;
    #[test]
    fn handles_hit_and_drag_constraints_survive_presets_and_snapping() {
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(800., 600.));
        let pivot = Point3::new(0., 0., 0.);
        for view in [View::Isometric, View::Top, View::Side] {
            let mut camera = Camera::default();
            camera.preset(view);
            for axis in 0..3 {
                let (handle, points) = handles(&camera, rect, pivot, false)
                    .into_iter()
                    .find(|(h, _)| *h == Handle::Axis(axis))
                    .unwrap();
                let start = points[0].lerp(points[1], 0.8);
                assert!(hit(&camera, rect, pivot, false, start).is_some());
                let mut drag = Drag::new(handle, start, pivot);
                drag.update(&camera, start + egui::vec2(31., -27.), rect, true);
                let delta = drag.matrix.transform_point(pivot) - pivot;
                assert!((delta[axis] * 2. - (delta[axis] * 2.).round()).abs() < 1e-8);
                for i in 0..3 {
                    if i != axis {
                        assert!(delta[i].abs() < 1e-8);
                    }
                }
            }
            assert!(hit(&camera, rect, pivot, true, Pos2::new(-100., -100.)).is_none());
        }
        let mut camera = Camera::default();
        camera.preset(View::Top);
        let start = camera.project(Point3::new(1., 0., 0.), rect).0;
        let end = camera.project(Point3::new(0., 1., 0.), rect).0;
        let events = vec![
            egui::Event::PointerButton {
                pos: start,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: egui::Modifiers::NONE,
            },
            egui::Event::PointerMoved(end),
            egui::Event::PointerButton {
                pos: end,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: egui::Modifiers::NONE,
            },
        ];
        assert_eq!(press_origin(&events), Some(start));
        let mut drag = Drag::new(Handle::Ring(2), press_origin(&events).unwrap(), pivot);
        drag.update(&camera, end, rect, true);
        assert!(
            (drag.matrix.transform_point(Point3::new(1., 0., 0.)) - Point3::new(0., 1., 0.))
                .magnitude()
                < 1e-5
        );
        let mut drag = Drag::new(Handle::Plane(2), start, pivot);
        drag.update(&camera, start + egui::vec2(33., -18.), rect, true);
        let delta = drag.matrix.transform_point(pivot) - pivot;
        assert_eq!(delta.z, 0.);
        assert_eq!(delta.x, 0.5);
        assert_eq!(delta.y, 0.5);
    }
}
