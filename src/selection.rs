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
                pick.1.is_some_and(|i| i < mode.count(g))
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
pub fn surface_hit(
    scene: &[(u64, Geometry)],
    camera: &Camera,
    p: egui::Pos2,
    rect: egui::Rect,
) -> Option<(f64, u64, usize)> {
    let (o, d) = camera.ray(p, rect);
    let hits: Vec<_> = scene
        .iter()
        .flat_map(|(id, g)| {
            g.triangles.iter().enumerate().filter_map(move |(i, t)| {
                triangle_hit(o, d, t)
                    .filter(|depth| *depth <= camera.span as f64 * 100.)
                    .map(|depth| (depth, *id, g.triangle_faces[i]))
            })
        })
        .collect();
    let nearest = hits.iter().map(|hit| hit.0).min_by(f64::total_cmp)?;
    // Prefer the latest profile only when surfaces are actually coincident.
    hits.into_iter()
        .filter(|hit| hit.0 <= nearest + 1e-7)
        .max_by_key(|hit| (hit.1, hit.2))
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
        // A visible standalone line needs its screen-space acquisition radius
        // even when it lies on a filled face; otherwise the face always wins.
        candidates.push((6.5, depth, id, None));
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
