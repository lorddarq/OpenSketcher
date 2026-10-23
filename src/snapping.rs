//! Drawing planes and screen-space acquisition of actual evaluated geometry.
use crate::{
    model::{Body, Geometry, Shape},
    selection,
    viewport::Camera,
};
use eframe::egui::{Pos2, Rect};
use serde::{Deserialize, Serialize};
use truck_modeling::{EuclideanSpace, InnerSpace, Matrix3, Point3, Vector3};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Preferences {
    pub geometry: bool,
    pub grid_axes: bool,
    pub cut_on_creation: bool,
    pub project_to_base_plane: bool,
}
impl Default for Preferences {
    fn default() -> Self {
        Self {
            geometry: true,
            grid_axes: true,
            cut_on_creation: false,
            project_to_base_plane: false,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Plane {
    pub origin: Point3,
    pub u: Vector3,
    pub v: Vector3,
    pub normal: Vector3,
}
impl Plane {
    pub fn ground() -> Self {
        Self {
            origin: Point3::new(0., 0., 0.),
            u: Vector3::unit_x(),
            v: Vector3::unit_y(),
            normal: Vector3::unit_z(),
        }
    }
    pub fn face(g: &Geometry, face: usize) -> Option<Self> {
        let triangles: Vec<_> = g
            .triangles
            .iter()
            .zip(&g.triangle_faces)
            .filter(|(_, f)| **f == face)
            .map(|(t, _)| t)
            .collect();
        let first = triangles.first()?;
        let normal = (first[1] - first[0]).cross(first[2] - first[0]);
        if normal.magnitude2() < 1e-20 {
            return None;
        }
        let normal = normal.normalize();
        if triangles
            .iter()
            .flat_map(|t| t.iter())
            .any(|p| (*p - first[0]).dot(normal).abs() > 1e-6)
        {
            return None;
        }
        let reference = if normal.x.abs() < 0.9 {
            Vector3::unit_x()
        } else {
            Vector3::unit_y()
        };
        let u = (reference - normal * reference.dot(normal)).normalize();
        let v = normal.cross(u);
        Some(Self {
            origin: Point3::from_vec(normal * first[0].to_vec().dot(normal)),
            u,
            v,
            normal,
        })
    }
    fn coordinates(self, p: Point3) -> [f64; 2] {
        let d = p - self.origin;
        [d.dot(self.u), d.dot(self.v)]
    }
    fn world(self, p: [f64; 2]) -> Point3 {
        self.origin + self.u * p[0] + self.v * p[1]
    }
    fn contains(self, p: Point3) -> bool {
        (p - self.origin).dot(self.normal).abs() < 1e-6
    }
    pub fn body(self, shape: Shape, a: Point3, b: Point3, cut: bool) -> Result<Body, String> {
        let a = self.coordinates(a);
        let b = self.coordinates(b);
        let mut body = Body::from_points(shape, [a[0], a[1], 0.], [b[0], b[1], 0.])?;
        body.position = self.world([body.position[0], body.position[1]]).into();
        body.rotation = crate::regions::rotation_angles(
            Matrix3::from_cols(self.u, self.v, self.normal)
                * Matrix3::from_angle_z(truck_modeling::Deg(body.rotation[2])),
        );
        body.cuts_profiles = cut;
        Ok(body)
    }
    fn intersect(self, camera: &Camera, p: Pos2, rect: Rect) -> Option<Point3> {
        let (o, d) = camera.ray(p, rect);
        let denominator = d.dot(self.normal);
        if denominator.abs() < 1e-6 {
            return None;
        }
        let t = (self.origin - o).dot(self.normal) / denominator;
        (t >= 0.).then(|| o + d * t)
    }
}

pub struct SnapPoint {
    pub position: Point3,
    pub plane: Plane,
    pub label: &'static str,
    pub guide: Option<[Point3; 2]>,
}
struct Target {
    point: Point3,
    owner: u64,
    label: &'static str,
    priority: u8,
    distance: f32,
}
fn targets(
    scene: &[(u64, Geometry)],
    camera: &Camera,
    rect: Rect,
    pointer: Pos2,
    plane: Option<Plane>,
) -> Vec<Target> {
    let mut targets = Vec::new();
    let mut add = |point: Point3, owner, label, priority, tolerance| {
        let distance = camera.project(point, rect).0.distance(pointer);
        if distance <= tolerance
            && plane.is_none_or(|plane| plane.contains(point))
            && selection::visible(scene, camera, point, rect)
        {
            targets.push(Target {
                point,
                owner,
                label,
                priority,
                distance,
            });
        }
    };
    for (owner, g) in scene {
        for point in &g.points {
            add(*point, *owner, "Endpoint", 0, 10.);
        }
        let mut lengths = vec![0.; g.edge_count];
        for (edge, group) in g.edges.iter().zip(&g.edge_groups) {
            lengths[*group] += (edge[1] - edge[0]).magnitude();
        }
        let mut traveled = vec![0.; g.edge_count];
        for (edge, group) in g.edges.iter().zip(&g.edge_groups) {
            let length = (edge[1] - edge[0]).magnitude();
            let half = lengths[*group] / 2.;
            if length > 1e-12 && traveled[*group] <= half && traveled[*group] + length >= half {
                add(
                    edge[0] + (edge[1] - edge[0]) * ((half - traveled[*group]) / length),
                    *owner,
                    "Midpoint",
                    1,
                    9.,
                );
            }
            traveled[*group] += length;
            let a = camera.project(edge[0], rect).0;
            let b = camera.project(edge[1], rect).0;
            let ab = b - a;
            let t = ((pointer - a).dot(ab) / ab.length_sq().max(1e-8)).clamp(0., 1.) as f64;
            add(edge[0] + (edge[1] - edge[0]) * t, *owner, "Edge", 2, 8.);
        }
    }
    targets.sort_by(|a, b| {
        a.priority
            .cmp(&b.priority)
            .then(a.distance.total_cmp(&b.distance))
    });
    targets
}

pub fn acquire_plane(
    scene: &[(u64, Geometry)],
    camera: &Camera,
    rect: Rect,
    pointer: Pos2,
    selected: Option<(u64, usize)>,
    geometry: bool,
    base_plane: bool,
) -> Result<Plane, String> {
    if let Some((owner, face)) = selected {
        return scene.iter().find(|(id,_)|*id==owner).and_then(|(_,g)|Plane::face(g,face))
            .ok_or_else(||"Drawing requires a planar support face. Select a planar face or clear the selection.".into());
    }
    if base_plane {
        return Ok(Plane::ground());
    }
    // Choosing a support face is placement, not snapping; disabling Snap must
    // not silently move drawing back to the ground plane.
    if let Some((_, owner, face)) = selection::surface_hit(scene, camera, pointer, rect) {
        return scene
            .iter()
            .find(|(id, _)| *id == owner)
            .and_then(|(_, g)| Plane::face(g, face))
            .ok_or_else(|| {
                "This surface is not planar. Select a planar support face before drawing.".into()
            });
    }
    if geometry {
        // Acquire silhouettes just outside a face too, using its visible boundary.
        let nearby = targets(scene, camera, rect, pointer, None);
        for target in &nearby {
            if let Some((_, g)) = scene.iter().find(|(id, _)| *id == target.owner) {
                let (_, direction) = camera.ray(pointer, rect);
                if let Some(plane) = (0..g.face_count)
                    .filter_map(|f| Plane::face(g, f))
                    .filter(|p| p.contains(target.point) && p.normal.dot(direction) < -1e-6)
                    .max_by(|a, b| {
                        a.normal
                            .dot(direction)
                            .abs()
                            .total_cmp(&b.normal.dot(direction).abs())
                    })
                {
                    return Ok(plane);
                }
            }
        }
        if let Some(target) = nearby.first() {
            let mut plane = Plane::ground();
            plane.origin.z = target.point.z;
            return Ok(plane);
        }
    }
    Ok(Plane::ground())
}

pub fn point(
    scene: &[(u64, Geometry)],
    camera: &Camera,
    rect: Rect,
    pointer: Pos2,
    plane: Plane,
    anchor: Option<Point3>,
    preferences: &Preferences,
) -> Result<SnapPoint, String> {
    let raw = plane
        .intersect(camera, pointer, rect)
        .ok_or("The drawing plane is edge-on. Change view or select another face.")?;
    if preferences.geometry
        && let Some(target) = targets(scene, camera, rect, pointer, Some(plane)).first()
    {
        return Ok(SnapPoint {
            position: target.point,
            plane,
            label: target.label,
            guide: None,
        });
    }
    let raw_uv = plane.coordinates(raw);
    let mut uv = raw_uv;
    let mut label = "Drawing plane";
    let mut guide = None;
    if preferences.grid_axes {
        uv = uv.map(|v| (v * 2.).round() / 2.);
        label = "Grid";
        if let Some(anchor) = anchor {
            let a = plane.coordinates(anchor);
            for axis in 0..2 {
                let mut aligned = raw_uv;
                aligned[axis] = a[axis];
                if camera
                    .project(plane.world(aligned), rect)
                    .0
                    .distance(pointer)
                    < 8.
                {
                    uv[axis] = a[axis];
                    guide = Some([anchor, plane.world(uv)]);
                    label = "Anchor alignment";
                }
            }
        }
    }
    if preferences.geometry {
        // Project visible boundary coordinates across the active plane. This infers
        // limits away from the actual edge without ever falling back to its base Z.
        for axis in 0..2 {
            let reference = scene
                .iter()
                .flat_map(|(_, g)| g.points.iter())
                .filter(|p| plane.contains(**p))
                .filter(|p| selection::visible(scene, camera, **p, rect))
                .filter_map(|p| {
                    let mut aligned = raw_uv;
                    aligned[axis] = plane.coordinates(*p)[axis];
                    let distance = camera
                        .project(plane.world(aligned), rect)
                        .0
                        .distance(pointer);
                    (distance < 8.).then_some((*p, aligned[axis], distance))
                })
                .min_by(|a, b| a.2.total_cmp(&b.2));
            if let Some((reference, coordinate, _)) = reference {
                uv[axis] = coordinate;
                guide = Some([reference, plane.world(uv)]);
                label = "Projected geometry limit";
            }
        }
    }
    Ok(SnapPoint {
        position: plane.world(uv),
        plane,
        label,
        guide,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        model::{evaluate, evaluate_scene},
        project::Project,
        viewport::View,
    };
    #[test]
    fn geometry_snaps_keep_extruded_support_planes_and_cutting_is_opt_in() {
        let mut host = Body::new(Shape::Rectangle);
        host.id = 1;
        host.width = 3.3;
        host.depth = 4.7;
        host.extrusion = 2.3;
        host.position = [1.2, -0.7, 0.4];
        let scene = evaluate_scene(&[host.clone()]).unwrap();
        let g = &scene[0].1;
        let top = 2.7;
        let face = g
            .triangles
            .iter()
            .zip(&g.triangle_faces)
            .find(|(t, _)| t.iter().all(|p| (p.z - top).abs() < 1e-7))
            .map(|(_, face)| *face)
            .unwrap();
        let mut camera = Camera::default();
        camera.preset(View::Top);
        let rect = Rect::from_min_size(Pos2::ZERO, eframe::egui::vec2(800., 600.));
        let preferences = Preferences::default();
        let corner = g
            .points
            .iter()
            .find(|p| (p.z - top).abs() < 1e-7 && (p.x - 2.85).abs() < 1e-7)
            .copied()
            .unwrap();
        let pointer = camera.project(corner, rect).0 + eframe::egui::vec2(2., 2.);
        let plane = acquire_plane(&scene, &camera, rect, pointer, None, true, false).unwrap();
        assert!(
            plane.contains(corner),
            "plane {plane:?}, corner {corner:?}, visible {}, targets {}",
            selection::visible(&scene, &camera, corner, rect),
            targets(&scene, &camera, rect, pointer, None).len()
        );
        let endpoint = point(&scene, &camera, rect, pointer, plane, None, &preferences).unwrap();
        assert_eq!(endpoint.label, "Endpoint");
        assert!((endpoint.position - corner).magnitude() < 1e-7);
        assert!((endpoint.position.z - top).abs() < 1e-7); // Neither base Z nor grid-rounded coordinates.
        let edge = g
            .edges
            .iter()
            .find(|e| e.iter().all(|p| (p.z - top).abs() < 1e-7))
            .unwrap();
        let middle = edge[0] + (edge[1] - edge[0]) * 0.5;
        let midpoint = point(
            &scene,
            &camera,
            rect,
            camera.project(middle, rect).0,
            plane,
            None,
            &preferences,
        )
        .unwrap();
        assert_eq!(midpoint.label, "Midpoint");
        assert!((midpoint.position - middle).magnitude() < 1e-7);
        let edge_point = edge[0] + (edge[1] - edge[0]) * 0.3;
        let snapped = point(
            &scene,
            &camera,
            rect,
            camera.project(edge_point, rect).0,
            plane,
            None,
            &preferences,
        )
        .unwrap();
        assert_eq!(snapped.label, "Edge");
        assert!((snapped.position - edge_point).magnitude() < 1e-6);
        // Beyond the face, infer its actual non-grid X limit while retaining captured depth.
        let raw = Point3::new(2.79, 3., top);
        let projected = point(
            &scene,
            &camera,
            rect,
            camera.project(raw, rect).0,
            plane,
            Some(corner),
            &preferences,
        )
        .unwrap();
        assert_eq!(projected.label, "Projected geometry limit");
        assert!((projected.position.x - 2.85).abs() < 1e-7);
        assert!((projected.position.z - top).abs() < 1e-7);
        let line = plane.body(Shape::Line, corner, middle, false).unwrap();
        assert!(
            evaluate(&line)
                .unwrap()
                .points
                .iter()
                .all(|p| (p.z - top).abs() < 1e-6)
        );
        let no_snap_plane = acquire_plane(
            &scene,
            &camera,
            rect,
            camera.project(Point3::new(1.2, -0.7, top), rect).0,
            None,
            false,
            false,
        )
        .unwrap();
        assert!(no_snap_plane.contains(corner));
        let base = acquire_plane(&scene, &camera, rect, rect.center(), None, true, true).unwrap();
        assert_eq!(base.origin.z, 0.);
        let pinned = acquire_plane(
            &scene,
            &camera,
            rect,
            Pos2::ZERO,
            Some((1, face)),
            false,
            false,
        )
        .unwrap();
        assert!(pinned.contains(corner));
        let disabled = Preferences {
            geometry: false,
            grid_axes: false,
            cut_on_creation: false,
            project_to_base_plane: false,
        };
        let raw = Point3::new(1.12, -0.13, top);
        let unsnapped = point(
            &scene,
            &camera,
            rect,
            camera.project(raw, rect).0,
            pinned,
            None,
            &disabled,
        )
        .unwrap();
        assert!((unsnapped.position - raw).magnitude() < 1e-5);
        // Vertical supports use Y/Z coordinates and never fall back to the world ground.
        camera.preset(View::Side);
        let side = g
            .triangles
            .iter()
            .zip(&g.triangle_faces)
            .find(|(t, _)| t.iter().all(|p| (p.x - 2.85).abs() < 1e-7))
            .map(|(_, face)| *face)
            .unwrap();
        let plane = acquire_plane(
            &scene,
            &camera,
            rect,
            rect.center(),
            Some((1, side)),
            true,
            false,
        )
        .unwrap();
        let patch = plane
            .body(
                Shape::Rectangle,
                Point3::new(2.85, -1., 1.),
                Point3::new(2.85, 0., 2.),
                false,
            )
            .unwrap();
        assert!(
            evaluate(&patch)
                .unwrap()
                .triangles
                .iter()
                .flatten()
                .all(|p| (p.x - 2.85).abs() < 1e-6)
        );
        assert!(!preferences.cut_on_creation && !Body::new(Shape::Rectangle).cuts_profiles);
        let mut inner = Body::new(Shape::Rectangle);
        inner.id = 2;
        inner.width = 1.;
        inner.depth = 2.;
        inner.position = host.position;
        let area = |g: &Geometry| {
            g.triangles
                .iter()
                .filter(|t| t.iter().all(|p| (p.z - top).abs() < 1e-7))
                .map(|t| (t[1] - t[0]).cross(t[2] - t[0]).magnitude() / 2.)
                .sum::<f64>()
        };
        assert!(
            (area(&evaluate_scene(&[host.clone(), inner.clone()]).unwrap()[0].1) - 15.51).abs()
                < 1e-6
        );
        inner.cuts_profiles = true;
        assert!(
            (area(&evaluate_scene(&[host.clone(), inner.clone()]).unwrap()[0].1) - 13.51).abs()
                < 1e-6
        );
        let mut project = Project::new(vec![host, inner], camera, true, None);
        project.drawing.geometry = false;
        project.drawing.cut_on_creation = true;
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("snap.osk");
        project.save(&path).unwrap();
        assert_eq!(Project::load(&path).unwrap(), project);
        // Old files keep their automatic cuts rather than silently healing old geometry.
        let mut legacy = serde_json::to_value(&project).unwrap();
        legacy["schema_version"] = 3.into();
        legacy["modeling_version"] = 3.into();
        legacy.as_object_mut().unwrap().remove("drawing");
        for body in legacy["bodies"].as_array_mut().unwrap() {
            body.as_object_mut().unwrap().remove("cuts_profiles");
        }
        std::fs::write(&path, serde_json::to_vec(&legacy).unwrap()).unwrap();
        let old = Project::load(&path).unwrap();
        assert!(old.bodies.iter().all(|b| b.cuts_profiles));
        assert!(!old.drawing.cut_on_creation);
        assert!((area(&evaluate_scene(&old.bodies).unwrap()[0].1) - 13.51).abs() < 1e-6);
        legacy["schema_version"] = 1.into();
        legacy["modeling_version"] = 1.into();
        legacy.as_object_mut().unwrap().remove("selection_mode");
        legacy.as_object_mut().unwrap().remove("selected_element");
        std::fs::write(&path, serde_json::to_vec(&legacy).unwrap()).unwrap();
        let original = Project::load(&path).unwrap();
        assert!(original.bodies.iter().all(|b| !b.cuts_profiles));
        assert!((area(&evaluate_scene(&original.bodies).unwrap()[0].1) - 15.51).abs() < 1e-6);
    }
}
