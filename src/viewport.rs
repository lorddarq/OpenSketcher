#[cfg(test)]
use crate::model::evaluate_scene;
use crate::model::{Body, Geometry};
use crate::selection::{self, SelectionMode};
use eframe::{egui, egui_wgpu, wgpu};
use serde::{Deserialize, Serialize};
use truck_modeling::{InnerSpace, Point3, Vector3};
use wgpu::util::DeviceExt;

// Smooth render samples within a logical face, retaining sharp authored boundaries.
fn surface_normals(g: &Geometry) -> Vec<[Vector3; 3]> {
    let key = |face, p: Point3| (face, [p.x, p.y, p.z].map(|v| (v * 1e6).round() as i64));
    let mut sums = std::collections::HashMap::new();
    for (t, face) in g.triangles.iter().zip(&g.triangle_faces) {
        let normal = (t[1] - t[0]).cross(t[2] - t[0]);
        for p in t {
            *sums
                .entry(key(g.face_group(*face), *p))
                .or_insert(Vector3::new(0., 0., 0.)) += normal;
        }
    }
    g.triangles
        .iter()
        .zip(&g.triangle_faces)
        .map(|(t, face)| {
            t.map(|p| {
                if let Some(Some((ring, axis))) = g.cylinder_supports.get(*face) {
                    let ring = &g.circles[*ring];
                    let plane_normal = ring.u.cross(ring.v).normalize();
                    let delta = p - ring.center;
                    let radial = delta - *axis * (delta.dot(plane_normal) / axis.dot(plane_normal));
                    let uu = ring.u.dot(ring.u);
                    let uv = ring.u.dot(ring.v);
                    let vv = ring.v.dot(ring.v);
                    let det = uu * vv - uv * uv;
                    let a = (radial.dot(ring.u) * vv - radial.dot(ring.v) * uv) / det;
                    let b = (radial.dot(ring.v) * uu - radial.dot(ring.u) * uv) / det;
                    let normal = (ring.v * a - ring.u * b).cross(*axis).normalize();
                    let outward = (t[1] - t[0]).cross(t[2] - t[0]);
                    return normal * if normal.dot(outward) < 0. { -1. } else { 1. };
                }
                let n = sums[&key(g.face_group(*face), p)];
                if n.magnitude2() > 1e-20 {
                    n.normalize()
                } else {
                    (t[1] - t[0]).cross(t[2] - t[0]).normalize()
                }
            })
        })
        .collect()
}

// Immutable display work belongs to the evaluated scene, not to selection refreshes.
#[derive(Default)]
pub struct SceneDisplay {
    pub index: selection::SpatialIndex,
    shades: Vec<Vec<[f32; 3]>>,
}
impl SceneDisplay {
    pub fn new(scene: &[(u64, Geometry)]) -> Self {
        let light = Vector3::new(0.3, -0.4, 0.85).normalize();
        Self {
            index: selection::SpatialIndex::new(scene),
            shades: scene
                .iter()
                .map(|(_, g)| {
                    surface_normals(g)
                        .into_iter()
                        .map(|normals| normals.map(|n| (0.45 + 0.4 * n.dot(light).abs()) as f32))
                        .collect()
                })
                .collect(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum View {
    Isometric,
    Top,
    Side,
    Free,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Camera {
    pub view: View,
    yaw: f32,
    pitch: f32,
    pub span: f32,
    target: [f32; 3],
}

impl Default for Camera {
    fn default() -> Self {
        let mut camera = Self {
            view: View::Isometric,
            yaw: 0.0,
            pitch: 0.0,
            span: 12.0,
            target: [0.0, 0.0, 1.0],
        };
        camera.preset(View::Isometric);
        camera
    }
}

impl Camera {
    pub fn validate(&self) -> Result<(), String> {
        if !self
            .target
            .iter()
            .chain([&self.yaw, &self.pitch, &self.span])
            .all(|v| v.is_finite())
            || !(0.05..=1000000.).contains(&self.span)
            || self.target.iter().any(|v| v.abs() > 1000000.)
            || self.yaw.abs() > 1000000.
            || self.pitch.abs() > std::f32::consts::FRAC_PI_2 + 1e-5
        {
            return Err("Invalid camera properties.".into());
        }
        Ok(())
    }
    pub fn preset(&mut self, view: View) {
        self.view = view;
        (self.yaw, self.pitch) = match view {
            View::Top => (-std::f32::consts::FRAC_PI_2, std::f32::consts::FRAC_PI_2),
            View::Side => (0.0, 0.0),
            _ => (
                std::f32::consts::FRAC_PI_4,
                (1.0_f32 / 3.0_f32.sqrt()).asin(),
            ),
        };
    }

    fn basis(&self) -> [[f32; 3]; 3] {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        [
            [-sy, cy, 0.0],
            [-sp * cy, -sp * sy, cp],
            [cp * cy, cp * sy, sp],
        ]
    }

    pub fn project(&self, p: Point3, rect: egui::Rect) -> (egui::Pos2, f32) {
        let [r, u, b] = self.basis();
        let a = [
            p.x as f32 - self.target[0],
            p.y as f32 - self.target[1],
            p.z as f32 - self.target[2],
        ];
        let dot = |v: [f32; 3]| a.iter().zip(v).map(|(a, b)| a * b).sum::<f32>();
        (
            egui::pos2(
                rect.center().x + dot(r) * rect.height() / self.span,
                rect.center().y - dot(u) * rect.height() / self.span,
            ),
            dot(b),
        )
    }
    pub fn ray(&self, point: egui::Pos2, rect: egui::Rect) -> (Point3, Vector3) {
        let [r, u, b] = self.basis();
        let x = (point.x - rect.center().x) * self.span / rect.height().max(1.);
        let y = (rect.center().y - point.y) * self.span / rect.height().max(1.);
        let p = std::array::from_fn::<_, 3, _>(|i| {
            (self.target[i] + r[i] * x + u[i] * y + b[i] * self.span * 50.) as f64
        });
        (Point3::from(p), -Vector3::from(b.map(f64::from)))
    }
    pub fn plane(
        &self,
        point: egui::Pos2,
        rect: egui::Rect,
        origin: [f64; 3],
        axis: usize,
    ) -> Option<[f64; 3]> {
        let (o, d) = self.ray(point, rect);
        if d[axis].abs() < 1e-5 {
            return None;
        }
        let t = (origin[axis] - o[axis]) / d[axis];
        (t >= 0.).then(|| (o + d * t).into())
    }
    pub fn screen_plane(
        &self,
        point: egui::Pos2,
        rect: egui::Rect,
        pivot: Point3,
    ) -> Option<Point3> {
        let (o, d) = self.ray(point, rect);
        let normal = Vector3::from(self.basis()[2].map(f64::from));
        let denominator = d.dot(normal);
        (denominator.abs() > 1e-8).then(|| o + d * ((pivot - o).dot(normal) / denominator))
    }
    pub fn fit(&mut self, scene: &[(u64, Geometry)]) {
        let points: Vec<_> = scene
            .iter()
            .flat_map(|(_, g)| g.edges.iter().flatten())
            .collect();
        if points.is_empty() {
            *self = Self::default();
            return;
        }
        let low = std::array::from_fn::<_, 3, _>(|i| {
            points.iter().map(|p| p[i]).fold(f64::INFINITY, f64::min)
        });
        let high = std::array::from_fn::<_, 3, _>(|i| {
            points
                .iter()
                .map(|p| p[i])
                .fold(f64::NEG_INFINITY, f64::max)
        });
        self.target = std::array::from_fn(|i| ((low[i] + high[i]) / 2.) as f32);
        self.span = (0..3).map(|i| high[i] - low[i]).fold(0.5, f64::max) as f32 * 2.5;
    }
    pub fn interact(&mut self, response: &egui::Response, ui: &egui::Ui, orbit: bool) {
        let delta = ui.input(|i| i.pointer.delta());
        if orbit
            && response.dragged_by(egui::PointerButton::Primary)
            && !ui.input(|i| i.modifiers.shift)
        {
            self.yaw -= delta.x * 0.008;
            self.pitch = (self.pitch + delta.y * 0.008).clamp(-1.56, 1.56);
            self.view = View::Free;
        } else if response.dragged_by(egui::PointerButton::Middle)
            || response.dragged_by(egui::PointerButton::Secondary)
            || (response.dragged() && ui.input(|i| i.modifiers.shift))
        {
            let [right, up, _] = self.basis();
            let scale = self.span / response.rect.height().max(1.0);
            for axis in 0..3 {
                self.target[axis] += (-right[axis] * delta.x + up[axis] * delta.y) * scale;
            }
        }
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            self.span = (self.span * (-scroll * 0.003).exp()).clamp(0.05, 100000.0);
        }
    }
}

pub struct Viewport {
    pub scene: Vec<(u64, Geometry)>,
    display: std::sync::Arc<SceneDisplay>,
    state: egui_wgpu::RenderState,
    triangles: wgpu::RenderPipeline,
    profile_overlay: wgpu::RenderPipeline,
    lines: wgpu::RenderPipeline,
    vertices: wgpu::Buffer,
    triangle_count: u32,
    overlay_start: u32,
    vertex_count: u32,
    uniform: wgpu::Buffer,
    bind: wgpu::BindGroup,
    target: Option<(
        wgpu::TextureView,
        wgpu::TextureView,
        egui::TextureId,
        [u32; 2],
    )>,
}

// Selected standalone profiles need a coplanar display pass, not a change to
// authored positions. Keep source geometry and picking at their exact depth.
fn triangle_order(
    bodies: &[Body],
    scene: &[(u64, Geometry)],
    selected: &[selection::Pick],
) -> (Vec<usize>, u32) {
    let overlay: std::collections::HashSet<_> = bodies
        .iter()
        .filter(|b| b.extrusion == 0. && selected.iter().any(|p| p.0 == b.id))
        .map(|b| b.id)
        .collect();
    let mut order: Vec<_> = (0..scene.len()).collect();
    order.sort_by_key(|index| overlay.contains(&scene[*index].0));
    let start = scene
        .iter()
        .filter(|(id, _)| !overlay.contains(id))
        .map(|(_, g)| g.triangles.len() as u32 * 3)
        .sum();
    (order, start)
}

fn bytes(values: &[f32]) -> Vec<u8> {
    values.iter().flat_map(|v| v.to_ne_bytes()).collect()
}

impl Viewport {
    pub fn new(state: egui_wgpu::RenderState) -> Self {
        let device = &state.device;
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Viewport"),
            source: wgpu::ShaderSource::Wgsl(include_str!("viewport.wgsl").into()),
        });
        let uniform = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Camera"),
            size: 80,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: None,
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform.as_entire_binding(),
            }],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipeline = |topology, bias| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("Depth-tested viewport"),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some("vertex"),
                    compilation_options: Default::default(),
                    buffers: &[wgpu::VertexBufferLayout {
                        array_stride: 24,
                        step_mode: wgpu::VertexStepMode::Vertex,
                        attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3],
                    }],
                },
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: Some("fragment"),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::TextureFormat::Rgba8Unorm.into())],
                }),
                primitive: wgpu::PrimitiveState {
                    topology,
                    ..Default::default()
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: wgpu::TextureFormat::Depth32Float,
                    depth_write_enabled: Some(true),
                    depth_compare: Some(wgpu::CompareFunction::LessEqual),
                    stencil: Default::default(),
                    bias: wgpu::DepthBiasState {
                        constant: bias,
                        ..Default::default()
                    },
                }),
                multisample: Default::default(),
                multiview_mask: None,
                cache: None,
            })
        };
        let triangles = pipeline(wgpu::PrimitiveTopology::TriangleList, 2);
        let profile_overlay = pipeline(wgpu::PrimitiveTopology::TriangleList, 0);
        let lines = pipeline(wgpu::PrimitiveTopology::LineList, 0);
        let vertices = device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: 24,
            usage: wgpu::BufferUsages::VERTEX,
            mapped_at_creation: false,
        });
        let triangle_count = 0;
        let vertex_count = 0;
        Self {
            scene: Vec::new(),
            display: std::sync::Arc::default(),
            state,
            triangles,
            profile_overlay,
            lines,
            vertices,
            triangle_count,
            overlay_start: 0,
            vertex_count,
            uniform,
            bind,
            target: None,
        }
    }

    pub fn update_scene(
        &mut self,
        bodies: &[Body],
        scene: Vec<(u64, Geometry)>,
        display: std::sync::Arc<SceneDisplay>,
        picks: &[selection::Pick],
        mode: SelectionMode,
    ) {
        self.display = display;
        let mut data = Vec::new();
        let chosen: std::collections::HashSet<_> = picks
            .iter()
            .map(|(id, element)| {
                (
                    *id,
                    if mode == SelectionMode::Face {
                        element.map(|face| {
                            scene
                                .iter()
                                .find(|(owner, _)| owner == id)
                                .map_or(face, |(_, g)| g.face_group(face))
                        })
                    } else if mode == SelectionMode::Edge {
                        element.map(|edge| {
                            scene
                                .iter()
                                .find(|(owner, _)| owner == id)
                                .map_or(edge, |(_, g)| g.edge_group(edge))
                        })
                    } else {
                        *element
                    },
                )
            })
            .collect();
        let mut vertex = |p: Point3, c: [f32; 3]| {
            data.extend([p.x as f32, p.y as f32, p.z as f32]);
            data.extend(c);
        };
        let (order, overlay_start) = triangle_order(bodies, &scene, picks);
        for index in order {
            let (id, g) = &scene[index];
            let shades = &self.display.shades[index];
            for (index, tri) in g.triangles.iter().enumerate() {
                let selected = matches!(mode, SelectionMode::Volume | SelectionMode::Face)
                    && chosen.contains(&(
                        *id,
                        if mode == SelectionMode::Volume {
                            None
                        } else {
                            Some(g.face_group(g.triangle_faces[index]))
                        },
                    ));
                for (p, shade) in tri.iter().zip(shades[index]) {
                    vertex(
                        *p,
                        if selected {
                            [shade * 0.55, shade * 0.8, shade]
                        } else {
                            [shade * 0.88, shade * 0.95, shade]
                        },
                    );
                }
            }
        }
        let triangle_count = (data.len() / 6) as u32;
        let mut vertex = |p: Point3, c: [f32; 3]| {
            data.extend([p.x as f32, p.y as f32, p.z as f32]);
            data.extend(c);
        };
        for n in -10..=10 {
            for p in [
                [n as f64, -10., -0.002],
                [n as f64, 10., -0.002],
                [-10., n as f64, -0.002],
                [10., n as f64, -0.002],
            ] {
                vertex(p.into(), [0.20, 0.23, 0.27]);
            }
        }
        for (end, c) in [
            ([4., 0., 0.], [0.9, 0.25, 0.25]),
            ([0., 4., 0.], [0.3, 0.8, 0.4]),
            ([0., 0., 4.], [0.3, 0.55, 1.]),
        ] {
            vertex(Point3::new(0., 0., 0.), c);
            vertex(end.into(), c);
        }
        for (id, g) in &scene {
            for (index, edge) in g.edges.iter().enumerate() {
                if g.internal_edges.contains(&g.edge_groups[index]) {
                    continue;
                }
                for p in edge {
                    vertex(
                        *p,
                        if (mode == SelectionMode::Volume && chosen.contains(&(*id, None)))
                            || (mode == SelectionMode::Edge
                                && chosen
                                    .contains(&(*id, Some(g.edge_group(g.edge_groups[index])))))
                        {
                            [0.35, 0.75, 1.]
                        } else if bodies.iter().any(|b| b.id == *id && b.construction) {
                            [0.8, 0.6, 0.3]
                        } else {
                            [0.55, 0.6, 0.65]
                        },
                    );
                }
            }
        }
        self.vertex_count = (data.len() / 6) as u32;
        self.triangle_count = triangle_count;
        self.overlay_start = overlay_start;
        self.vertices = self
            .state
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Editable model"),
                contents: &bytes(&data),
                usage: wgpu::BufferUsages::VERTEX,
            });
        self.scene = scene;
    }

    pub fn query(&self) -> selection::Query<'_> {
        selection::Query::indexed(&self.scene, &self.display.index)
    }

    pub fn pick(
        &self,
        camera: &Camera,
        point: egui::Pos2,
        rect: egui::Rect,
        mode: SelectionMode,
    ) -> Option<(u64, Option<usize>)> {
        self.query().pick(camera, point, rect, mode)
    }

    pub fn render(&mut self, size: [u32; 2], camera: &Camera) -> egui::TextureId {
        let device = &self.state.device;
        let limit = device.limits().max_texture_dimension_2d;
        let size = size.map(|v| v.clamp(1, limit));
        if self.target.as_ref().is_none_or(|t| t.3 != size) {
            if let Some(old) = self.target.take() {
                self.state.renderer.write().free_texture(&old.2);
            }
            let texture = |format, usage| {
                device
                    .create_texture(&wgpu::TextureDescriptor {
                        label: Some("Viewport attachment"),
                        size: wgpu::Extent3d {
                            width: size[0],
                            height: size[1],
                            depth_or_array_layers: 1,
                        },
                        mip_level_count: 1,
                        sample_count: 1,
                        dimension: wgpu::TextureDimension::D2,
                        format,
                        usage,
                        view_formats: &[],
                    })
                    .create_view(&Default::default())
            };
            let color = texture(
                wgpu::TextureFormat::Rgba8Unorm,
                wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            );
            let depth = texture(
                wgpu::TextureFormat::Depth32Float,
                wgpu::TextureUsages::RENDER_ATTACHMENT,
            );
            let id = self.state.renderer.write().register_native_texture(
                device,
                &color,
                wgpu::FilterMode::Linear,
            );
            self.target = Some((color, depth, id, size));
        }
        let mut values = Vec::new();
        for axis in camera.basis() {
            values.extend(axis);
            values.push(0.0);
        }
        values.extend(camera.target);
        values.push(camera.span);
        values.extend([size[0] as f32 / size[1] as f32, 0., 0., 0.]);
        self.state
            .queue
            .write_buffer(&self.uniform, 0, &bytes(&values));
        let (color, depth, id, _) = self.target.as_ref().unwrap();
        let mut encoder = device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("3D viewport"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: color,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.105,
                            g: 0.125,
                            b: 0.15,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_bind_group(0, &self.bind, &[]);
            pass.set_vertex_buffer(0, self.vertices.slice(..));
            pass.set_pipeline(&self.triangles);
            pass.draw(0..self.overlay_start, 0..1);
            pass.set_pipeline(&self.profile_overlay);
            pass.draw(self.overlay_start..self.triangle_count, 0..1);
            pass.set_pipeline(&self.lines);
            pass.draw(self.triangle_count..self.vertex_count, 0..1);
        }
        self.state.queue.submit([encoder.finish()]);
        *id
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
        a.into_iter().zip(b).map(|(a, b)| a * b).sum()
    }
    #[test]
    fn display_shading_matches_geometry_and_is_shared_across_refreshes() {
        let mut circle = Body::new(crate::model::Shape::Circle);
        circle.extrusion = 3.;
        let scene = evaluate_scene(&[circle]).unwrap();
        let display = std::sync::Arc::new(SceneDisplay::new(&scene));
        let refreshed = display.clone();
        assert!(std::sync::Arc::ptr_eq(&display, &refreshed));
        assert_eq!(display.shades.len(), scene.len());
        for ((_, g), shades) in scene.iter().zip(&display.shades) {
            assert_eq!(shades.len(), g.triangles.len());
            for (normals, shades) in surface_normals(g).iter().zip(shades) {
                for (normal, shade) in normals.iter().zip(shades) {
                    let expected = (0.45
                        + 0.4 * normal.dot(Vector3::new(0.3, -0.4, 0.85).normalize()).abs())
                        as f32;
                    assert_eq!(*shade, expected);
                    assert!(shade.is_finite());
                }
            }
        }
        assert!(SceneDisplay::new(&[]).shades.is_empty());
    }

    #[test]
    #[ignore = "manual CPU audit: cargo test --release geometry_workload_timings -- --ignored --nocapture"]
    fn geometry_workload_timings() {
        use crate::model::{Body, Shape};
        use std::time::Instant;
        // Match the native evaluator's bounded pool rather than the global pool.
        rayon::ThreadPoolBuilder::new().num_threads(2).build().unwrap().install(|| {
        let measure = |name: &str, mut bodies: Vec<Body>| {
            let mut builds = Vec::new();
            let mut normals = Vec::new();
            let mut copies = Vec::new();
            let mut triangles = 0;
            let mut point_overlay = None;
            let mut indexed_overlay = None;
            let mut index_build = Vec::new();
            let mut index_bytes = 0;
            for _ in 0..3 {
                bodies[0].extrusion += 0.01;
                let start = Instant::now();
                let scene = evaluate_scene(&bodies).unwrap();
                builds.push(start.elapsed());
                triangles = scene.iter().map(|(_, g)| g.triangles.len()).sum::<usize>();
                let start = Instant::now();
                let cloned = scene.clone();
                copies.push(start.elapsed());
                std::hint::black_box(cloned);
                let start = Instant::now();
                for (_, g) in &scene {
                    std::hint::black_box(surface_normals(g));
                }
                normals.push(start.elapsed());
                let start=Instant::now();let index=selection::SpatialIndex::new(&scene);index_build.push(start.elapsed());index_bytes=index.allocated_bytes();
                if point_overlay.is_none() && bodies.len() <= 25 {
                    let mut camera = Camera::default();
                    camera.fit(&scene);
                    let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(1280., 850.));
                    let start = Instant::now();
                    let mut points = 0;
                    for (_, g) in &scene {
                        for p in &g.points {
                            std::hint::black_box(selection::visible(&scene, &camera, *p, rect));
                            points += 1;
                        }
                    }
                    point_overlay = Some((points, start.elapsed()));
                    let fast=selection::Query::indexed(&scene,&index);let start=Instant::now();
                    for (_,g) in &scene {for p in &g.points {std::hint::black_box(fast.visible(&camera,*p,rect));}}
                    indexed_overlay=Some(start.elapsed());
                }
            }
            eprintln!(
                "{name}: {} bodies, {triangles} visible triangles; rebuild={builds:?}; normals={normals:?}; scene clone={copies:?}; point overlay={point_overlay:?}; indexed overlay={indexed_overlay:?}; index build={index_build:?}; index bytes={index_bytes}",
                bodies.len()
            );
        };
        for count in [1, 25, 100] {
            let bodies = (0..count)
                .map(|i| {
                    let mut b = Body::new(Shape::Circle);
                    b.id = i + 1;
                    b.width = 20.;
                    b.extrusion = 3.;
                    b.position = [(i % 10) as f64 * 30., (i / 10) as f64 * 30., 0.];
                    b
                })
                .collect();
            measure(&format!("cylinders-{count}"), bodies);
        }
        if let Ok(path) = std::env::var("OSK_PROFILE_PROJECT") {
            let project = crate::project::Project::load(std::path::Path::new(&path)).unwrap();
            for _ in 0..3 {
                evaluate_scene(&[]).unwrap();
                let start = Instant::now();
                let scene = evaluate_scene(&project.bodies).unwrap();
                eprintln!(
                    "project replay: {} authored bodies, {} visible triangles, cold evaluation {:?}",
                    project.bodies.len(),
                    scene.iter().map(|(_, g)| g.triangles.len()).sum::<usize>(),
                    start.elapsed()
                );
            }
        }
        });
    }

    #[test]
    fn clipped_circle_extrusion_has_no_old_rim_continuation_edge() {
        use crate::{
            booleans::{Boolean, Kind},
            components,
            edge_treatments::FaceTarget,
            model::{Shape, evaluate_scene},
        };
        let mut source = Body::new(Shape::Circle);
        source.id = 1;
        source.width = 4.;
        source.extrusion = 3.;
        let mut profile = Body::new(Shape::Circle);
        profile.id = 2;
        profile.width = 3.;
        profile.position = [1., 0., 3.];
        let mut bodies = vec![source.clone(), profile];
        let scene = evaluate_scene(&bodies).unwrap();
        let face = (0..scene[0].1.face_count)
            .find(|f| {
                crate::edge_treatments::target(&source, &scene[0].1, *f) == Ok(FaceTarget::Top)
            })
            .unwrap();
        let mut feature = Body::new(Shape::Rectangle);
        feature.id = 3;
        feature.boolean = Some(Boolean::new(
            Kind::Imprint,
            &[(1, Some(face)), (2, None)],
            SelectionMode::Face,
            &scene,
        ));
        bodies.push(feature.clone());
        let scene = evaluate_scene(&bodies).unwrap();
        let g = &scene[2].1;
        let cap = (0..g.face_count)
            .filter(|f| crate::snapping::Plane::face(g, *f).is_some_and(|p| p.normal.z > 0.99))
            .max_by(|a, b| {
                let center = |face: usize| {
                    let xs = g
                        .triangles
                        .iter()
                        .zip(&g.triangle_faces)
                        .filter(|(_, f)| **f == face)
                        .flat_map(|(t, _)| t.iter())
                        .map(|p| p.x)
                        .collect::<Vec<_>>();
                    xs.iter().sum::<f64>() / xs.len() as f64
                };
                center(*a).total_cmp(&center(*b))
            })
            .unwrap();
        bodies[2] = components::sweep_face(&feature, g, cap, 0.7).unwrap();
        let scene = evaluate_scene(&bodies).unwrap();
        let g = &scene[2].1;
        let mut continuations = 0;
        for edge in 0..g.edge_count {
            let owners = (0..g.face_count)
                .filter(|face| g.topology.faces[*face].contains(&edge))
                .collect::<Vec<_>>();
            if owners.len() != 2
                || owners
                    .iter()
                    .any(|face| crate::snapping::Plane::face(g, *face).is_some())
            {
                continue;
            }
            let points = g
                .edges
                .iter()
                .zip(&g.edge_groups)
                .filter(|(_, group)| **group == edge)
                .flat_map(|(e, _)| e.iter())
                .collect::<Vec<_>>();
            if points.is_empty()
                || !points
                    .iter()
                    .all(|p| (p.z - 3.).abs() < 1e-7 && (p.x * p.x + p.y * p.y - 4.).abs() < 1e-6)
            {
                continue;
            }
            continuations += 1;
            assert!(
                g.internal_edges.contains(&edge),
                "Old rim edge {edge} between faces {owners:?} is internal to the same cylinder"
            );
            assert_eq!(g.face_group(owners[0]), g.face_group(owners[1]));
        }
        assert!(continuations > 0);
        let normals = surface_normals(g);
        for (index, triangle) in g.triangles.iter().enumerate() {
            if g.cylinder_supports
                .get(g.triangle_faces[index])
                .is_none_or(Option::is_none)
            {
                continue;
            }
            for (point, normal) in triangle.iter().zip(normals[index]) {
                // Exact radial normals must not depend on how either patch was triangulated.
                if (point.x * point.x + point.y * point.y - 4.).abs() < 1e-6 {
                    let expected = Vector3::new(point.x, point.y, 0.).normalize();
                    assert!(normal.dot(expected) > 1. - 1e-10);
                }
            }
        }
    }

    #[test]
    fn circular_extrusions_have_one_editable_wall_without_patch_edges() {
        use crate::{
            components, face_operations,
            model::{Model, Shape, evaluate},
            selection,
        };
        use truck_modeling::{Matrix4, Transform};
        for affine in [false, true] {
            let mut body = Body::new(Shape::Circle);
            body.id = 1;
            body.width = 4.;
            body.extrusion = 3.;
            if affine {
                body.position = [2., -3., 1.];
                body.rotation = [15., 25., 35.];
                body.scale = [1.4, 0.7, 1.2];
            }
            let g = evaluate(&body).unwrap();
            // Preserve raw topology for old feature guards, but remove its
            // circle-arc junctions from modeling selection and snapping.
            let raw =
                crate::model::tessellate(&crate::model::primitive_shell(&body).unwrap()).unwrap();
            assert_eq!(g.topology, raw.topology);
            assert_eq!(g.face_elements(1), vec![1, 2, 3]);
            assert_eq!(g.internal_edges.len(), 3);
            let rims: std::collections::HashSet<_> = (0..g.edge_count)
                .filter(|edge| !g.internal_edges.contains(edge))
                .map(|edge| g.edge_group(edge))
                .collect();
            assert_eq!(rims.len(), 2);
            assert!(rims.iter().all(|edge| g.edge_elements(*edge).len() == 3));
            for edge in &g.internal_edges {
                assert!(!selection::valid(
                    (1, Some(*edge)),
                    &[(1, g.clone())],
                    SelectionMode::Edge
                ));
            }
            let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(1000., 800.));
            let mut camera = Camera::default();
            camera.fit(&[(1, g.clone())]);
            let mut visible = 0;
            for edge in &g.internal_edges {
                assert!(
                    components::transformed(
                        &body,
                        &g,
                        SelectionMode::Edge,
                        Some(*edge),
                        Matrix4::from_translation(Vector3::new(0.2, 0., 0.))
                    )
                    .unwrap_err()
                    .contains("not editable")
                );
                let segment = g
                    .edges
                    .iter()
                    .zip(&g.edge_groups)
                    .find(|(_, group)| **group == *edge)
                    .unwrap()
                    .0;
                let midpoint = segment[0] + (segment[1] - segment[0]) * 0.5;
                if selection::visible(&[(1, g.clone())], &camera, midpoint, rect) {
                    visible += 1;
                    assert_ne!(
                        selection::pick(
                            &[(1, g.clone())],
                            &camera,
                            camera.project(midpoint, rect).0,
                            rect,
                            SelectionMode::Edge
                        ),
                        Some((1, Some(*edge)))
                    );
                }
            }
            assert!(visible > 0);
            let normals = surface_normals(&g);
            let mut shared = std::collections::HashMap::<[i64; 3], Vector3>::new();
            for (index, t) in g.triangles.iter().enumerate() {
                if g.face_group(g.triangle_faces[index]) != 1 {
                    continue;
                }
                for (point, normal) in t.iter().zip(normals[index]) {
                    let key = [point.x, point.y, point.z].map(|v| (v * 1e6).round() as i64);
                    if let Some(previous) = shared.insert(key, normal) {
                        // Positions share a quantized key, not necessarily identical coordinates.
                        assert!((previous - normal).magnitude() < 1e-6);
                    }
                }
            }
            let moved = components::transformed(
                &body,
                &g,
                SelectionMode::Face,
                Some(1),
                Matrix4::from_translation(Vector3::new(0.2, 0., 0.)),
            )
            .unwrap();
            let edit = moved.component_edits.last().unwrap();
            assert_eq!(edit.element, 1);
            assert_eq!(edit.elements, vec![2, 3]);
            let changed = evaluate(&moved).unwrap();
            assert!(changed.shell.is_some());
            assert_eq!(changed.circles.len(), g.circles.len());
            assert!(changed.shell.as_ref().unwrap().is_geometric_consistent());
            for (before, after) in g.points.iter().zip(&changed.points) {
                assert!(
                    (*after
                        - Matrix4::from_translation(Vector3::new(0.2, 0., 0.))
                            .transform_point(*before))
                    .magnitude()
                        < 1e-7
                );
            }
            let deleted = face_operations::selected(
                std::slice::from_ref(&body),
                &[(1, g.clone())],
                &[(1, Some(2))],
                face_operations::FaceEdit::Open,
            )
            .unwrap();
            let opened = evaluate(&deleted[0]).unwrap();
            assert!(!opened.is_solid);
            assert_eq!(opened.face_count, 2);
            let mut model = Model::from_bodies(vec![body.clone()]);
            model.commit(moved.clone()).unwrap();
            model.undo();
            assert_eq!(model.bodies, vec![body]);
            model.redo();
            assert_eq!(model.bodies, vec![moved]);
        }
    }

    #[test]
    fn logical_surfaces_smooth_internal_samples_but_preserve_boundaries() {
        let a = Point3::new(0., 0., 0.);
        let b = Point3::new(1., 0., 0.);
        let mut g = Geometry {
            triangles: vec![
                [a, b, Point3::new(0., 1., 0.)],
                [b, a, Point3::new(0., 0., 1.)],
            ],
            triangle_faces: vec![0, 0],
            ..Default::default()
        };
        let normals = surface_normals(&g);
        assert!((normals[0][0] - normals[1][1]).magnitude() < 1e-8);
        assert!(normals[0][0].y > 0.5 && normals[0][0].z > 0.5);
        g.triangle_faces[1] = 1;
        let normals = surface_normals(&g);
        assert!((normals[0][0] - Vector3::unit_z()).magnitude() < 1e-8);
        assert!((normals[1][1] - Vector3::unit_y()).magnitude() < 1e-8);
    }
    #[test]
    fn presets_have_orthonormal_axes_and_isometric_foreshortening() {
        let mut solid = Body::new(crate::model::Shape::Rectangle);
        solid.id = 2;
        solid.extrusion = 2.;
        let mut profile = Body::new(crate::model::Shape::Rectangle);
        profile.id = 1;
        profile.position[2] = 2.;
        let bodies = vec![profile, solid];
        let scene = evaluate_scene(&bodies).unwrap();
        let (order, start) = triangle_order(&bodies, &scene, &[(1, None)]);
        assert_eq!(order, vec![1, 0]);
        assert_eq!(start, scene[1].1.triangles.len() as u32 * 3);
        let (order, start) = triangle_order(&bodies, &scene, &[(2, None)]);
        assert_eq!(order, vec![0, 1]);
        assert_eq!(
            start,
            scene
                .iter()
                .map(|(_, g)| g.triangles.len() as u32 * 3)
                .sum::<u32>()
        );
        let mut camera = Camera::default();
        for view in [View::Isometric, View::Top, View::Side] {
            camera.preset(view);
            let [r, u, b] = camera.basis();
            for axis in [r, u, b] {
                assert!((dot(axis, axis) - 1.).abs() < 1e-5);
            }
            assert!(dot(r, u).abs() < 1e-5 && dot(r, b).abs() < 1e-5 && dot(u, b).abs() < 1e-5);
            if view == View::Isometric {
                assert!((b[0] - b[1]).abs() < 1e-5 && (b[1] - b[2]).abs() < 1e-5);
            }
            let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(800., 600.));
            if view == View::Side {
                assert!(camera.plane(rect.center(), rect, [0.; 3], 2).is_none());
            } else {
                let point = Point3::new(2., -1., 0.);
                let projected = camera.project(point, rect).0;
                let recovered = camera.plane(projected, rect, [0.; 3], 2).unwrap();
                assert!(
                    (recovered[0] - point.x).abs() < 0.001
                        && (recovered[1] - point.y).abs() < 0.001
                );
            }
            if view == View::Top {
                assert!((b[2] - 1.).abs() < 1e-5);
                assert!((r[0] - 1.).abs() < 1e-5 && (u[1] - 1.).abs() < 1e-5);
            }
            if view == View::Side {
                assert!((b[0] - 1.).abs() < 1e-5);
            }
        }
    }
}
