use crate::model::{Body, Geometry, evaluate_scene};
use crate::selection::{self, SelectionMode};
use eframe::{egui, egui_wgpu, wgpu};
use serde::{Deserialize, Serialize};
use truck_modeling::{InnerSpace, Point3, Vector3};
use wgpu::util::DeviceExt;

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
    state: egui_wgpu::RenderState,
    triangles: wgpu::RenderPipeline,
    lines: wgpu::RenderPipeline,
    vertices: wgpu::Buffer,
    triangle_count: u32,
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
        let pipeline = |topology| {
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
                        constant: if topology == wgpu::PrimitiveTopology::TriangleList {
                            2
                        } else {
                            0
                        },
                        ..Default::default()
                    },
                }),
                multisample: Default::default(),
                multiview_mask: None,
                cache: None,
            })
        };
        let triangles = pipeline(wgpu::PrimitiveTopology::TriangleList);
        let lines = pipeline(wgpu::PrimitiveTopology::LineList);
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
            state,
            triangles,
            lines,
            vertices,
            triangle_count,
            vertex_count,
            uniform,
            bind,
            target: None,
        }
    }

    pub fn update(
        &mut self,
        bodies: &[Body],
        selected: Option<u64>,
        mode: SelectionMode,
        element: Option<usize>,
    ) -> Result<(), String> {
        let scene = evaluate_scene(bodies)?;
        let mut data = Vec::new();
        let mut vertex = |p: Point3, c: [f32; 3]| {
            data.extend([p.x as f32, p.y as f32, p.z as f32]);
            data.extend(c);
        };
        for (id, g) in &scene {
            for (index, tri) in g.triangles.iter().enumerate() {
                let n = (tri[1] - tri[0]).cross(tri[2] - tri[0]).normalize();
                let shade =
                    (0.45 + 0.4 * n.dot(Vector3::new(0.3, -0.4, 0.85).normalize()).abs()) as f32;
                let color = if Some(*id) == selected
                    && (mode == SelectionMode::Volume
                        || (mode == SelectionMode::Face
                            && element == Some(g.triangle_faces[index])))
                {
                    [shade * 0.55, shade * 0.8, shade]
                } else {
                    [shade * 0.88, shade * 0.95, shade]
                };
                for p in tri {
                    vertex(*p, color);
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
                for p in edge {
                    vertex(
                        *p,
                        if Some(*id) == selected
                            && (mode == SelectionMode::Volume
                                || (mode == SelectionMode::Edge
                                    && element == Some(g.edge_groups[index])))
                        {
                            [0.35, 0.75, 1.]
                        } else {
                            [0.55, 0.6, 0.65]
                        },
                    );
                }
            }
        }
        self.vertex_count = (data.len() / 6) as u32;
        self.triangle_count = triangle_count;
        self.vertices = self
            .state
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("Editable model"),
                contents: &bytes(&data),
                usage: wgpu::BufferUsages::VERTEX,
            });
        self.scene = scene;
        Ok(())
    }

    pub fn pick(
        &self,
        camera: &Camera,
        point: egui::Pos2,
        rect: egui::Rect,
        mode: SelectionMode,
    ) -> Option<(u64, Option<usize>)> {
        selection::pick(&self.scene, camera, point, rect, mode)
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
            pass.draw(0..self.triangle_count, 0..1);
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
    fn presets_have_orthonormal_axes_and_isometric_foreshortening() {
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
