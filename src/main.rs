#![cfg_attr(target_os = "windows", windows_subsystem = "windows")]

use eframe::egui;
mod booleans;
mod components;
mod curve_imprints;
mod drafting;
mod edge_treatments;
mod evaluation;
mod export;
mod face_extrusions;
mod face_operations;
mod gizmo;
mod model;
mod operators;
mod profile_imprints;
mod project;
mod regions;
mod section_edits;
mod selection;
mod shelling;
mod snapping;
use project::Project;
use selection::SelectionMode;
use std::path::PathBuf;
mod viewport;
use model::{Body, Model, Shape};
use truck_modeling::{InnerSpace, Point3, Transform};
use viewport::{Camera, View, Viewport};

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280., 850.])
            .with_min_inner_size([800., 600.]),
        ..Default::default()
    };
    eframe::run_native(
        "OpenSketcher",
        options,
        Box::new(|cc| {
            let state = cc
                .wgpu_render_state
                .clone()
                .expect("wgpu renderer required");
            let mut viewport = Viewport::new(state);
            viewport.update_scene(
                &[],
                Vec::new(),
                std::sync::Arc::default(),
                &[],
                SelectionMode::Volume,
            );
            Ok(Box::new(OpenSketcher {
                camera: Camera::default(),
                viewport,
                model: Model::default(),
                evaluator: evaluation::Evaluator::default(),
                committed_scene: Vec::new(),
                committed_display: std::sync::Arc::default(),
                selected: None,
                selection_mode: SelectionMode::Volume,
                selected_element: None,
                selection: Vec::new(),
                tangent_chain: false,
                draft: None,
                face_offset: None,
                anchor: None,
                tool: Tool::Select,
                transform: None,
                pivot: None,
                snap: true,
                drawing: snapping::Preferences::default(),
                construction: None,
                construction_support: None,
                show_preferences: false,
                error: None,
                path: None,
                saved: Project::new(Vec::new(), Camera::default(), true, None),
                pending: None,
                allow_close: false,
            }))
        }),
    )
}

#[derive(Clone, Copy, PartialEq)]
enum Tool {
    Select,
    Orbit,
    Move,
    Rotate,
    Create(Shape),
}
#[derive(Clone, Copy)]
enum FileAction {
    New,
    Open,
    Close,
}

struct TransformGesture {
    bodies: Vec<Body>,
    scene: Vec<(u64, model::Geometry)>,
    selection: Vec<selection::Pick>,
    drag: gizmo::Drag,
    preview: Vec<Body>,
    valid: bool,
}

struct FaceOffset {
    source: Body,
    geometry: model::Geometry,
    face: components::OffsetTarget,
    distance: f64,
    valid: bool,
    extrude: bool,
}

struct OpenSketcher {
    camera: Camera,
    viewport: Viewport,
    model: Model,
    evaluator: evaluation::Evaluator,
    committed_scene: Vec<(u64, model::Geometry)>,
    committed_display: std::sync::Arc<viewport::SceneDisplay>,
    selected: Option<u64>,
    selection_mode: SelectionMode,
    selected_element: Option<usize>,
    selection: Vec<selection::Pick>,
    tangent_chain: bool,
    draft: Option<Body>,
    face_offset: Option<FaceOffset>,
    anchor: Option<[f64; 3]>,
    tool: Tool,
    transform: Option<TransformGesture>,
    pivot: Option<Point3>,
    snap: bool,
    drawing: snapping::Preferences,
    construction: Option<snapping::Plane>,
    construction_support: Option<operators::Bore>,
    show_preferences: bool,
    error: Option<String>,
    path: Option<PathBuf>,
    saved: Project,
    pending: Option<FileAction>,
    allow_close: bool,
}
impl OpenSketcher {
    fn snapshot(&self) -> Project {
        let mut project = Project::new(
            self.model.bodies.clone(),
            self.camera.clone(),
            self.snap,
            self.selected,
        );
        project.drawing = self.drawing.clone();
        project.selection_mode = self.selection_mode;
        project.selected_element = self.selected_element;
        project.selection = self.selection.clone();
        project
    }
    fn dirty(&self) -> bool {
        self.snapshot() != self.saved || self.evaluator.pending()
    }
    fn export(&mut self, format: export::Format) {
        if self.evaluator.pending()
            || self.transform.is_some()
            || self.anchor.is_some()
            || self.face_offset.as_ref().is_some_and(|edit| !edit.valid)
            || self
                .draft
                .as_ref()
                .is_some_and(|b| !self.model.bodies.contains(b))
        {
            self.error = Some(
                "Finish or cancel the preview and correct invalid properties before exporting."
                    .into(),
            );
            return;
        }
        self.finish_model_edit();
        let extension = format.extension();
        let name = self
            .path
            .as_ref()
            .and_then(|p| p.file_stem())
            .unwrap_or_else(|| std::ffi::OsStr::new("Untitled"))
            .to_string_lossy();
        let mut dialog = rfd::FileDialog::new()
            .add_filter(format.label(), &[extension])
            .set_file_name(format!("{name}.{extension}"));
        if let Some(parent) = self.path.as_ref().and_then(|p| p.parent()) {
            dialog = dialog.set_directory(parent);
        }
        let Some(mut path) = dialog.save_file() else {
            return;
        };
        if !path
            .extension()
            .is_some_and(|e| e.eq_ignore_ascii_case(extension))
        {
            let mut name = path.as_os_str().to_os_string();
            name.push(format!(".{extension}"));
            path = PathBuf::from(name);
            if path.exists()
                && rfd::MessageDialog::new()
                    .set_title("Replace export?")
                    .set_description(format!("Replace {}?", path.display()))
                    .set_buttons(rfd::MessageButtons::YesNo)
                    .show()
                    != rfd::MessageDialogResult::Yes
            {
                return;
            }
        }
        self.error = export::save(&self.model.bodies, format, &path).err();
    }
    fn save(&mut self, save_as: bool) -> bool {
        if self.evaluator.pending() {
            self.error=Some("Geometry is still computing. Wait for it to finish, or press Escape to cancel before saving.".into());
            return false;
        }
        if self.transform.is_some() {
            self.error = Some("Finish or cancel the transform before saving.".into());
            return false;
        }
        if self.anchor.is_some() {
            self.error = Some("Finish or cancel the shape preview before saving.".into());
            return false;
        }
        if self.face_offset.as_ref().is_some_and(|edit| !edit.valid)
            || self
                .draft
                .as_ref()
                .is_some_and(|b| !self.model.bodies.contains(b))
        {
            self.error = Some("Correct the invalid object properties before saving.".into());
            return false;
        }
        self.finish_model_edit();
        let path = if save_as || self.path.is_none() {
            let mut dialog = rfd::FileDialog::new().add_filter("OpenSketcher project", &["osk"]);
            if let Some(path) = &self.path {
                if let Some(parent) = path.parent() {
                    dialog = dialog.set_directory(parent);
                }
                dialog =
                    dialog.set_file_name(path.file_name().unwrap_or_default().to_string_lossy());
            } else {
                dialog = dialog.set_file_name("Untitled.osk");
            }
            let Some(mut path) = dialog.save_file() else {
                return false;
            };
            if !path
                .extension()
                .is_some_and(|ext| ext.eq_ignore_ascii_case("osk"))
            {
                let mut name = path.as_os_str().to_os_string();
                name.push(".osk");
                path = PathBuf::from(name);
                if path.exists()
                    && rfd::MessageDialog::new()
                        .set_title("Replace project?")
                        .set_description(format!("Replace {}?", path.display()))
                        .set_buttons(rfd::MessageButtons::YesNo)
                        .show()
                        != rfd::MessageDialogResult::Yes
                {
                    return false;
                }
            }
            path
        } else {
            self.path.clone().unwrap()
        };
        let project = self.snapshot();
        match project.save(&path) {
            Ok(()) => {
                self.path = Some(path);
                self.saved = project;
                self.error = None;
                true
            }
            Err(e) => {
                self.error = Some(e);
                false
            }
        }
    }
    fn request_file_action(&mut self, action: FileAction, ctx: &egui::Context) {
        if self.pending.is_some() {
            return;
        }
        if self.dirty() {
            self.pending = Some(action);
        } else {
            self.file_action(action, ctx);
        }
    }
    fn file_action(&mut self, action: FileAction, ctx: &egui::Context) {
        self.evaluator.cancel();
        match action {
            FileAction::Close => {
                self.allow_close = true;
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            FileAction::New => {
                self.transform = None;
                self.pivot = None;
                self.model = Model::default();
                self.committed_scene.clear();
                self.committed_display = std::sync::Arc::default();
                model::cache_scene(&[], &[]);
                self.refresh();
                self.camera = Camera::default();
                self.snap = true;
                self.drawing = snapping::Preferences::default();
                self.selected = None;
                self.selection_mode = SelectionMode::Volume;
                self.selected_element = None;
                self.selection.clear();
                self.draft = None;
                self.face_offset = None;
                self.anchor = None;
                self.construction = None;
                self.construction_support = None;
                self.tool = Tool::Select;
                self.path = None;
                self.saved = self.snapshot();
                self.error = None;
            }
            FileAction::Open => {
                let Some(path) = rfd::FileDialog::new()
                    .add_filter("OpenSketcher project", &["osk"])
                    .pick_file()
                else {
                    return;
                };
                self.request_change(evaluation::Change::Open(path));
            }
        }
    }
    fn apply_edge_treatment(&mut self, kind: edge_treatments::Kind) {
        match edge_treatments::selected_operations(
            &self.model.bodies,
            &self.committed_scene,
            &self.selection,
            self.selection_mode,
            kind,
        ) {
            Ok(changes) => {
                self.selection_mode = SelectionMode::Volume;
                self.set_picks(changes.iter().map(|b| (b.id, None)).collect());
                self.request_change(evaluation::Change::Transform(changes));
            }
            Err(error) => self.error = Some(error),
        }
    }

    fn apply_boolean(&mut self, kind: booleans::Kind, picks: Vec<selection::Pick>) {
        let mut body = Body::new(Shape::Rectangle);
        body.boolean = Some(booleans::Boolean::new(
            kind,
            &picks,
            self.selection_mode,
            &self.committed_scene,
        ));
        self.request_change(evaluation::Change::Edit(Box::new(body), false));
    }
    fn apply_face_operation(&mut self, kind: face_operations::FaceEdit) {
        match face_operations::selected(
            &self.model.bodies,
            &self.committed_scene,
            &self.selection,
            kind,
        ) {
            Ok(changes) => {
                self.selection_mode = SelectionMode::Volume;
                self.set_picks(changes.iter().map(|b| (b.id, None)).collect());
                self.request_change(evaluation::Change::Transform(changes));
            }
            Err(error) => self.error = Some(error),
        }
    }
    fn apply_shell(&mut self) {
        let result = (|| {
            if self.selection_mode != SelectionMode::Face || self.selection.len() != 1 {
                return Err("Select one planar source face to open the shell.".to_owned());
            }
            let (id, Some(face)) = self.selection[0] else {
                return Err("Select a planar source face.".into());
            };
            let mut body = self
                .model
                .bodies
                .iter()
                .find(|b| b.id == id)
                .ok_or("Body no longer exists.")?
                .clone();
            let geometry = &self
                .committed_scene
                .iter()
                .find(|g| g.0 == id)
                .ok_or("Geometry no longer exists.")?
                .1;
            let opening = edge_treatments::target(&body, geometry, face)?;
            body.hollow = Some(shelling::Hollow::new(&body, opening)?);
            Ok(body)
        })();
        match result {
            Ok(body) => {
                self.selection_mode = SelectionMode::Volume;
                self.set_picks(vec![(body.id, None)]);
                self.request_change(evaluation::Change::Transform(vec![body]));
            }
            Err(error) => self.error = Some(error),
        }
    }

    fn refresh(&mut self) {
        self.viewport.update_scene(
            &self.model.bodies,
            self.committed_scene.clone(),
            self.committed_display.clone(),
            &self.selection,
            self.selection_mode,
        );
    }
    fn has_uncommitted_input(&self) -> bool {
        self.draft.as_ref().is_some_and(|draft| {
            self.model.bodies.iter().find(|body| body.id == draft.id) != Some(draft)
        }) || self.face_offset.as_ref().is_some_and(|edit| !edit.valid)
            || self
                .transform
                .as_ref()
                .is_some_and(|gesture| !gesture.valid)
    }
    fn finish_model_edit(&mut self) {
        self.model.finish_edit();
        self.evaluator.finish_edit();
    }
    fn request_change(&mut self, change: evaluation::Change) {
        self.error = None;
        self.evaluator.request(&self.model, change);
    }
    fn poll_evaluation(&mut self, ctx: &egui::Context) {
        let offset_pending = self
            .face_offset
            .as_ref()
            .is_some_and(|edit| self.evaluator.editing(edit.source.id));
        if let Some(result) = self.evaluator.poll() {
            match result {
                Err(error) => {
                    self.error = Some(error);
                    if self.transform.is_none() {
                        self.refresh();
                    }
                    if offset_pending && let Some(edit) = &mut self.face_offset {
                        edit.valid = false;
                    }
                    if let Some(gesture) = &mut self.transform {
                        gesture.valid = false;
                    }
                }
                Ok(update) => {
                    self.error = None;
                    if let evaluation::Change::Preview(bodies) = &update.change {
                        if self.transform.is_some() {
                            self.viewport.update_scene(
                                bodies,
                                update.scene,
                                update.display,
                                &self.selection,
                                self.selection_mode,
                            );
                        }
                    } else {
                        self.model = update.model;
                        self.committed_scene = update.scene;
                        self.committed_display = update.display;
                        model::cache_scene(&self.model.bodies, &self.committed_scene);
                        self.refresh();
                        if let Some((path, p)) = update.loaded {
                            self.face_offset = None;
                            self.transform = None;
                            self.pivot = None;
                            self.camera = p.camera.clone();
                            self.snap = p.snapping;
                            self.drawing = p.drawing.clone();
                            self.selected = p.selected;
                            self.selection_mode = p.selection_mode;
                            self.selected_element = p.selected_element;
                            self.selection = p.picks();
                            self.draft = self
                                .model
                                .bodies
                                .iter()
                                .find(|b| {
                                    Some(b.id) == self.selected
                                        && self.selection_mode == SelectionMode::Volume
                                        && self.selection.len() == 1
                                })
                                .cloned();
                            self.anchor = None;
                            self.construction = None;
                            self.construction_support = None;
                            self.tool = Tool::Select;
                            self.path = Some(path);
                            self.saved = p;
                            self.refresh();
                        } else {
                            if self.drawing.plane.is_some_and(|id| {
                                !self
                                    .model
                                    .bodies
                                    .iter()
                                    .any(|b| b.id == id && b.construction)
                            }) {
                                self.drawing.plane = None;
                            }
                            match update.change {
                                evaluation::Change::Edit(_, false) => {
                                    if update.selected.is_some_and(|id| {
                                        self.model
                                            .bodies
                                            .iter()
                                            .any(|b| b.id == id && b.construction)
                                    }) {
                                        self.drawing.plane = update.selected;
                                    }
                                    self.select(update.selected);
                                }
                                evaluation::Change::Delete(_) => self.select(None),
                                evaluation::Change::History(_)
                                | evaluation::Change::Transform(_) => {
                                    let picks = self
                                        .selection
                                        .iter()
                                        .copied()
                                        .filter(|p| {
                                            selection::valid(
                                                *p,
                                                &self.viewport.scene,
                                                self.selection_mode,
                                            )
                                        })
                                        .collect();
                                    self.set_picks(picks);
                                }
                                _ => {
                                    if let Some(edit) = &mut self.face_offset {
                                        edit.valid = true;
                                        if let Some(body) = self
                                            .model
                                            .bodies
                                            .iter()
                                            .find(|b| b.id == edit.source.id)
                                            && let Some((_, g)) = self
                                                .committed_scene
                                                .iter()
                                                .find(|(id, _)| *id == body.id)
                                        {
                                            self.selected_element = (0..g.face_count).find(|f| {
                                                if let components::OffsetTarget::EvaluatedPlane(
                                                    face,
                                                ) = edit.face
                                                {
                                                    let original =
                                                        snapping::Plane::face(&edit.geometry, face)
                                                            .unwrap();
                                                    snapping::Plane::face(g, *f).is_some_and(
                                                        |plane| {
                                                            plane.normal.dot(original.normal)
                                                                > 1. - 1e-6
                                                                && (plane.origin
                                                                    - original.origin
                                                                    - original.normal
                                                                        * edit.distance)
                                                                    .dot(original.normal)
                                                                    .abs()
                                                                    < 1e-6
                                                        },
                                                    )
                                                } else if edit.extrude {
                                                    face_extrusions::target(body, g, *f)
                                                        .map(components::OffsetTarget::Plane)
                                                        == Ok(edit.face)
                                                } else {
                                                    components::offset_target(body, g, *f)
                                                        == Ok(edit.face)
                                                }
                                            });
                                            self.selected = self.selected_element.map(|_| body.id);
                                            self.selection = self
                                                .selected_element
                                                .map(|face| vec![(body.id, Some(face))])
                                                .unwrap_or_default();
                                            self.pivot = None;
                                            self.refresh();
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        if self.evaluator.running() {
            ctx.request_repaint_after(std::time::Duration::from_millis(16));
        }
    }
    fn select(&mut self, id: Option<u64>) {
        self.set_selection(id, SelectionMode::Volume, None);
    }
    fn set_selection(&mut self, id: Option<u64>, mode: SelectionMode, element: Option<usize>) {
        self.selection_mode = mode;
        self.set_picks(id.map(|id| vec![(id, element)]).unwrap_or_default());
    }
    fn set_picks(&mut self, picks: Vec<selection::Pick>) {
        self.evaluator.cancel_preview();
        self.face_offset = None;
        self.transform = None;
        self.pivot = None;
        self.finish_model_edit();
        self.selection = picks
            .into_iter()
            .filter_map(|(id, element)| {
                let geometry = self
                    .committed_scene
                    .iter()
                    .find(|(owner, _)| *owner == id)
                    .map(|(_, g)| g);
                if self.selection_mode == SelectionMode::Edge
                    && element.is_some_and(|edge| {
                        geometry.is_some_and(|g| g.internal_edges.contains(&edge))
                    })
                {
                    return None;
                }
                Some((
                    id,
                    if self.selection_mode == SelectionMode::Face {
                        element.map(|face| geometry.map_or(face, |g| g.face_group(face)))
                    } else if self.selection_mode == SelectionMode::Edge {
                        element.map(|edge| geometry.map_or(edge, |g| g.edge_group(edge)))
                    } else {
                        element
                    },
                ))
            })
            .collect();
        let mut seen = std::collections::HashSet::new();
        self.selection.retain(|pick| seen.insert(*pick));
        self.selected = self.selection.last().map(|p| p.0);
        self.selected_element = self.selection.last().and_then(|p| p.1);
        self.draft = if self.selection_mode == SelectionMode::Volume && self.selection.len() == 1 {
            self.model
                .bodies
                .iter()
                .find(|b| Some(b.id) == self.selected)
                .cloned()
        } else {
            None
        };
        self.refresh();
    }
    fn pick(&mut self, picked: Option<selection::Pick>, extend: bool) {
        let mut picks = self.selection.clone();
        if self.tangent_chain && self.selection_mode == SelectionMode::Edge {
            let chain = picked
                .and_then(|(id, element)| Some((id, element?)))
                .and_then(|(id, element)| {
                    self.committed_scene
                        .iter()
                        .find(|(owner, _)| *owner == id)
                        .map(|(_, g)| {
                            selection::tangent_chain(g, element)
                                .into_iter()
                                .map(|edge| (id, Some(edge)))
                                .collect::<Vec<_>>()
                        })
                })
                .unwrap_or_default();
            selection::click_many(&mut picks, &chain, extend);
        } else {
            selection::click(&mut picks, picked, extend);
        }
        self.set_picks(picks);
    }
    fn offset_face_properties(&mut self, ui: &mut egui::Ui) {
        if self.selection_mode != SelectionMode::Face
            || (self.selection.len() != 1 && self.face_offset.is_none())
        {
            return;
        }
        if self.face_offset.is_none() {
            let resolve = (|| {
                let (id, Some(element)) = self.selection[0] else {
                    return Err("Select a planar source face.".to_owned());
                };
                let body = self
                    .model
                    .bodies
                    .iter()
                    .find(|b| b.id == id)
                    .ok_or("Selected body no longer exists.")?;
                if regions::boundaries(body, &self.model.bodies)?.is_some() {
                    return Err("Offset face does not yet support partitioned faces.".into());
                }
                let geometry = &self
                    .viewport
                    .scene
                    .iter()
                    .find(|(owner, _)| *owner == id)
                    .ok_or("Selected geometry no longer exists.")?
                    .1;
                let face = if (!body.component_edits.is_empty()
                    || body.boolean.is_some()
                    || face_extrusions::target(body, geometry, element).is_err())
                    && geometry.is_solid
                    && geometry.shell.is_some()
                    && snapping::Plane::face(geometry, element).is_some()
                {
                    components::OffsetTarget::EvaluatedPlane(element)
                } else {
                    let face = face_extrusions::target(body, geometry, element)
                        .map(components::OffsetTarget::Plane)
                        .or_else(|_| components::offset_target(body, geometry, element))?;
                    components::offset_source(body, face, 0.)?;
                    face
                };
                Ok(FaceOffset {
                    source: body.clone(),
                    geometry: geometry.clone(),
                    face,
                    distance: 0.,
                    valid: true,
                    extrude: true,
                })
            })();
            match resolve {
                Ok(edit) => self.face_offset = Some(edit),
                Err(reason) => {
                    ui.add_enabled(false, egui::Button::new("Offset face"))
                        .on_disabled_hover_text(reason);
                    return;
                }
            }
        }
        let mut edit = self.face_offset.take().unwrap();
        if matches!(
            edit.face,
            components::OffsetTarget::Plane(_) | components::OffsetTarget::EvaluatedPlane(_)
        ) {
            let previous = edit.extrude;
            ui.horizontal(|ui| {
                ui.selectable_value(&mut edit.extrude, true, "Extrude face");
                if !matches!(edit.face, components::OffsetTarget::EvaluatedPlane(_)) {
                    ui.selectable_value(&mut edit.extrude, false, "Offset source");
                }
            });
            if previous != edit.extrude {
                self.finish_model_edit();
                edit.source = self
                    .model
                    .bodies
                    .iter()
                    .find(|b| b.id == edit.source.id)
                    .unwrap()
                    .clone();
                edit.distance = 0.;
            }
        } else {
            edit.extrude = false;
        }
        let before = edit.distance;
        ui.label(format!(
            "{} · {}",
            if edit.extrude {
                "Extrude face"
            } else {
                "Offset face"
            },
            edit.face.label()
        ));
        let response = ui
            .scope_builder(
                egui::UiBuilder::new().id(egui::Id::new("face_offset_distance")),
                |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Distance");
                        ui.add(
                            egui::DragValue::new(&mut edit.distance)
                                .speed(0.05)
                                .suffix(" mm"),
                        )
                    })
                    .inner
                },
            )
            .inner;
        ui.weak(if edit.extrude {"+ adds material; − subtracts it. Passing the far wall opens a through-cut; no body emerges."} else if edit.face == components::OffsetTarget::CylinderWall {"+ grows radius · − shrinks radius. Axis and cap positions stay fixed."} else {"From the selected position: + outward, − inward. Edits the source shape; the opposite face stays fixed."});
        if edit.distance != before {
            let changed = if let components::OffsetTarget::EvaluatedPlane(face) = edit.face {
                components::sweep_face(&edit.source, &edit.geometry, face, edit.distance)
            } else if edit.extrude {
                let mut body = edit.source.clone();
                if let components::OffsetTarget::Plane(face) = edit.face
                    && edit.distance != 0.
                {
                    body.face_extrusions.push(face_extrusions::Extrusion {
                        face,
                        distance: edit.distance,
                    });
                }
                body.validate().map(|_| body)
            } else {
                components::offset_source(&edit.source, edit.face, edit.distance)
            };
            match changed {
                Ok(body) => {
                    edit.valid = true;
                    self.request_change(evaluation::Change::Edit(Box::new(body), true));
                }
                Err(reason) => {
                    edit.valid = false;
                    self.error = Some(reason);
                }
            }
        }
        if response.drag_stopped() || response.lost_focus() {
            self.finish_model_edit();
        }
        self.face_offset = Some(edit);
    }
    fn gizmo_pivot(&self) -> Option<Point3> {
        self.pivot.or_else(|| {
            let pivots: Vec<_> = self
                .selection
                .iter()
                .filter_map(|(id, element)| {
                    let (_, geometry) =
                        self.viewport.scene.iter().find(|(owner, _)| owner == id)?;
                    components::pivot(geometry, self.selection_mode, *element)
                })
                .collect();
            if pivots.is_empty() {
                None
            } else {
                Some(Point3::new(
                    pivots.iter().map(|p| p.x).sum::<f64>() / pivots.len() as f64,
                    pivots.iter().map(|p| p.y).sum::<f64>() / pivots.len() as f64,
                    pivots.iter().map(|p| p.z).sum::<f64>() / pivots.len() as f64,
                ))
            }
        })
    }
    fn commit(&mut self, body: Body) {
        self.request_change(evaluation::Change::Edit(Box::new(body), false));
    }
    fn delete_selected(&mut self) {
        self.transform = None;
        self.evaluator.cancel_preview();
        let ids: Vec<_> = self.selection.iter().map(|p| p.0).collect();
        if !ids.is_empty() {
            self.request_change(evaluation::Change::Delete(ids));
        }
    }
    fn history(&mut self, redo: bool) {
        if self.evaluator.loading() {
            self.evaluator.cancel();
            return;
        }
        if self.transform.take().is_some() {
            self.evaluator.cancel();
            self.refresh();
            return;
        }
        if self.anchor.take().is_some() {
            return;
        }
        self.request_change(evaluation::Change::History(redo));
    }
    fn creation_point(
        &self,
        point: egui::Pos2,
        rect: egui::Rect,
    ) -> Result<snapping::SnapPoint, String> {
        let mut preferences = self.drawing.clone();
        if !self.snap {
            preferences.geometry = false;
            preferences.grid_axes = false;
        }
        let selected = if self.selection_mode == SelectionMode::Face {
            self.selected.zip(self.selected_element)
        } else {
            None
        };
        let plane = if let Some(plane) = self.construction.filter(|_| self.anchor.is_some()) {
            plane
        } else if selected.is_none()
            && let Some(id) = self.drawing.plane
        {
            snapping::Plane::reference(
                self.model
                    .bodies
                    .iter()
                    .find(|b| b.id == id)
                    .ok_or("Active construction plane no longer exists.")?,
            )?
        } else {
            snapping::acquire_plane(
                self.viewport.query(),
                &self.camera,
                rect,
                point,
                selected,
                preferences.geometry,
                preferences.project_to_base_plane,
            )?
        };
        snapping::point(
            self.viewport.query(),
            &self.camera,
            rect,
            point,
            plane,
            self.anchor.map(Point3::from),
            &preferences,
        )
    }
    fn capture_support(
        &self,
        target: &snapping::SnapPoint,
        pointer: egui::Pos2,
        rect: egui::Rect,
    ) -> Option<operators::Bore> {
        let selected = (self.selection_mode == SelectionMode::Face)
            .then_some(self.selected.zip(self.selected_element))
            .flatten();
        if self.drawing.plane.is_some() && selected.is_none() {
            return None;
        }
        let hit = self
            .viewport
            .query()
            .surface_hit(&self.camera, pointer, rect)
            .map(|(_, id, face)| (id, face));
        let (owner, face) = selected.or(hit).or_else(|| {
            let id = target.owner?;
            let g = &self
                .viewport
                .scene
                .iter()
                .find(|(owner, _)| *owner == id)?
                .1;
            let face = (0..g.face_count).find(|f| {
                snapping::Plane::face(g, *f).is_some_and(|plane| {
                    plane.normal.dot(target.plane.normal) > 1. - 1e-6
                        && (plane.origin - target.plane.origin).dot(plane.normal).abs() < 1e-6
                })
            })?;
            Some((id, face))
        })?;
        let body = self.model.bodies.iter().find(|b| b.id == owner)?;
        let g = &self.viewport.scene.iter().find(|(id, _)| *id == owner)?.1;
        let role = edge_treatments::target(body, g, face).ok()?;
        let mut support = operators::Bore::new(body, role).ok()?;
        if target.label == "Circle rim" {
            support.mode = operators::BoreMode::Borer;
        }
        Some(support)
    }
}

fn dimensions(ui: &mut egui::Ui, b: &mut Body) -> bool {
    let label = match b.shape {
        Shape::Rectangle => "Width",
        Shape::Line => "Length",
        _ => "Diameter",
    };
    let mut finished = scalar(ui, label, &mut b.width, 0.01..=100.);
    if b.shape == Shape::Rectangle {
        finished |= scalar(ui, "Depth", &mut b.depth, 0.01..=100.);
    }
    if b.bore.is_none() {
        finished |= scalar(ui, "Extrusion", &mut b.extrusion, 0.0..=100.);
    }
    if b.shape == Shape::Line {
        ui.weak("Extruding a line creates a surface.");
    }
    finished
}
fn history_shortcut(input: &mut egui::InputState) -> Option<bool> {
    // egui allows extra Shift when matching Command: consume the specific chord first.
    if input.consume_key(
        egui::Modifiers::COMMAND | egui::Modifiers::SHIFT,
        egui::Key::Z,
    ) {
        Some(true)
    } else if input.consume_key(egui::Modifiers::COMMAND, egui::Key::Z) {
        Some(false)
    } else {
        None
    }
}

fn scalar(
    ui: &mut egui::Ui,
    label: &str,
    v: &mut f64,
    range: std::ops::RangeInclusive<f64>,
) -> bool {
    let response = scalar_response(ui, label, v, range);
    response.drag_stopped() || response.lost_focus()
}
fn scalar_response(
    ui: &mut egui::Ui,
    label: &str,
    v: &mut f64,
    range: std::ops::RangeInclusive<f64>,
) -> egui::Response {
    // A sheet becoming solid changes preceding controls during the same drag.
    // Global scope also keeps egui's child auto IDs independent of that order.
    ui.scope_builder(
        egui::UiBuilder::new().id(ui.make_persistent_id(("scalar", label))),
        |ui| {
            ui.horizontal(|ui| {
                ui.label(label);
                ui.add(egui::DragValue::new(v).speed(0.05).range(range))
            })
            .inner
        },
    )
    .inner
}
fn vector(
    ui: &mut egui::Ui,
    label: &str,
    values: &mut [f64; 3],
    range: std::ops::RangeInclusive<f64>,
) -> bool {
    ui.label(label);
    ui.horizontal(|ui| {
        let mut finished = false;
        for (i, value) in values.iter_mut().enumerate() {
            ui.label(["X", "Y", "Z"][i]);
            let response = ui.add(egui::DragValue::new(value).speed(0.1).range(range.clone()));
            finished |= response.drag_stopped() || response.lost_focus();
        }
        finished
    })
    .inner
}

impl eframe::App for OpenSketcher {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.poll_evaluation(&ctx);
        if ctx.input(|i| i.viewport().close_requested()) && !self.allow_close && self.dirty() {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.request_file_action(FileAction::Close, &ctx);
        }
        if self.pending.is_none() {
            if ctx.input_mut(|i| {
                i.consume_key(
                    egui::Modifiers::COMMAND | egui::Modifiers::SHIFT,
                    egui::Key::S,
                )
            }) {
                self.save(true);
            } else if ctx.input_mut(|i| i.consume_key(egui::Modifiers::COMMAND, egui::Key::S)) {
                self.save(false);
            }
            if ctx.input_mut(|i| i.consume_key(egui::Modifiers::COMMAND, egui::Key::O)) {
                self.request_file_action(FileAction::Open, &ctx);
            }
            if ctx.input_mut(|i| i.consume_key(egui::Modifiers::COMMAND, egui::Key::N)) {
                self.request_file_action(FileAction::New, &ctx);
            }
        }
        if let Some(action) = self.pending {
            egui::Modal::new(egui::Id::new("unsaved_changes")).show(&ctx, |ui| {
                ui.heading("Save changes to this project?");
                ui.label("Unsaved changes will be lost if you discard them.");
                ui.horizontal(|ui| {
                    if ui.button("Save").clicked() && self.save(false) {
                        self.pending = None;
                        self.file_action(action, &ctx);
                    }
                    if ui.button("Discard").clicked() {
                        self.pending = None;
                        self.file_action(action, &ctx);
                    }
                    if ui.button("Cancel").clicked() {
                        self.pending = None;
                    }
                });
            });
            ui.disable();
        }
        if self.pending.is_none() && !ui.ctx().egui_wants_keyboard_input() {
            if let Some(redo) = ui.input_mut(history_shortcut) {
                self.history(redo);
            }
            if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                if self.evaluator.pending() || self.has_uncommitted_input() {
                    self.evaluator.cancel();
                    self.face_offset = None;
                    self.draft = self
                        .model
                        .bodies
                        .iter()
                        .find(|b| {
                            Some(b.id) == self.selected
                                && self.selection_mode == SelectionMode::Volume
                        })
                        .cloned();
                    self.error = None;
                }
                if self.transform.take().is_some() {
                    self.refresh();
                } else if self.anchor.take().is_none() {
                    self.tool = Tool::Select;
                }
            }
            if ui.input(|i| i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace))
                && self.selection_mode == SelectionMode::Volume
                && !self.evaluator.loading()
            {
                self.delete_selected();
            }
        }
        egui::Panel::top("header").show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading("OpenSketcher");
                if self.evaluator.pending() {
                    ui.spinner()
                        .on_hover_text("Computing geometry · Escape cancels");
                }
                ui.separator();
                let name = self
                    .path
                    .as_ref()
                    .and_then(|p| p.file_name())
                    .map(|s| s.to_string_lossy().into_owned())
                    .unwrap_or_else(|| "Untitled".into());
                ui.label(format!("{}{}", name, if self.dirty() { " *" } else { "" }));
                ui.separator();
                if ui.button("New").clicked() {
                    self.request_file_action(FileAction::New, &ctx);
                }
                if ui.button("Open…").clicked() {
                    self.request_file_action(FileAction::Open, &ctx);
                }
                if ui.button("Save").clicked() {
                    self.save(false);
                }
                if ui.button("Save As…").clicked() {
                    self.save(true);
                }
                ui.menu_button("Export…", |ui| {
                    ui.label("1 scene unit = 1 mm");
                    for format in export::Format::ALL {
                        if ui.button(format.label()).clicked() {
                            ui.close();
                            self.export(format);
                        }
                    }
                });
                if ui.button("Preferences").clicked() {
                    self.show_preferences = !self.show_preferences;
                }
                ui.separator();
                if ui
                    .add_enabled(
                        self.model.can_undo() || self.anchor.is_some(),
                        egui::Button::new("Undo"),
                    )
                    .clicked()
                {
                    self.history(false);
                }
                if ui
                    .add_enabled(self.model.can_redo(), egui::Button::new("Redo"))
                    .clicked()
                {
                    self.history(true);
                }
            });
        });
        egui::Window::new("Drawing preferences")
            .open(&mut self.show_preferences).resizable(false).collapsible(false)
            .enabled(self.pending.is_none()).show(&ctx,|ui| {
                ui.label("Saved with this project. New projects start with geometry/grid snapping on and cutting off.");
                ui.checkbox(&mut self.drawing.geometry,"Snap to geometry");
                ui.weak("Visible endpoints, midpoints, edges, and projected limits. Selected faces pin the drawing plane.");
                ui.checkbox(&mut self.drawing.grid_axes,"Snap to grid and anchor axes");
                ui.checkbox(&mut self.drawing.project_to_base_plane,"Project drawing to base XY plane");
                ui.weak("Off by default. A selected planar face still takes priority.");
                ui.checkbox(&mut self.drawing.cut_on_creation,"New profiles cut coplanar regions");
                ui.weak("Off by default. Applies to new profiles only; existing cuts are retained. This is not a general solid Boolean tool.");
            });
        egui::Panel::bottom("status").show_inside(ui,|ui| {
            if let Some(e)=&self.error {ui.colored_label(egui::Color32::LIGHT_RED,e);}
            ui.label(match self.tool {
                Tool::Select=>match self.selection_mode {
                    SelectionMode::Volume=>"Volume selection · Shift-click toggles · Click an object · Properties update automatically · Shift-drag / right / middle to pan",
                    SelectionMode::Face=>"Face selection · Shift-click toggles · Click a visible face · Move/Rotate handles edit the selected face",
                    SelectionMode::Edge=>"Edge selection · Shift-click toggles · Click near a visible boundary edge · Move/Rotate handles edit the selected edge",
                    SelectionMode::Point=>"Point selection · Shift-click toggles · Click a visible topology vertex · Control-node editing is separate",
                },
                Tool::Move=>"Move · Select geometry, then drag an axis arrow or plane handle · Snap: 0.5 units · Escape cancels · Alt-click sets pivot",
                Tool::Rotate=>"Rotate · Select geometry, then drag an axis ring · Snap: 15° · Alt-click sets pivot (needed for a single point) · Escape cancels",
                Tool::Orbit=>"Drag to orbit · Shift-drag / right / middle to pan · Scroll to zoom",
                Tool::Create(_)=>"Draw on a selected face or active construction plane; otherwise visible plane / XY · Snap: geometry + grid · Escape cancels",
            });
        });
        let canvas = ui.available_rect_before_wrap();
        let panel_height = (canvas.height() - 24.).max(100.);
        egui::Window::new("Tools")
            .default_pos(canvas.left_top() + egui::vec2(12., 12.))
            .constrain_to(canvas)
            .default_width(135.)
            .resizable(false)
            .collapsible(false)
            .enabled(self.pending.is_none())
            .show(&ctx, |ui| {
                egui::ScrollArea::vertical()
                    .max_height((panel_height - 36.).max(64.))
                    .show(ui, |ui| {
                        for (tool, label) in [
                            (Tool::Select, "Select"),
                            (Tool::Orbit, "Orbit"),
                            (Tool::Move, "Move"),
                            (Tool::Rotate, "Rotate"),
                            (Tool::Create(Shape::Rectangle), "Rectangle"),
                            (Tool::Create(Shape::Circle), "Circle"),
                            (Tool::Create(Shape::Line), "Line"),
                            (Tool::Create(Shape::Semicircle), "Semicircle"),
                        ] {
                            if ui.selectable_label(self.tool == tool, label).clicked() {
                                self.transform = None;
                                self.refresh();
                                self.tool = tool;
                                self.anchor = None;
                                self.construction = None;
                                self.finish_model_edit();
                            }
                        }

                        ui.separator();
                        if matches!(self.tool, Tool::Move | Tool::Rotate) {
                            ui.weak("World X / Y / Z");
                            ui.weak("Alt-click: set pivot").on_hover_text("Place the pivot on the view-aligned plane through its current position. A point needs an offset pivot to rotate.");
                            if ui.button("Reset pivot").clicked() {
                                self.transform = None;
                                self.pivot = None;
                                self.refresh();
                            }
                        }
                        if ui.button("Add construction plane").clicked() {
                            self.anchor=None;self.construction=None;self.construction_support=None;
                            self.tool=Tool::Select;
                            let mut plane=Body::new(Shape::Rectangle);plane.construction=true;plane.width=10.;plane.depth=10.;
                            self.commit(plane);
                        }
                        if let Some(id)=self.drawing.plane {
                            ui.weak(format!("Drawing on plane {id}"));
                            if ui.button("Auto drawing plane").clicked(){self.drawing.plane=None;self.anchor=None;self.construction=None;self.construction_support=None;}
                        }
                        ui.checkbox(&mut self.snap, "Snap")
                            .on_hover_text("Creation: geometry limits plus grid/anchor alignment. Configure both in Preferences. Move: 0.5-unit offsets. Rotate: 15° offsets.");
                        ui.separator();
                        ui.label("Selection");
                        for mode in [
                            SelectionMode::Volume,
                            SelectionMode::Face,
                            SelectionMode::Edge,
                            SelectionMode::Point,
                        ] {
                            if ui
                                .selectable_label(self.selection_mode == mode, mode.label())
                                .clicked()
                            {
                                self.select(None);
                                self.selection_mode = mode;
                                if !matches!(self.tool, Tool::Move | Tool::Rotate) {
                                    self.tool = Tool::Select;
                                }
                                self.anchor = None;
                                self.construction = None;
                                self.refresh();
                            }
                        }

                        if self.selection_mode == SelectionMode::Edge {
                            ui.checkbox(&mut self.tangent_chain, "Tangent chain")
                                .on_hover_text("Expand the next edge click along tangent curves. Shift-click toggles the chain.");
                        }
                        ui.separator();
                        ui.label("View");
                        for (view, label) in [
                            (View::Isometric, "Isometric"),
                            (View::Top, "Top"),
                            (View::Side, "Side"),
                        ] {
                            if ui
                                .selectable_label(self.camera.view == view, label)
                                .clicked()
                            {
                                self.transform = None;
                                self.refresh();
                                self.camera.preset(view);
                            }
                        }

                        if self.camera.view == View::Free {
                            ui.weak("Free orbit");
                        }
                    });
            });
        egui::Window::new("Model & properties")
            .default_pos(egui::pos2(canvas.right()-342.,canvas.top()+12.))
            .constrain_to(canvas)
            .default_width(330.).max_height(panel_height)
            .resizable(false).collapsible(false)
            .enabled(self.pending.is_none() && self.transform.is_none() && !self.evaluator.loading())
            .show(&ctx,|ui| {
            egui::ScrollArea::vertical().auto_shrink([false,true]).max_height((panel_height-36.).max(64.)).show(ui,|ui| {
                if let Tool::Create(shape)=self.tool {
                    ui.heading(format!("Create {}",shape.label()));
                    ui.label(match shape { Shape::Rectangle=>"Pick opposite corners on the drawing plane.", Shape::Circle=>"Pick center, then a point on the radius.", Shape::Line=>"Pick the start and end points.", Shape::Semicircle=>"Pick the two diameter endpoints." });
                    ui.weak("A selected planar face pins the plane; otherwise the first click captures a visible planar face.");
                    ui.weak(if self.drawing.cut_on_creation {"New closed profiles cut older coplanar regions."}else{"New profiles are independent; automatic cutting is off."});
                    ui.weak(if self.anchor.is_some() {"Pick the second point. Escape cancels."}else{"Pick the first point in the viewport."});
                    if shape==Shape::Semicircle {ui.weak("The filled half-disc lies to the left of the directed diameter.");}
                    ui.separator();
                }
                ui.heading("Model");
                if self.model.bodies.is_empty(){ui.weak("No geometry. Choose a creation tool.");}
                let mut next=None;
                for body in &self.model.bodies {
                    if ui.selectable_label(self.selection.iter().any(|p|p.0==body.id),body.label()).clicked(){next=Some(body.id);}
                }
                if let Some(id)=next {
                    let extend=ui.input(selection::extend) && self.selection_mode==SelectionMode::Volume;
                    if !extend {self.selection_mode=SelectionMode::Volume;}
                    self.pick(Some((id,None)),extend);
                }
                if self.selection.len()>1 {ui.label(format!("{} {} items selected · Shift-click toggles",self.selection.len(),self.selection_mode.label()));}
                if self.selection.len()==2 && matches!(self.selection_mode,SelectionMode::Volume|SelectionMode::Face) {
                    ui.separator();
                    ui.label(format!("Target: {} | Tool: {}",self.selection[0].0,self.selection[1].0));
                    if ui.button("Swap target / tool").clicked() {let mut picks=self.selection.clone();picks.swap(0,1);self.set_picks(picks);}
                    ui.horizontal(|ui| {
                        for kind in [booleans::Kind::Union,booleans::Kind::Subtract,booleans::Kind::Intersect] {
                            if ui.add_enabled(!self.evaluator.pending(),egui::Button::new(kind.label())).clicked() {
                                self.apply_boolean(kind,self.selection.clone());
                            }
                        }
                    });
                    if self.selection_mode==SelectionMode::Face && ui.add_enabled(!self.evaluator.pending(),egui::Button::new("Replace face")).clicked() {
                        self.apply_boolean(booleans::Kind::ReplaceFace,self.selection.clone());
                    }
                    if self.selection_mode==SelectionMode::Face && ui.add_enabled(!self.evaluator.pending(),egui::Button::new("Imprint")).clicked() {
                        self.apply_boolean(booleans::Kind::Imprint,self.selection.clone());
                    }
                    ui.weak("First face is the source; second is the reference. Replace face extends/trims the source solid to the reference plane and keeps the reference body. Imprint splits the selected solid face with a coplanar profile. Face set operations create a sheet.");
                }
                if self.selection.len()>1 && self.selection_mode==SelectionMode::Volume && ui.button("Delete selected volumes").clicked() {self.delete_selected();}
                if self.has_uncommitted_input() {
                    if self.evaluator.pending() {ui.weak("Computing requested values…");}
                    else {ui.colored_label(egui::Color32::YELLOW,"Input not applied");ui.weak("Geometry, Save and Export use the last accepted values. Correct the input or press Escape to discard it.");}
                }
                if self.selection_mode!=SelectionMode::Volume {
                    ui.separator();
                    if let (Some(id),Some(element))=(self.selected,self.selected_element) {
                        ui.label(format!("{} {} · Object {}",self.selection_mode.label(),element+1,id));
                    }else{ui.label(format!("Pick a {} in the viewport.",self.selection_mode.label().to_lowercase()));}
                    ui.weak("Choose Move or Rotate and drag the gizmo to edit this component. Adjacent boundaries rebuild together.");
                    self.offset_face_properties(ui);
                    if matches!(self.selection_mode,SelectionMode::Face|SelectionMode::Edge) && !self.selection.is_empty() {
                        ui.horizontal(|ui| {
                            if ui.button("Chamfer").clicked() { self.apply_edge_treatment(edge_treatments::Kind::Chamfer); }
                            if ui.button("Bevel").clicked() { self.apply_edge_treatment(edge_treatments::Kind::Bevel); }
                            if self.selection_mode==SelectionMode::Face && self.selection.len()==1 && ui.button("Shell").clicked() {self.apply_shell();}
                        });
                        ui.weak("Shift-select faces or edges. Box edges are treated individually; circular rims use their full perimeter. One action creates one undo step.");
                    }
                    if self.selection_mode==SelectionMode::Face && self.selection.len()==1 {
                        if self.committed_scene.iter().any(|(id,g)|Some(*id)==self.selected && g.is_solid) {
                            let curves=self.model.bodies.iter().filter(|b|!b.construction && Some(b.id)!=self.selected && self.committed_scene.iter().any(|(id,g)|*id==b.id && g.triangles.is_empty() && g.edge_count==1 && !g.edges.is_empty())).map(|b|(b.id,b.label())).collect::<Vec<_>>();
                            if !curves.is_empty() {ui.menu_button("Imprint using curve",|ui| {
                                for (id,label) in curves {
                                    if ui.add_enabled(!self.evaluator.pending(),egui::Button::new(label)).clicked() {
                                        self.apply_boolean(booleans::Kind::ImprintCurve,vec![self.selection[0],(id,None)]);ui.close();
                                    }
                                }
                            });}
                            let profiles=self.model.bodies.iter().filter(|b|!b.construction && Some(b.id)!=self.selected && self.committed_scene.iter().any(|(id,g)|*id==b.id && !g.is_solid && !g.triangles.is_empty() && g.shell.is_some())).map(|b|(b.id,b.label())).collect::<Vec<_>>();
                            if !profiles.is_empty() {ui.menu_button("Imprint using profile",|ui| {
                                for (id,label) in profiles {
                                    if ui.add_enabled(!self.evaluator.pending(),egui::Button::new(label)).clicked() {
                                        self.apply_boolean(booleans::Kind::Imprint,vec![self.selection[0],(id,None)]);ui.close();
                                    }
                                }
                            });}
                        }
                        let planes=self.model.bodies.iter().filter(|b|b.construction).map(|b|(b.id,b.label())).collect::<Vec<_>>();
                        if !planes.is_empty() {ui.menu_button("Replace using plane",|ui| {
                            for (id,label) in planes {
                                if ui.add_enabled(!self.evaluator.pending(),egui::Button::new(label)).clicked() {
                                    self.apply_boolean(booleans::Kind::ReplaceFace,vec![self.selection[0],(id,Some(0))]);
                                    ui.close();
                                }
                            }
                        });}
                    }
                    if self.selection_mode==SelectionMode::Face && !self.selection.is_empty() {
                        ui.horizontal_wrapped(|ui| {
                            for kind in [face_operations::FaceEdit::Open,face_operations::FaceEdit::Cap,face_operations::FaceEdit::Heal] {
                                if ui.button(kind.label()).clicked() {self.apply_face_operation(kind);}
                            }
                        });
                        ui.weak("Open leaves a sheet. Cap closes planar openings. Heal extends supported neighboring planes.");
                    }
                    if self.selection_mode==SelectionMode::Point { ui.weak("Rotation around a point itself has no effect. Alt-click to place an offset pivot."); }
                    if self.selection_mode==SelectionMode::Point {ui.weak("Points are topology vertices; deformation control nodes are separate future editing tools.");}
                }
                if self.selection_mode==SelectionMode::Volume && self.selection.len()==1
                    && self.committed_scene.iter().any(|(id,g)|Some(*id)==self.selected && !g.is_solid && !g.triangles.is_empty() && g.shell.is_some())
                    && ui.button("Cap openings").clicked() {
                    self.apply_face_operation(face_operations::FaceEdit::CapOpenings);
                }
                if self.draft.as_ref().is_some_and(|b| b.boolean.is_some()) {
                    let mut body=self.draft.take().unwrap();
                    let before=body.clone();
                    let operation=body.boolean.as_mut().unwrap();
                    let replacing=operation.kind==booleans::Kind::ReplaceFace;
                    let imprinting=matches!(operation.kind,booleans::Kind::Imprint|booleans::Kind::ImprintCurve);
                    ui.heading(if replacing {"Replace face result"}else if imprinting {"Imprint result"}else{"Boolean result"});
                    ui.label(format!("{} · {}",operation.kind.label(),operation.references()));
                    if !replacing {ui.checkbox(&mut operation.keep_tools,"Keep tools");}
                    if ui.button("Swap operands").clicked() {operation.operands.swap(0,1);operation.topology.swap(0,1);}
                    if body!=before {self.request_change(evaluation::Change::Edit(Box::new(body.clone()),false));}
                    if ui.button(if replacing {"Remove replacement / restore source"}else if imprinting {"Remove imprint / restore inputs"}else{"Remove Boolean / restore inputs"}).clicked() {self.request_change(evaluation::Change::Delete(vec![body.id]));}
                    self.draft=Some(body);
                } else if let Some(mut draft)=self.draft.take() {
                    ui.separator();ui.heading(draft.label());
                    let before=draft.clone();
                    let mut detach=false;
                    let mut finished = false;
                    if draft.construction {
                        ui.weak("Reference only · excluded from exports and Boolean tools. Use Volume Move/Rotate or edit placement below.");
                        if ui.button("Draw on this plane").clicked(){self.drawing.plane=Some(draft.id);self.anchor=None;self.construction=None;self.construction_support=None;}
                        ui.horizontal(|ui| {
                            for (label,rotation) in [("XY",[0.,0.,0.]),("XZ",[90.,0.,0.]),("YZ",[0.,90.,0.])] {
                                if ui.button(label).clicked(){draft.rotation=rotation;finished=true;}
                            }
                        });
                    }
                    if let Some(bore)=&mut draft.bore {
                        ui.label(format!("Attached to body {} · {}",bore.target,bore.face.label()));
                        egui::ComboBox::from_id_salt("sweep_operation").selected_text(bore.mode.label()).show_ui(ui,|ui| {
                            for mode in [operators::BoreMode::Generate,operators::BoreMode::Borer,operators::BoreMode::GeneratorAndBorer] {
                                finished |= ui.selectable_value(&mut bore.mode,mode,mode.label()).changed();
                            }
                        });
                        use operators::Extent;
                        egui::ComboBox::from_id_salt("sweep_extent").selected_text(bore.extent.label()).show_ui(ui,|ui| {
                            for (active,value) in [
                                (bore.extent==Extent::Distance,Extent::Distance),
                                (bore.extent==Extent::ThroughAll,Extent::ThroughAll),
                                (matches!(bore.extent,Extent::UpToNext{..}),Extent::UpToNext{target:bore.target}),
                                (matches!(bore.extent,Extent::UpToFace{..}),Extent::UpToFace{target:bore.target,face:bore.face.opposite()})
                            ] {
                                if ui.selectable_label(active,value.label()).clicked() && !active {bore.extent=value;if value!=Extent::ThroughAll {bore.through_targets.clear();}finished=true;}
                            }
                        });
                        if bore.extent==Extent::Distance {finished|=scalar(ui,"Travel · mm",&mut bore.travel,-10000.0..=10000.0);}
                        else {
                            let mut inward=if bore.travel==0. {bore.mode!=operators::BoreMode::Generate}else{bore.travel<0.};
                            if ui.checkbox(&mut inward,"Inward direction").changed() {bore.travel=bore.travel.abs().max(1.)*if inward {-1.}else{1.};finished=true;}
                            if let Extent::UpToNext{target}|Extent::UpToFace{target,..}=&mut bore.extent {
                                egui::ComboBox::from_id_salt("extent_target").selected_text(format!("End target: body {target}")).show_ui(ui,|ui| {
                                    for body in self.model.bodies.iter().filter(|body|body.id<draft.id && body.boolean.is_none()) {
                                        if self.committed_scene.iter().any(|(id,g)|*id==body.id && g.is_solid) {finished|=ui.selectable_value(target,body.id,body.label()).changed();}
                                    }
                                });
                            }
                            if let Extent::UpToFace{target,face}=&mut bore.extent {
                                egui::ComboBox::from_id_salt("extent_face").selected_text(format!("End face: {}",face.label())).show_ui(ui,|ui| {
                                    if let Some(body)=self.model.bodies.iter().find(|b|b.id==*target) {
                                        for role in [edge_treatments::FaceTarget::Top,edge_treatments::FaceTarget::Bottom,edge_treatments::FaceTarget::Left,edge_treatments::FaceTarget::Right,edge_treatments::FaceTarget::Front,edge_treatments::FaceTarget::Back] {
                                            if role.supported(body) {finished|=ui.selectable_value(face,role,role.label()).changed();}
                                        }
                                    }
                                });
                            }
                            if bore.extent==Extent::ThroughAll {
                                ui.label(format!("Through-all scope · attached body {}",bore.target));
                                for body in self.model.bodies.iter().filter(|b|b.id<draft.id&&b.id!=bore.target&&!b.construction&&b.boolean.is_none()) {
                                    if self.committed_scene.iter().any(|(id,g)|*id==body.id&&g.is_solid&&g.shell.is_some()&&!g.triangles.is_empty()) {
                                        let mut included=bore.through_targets.contains(&body.id);
                                        if ui.checkbox(&mut included,body.label()).changed() {
                                            if included {bore.through_targets.push(body.id);bore.through_targets.sort_unstable();}
                                            else {bore.through_targets.retain(|id|*id!=body.id);}
                                            finished=true;
                                        }
                                    }
                                }
                                ui.weak("Borer modes cut only the attached and checked bodies. Through all stops at the scope end; use Distance for an emerging extension.");
                            }else{ui.weak("Planar end-face termination; target changes regenerate this sweep.");}
                        }
                        ui.weak(if bore.extent==Extent::ThroughAll {"Through all follows the current scope. New body generates without cutting."}else{"+ outward · − inward. Generator + borer creates a separate body beyond the target exit."});
                        detach=ui.button("Detach / remove operation").clicked();
                    }
                    if detach {
                        let bore=draft.bore.as_ref().unwrap();
                        match self.model.bodies.iter().find(|b| b.id==bore.target).ok_or_else(||"Support target no longer exists.".to_owned()).and_then(|target|operators::resolve_profile(&draft,target,bore)) {
                            Ok(source)=>{draft=source;finished=true;self.finish_model_edit();}
                            Err(e)=>self.error=Some(e),
                        }
                    }
                    if !draft.component_edits.is_empty() {
                        ui.label(format!("Component edits: {}",draft.component_edits.len()));
                        ui.weak("Curved and nonplanar component results use a regenerated faceted boundary.");
                        if ui.button("Remove component edits").clicked() { draft.component_edits.clear(); }
                        ui.label("Source profile parameters");
                    }
                    if !draft.edge_treatments.is_empty() {
                        ui.label("Applied edge operations · local units");
                        let mut remove = None;
                        for (index, modifier) in draft.edge_treatments.iter_mut().enumerate() {
                            ui.push_id(index, |ui| {
                                ui.horizontal(|ui| {
                                    ui.label(modifier.label());
                                    if ui.small_button("Remove").clicked() { remove = Some(index); finished = true; }
                                });
                                if modifier.kind == edge_treatments::Kind::Bevel {
                                    ui.horizontal(|ui| {
                                        finished |= ui.selectable_value(&mut modifier.style, edge_treatments::Style::Rounded, "Rounded").changed();
                                        finished |= ui.selectable_value(&mut modifier.style, edge_treatments::Style::Flat, "Flat / segmented").changed();
                                    });
                                }
                                ui.horizontal(|ui| {
                                    ui.label(if modifier.style == edge_treatments::Style::Rounded { "Radius" } else if modifier.kind==edge_treatments::Kind::Chamfer {"First distance"} else { "Distance" });
                                    let response = ui.add(egui::DragValue::new(&mut modifier.amount).speed(0.01));
                                    finished |= response.drag_stopped() || response.lost_focus();
                                });
                                if modifier.kind==edge_treatments::Kind::Chamfer {
                                    use edge_treatments::ChamferDefinition;
                                    egui::ComboBox::from_id_salt("chamfer_definition").selected_text(match modifier.chamfer {ChamferDefinition::Equal=>"Equal distance",ChamferDefinition::TwoDistances{..}=>"Two distances",ChamferDefinition::DistanceAngle{..}=>"Distance and angle"}).show_ui(ui,|ui| {
                                        for (active,label,value) in [
                                            (matches!(modifier.chamfer,ChamferDefinition::Equal),"Equal distance",ChamferDefinition::Equal),
                                            (matches!(modifier.chamfer,ChamferDefinition::TwoDistances{..}),"Two distances",ChamferDefinition::TwoDistances{second:modifier.amount}),
                                            (matches!(modifier.chamfer,ChamferDefinition::DistanceAngle{..}),"Distance and angle",ChamferDefinition::DistanceAngle{degrees:45.})
                                        ] {
                                            if ui.selectable_label(active,label).clicked() && !active {modifier.chamfer=value;finished=true;}
                                        }
                                    });
                                    match &mut modifier.chamfer {
                                        ChamferDefinition::Equal=>{},
                                        ChamferDefinition::TwoDistances{second}=> {finished|=scalar(ui,"Second distance",second,0.00001..=100.);},
                                        ChamferDefinition::DistanceAngle{degrees}=> {finished|=scalar(ui,"Angle · degrees",degrees,0.001..=89.999);},
                                    }
                                    if !matches!(modifier.chamfer,ChamferDefinition::Equal) {
                                        finished|=ui.checkbox(&mut modifier.reverse,"Reverse direction").changed();
                                        ui.weak(if modifier.reverse {"First distance and angle reference the adjacent faces."}else{"First distance and angle reference the selected face."});
                                    }
                                }
                                if modifier.style == edge_treatments::Style::Rounded && draft.shape != Shape::Circle {
                                    ui.horizontal(|ui| {
                                        ui.label("Segments");
                                        let response=ui.add(egui::DragValue::new(&mut modifier.segments).range(2..=16));
                                        finished |= response.drag_stopped() || response.lost_focus();
                                    });
                                } else if modifier.style == edge_treatments::Style::Flat && modifier.kind == edge_treatments::Kind::Bevel { ui.weak("Flat bevel uses a straight cross-section, matching an equal-distance chamfer."); }
                                ui.separator();
                            });
                        }
                        if let Some(index) = remove { self.finish_model_edit(); draft.edge_treatments.remove(index); }
                        ui.weak("Applied geometry stays editable. Source parameters are retained as modeling history.");
                    }
                    if let Some(operation)=&mut draft.taper {
                        ui.separator();ui.label("Draft · source side walls");
                        finished|=scalar(ui,"Angle · degrees",&mut operation.degrees,-80.0..=80.0);
                        ui.horizontal(|ui| {
                            ui.label("Neutral cap");
                            finished|=ui.selectable_value(&mut operation.neutral,edge_treatments::FaceTarget::Bottom,"Bottom").changed();
                            finished|=ui.selectable_value(&mut operation.neutral,edge_treatments::FaceTarget::Top,"Top").changed();
                        });
                        ui.weak("The neutral cap stays fixed. Positive angles expand away from it; negative angles taper inward. Applied before object scaling.");
                        if ui.button("Remove draft").clicked(){self.finish_model_edit();draft.taper=None;finished=true;}
                    }else if draft.extrusion>0. && draft.shape!=Shape::Line && draft.bore.is_none() {
                        let supported=matches!(draft.shape,Shape::Rectangle|Shape::Circle|Shape::Semicircle) && draft.hollow.is_none() && draft.edge_treatments.is_empty() && draft.component_edits.is_empty();
                        if ui.add_enabled(supported,egui::Button::new("Draft walls")).on_disabled_hover_text("Source prisms are supported. Drafting treated, shelled or deformed geometry requires general trimming.").clicked(){self.finish_model_edit();draft.taper=Some(drafting::Draft{degrees:3.,neutral:edge_treatments::FaceTarget::Bottom});finished=true;}
                    }
                    if let Some(operation)=&mut draft.hollow {
                        ui.separator();ui.label("Shell · source-local units");
                        ui.horizontal(|ui| {
                            finished|=ui.selectable_value(&mut operation.outward,false,"Inward").changed();
                            finished|=ui.selectable_value(&mut operation.outward,true,"Outward").changed();
                        });
                        finished|=scalar(ui,"Wall thickness",&mut operation.thickness,0.00001..=100.);
                        finished|=ui.checkbox(&mut operation.closed,"Closed cavity · keep all outer faces").changed();
                        ui.add_enabled_ui(!operation.closed,|ui| {
                        egui::ComboBox::from_id_salt("shell_opening").selected_text(format!("Opening · {}",operation.opening.label())).show_ui(ui,|ui| {
                            for face in [edge_treatments::FaceTarget::Top,edge_treatments::FaceTarget::Bottom,edge_treatments::FaceTarget::Left,edge_treatments::FaceTarget::Right,edge_treatments::FaceTarget::Front,edge_treatments::FaceTarget::Back] {
                                if face.supported(&before) {finished|=ui.selectable_value(&mut operation.opening,face,face.label()).changed();}
                            }
                        });
                        });
                        ui.weak(if operation.closed {"Uniform thickness is applied before object scaling. All outer faces remain. Inward preserves the exterior; outward preserves the cavity at the source dimensions."}else{"Uniform thickness is applied before object scaling. The opening plane stays fixed. Inward preserves the exterior; outward preserves the cavity at the source dimensions."});
                        if ui.button("Remove shell").clicked(){self.finish_model_edit();draft.hollow=None;finished=true;}
                    }else if draft.extrusion>0. && draft.shape!=Shape::Line && draft.bore.is_none() {
                        let supported=draft.taper.is_none() && draft.edge_treatments.is_empty() && draft.component_edits.is_empty();
                        if ui.add_enabled(supported,egui::Button::new("Shell · open top")).on_disabled_hover_text("Shell requires a source prism; treated, drafted or deformed solids need general offset/retrim.").clicked(){
                            match shelling::Hollow::new(&draft,edge_treatments::FaceTarget::Top) {
                                Ok(operation)=>{self.finish_model_edit();draft.hollow=Some(operation);finished=true;}
                                Err(error)=>self.error=Some(error),
                            }
                        }
                    }
                    if draft.construction {
                        finished|=scalar(ui,"Reference width",&mut draft.width,0.01..=100.);
                        finished|=scalar(ui,"Reference depth",&mut draft.depth,0.01..=100.);
                    }else{finished |= dimensions(ui,&mut draft);}
                    if !draft.construction && draft.shape!=Shape::Line && draft.bore.is_none() {finished|=ui.checkbox(&mut draft.cuts_profiles,"Cut older coplanar profiles").changed();}
                    ui.add_space(8.);finished |= vector(ui,if draft.bore.is_some() {"Position · attached reference placement"}else{"Position · world units"},&mut draft.position,-1000.0..=1000.0);
                    finished |= vector(ui,if draft.bore.is_some() {"Rotation · reference placement · degrees"}else{"Rotation · degrees · local X, Y, Z order"},&mut draft.rotation,-36000.0..=36000.0);
                    finished |= vector(ui,"Scale · local axes",&mut draft.scale,0.01..=100.0);
                    ui.weak(if draft.construction {"Placement and reference size update automatically. Drawings retain their captured placement."}else{"Changes update automatically. Rotation and scale pivot at the profile center; extrusion follows local +Z."});
                    let delete=ui.button("Delete selected").clicked();
                    if draft != before {
                        self.request_change(evaluation::Change::Edit(Box::new(draft.clone()),true));
                    }
                    if finished {self.finish_model_edit();}
                    self.draft=Some(draft);
                    if delete {self.delete_selected();}
                }
                ui.separator();
                if ui.button("Fit model").clicked(){self.camera.fit(&self.viewport.scene);}
            });
        });
        if !ui.input(|i| i.focused) {
            self.finish_model_edit();
            if self.transform.take().is_some() {
                self.evaluator.cancel();
                self.refresh();
            }
        }
        egui::CentralPanel::default()
            .frame(egui::Frame::NONE)
            .show_inside(ui, |ui| {
                let size = ui.available_size();
                let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click_and_drag());
                let transform_tool = matches!(self.tool, Tool::Move | Tool::Rotate);
                let rotate = self.tool == Tool::Rotate;
                let mut gizmo_handled = self.transform.is_some();
                if self.transform.is_none() {
                    self.camera
                        .interact(&response, ui, self.tool == Tool::Orbit);
                }
                if transform_tool
                    && self.pending.is_none()
                    && !self.evaluator.pending()
                    && response.hovered()
                    && !ui.input(|i| selection::extend(i) || i.modifiers.alt)
                    && ui.input(|i| i.pointer.primary_pressed())
                    && let (Some(point), Some(pivot)) = (
                        ui.input(|i| gizmo::press_origin(&i.events)),
                        self.gizmo_pivot(),
                    )
                    && let Some(handle) = gizmo::hit(&self.camera, rect, pivot, rotate, point)
                    && !self.selection.is_empty()
                {
                    self.face_offset = None;
                    self.finish_model_edit();
                    self.transform = Some(TransformGesture {
                        preview: self.model.bodies.clone(),
                        bodies: self.model.bodies.clone(),
                        scene: self.viewport.scene.clone(),
                        selection: self.selection.clone(),
                        drag: gizmo::Drag::new(handle, point, pivot),
                        valid: true,
                    });
                    self.error = None;
                    gizmo_handled = true;
                }
                if let Some(mut gesture) = self.transform.take() {
                    if let Some(point) = ui.input(|i| i.pointer.interact_pos()) {
                        gesture.drag.update(&self.camera, point, rect, self.snap);
                        let result = components::transform_selection(
                            &gesture.bodies,
                            &gesture.scene,
                            self.selection_mode,
                            &gesture.selection,
                            gesture.drag.matrix,
                        );
                        match result {
                            Ok(preview) => {
                                if gesture.preview != preview {
                                    self.request_change(evaluation::Change::Preview(
                                        preview.clone(),
                                    ));
                                }
                                gesture.preview = preview;
                                gesture.valid = true;
                                self.error = None;
                            }
                            Err(error) => {
                                gesture.valid = false;
                                self.error = Some(error);
                            }
                        }
                    }
                    if ui.input(|i| i.pointer.primary_released()) {
                        if gesture.valid {
                            self.request_change(evaluation::Change::Transform(gesture.preview));
                        } else {
                            self.refresh();
                        }
                    } else {
                        self.transform = Some(gesture);
                    }
                }
                if response.clicked()
                    && !gizmo_handled
                    && let Some(point) = response.interact_pointer_pos()
                {
                    match self.tool {
                        Tool::Select | Tool::Move | Tool::Rotate => {
                            if transform_tool && ui.input(|i| i.modifiers.alt) {
                                if let Some(pivot) = self.gizmo_pivot() {
                                    self.pivot = self.camera.screen_plane(point, rect, pivot);
                                }
                            } else {
                                let mode = self.selection_mode;
                                let picked = self.viewport.pick(&self.camera, point, rect, mode);
                                self.pick(picked, ui.input(selection::extend));
                            }
                        }
                        Tool::Create(shape)
                            if !ui.input(selection::extend) && !self.evaluator.loading() =>
                        {
                            match self.creation_point(point, rect) {
                                Ok(target) => {
                                    if let Some(anchor) = self.anchor {
                                        match target.plane.body(
                                            shape,
                                            anchor.into(),
                                            target.position,
                                            self.drawing.cut_on_creation,
                                        ) {
                                            Ok(mut body) => {
                                                if shape != Shape::Line
                                                    && let Some(support) =
                                                        self.construction_support.take()
                                                {
                                                    body.cuts_profiles = false;
                                                    body.bore = Some(support);
                                                }
                                                self.commit(body);
                                                self.anchor = None;
                                                self.construction = None;
                                            }
                                            Err(error) => self.error = Some(error),
                                        }
                                    } else {
                                        self.anchor = Some(target.position.into());
                                        self.construction = Some(target.plane);
                                        self.construction_support = (shape != Shape::Line)
                                            .then(|| self.capture_support(&target, point, rect))
                                            .flatten();
                                        self.error = None;
                                    }
                                }
                                Err(error) => self.error = Some(error),
                            }
                        }
                        _ => {}
                    }
                }
                let pixels = ui.ctx().pixels_per_point();
                let id = self.viewport.render(
                    [(size.x * pixels) as u32, (size.y * pixels) as u32],
                    &self.camera,
                );
                ui.painter().image(
                    id,
                    rect,
                    egui::Rect::from_min_max(egui::pos2(0., 0.), egui::pos2(1., 1.)),
                    egui::Color32::WHITE,
                );
                if self.selection_mode == SelectionMode::Point {
                    let painter = ui.painter().with_clip_rect(rect);
                    for (id, g) in &self.viewport.scene {
                        for (index, p) in g.points.iter().enumerate() {
                            if self.viewport.query().visible(&self.camera, *p, rect) {
                                let chosen = self.selection.contains(&(*id, Some(index)));
                                let point = self.camera.project(*p, rect).0;
                                painter.circle_filled(
                                    point,
                                    if chosen { 5. } else { 3.5 },
                                    if chosen {
                                        egui::Color32::LIGHT_BLUE
                                    } else {
                                        egui::Color32::WHITE
                                    },
                                );
                                painter.circle_stroke(
                                    point,
                                    if chosen { 6. } else { 4. },
                                    egui::Stroke::new(1., egui::Color32::BLACK),
                                );
                            }
                        }
                    }
                }
                if transform_tool && let Some(pivot) = self.gizmo_pivot() {
                    let (pivot, active) = if let Some(gesture) = &self.transform {
                        let p = if rotate {
                            gesture.drag.pivot
                        } else {
                            gesture.drag.matrix.transform_point(gesture.drag.pivot)
                        };
                        (p, Some(gesture.drag.handle))
                    } else {
                        (pivot, None)
                    };
                    gizmo::draw(
                        ui.painter(),
                        &self.camera,
                        rect,
                        pivot,
                        rotate,
                        active,
                        response.hover_pos(),
                    );
                    if let Some(gesture) = &self.transform {
                        ui.painter().text(
                            self.camera.project(pivot, rect).0 + egui::vec2(12., 22.),
                            egui::Align2::LEFT_TOP,
                            &gesture.drag.value,
                            egui::FontId::monospace(13.),
                            egui::Color32::WHITE,
                        );
                    }
                }
                if let Tool::Create(shape) = self.tool
                    && let Some(pointer) = response.hover_pos()
                    && let Ok(target) = self.creation_point(pointer, rect)
                {
                    let p = target.position;
                    let screen = self.camera.project(p, rect).0;
                    if let Some(guide) = target.guide {
                        ui.painter().line_segment(
                            guide.map(|p| self.camera.project(p, rect).0),
                            egui::Stroke::new(1., egui::Color32::LIGHT_GREEN),
                        );
                    }
                    ui.painter().text(
                        screen + egui::vec2(10., 12.),
                        egui::Align2::LEFT_TOP,
                        target.label,
                        egui::FontId::proportional(12.),
                        egui::Color32::LIGHT_GREEN,
                    );
                    ui.painter().circle_stroke(
                        screen,
                        4.,
                        egui::Stroke::new(1.5, egui::Color32::LIGHT_GREEN),
                    );
                    if let Some(a) = self.anchor {
                        if let Ok(body) =
                            target
                                .plane
                                .body(shape, a.into(), p, self.drawing.cut_on_creation)
                        {
                            if let Ok(g) = model::evaluate(&body) {
                                for edge in g.edges {
                                    ui.painter().line_segment(
                                        edge.map(|p| self.camera.project(p, rect).0),
                                        egui::Stroke::new(1.5, egui::Color32::LIGHT_GREEN),
                                    );
                                }
                            }
                            ui.painter().text(
                                screen + egui::vec2(12., -12.),
                                egui::Align2::LEFT_BOTTOM,
                                format!(
                                    "{:.2}{}",
                                    body.width,
                                    if shape == Shape::Rectangle {
                                        format!(" × {:.2}", body.depth)
                                    } else {
                                        String::new()
                                    }
                                ),
                                egui::FontId::monospace(12.),
                                egui::Color32::WHITE,
                            );
                        }
                        ui.painter().circle_filled(
                            self.camera.project(a.into(), rect).0,
                            4.,
                            egui::Color32::LIGHT_GREEN,
                        );
                    }
                }
            });
    }
}

#[cfg(test)]
mod shortcut_tests {
    use super::*;
    #[test]
    fn extrusion_drag_survives_inspector_controls_disappearing() {
        let ctx = egui::Context::default();
        let mut value = 0.;
        let mut time = 0.;
        let mut frame = |events, flat| {
            time += 0.02;
            let mut response = None;
            let _ = ctx.run_ui(
                egui::RawInput {
                    time: Some(time),
                    screen_rect: Some(egui::Rect::from_min_size(
                        egui::Pos2::ZERO,
                        egui::vec2(600., 400.),
                    )),
                    events,
                    ..Default::default()
                },
                |root| {
                    egui::CentralPanel::default().show_inside(root, |ui| {
                        // The actual inspector offers this only while the profile is a sheet.
                        if flat {
                            let _ = ui.button("Cap openings");
                        }
                        response = Some(scalar_response(ui, "Extrusion", &mut value, 0.0..=100.));
                    });
                },
            );
            (response.unwrap(), value)
        };
        let (initial, _) = frame(vec![], true);
        let start = initial.rect.center();
        let button = |pos, pressed| egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: egui::Modifiers::NONE,
        };
        frame(
            vec![egui::Event::PointerMoved(start), button(start, true)],
            true,
        );
        let moved = start + egui::vec2(30., 0.);
        let (drag, first) = frame(vec![egui::Event::PointerMoved(moved)], true);
        assert!(drag.dragged());
        let (continued, second) = frame(
            vec![egui::Event::PointerMoved(moved + egui::vec2(30., 0.))],
            false,
        );
        assert_eq!(initial.id, continued.id);
        assert!(
            continued.dragged(),
            "Geometry updates must not release the numeric drag"
        );
        assert!(
            second > first,
            "The held gesture must keep adjusting extrusion"
        );
        let (released, _) = frame(vec![button(moved + egui::vec2(30., 0.), false)], false);
        assert!(released.drag_stopped());
    }

    #[test]
    fn redo_chord_is_not_consumed_as_undo() {
        for (modifiers, redo) in [
            (egui::Modifiers::COMMAND, false),
            (egui::Modifiers::COMMAND | egui::Modifiers::SHIFT, true),
        ] {
            let mut input = egui::InputState::default();
            input.events.push(egui::Event::Key {
                key: egui::Key::Z,
                physical_key: None,
                pressed: true,
                repeat: false,
                modifiers,
            });
            assert_eq!(history_shortcut(&mut input), Some(redo));
            assert_eq!(history_shortcut(&mut input), None);
        }
    }
}
