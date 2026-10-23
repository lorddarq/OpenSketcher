#![cfg_attr(target_os = "windows", windows_subsystem = "windows")]

use eframe::egui;
mod model;
mod project;
use project::Project;
use std::path::PathBuf;
mod viewport;
use model::{Body, Model, Shape};
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
            viewport.update(&[], None).expect("empty scene");
            Ok(Box::new(OpenSketcher {
                camera: Camera::default(),
                viewport,
                model: Model::default(),
                selected: None,
                draft: None,
                anchor: None,
                tool: Tool::Select,
                snap: true,
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
    Create(Shape),
}
#[derive(Clone, Copy)]
enum FileAction {
    New,
    Open,
    Close,
}

struct OpenSketcher {
    camera: Camera,
    viewport: Viewport,
    model: Model,
    selected: Option<u64>,
    draft: Option<Body>,
    anchor: Option<[f64; 3]>,
    tool: Tool,
    snap: bool,
    error: Option<String>,
    path: Option<PathBuf>,
    saved: Project,
    pending: Option<FileAction>,
    allow_close: bool,
}
impl OpenSketcher {
    fn snapshot(&self) -> Project {
        Project::new(
            self.model.bodies.clone(),
            self.camera.clone(),
            self.snap,
            self.selected,
        )
    }
    fn dirty(&self) -> bool {
        self.snapshot() != self.saved
    }
    fn save(&mut self, save_as: bool) -> bool {
        if self.anchor.is_some() {
            self.error = Some("Finish or cancel the shape preview before saving.".into());
            return false;
        }
        if self
            .draft
            .as_ref()
            .is_some_and(|b| !self.model.bodies.contains(b))
        {
            self.error = Some("Correct the invalid object properties before saving.".into());
            return false;
        }
        self.model.finish_edit();
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
        match action {
            FileAction::Close => {
                self.allow_close = true;
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            FileAction::New => {
                if let Err(e) = self.viewport.update(&[], None) {
                    self.error = Some(e);
                    return;
                }
                self.model = Model::default();
                self.camera = Camera::default();
                self.snap = true;
                self.selected = None;
                self.draft = None;
                self.anchor = None;
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
                let loaded = Project::load(&path)
                    .and_then(|p| self.viewport.update(&p.bodies, p.selected).map(|()| p));
                match loaded {
                    Ok(p) => {
                        self.model = Model::from_bodies(p.bodies.clone());
                        self.camera = p.camera.clone();
                        self.snap = p.snapping;
                        self.selected = p.selected;
                        self.draft = self
                            .model
                            .bodies
                            .iter()
                            .find(|b| Some(b.id) == self.selected)
                            .cloned();
                        self.anchor = None;
                        self.tool = Tool::Select;
                        self.path = Some(path);
                        self.saved = p;
                        self.error = None;
                    }
                    Err(e) => self.error = Some(e),
                }
            }
        }
    }
    fn refresh(&mut self) {
        if let Err(e) = self.viewport.update(&self.model.bodies, self.selected) {
            self.error = Some(e);
        }
    }
    fn select(&mut self, id: Option<u64>) {
        self.model.finish_edit();
        self.selected = id;
        self.draft = self.model.bodies.iter().find(|b| Some(b.id) == id).cloned();
        self.refresh();
    }
    fn commit(&mut self, body: Body) {
        match self.model.commit(body) {
            Ok(id) => {
                self.error = None;
                self.select(Some(id));
            }
            Err(e) => self.error = Some(e),
        }
    }
    fn history(&mut self, redo: bool) {
        if self.anchor.take().is_some() {
            return;
        }
        if redo {
            self.model.redo();
        } else {
            self.model.undo();
        }
        let id = self
            .selected
            .filter(|id| self.model.bodies.iter().any(|b| b.id == *id));
        self.select(id);
    }
    fn creation_point(&self, point: egui::Pos2, rect: egui::Rect) -> Option<[f64; 3]> {
        let mut p = self.camera.ground(point, rect)?;
        if self.snap {
            for v in &mut p {
                *v = (*v * 2.).round() / 2.;
            }
            if let Some(a) = self.anchor {
                let tolerance = (self.camera.span / rect.height().max(1.) * 8.) as f64;
                for axis in 0..2 {
                    if (p[axis] - a[axis]).abs() < tolerance {
                        p[axis] = a[axis];
                    }
                }
            }
        }
        Some(p)
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
    finished |= scalar(ui, "Extrusion", &mut b.extrusion, 0.0..=100.);
    if b.shape == Shape::Line {
        ui.weak("Extruding a line creates a surface.");
    }
    finished
}
fn scalar(
    ui: &mut egui::Ui,
    label: &str,
    v: &mut f64,
    range: std::ops::RangeInclusive<f64>,
) -> bool {
    ui.horizontal(|ui| {
        ui.label(label);
        let response = ui.add(egui::DragValue::new(v).speed(0.05).range(range));
        response.drag_stopped() || response.lost_focus()
    })
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
            if ui.input_mut(|i| i.consume_key(egui::Modifiers::COMMAND, egui::Key::Z)) {
                self.history(false);
            }
            if ui.input_mut(|i| {
                i.consume_key(
                    egui::Modifiers::COMMAND | egui::Modifiers::SHIFT,
                    egui::Key::Z,
                )
            }) {
                self.history(true);
            }
            if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                if self.anchor.take().is_none() {
                    self.tool = Tool::Select;
                }
                self.select(self.selected);
            }
            if ui.input(|i| i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace))
                && let Some(id) = self.selected
            {
                self.model.delete(id);
                self.select(None);
            }
        }
        egui::Panel::top("header").show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading("OpenSketcher");
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
            ui.horizontal_wrapped(|ui| {
                for (tool, label) in [
                    (Tool::Select, "Select"),
                    (Tool::Orbit, "Orbit"),
                    (Tool::Create(Shape::Rectangle), "Rectangle"),
                    (Tool::Create(Shape::Circle), "Circle"),
                    (Tool::Create(Shape::Line), "Line"),
                    (Tool::Create(Shape::Semicircle), "Semicircle"),
                ] {
                    if ui.selectable_label(self.tool == tool, label).clicked() {
                        self.tool = tool;
                        self.anchor = None;
                        self.model.finish_edit();
                    }
                }
                ui.separator();
                ui.checkbox(&mut self.snap, "Snap").on_hover_text(
                    "Enabled by default: 0.5-unit grid and X/Y alignment to the first point",
                );
            });
        });
        egui::Panel::bottom("status").show_inside(ui,|ui| {
            if let Some(e)=&self.error {ui.colored_label(egui::Color32::LIGHT_RED,e);}
            ui.label(match self.tool {
                Tool::Select=>"Click geometry to select · Values update automatically · Shift-drag / right / middle to pan · Scroll to zoom",
                Tool::Orbit=>"Drag to orbit · Shift-drag / right / middle to pan · Scroll to zoom",
                Tool::Create(_)=>"Pick two points on the XY plane · Escape cancels the preview · Snap is on by default",
            });
        });
        egui::Panel::right("inspector").default_size(330.).show_inside(ui,|ui| {
            egui::ScrollArea::vertical().show(ui,|ui| {
                if let Tool::Create(shape)=self.tool {
                    ui.heading(format!("Create {}",shape.label()));
                    ui.label(match shape { Shape::Rectangle=>"Pick opposite corners on the XY plane.", Shape::Circle=>"Pick center, then a point on the radius.", Shape::Line=>"Pick the start and end points.", Shape::Semicircle=>"Pick the two diameter endpoints." });
                    ui.weak(if self.anchor.is_some() {"Pick the second point. Escape cancels."}else{"Pick the first point in the viewport."});
                    if shape==Shape::Semicircle {ui.weak("The filled half-disc lies to the left of the directed diameter.");}
                    ui.separator();
                }
                ui.heading("Model");
                if self.model.bodies.is_empty(){ui.weak("No geometry. Choose a creation tool.");}
                let mut next=None;
                for body in &self.model.bodies {
                    if ui.selectable_label(self.selected==Some(body.id),format!("{} {}",body.shape.label(),body.id)).clicked(){next=Some(body.id);}
                }
                if let Some(id)=next {self.select(Some(id));}
                if let Some(mut draft)=self.draft.take() {
                    ui.separator();ui.heading(format!("{} {}",draft.shape.label(),draft.id));
                    let before=draft.clone();
                    let mut finished=dimensions(ui,&mut draft);
                    ui.add_space(8.);finished |= vector(ui,"Position · world units",&mut draft.position,-1000.0..=1000.0);
                    finished |= vector(ui,"Rotation · degrees · local X, Y, Z order",&mut draft.rotation,-36000.0..=36000.0);
                    finished |= vector(ui,"Scale · local axes",&mut draft.scale,0.01..=100.0);
                    ui.weak("Changes update automatically. Rotation and scale pivot at the profile center; extrusion follows local +Z.");
                    let delete=ui.button("Delete selected").clicked();
                    if draft != before {
                        match self.model.edit(draft.clone()) {
                            Ok(_)=>{self.error=None;self.refresh();}
                            Err(e)=>self.error=Some(e),
                        }
                    }
                    if finished {self.model.finish_edit();}
                    self.draft=Some(draft);
                    if delete {if let Some(id) = self.selected { self.model.delete(id); }self.select(None);}
                }
                ui.separator();
                if ui.button("Fit model").clicked(){self.camera.fit(&self.viewport.scene);}
            });
        });
        if !ui.input(|i| i.focused) {
            self.model.finish_edit();
        }
        egui::CentralPanel::default().show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                for (view, label) in [
                    (View::Isometric, "Isometric"),
                    (View::Top, "Top"),
                    (View::Side, "Side"),
                ] {
                    if ui
                        .selectable_label(self.camera.view == view, label)
                        .clicked()
                    {
                        self.camera.preset(view);
                    }
                }
                if self.camera.view == View::Free {
                    ui.weak("Free orbit");
                }
                ui.separator();
                ui.weak("Orthographic · X red / Y green / Z blue");
            });
            let size = ui.available_size();
            let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click_and_drag());
            self.camera
                .interact(&response, ui, self.tool == Tool::Orbit);
            if response.clicked()
                && !ui.input(|i| i.modifiers.shift)
                && let Some(point) = response.interact_pointer_pos()
            {
                match self.tool {
                    Tool::Select => {
                        let id = self.viewport.pick(&self.camera, point, rect);
                        self.select(id);
                    }
                    Tool::Create(shape) => {
                        if let Some(p) = self.creation_point(point, rect) {
                            if let Some(a) = self.anchor {
                                match Body::from_points(shape, a, p) {
                                    Ok(body) => {
                                        self.commit(body);
                                        self.anchor = None;
                                    }
                                    Err(e) => self.error = Some(e),
                                }
                            } else {
                                self.anchor = Some(p);
                                self.error = None;
                            }
                        } else {
                            self.error = Some(
                                "The XY plane is edge-on. Switch to Top or Isometric to draw."
                                    .into(),
                            );
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
            if let Tool::Create(shape) = self.tool
                && let Some(pointer) = response.hover_pos()
                && let Some(p) = self.creation_point(pointer, rect)
            {
                let screen = self.camera.project(p.into(), rect).0;
                ui.painter().circle_stroke(
                    screen,
                    4.,
                    egui::Stroke::new(1.5, egui::Color32::LIGHT_GREEN),
                );
                if let Some(a) = self.anchor {
                    if let Ok(body) = Body::from_points(shape, a, p) {
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
