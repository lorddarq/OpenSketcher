#![cfg_attr(target_os = "windows", windows_subsystem = "windows")]

use eframe::egui;
mod components;
mod export;
mod gizmo;
mod model;
mod project;
mod regions;
mod selection;
mod snapping;
use project::Project;
use selection::SelectionMode;
use std::path::PathBuf;
mod viewport;
use model::{Body, Model, Shape};
use truck_modeling::{Point3, Transform};
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
            viewport
                .update(&[], None, SelectionMode::Volume, None)
                .expect("empty scene");
            Ok(Box::new(OpenSketcher {
                camera: Camera::default(),
                viewport,
                model: Model::default(),
                selected: None,
                selection_mode: SelectionMode::Volume,
                selected_element: None,
                draft: None,
                anchor: None,
                tool: Tool::Select,
                transform: None,
                pivot: None,
                snap: true,
                drawing: snapping::Preferences::default(),
                construction: None,
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
    body: Body,
    geometry: model::Geometry,
    drag: gizmo::Drag,
    preview: Body,
    valid: bool,
}

struct OpenSketcher {
    camera: Camera,
    viewport: Viewport,
    model: Model,
    selected: Option<u64>,
    selection_mode: SelectionMode,
    selected_element: Option<usize>,
    draft: Option<Body>,
    anchor: Option<[f64; 3]>,
    tool: Tool,
    transform: Option<TransformGesture>,
    pivot: Option<Point3>,
    snap: bool,
    drawing: snapping::Preferences,
    construction: Option<snapping::Plane>,
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
        project
    }
    fn dirty(&self) -> bool {
        self.snapshot() != self.saved
    }
    fn export(&mut self, format: export::Format) {
        if self.transform.is_some()
            || self.anchor.is_some()
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
        self.model.finish_edit();
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
        if self.transform.is_some() {
            self.error = Some("Finish or cancel the transform before saving.".into());
            return false;
        }
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
                if let Err(e) = self.viewport.update(&[], None, SelectionMode::Volume, None) {
                    self.error = Some(e);
                    return;
                }
                self.transform = None;
                self.pivot = None;
                self.model = Model::default();
                self.camera = Camera::default();
                self.snap = true;
                self.drawing = snapping::Preferences::default();
                self.selected = None;
                self.selection_mode = SelectionMode::Volume;
                self.selected_element = None;
                self.draft = None;
                self.anchor = None;
                self.construction = None;
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
                let loaded = Project::load(&path).and_then(|p| {
                    self.viewport
                        .update(&p.bodies, p.selected, p.selection_mode, p.selected_element)
                        .map(|()| p)
                });
                match loaded {
                    Ok(p) => {
                        self.transform = None;
                        self.pivot = None;
                        self.model = Model::from_bodies(p.bodies.clone());
                        self.camera = p.camera.clone();
                        self.snap = p.snapping;
                        self.drawing = p.drawing.clone();
                        self.selected = p.selected;
                        self.selection_mode = p.selection_mode;
                        self.selected_element = p.selected_element;
                        self.draft = self
                            .model
                            .bodies
                            .iter()
                            .find(|b| {
                                Some(b.id) == self.selected
                                    && self.selection_mode == SelectionMode::Volume
                            })
                            .cloned();
                        self.anchor = None;
                        self.construction = None;
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
        if let Err(e) = self.viewport.update(
            &self.model.bodies,
            self.selected,
            self.selection_mode,
            self.selected_element,
        ) {
            self.error = Some(e);
        }
    }
    fn select(&mut self, id: Option<u64>) {
        self.set_selection(id, SelectionMode::Volume, None);
    }
    fn set_selection(&mut self, id: Option<u64>, mode: SelectionMode, element: Option<usize>) {
        self.transform = None;
        self.pivot = None;
        self.model.finish_edit();
        self.selected = id;
        self.selection_mode = mode;
        self.selected_element = element;
        self.draft = if mode == SelectionMode::Volume {
            self.model.bodies.iter().find(|b| Some(b.id) == id).cloned()
        } else {
            None
        };
        self.refresh();
    }
    fn gizmo_pivot(&self) -> Option<Point3> {
        self.pivot.or_else(|| {
            let (_, geometry) = self
                .viewport
                .scene
                .iter()
                .find(|(id, _)| Some(*id) == self.selected)?;
            components::pivot(geometry, self.selection_mode, self.selected_element)
        })
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
    fn delete_selected(&mut self) {
        if let Some(id) = self.selected {
            match self.model.delete(id) {
                Ok(()) => {
                    self.error = None;
                    self.select(None);
                }
                Err(error) => self.error = Some(error),
            }
        }
    }
    fn history(&mut self, redo: bool) {
        if self.transform.take().is_some() {
            self.refresh();
            return;
        }
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
        self.refresh();
        let valid = id.is_some_and(|id| {
            self.viewport.scene.iter().any(|(owner, g)| {
                *owner == id
                    && self
                        .selected_element
                        .is_some_and(|element| element < self.selection_mode.count(g))
            })
        });
        if self.selection_mode == SelectionMode::Volume {
            self.select(id);
        } else if valid {
            self.set_selection(id, self.selection_mode, self.selected_element);
        } else {
            self.set_selection(None, self.selection_mode, None);
        }
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
        } else {
            snapping::acquire_plane(
                &self.viewport.scene,
                &self.camera,
                rect,
                point,
                selected,
                preferences.geometry,
                preferences.project_to_base_plane,
            )?
        };
        snapping::point(
            &self.viewport.scene,
            &self.camera,
            rect,
            point,
            plane,
            self.anchor.map(Point3::from),
            &preferences,
        )
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
                if self.transform.take().is_some() {
                    self.refresh();
                } else if self.anchor.take().is_none() {
                    self.tool = Tool::Select;
                }
            }
            if ui.input(|i| i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace))
                && self.selection_mode == SelectionMode::Volume
            {
                self.delete_selected();
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
                    SelectionMode::Volume=>"Volume selection · Click an object · Properties update automatically · Shift-drag / right / middle to pan",
                    SelectionMode::Face=>"Face selection · Click a visible face · Move/Rotate handles edit the selected face",
                    SelectionMode::Edge=>"Edge selection · Click near a visible boundary edge · Move/Rotate handles edit the selected edge",
                    SelectionMode::Point=>"Point selection · Click a visible topology vertex · Control-node editing is separate",
                },
                Tool::Move=>"Move · Select geometry, then drag an axis arrow or plane handle · Snap: 0.5 units · Escape cancels · Alt-click sets pivot",
                Tool::Rotate=>"Rotate · Select geometry, then drag an axis ring · Snap: 15° · Alt-click sets pivot (needed for a single point) · Escape cancels",
                Tool::Orbit=>"Drag to orbit · Shift-drag / right / middle to pan · Scroll to zoom",
                Tool::Create(_)=>"Draw on a selected or visible planar face, otherwise XY · Snap: visible geometry + grid · Escape cancels",
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
                                self.model.finish_edit();
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
            .enabled(self.pending.is_none() && self.transform.is_none())
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
                    if ui.selectable_label(self.selected==Some(body.id),format!("{} {}",body.shape.label(),body.id)).clicked(){next=Some(body.id);}
                }
                if let Some(id)=next {self.select(Some(id));}
                if self.selection_mode!=SelectionMode::Volume {
                    ui.separator();
                    if let (Some(id),Some(element))=(self.selected,self.selected_element) {
                        ui.label(format!("{} {} · Object {}",self.selection_mode.label(),element+1,id));
                    }else{ui.label(format!("Pick a {} in the viewport.",self.selection_mode.label().to_lowercase()));}
                    ui.weak("Choose Move or Rotate and drag the gizmo to edit this component. Adjacent boundaries rebuild together.");
                    if self.selection_mode==SelectionMode::Point { ui.weak("Rotation around a point itself has no effect. Alt-click to place an offset pivot."); }
                    if self.selection_mode==SelectionMode::Point {ui.weak("Points are topology vertices; deformation control nodes are separate future editing tools.");}
                }
                if let Some(mut draft)=self.draft.take() {
                    ui.separator();ui.heading(format!("{} {}",draft.shape.label(),draft.id));
                    let before=draft.clone();
                    if !draft.component_edits.is_empty() {
                        ui.label(format!("Component edits: {}",draft.component_edits.len()));
                        ui.weak("Curved and nonplanar component results use a regenerated faceted boundary.");
                        if ui.button("Remove component edits").clicked() { draft.component_edits.clear(); }
                        ui.label("Source profile parameters");
                    }
                    let mut finished=dimensions(ui,&mut draft);
                    if draft.shape!=Shape::Line {finished|=ui.checkbox(&mut draft.cuts_profiles,"Cut older coplanar profiles").changed();}
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
                    if delete {self.delete_selected();}
                }
                ui.separator();
                if ui.button("Fit model").clicked(){self.camera.fit(&self.viewport.scene);}
            });
        });
        if !ui.input(|i| i.focused) {
            self.model.finish_edit();
            if self.transform.take().is_some() {
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
                    && response.hovered()
                    && !ui.input(|i| i.modifiers.shift || i.modifiers.alt)
                    && ui.input(|i| i.pointer.primary_pressed())
                    && let (Some(point), Some(pivot)) = (
                        ui.input(|i| gizmo::press_origin(&i.events)),
                        self.gizmo_pivot(),
                    )
                    && let Some(handle) = gizmo::hit(&self.camera, rect, pivot, rotate, point)
                    && let Some(body) = self
                        .model
                        .bodies
                        .iter()
                        .find(|b| Some(b.id) == self.selected)
                        .cloned()
                    && let Some((_, geometry)) =
                        self.viewport.scene.iter().find(|(id, _)| *id == body.id)
                {
                    self.model.finish_edit();
                    self.transform = Some(TransformGesture {
                        preview: body.clone(),
                        body,
                        geometry: geometry.clone(),
                        drag: gizmo::Drag::new(handle, point, pivot),
                        valid: true,
                    });
                    self.error = None;
                    gizmo_handled = true;
                }
                if let Some(mut gesture) = self.transform.take() {
                    if let Some(point) = ui.input(|i| i.pointer.interact_pos()) {
                        gesture.drag.update(&self.camera, point, rect, self.snap);
                        let result = components::transformed(
                            &gesture.body,
                            &gesture.geometry,
                            self.selection_mode,
                            self.selected_element,
                            gesture.drag.matrix,
                        )
                        .and_then(|preview| {
                            let mut bodies = self.model.bodies.clone();
                            if let Some(body) = bodies.iter_mut().find(|b| b.id == preview.id) {
                                *body = preview.clone();
                            }
                            self.viewport.update(
                                &bodies,
                                self.selected,
                                self.selection_mode,
                                self.selected_element,
                            )?;
                            Ok(preview)
                        });
                        match result {
                            Ok(preview) => {
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
                            match self.model.commit(gesture.preview) {
                                Ok(id) => {
                                    let pivot = self.pivot;
                                    self.set_selection(
                                        Some(id),
                                        self.selection_mode,
                                        self.selected_element,
                                    );
                                    self.pivot = pivot;
                                    self.error = None;
                                }
                                Err(error) => {
                                    self.error = Some(error);
                                    self.refresh();
                                }
                            }
                        } else {
                            self.refresh();
                        }
                    } else {
                        self.transform = Some(gesture);
                    }
                }
                if response.clicked()
                    && !gizmo_handled
                    && !ui.input(|i| i.modifiers.shift)
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
                                self.set_selection(
                                    picked.map(|p| p.0),
                                    mode,
                                    picked.and_then(|p| p.1),
                                );
                            }
                        }
                        Tool::Create(shape) => match self.creation_point(point, rect) {
                            Ok(target) => {
                                if let Some(anchor) = self.anchor {
                                    match target.plane.body(
                                        shape,
                                        anchor.into(),
                                        target.position,
                                        self.drawing.cut_on_creation,
                                    ) {
                                        Ok(body) => {
                                            self.commit(body);
                                            self.anchor = None;
                                            self.construction = None;
                                        }
                                        Err(error) => self.error = Some(error),
                                    }
                                } else {
                                    self.anchor = Some(target.position.into());
                                    self.construction = Some(target.plane);
                                    self.error = None;
                                }
                            }
                            Err(error) => self.error = Some(error),
                        },
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
                            if selection::visible(&self.viewport.scene, &self.camera, *p, rect) {
                                let chosen = self.selected == Some(*id)
                                    && self.selected_element == Some(index);
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
