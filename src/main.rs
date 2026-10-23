#![cfg_attr(target_os = "windows", windows_subsystem = "windows")]

use eframe::egui;
mod viewport;
use viewport::{Camera, View, Viewport};

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 800.0])
            .with_min_inner_size([640.0, 480.0]),
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
            Ok(Box::new(OpenSketcher {
                camera: Camera::default(),
                viewport: Viewport::new(state),
            }))
        }),
    )
}

struct OpenSketcher {
    camera: Camera,
    viewport: Viewport,
}

impl eframe::App for OpenSketcher {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::Panel::top("header").show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading("OpenSketcher");
                ui.separator();
                ui.label("Untitled");
            });
        });
        egui::Panel::bottom("status").show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(
                    "Drag to orbit · Shift-drag / middle / right drag to pan · Scroll to zoom",
                );
                ui.separator();
                ui.label("X red · Y green · Z blue");
            });
        });
        egui::Panel::right("inspector")
            .default_size(200.0)
            .show_inside(ui, |ui| {
                ui.heading("Model");
                ui.separator();
                ui.label("Reference cube");
                ui.weak("Viewport demonstration");
                ui.add_space(16.0);
                ui.label("Camera");
                ui.label(format!("Visible height: {:.1} units", self.camera.span));
                if ui.button("Fit reference").clicked() {
                    self.camera = Camera::default();
                }
            });
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
                ui.weak("Orthographic");
            });
            let size = ui.available_size();
            let (rect, response) = ui.allocate_exact_size(size, egui::Sense::drag());
            self.camera.interact(&response, ui);
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
        });
    }
}
