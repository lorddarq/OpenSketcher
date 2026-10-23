#![cfg_attr(target_os = "windows", windows_subsystem = "windows")]

use eframe::egui;

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
        Box::new(|_| Ok(Box::new(OpenSketcher))),
    )
}

struct OpenSketcher;

impl eframe::App for OpenSketcher {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::TopBottomPanel::top("header").show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading("OpenSketcher");
                ui.separator();
                ui.label("Untitled");
            });
        });
        egui::SidePanel::right("inspector")
            .default_width(220.0)
            .show_inside(ui, |ui| {
                ui.heading("Model");
                ui.label("No geometry yet.");
            });
        egui::CentralPanel::default().show_inside(ui, |ui| {
            ui.centered_and_justified(|ui| {
                ui.label("Modeling viewport — ready for implementation");
            });
        });
    }
}
