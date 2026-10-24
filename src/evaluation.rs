//! One geometry worker; pending input is coalesced instead of flooding CPU threads.
use crate::model::{Body, Geometry, Model, evaluate_scene};
use std::sync::mpsc::{self, Receiver, Sender, TryRecvError};

#[derive(Clone)]
pub enum Change {
    Edit(Box<Body>, bool),
    Delete(Vec<u64>),
    Transform(Vec<Body>),
    Preview(Vec<Body>),
    History(bool),
    Open(std::path::PathBuf),
    EndEdit,
}
#[derive(Clone)]
struct Request {
    revision: u64,
    model: Model,
    changes: Vec<Change>,
}
pub struct Update {
    pub model: Model,
    pub scene: Vec<(u64, Geometry)>,
    pub display: std::sync::Arc<crate::viewport::SceneDisplay>,
    pub change: Change,
    pub selected: Option<u64>,
    pub loaded: Option<(std::path::PathBuf, crate::project::Project)>,
}
type Response = (u64, Result<Update, String>);
pub struct Evaluator {
    sender: Sender<Request>,
    receiver: Receiver<Response>,
    desired: Option<Request>,
    running: bool,
    revision: u64,
    latest: std::sync::Arc<std::sync::atomic::AtomicU64>,
    finish: bool,
}
impl Default for Evaluator {
    fn default() -> Self {
        let (sender, requests) = mpsc::channel::<Request>();
        let (results, receiver) = mpsc::channel();
        let latest = std::sync::Arc::new(std::sync::atomic::AtomicU64::new(0));
        let worker_latest = latest.clone();
        std::thread::Builder::new()
            .name("OpenSketcher geometry".into())
            .spawn(move || {
                let threads = std::thread::available_parallelism().map_or(1, |n| n.get().min(2));
                let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build();
                while let Ok(request) = requests.recv() {
                    let revision = request.revision;
                    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        pool.as_ref()
                            .map_err(|e| format!("Cannot start geometry workers: {e}"))?
                            .install(|| execute(request, &worker_latest))
                    }))
                    .unwrap_or_else(|_| {
                        Err("Geometry evaluation failed; the committed model was preserved.".into())
                    });
                    if results.send((revision, result)).is_err() {
                        break;
                    }
                }
            })
            .expect("Cannot start the geometry worker");
        Self {
            sender,
            receiver,
            desired: None,
            running: false,
            revision: 0,
            latest,
            finish: false,
        }
    }
}
impl Evaluator {
    pub fn request(&mut self, model: &Model, change: Change) {
        self.revision += 1;
        self.latest
            .store(self.revision, std::sync::atomic::Ordering::Relaxed);
        self.finish = false;
        let request = self.desired.get_or_insert_with(|| Request {
            revision: self.revision,
            model: model.clone(),
            changes: Vec::new(),
        });
        request.revision = self.revision;
        let replace = match (request.changes.last(), &change) {
            (Some(Change::Edit(a, true)), Change::Edit(b, true)) => a.id == b.id,
            (Some(Change::Preview(_)), Change::Preview(_) | Change::Transform(_)) => true,
            _ => false,
        };
        if replace {
            request.changes.pop();
        }
        request.changes.push(change);
        self.start();
    }
    fn start(&mut self) {
        if !self.running
            && let Some(request) = &self.desired
        {
            // Exactly one request is ever in flight. New input only updates desired.
            self.running = self.sender.send(request.clone()).is_ok();
        }
    }
    pub fn pending(&self) -> bool {
        self.desired.is_some()
    }
    pub fn editing(&self, id: u64) -> bool {
        self.desired.as_ref().is_some_and(|request| {
            request
                .changes
                .iter()
                .any(|change| matches!(change,Change::Edit(body,_) if body.id==id))
        })
    }
    pub fn loading(&self) -> bool {
        self.desired
            .as_ref()
            .is_some_and(|r| r.changes.iter().any(|c| matches!(c, Change::Open(_))))
    }
    pub fn running(&self) -> bool {
        self.running
    }
    pub fn finish_edit(&mut self) {
        self.finish = true;
        if let Some(request) = &mut self.desired
            && matches!(request.changes.last(), Some(Change::Edit(_, true)))
        {
            request.changes.push(Change::EndEdit);
        }
    }
    pub fn cancel_preview(&mut self) {
        if self
            .desired
            .as_ref()
            .is_some_and(|r| matches!(r.changes.last(), Some(Change::Preview(_))))
        {
            self.cancel();
        }
    }
    pub fn cancel(&mut self) {
        self.revision += 1;
        self.latest
            .store(self.revision, std::sync::atomic::Ordering::Relaxed);
        self.desired = None;
    }
    pub fn poll(&mut self) -> Option<Result<Update, String>> {
        let response = match self.receiver.try_recv() {
            Ok(response) => response,
            Err(TryRecvError::Empty) => return None,
            Err(TryRecvError::Disconnected) => {
                self.running = false;
                return self.desired.take().map(|_| {
                    Err("The geometry worker stopped; the committed model was preserved.".into())
                });
            }
        };
        self.running = false;
        if response.0 != self.revision {
            self.start();
            return None;
        }
        self.desired = None;
        Some(response.1.map(|mut update| {
            if self.finish {
                update.model.finish_edit();
            }
            update
        }))
    }
}
fn execute(request: Request, latest: &std::sync::atomic::AtomicU64) -> Result<Update, String> {
    let current = || {
        if latest.load(std::sync::atomic::Ordering::Relaxed) == request.revision {
            Ok(())
        } else {
            Err("Geometry request superseded by newer input.".to_string())
        }
    };
    current()?;
    let mut model = request.model;
    let mut selected = None;
    let mut preview = None;
    let mut loaded = None;
    for change in &request.changes {
        current()?;
        match change {
            Change::Edit(body, live) => {
                selected = Some(if *live {
                    model.edit((**body).clone())?
                } else {
                    model.commit((**body).clone())?
                });
            }
            Change::Delete(ids) => {
                if ids.len() == 1 {
                    model.delete(ids[0])?
                } else {
                    model.delete_many(ids)?
                }
            }
            Change::Transform(bodies) => model.commit_changes(bodies)?,
            Change::Preview(bodies) => preview = Some(bodies.clone()),
            Change::History(redo) => {
                if *redo {
                    model.redo()
                } else {
                    model.undo()
                }
            }
            Change::Open(path) => {
                let project = crate::project::Project::load(path)?;
                model = Model::from_bodies(project.bodies.clone());
                loaded = Some((path.clone(), project));
            }
            Change::EndEdit => model.finish_edit(),
        }
    }
    current()?;
    let scene = evaluate_scene(preview.as_deref().unwrap_or(&model.bodies))?;
    current()?;
    let display = std::sync::Arc::new(crate::viewport::SceneDisplay::new(&scene));
    Ok(Update {
        model,
        scene,
        display,
        change: request.changes.last().unwrap().clone(),
        selected,
        loaded,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Shape;
    #[test]
    fn superseded_requests_stop_before_validation_or_display_work() {
        let latest = std::sync::atomic::AtomicU64::new(2);
        let mut invalid = Body::new(Shape::Circle);
        invalid.width = f64::NAN;
        let stale = Request {
            revision: 1,
            model: Model::default(),
            changes: vec![Change::Edit(Box::new(invalid), true)],
        };
        assert!(
            execute(stale, &latest)
                .err()
                .unwrap()
                .contains("superseded")
        );
        let current = Request {
            revision: 2,
            model: Model::default(),
            changes: vec![Change::EndEdit],
        };
        assert!(execute(current, &latest).unwrap().scene.is_empty());
    }

    #[test]
    fn failed_operations_report_references_and_recover_without_committing_input() {
        use crate::edge_treatments::{ChamferDefinition, FaceTarget, Kind, Modifier};
        let mut source = Body::new(Shape::Rectangle);
        source.id = 41;
        source.width = 4.;
        source.depth = 4.;
        source.extrusion = 4.;
        let mut treatment = Modifier::new(Kind::Chamfer, FaceTarget::Top, &source);
        treatment.edge = Some(FaceTarget::Right);
        source.edge_treatments.push(treatment);
        let model = Model::from_bodies(vec![source.clone()]);
        let committed = evaluate_scene(&model.bodies).unwrap();
        let mut input = source.clone();
        input.edge_treatments[0].chamfer = ChamferDefinition::TwoDistances { second: 100. };
        let receive = |worker: &mut Evaluator| {
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
            loop {
                assert!(std::time::Instant::now() < deadline);
                if let Some(response) = worker.poll() {
                    break response;
                }
                std::thread::yield_now();
            }
        };
        let mut worker = Evaluator::default();
        worker.request(&model, Change::Edit(Box::new(input.clone()), true));
        assert!(worker.editing(41));
        assert!(!worker.editing(42));
        let reason = receive(&mut worker).err().unwrap();
        for expected in [
            "Rectangle 41",
            "Operation 1",
            "Chamfer",
            "Top",
            "Right edge",
        ] {
            assert!(reason.contains(expected), "{reason}");
        }
        assert!(!worker.pending());
        assert!(!worker.editing(41));
        assert_eq!(model.bodies, vec![source.clone()]);
        assert_ne!(input, model.bodies[0]);
        assert!(!model.can_undo());
        assert_eq!(
            evaluate_scene(&model.bodies).unwrap()[0].1.triangles,
            committed[0].1.triangles
        );
        input.edge_treatments[0].chamfer = ChamferDefinition::TwoDistances { second: 0.5 };
        worker.request(&model, Change::Edit(Box::new(input.clone()), true));
        let mut update = receive(&mut worker).unwrap();
        assert_eq!(update.model.bodies[0], input);
        update.model.undo();
        assert_eq!(update.model.bodies, model.bodies);

        let mut plain = source.clone();
        plain.edge_treatments.clear();
        let geometry = crate::model::evaluate(&plain).unwrap();
        let mut topology = geometry.topology.clone();
        topology.faces.pop();
        plain
            .component_edits
            .push(crate::components::ComponentEdit {
                retrim: false,
                section_transform: false,
                sweep: None,
                tangent_walls: false,
                prismatic_cut: false,
                face_edit: None,
                edge_chamfer: None,
                prism_chamfer: false,
                prism_bevel: None,
                prism_curves: false,
                evaluated: false,
                mode: crate::selection::SelectionMode::Face,
                element: 0,
                elements: vec![],
                topology,
                matrix: truck_modeling::Matrix4::from_scale(1.).into(),
            });
        let reason = crate::model::evaluate(&plain).err().unwrap();
        assert!(
            reason.contains("Rectangle 41")
                && reason.contains("Component operation 1")
                && reason.contains("Face 1"),
            "{reason}"
        );

        let mut base = source;
        base.edge_treatments.clear();
        let mut tool = base.clone();
        tool.id = 42;
        tool.position[0] = 6.;
        let scene = evaluate_scene(&[base.clone(), tool.clone()]).unwrap();
        let mut result = Body::new(Shape::Rectangle);
        result.id = 43;
        result.boolean = Some(crate::booleans::Boolean::new(
            crate::booleans::Kind::Subtract,
            &[(41, None), (42, None)],
            crate::selection::SelectionMode::Volume,
            &scene,
        ));
        let mut boolean_model = Model::from_bodies(vec![base.clone(), tool, result]);
        let before = boolean_model.bodies.clone();
        let reason = boolean_model.delete(42).unwrap_err();
        for expected in [
            "Subtract 43",
            "target body 41 volume",
            "tool body 42 volume",
        ] {
            assert!(reason.contains(expected), "{reason}");
        }
        assert_eq!(boolean_model.bodies, before);
        let mut profile = Body::new(Shape::Circle);
        profile.id = 45;
        profile.width = 1.;
        profile.position[2] = 4.;
        profile.bore = Some(crate::operators::Bore::new(&base, FaceTarget::Top).unwrap());
        profile.bore.as_mut().unwrap().mode = crate::operators::BoreMode::Borer;
        let mut bore_model = Model::from_bodies(vec![base, profile]);
        let before = bore_model.bodies.clone();
        let reason = bore_model.delete(41).unwrap_err();
        for expected in ["Circle 45", "Borer only", "target body 41 / Top"] {
            assert!(reason.contains(expected), "{reason}");
        }
        assert_eq!(bore_model.bodies, before);
    }
    #[test]
    fn coalesces_pending_edits_and_discards_cancelled_results() {
        let mut body = Body::new(Shape::Rectangle);
        body.id = 1;
        let model = Model::from_bodies(vec![body.clone()]);
        let mut worker = Evaluator::default();
        for width in [3., 4., 5.] {
            body.width = width;
            worker.request(&model, Change::Edit(Box::new(body.clone()), true));
        }
        worker.finish_edit();
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        let mut update = loop {
            assert!(std::time::Instant::now() < deadline);
            if let Some(result) = worker.poll() {
                break result.unwrap();
            }
            std::thread::yield_now();
        };
        assert_eq!(update.model.bodies[0].width, 5.);
        let query = crate::selection::Query::indexed(&update.scene, &update.display.index);
        let camera = crate::viewport::Camera::default();
        let rect = eframe::egui::Rect::from_min_size(
            eframe::egui::Pos2::ZERO,
            eframe::egui::vec2(800., 600.),
        );
        for (_, geometry) in &update.scene {
            for point in &geometry.points {
                let pixel = camera.project(*point, rect).0;
                assert_eq!(
                    query.surface_hit(&camera, pixel, rect),
                    crate::selection::Query::brute(&update.scene).surface_hit(&camera, pixel, rect)
                );
            }
        }
        update.model.undo();
        assert_eq!(update.model.bodies[0].width, 2.);
        assert!(!update.model.can_undo());
        worker.request(&model, Change::Edit(Box::new(body), true));
        worker.cancel();
        while worker.running() {
            assert!(worker.poll().is_none());
            std::thread::yield_now();
        }
        assert!(!worker.pending());
        assert_eq!(
            model.bodies[0].width, 2.,
            "UI model stays untouched until acceptance"
        );
        // A second object's edit must not discard the first object's pending edit.
        let mut second = Body::new(Shape::Rectangle);
        second.id = 2;
        second.position = [4., 0., 0.];
        let two = Model::from_bodies(vec![model.bodies[0].clone(), second.clone()]);
        let mut first = two.bodies[0].clone();
        first.width = 3.;
        second.width = 4.;
        worker.request(&two, Change::Edit(Box::new(first), true));
        worker.request(&two, Change::Edit(Box::new(second), true));
        let result = loop {
            if let Some(result) = worker.poll() {
                break result.unwrap();
            }
            std::thread::yield_now();
        };
        assert_eq!(
            result
                .model
                .bodies
                .iter()
                .map(|b| b.width)
                .collect::<Vec<_>>(),
            vec![3., 4.]
        );
        let mut body = model.bodies[0].clone();
        body.width = 3.;
        worker.request(&model, Change::Edit(Box::new(body.clone()), true));
        worker.finish_edit();
        body.width = 4.;
        worker.request(&model, Change::Edit(Box::new(body), true));
        worker.finish_edit();
        let mut result = loop {
            if let Some(result) = worker.poll() {
                break result.unwrap();
            }
            std::thread::yield_now();
        };
        result.model.undo();
        assert_eq!(result.model.bodies[0].width, 3.);
        result.model.undo();
        assert_eq!(result.model.bodies[0].width, 2.);
    }
}
