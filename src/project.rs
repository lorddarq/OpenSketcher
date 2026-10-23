use crate::{
    model::{Body, evaluate},
    viewport::Camera,
};
use serde::{Deserialize, Serialize};
use std::{
    collections::HashSet,
    fs::File,
    io::{Read, Write},
    path::Path,
};

const MAX_BYTES: u64 = 16 * 1024 * 1024;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Project {
    format: String,
    schema_version: u32,
    modeling_version: u32,
    pub bodies: Vec<Body>,
    pub camera: Camera,
    pub snapping: bool,
    pub selected: Option<u64>,
}
impl Project {
    pub fn new(bodies: Vec<Body>, camera: Camera, snapping: bool, selected: Option<u64>) -> Self {
        Self {
            format: "OpenSketcher".into(),
            schema_version: 1,
            modeling_version: 1,
            bodies,
            camera,
            snapping,
            selected,
        }
    }
    pub fn validate(&self) -> Result<(), String> {
        if self.format != "OpenSketcher" {
            return Err("This is not an OpenSketcher project.".into());
        }
        if self.schema_version != 1 || self.modeling_version != 1 {
            return Err(
                "Unsupported .osk version. This application supports schema/modeling version 1."
                    .into(),
            );
        }
        if self.bodies.len() > 1000 {
            return Err("Project exceeds the current 1,000-object limit.".into());
        }
        self.camera.validate()?;
        let mut ids = HashSet::new();
        let mut triangles = 0;
        for body in &self.bodies {
            if body.id == 0 || body.id == u64::MAX || !ids.insert(body.id) {
                return Err("Invalid or duplicate object ID.".into());
            }
            let geometry = evaluate(body).map_err(|e| format!("Object {}: {e}", body.id))?;
            triangles += geometry.triangles.len();
            if triangles > 1_000_000 {
                return Err("Project exceeds the current geometry budget.".into());
            }
        }
        if self.selected.is_some_and(|id| !ids.contains(&id)) {
            return Err("Selected object reference does not exist.".into());
        }
        Ok(())
    }
    pub fn load(path: &Path) -> Result<Self, String> {
        let file = File::open(path).map_err(|e| format!("Cannot open project: {e}"))?;
        let mut bytes = Vec::new();
        file.take(MAX_BYTES + 1)
            .read_to_end(&mut bytes)
            .map_err(|e| format!("Cannot read project: {e}"))?;
        if bytes.len() as u64 > MAX_BYTES {
            return Err("Project exceeds the 16 MiB file limit.".into());
        }
        let project: Self =
            serde_json::from_slice(&bytes).map_err(|e| format!("Invalid .osk document: {e}"))?;
        project.validate()?;
        Ok(project)
    }
    pub fn save(&self, path: &Path) -> Result<(), String> {
        self.validate()?;
        let bytes = serde_json::to_vec_pretty(self)
            .map_err(|e| format!("Cannot serialize project: {e}"))?;
        if bytes.len() as u64 > MAX_BYTES {
            return Err("Project exceeds the 16 MiB file limit.".into());
        }
        // Same-directory temporary files permit atomic replacement on the destination filesystem.
        let parent = path
            .parent()
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or_else(|| Path::new("."));
        let mut temporary = tempfile::NamedTempFile::new_in(parent)
            .map_err(|e| format!("Cannot create temporary project: {e}"))?;
        temporary
            .write_all(&bytes)
            .and_then(|()| temporary.as_file().sync_all())
            .map_err(|e| format!("Cannot write project: {e}"))?;
        temporary.persist(path).map_err(|e| {
            format!(
                "Cannot replace project; previous file preserved: {}",
                e.error
            )
        })?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Model, Shape};
    #[test]
    fn editable_scene_round_trip_and_safe_failures() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("scene.osk");
        let mut model = Model::default();
        for shape in [
            Shape::Rectangle,
            Shape::Circle,
            Shape::Line,
            Shape::Semicircle,
        ] {
            let mut body = Body::new(shape);
            body.width = 2.123456789012345;
            body.extrusion = 2.;
            body.position = [1., 2., 3.];
            body.rotation = [10., 20., 30.];
            body.scale = [1., 2., 1.];
            model.commit(body).unwrap();
        }
        let project = Project::new(model.bodies, Camera::default(), false, Some(2));
        project.save(&path).unwrap();
        let restored = Project::load(&path).unwrap();
        assert_eq!(restored, project);
        let mut model = Model::from_bodies(restored.bodies);
        let mut edit = model.bodies[0].clone();
        edit.width = 5.;
        model.edit(edit).unwrap();
        assert_eq!(model.bodies[0].width, 5.);
        assert!(model.commit(Body::new(Shape::Line)).unwrap() > 4);
        // Replacing a valid project works, and invalid saves cannot overwrite it.
        project.save(&path).unwrap();
        let previous = std::fs::read(&path).unwrap();
        let mut invalid = project.clone();
        invalid.bodies[0].width = f64::NAN;
        assert!(invalid.save(&path).is_err());
        assert_eq!(std::fs::read(&path).unwrap(), previous);
        assert!(project.save(dir.path()).is_err());
        assert_eq!(std::fs::read(&path).unwrap(), previous);
        for change in ["version", "duplicate", "reference", "camera"] {
            let mut p = project.clone();
            match change {
                "version" => p.schema_version = 99,
                "duplicate" => p.bodies[1].id = p.bodies[0].id,
                "reference" => p.selected = Some(99),
                _ => p.camera.span = 0.,
            }
            std::fs::write(&path, serde_json::to_vec(&p).unwrap()).unwrap();
            assert!(Project::load(&path).is_err());
        }
        std::fs::write(&path, b"truncated {").unwrap();
        assert!(Project::load(&path).is_err());
    }
}
