use crate::selection::SelectionMode;
use crate::{
    model::{Body, evaluate_scene},
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
    #[serde(default)]
    pub drawing: crate::snapping::Preferences,
    pub selected: Option<u64>,
    #[serde(default)]
    pub selection_mode: SelectionMode,
    #[serde(default)]
    pub selected_element: Option<usize>,
    #[serde(default)]
    pub selection: Vec<crate::selection::Pick>,
}
impl Project {
    pub fn new(bodies: Vec<Body>, camera: Camera, snapping: bool, selected: Option<u64>) -> Self {
        Self {
            format: "OpenSketcher".into(),
            schema_version: 13,
            modeling_version: 13,
            bodies,
            camera,
            snapping,
            drawing: crate::snapping::Preferences::default(),
            selected,
            selection_mode: SelectionMode::Volume,
            selected_element: None,
            selection: Vec::new(),
        }
    }
    pub fn picks(&self) -> Vec<crate::selection::Pick> {
        if self.selection.is_empty() {
            self.selected
                .map(|id| vec![(id, self.selected_element)])
                .unwrap_or_default()
        } else {
            self.selection.clone()
        }
    }
    pub fn validate(&self) -> Result<(), String> {
        if self.format != "OpenSketcher" {
            return Err("This is not an OpenSketcher project.".into());
        }
        if !matches!(self.schema_version, 1..=13) || !matches!(self.modeling_version, 1..=13) {
            return Err(
                "Unsupported .osk version. This application supports schema versions 1–13 and modeling versions 1–13."
                    .into(),
            );
        }
        if (self.schema_version < 13 || self.modeling_version < 13)
            && self
                .bodies
                .iter()
                .any(|b| b.face_extrusions.iter().any(|e| e.distance < 0.))
        {
            return Err("Inward face extrusions require .osk version 13.".into());
        }
        if (self.schema_version < 12 || self.modeling_version < 12)
            && self
                .bodies
                .iter()
                .any(|b| b.component_edits.iter().any(|e| e.evaluated))
        {
            return Err("Evaluated component edits require .osk version 12.".into());
        }
        if (self.schema_version < 11 || self.modeling_version < 11)
            && self.bodies.iter().any(|b| !b.face_extrusions.is_empty())
        {
            return Err("Face extrusions require .osk version 11.".into());
        }
        if (self.schema_version < 7 || self.modeling_version < 7)
            && self.bodies.iter().any(|b| b.boolean.is_some())
        {
            return Err("Boolean features require .osk version 7.".into());
        }
        if (self.schema_version < 8 || self.modeling_version < 8)
            && self
                .bodies
                .iter()
                .flat_map(|b| &b.edge_treatments)
                .any(|m| m.chamfer != crate::edge_treatments::ChamferDefinition::Equal || m.reverse)
        {
            return Err("Asymmetric chamfers require .osk schema/modeling version 8.".into());
        }
        if (self.schema_version < 9 || self.modeling_version < 9)
            && self
                .bodies
                .iter()
                .flat_map(|b| &b.edge_treatments)
                .any(|m| m.edge.is_some())
        {
            return Err("Selected-edge treatments require .osk schema/modeling version 9.".into());
        }
        if (self.schema_version < 10 || self.modeling_version < 10)
            && self
                .bodies
                .iter()
                .filter_map(|b| b.bore.as_ref())
                .any(|b| b.extent != crate::operators::Extent::Distance)
        {
            return Err("Automatic sweep extents require .osk schema/modeling version 10.".into());
        }
        if self.bodies.len() > 1000 {
            return Err("Project exceeds the current 1,000-object limit.".into());
        }
        if (self.schema_version < 6 || self.modeling_version < 6)
            && self.bodies.iter().any(|b| b.bore.is_some())
        {
            return Err(
                "Attached profile sweeps and boring require .osk schema/modeling version 6.".into(),
            );
        }
        if (self.schema_version < 3 || self.modeling_version < 3)
            && self
                .bodies
                .iter()
                .any(|body| !body.component_edits.is_empty())
        {
            return Err("Component edits require .osk schema/modeling version 3.".into());
        }
        if (self.schema_version < 5 || self.modeling_version < 5)
            && (self.selection.len() > 1
                || self.bodies.iter().any(|b| {
                    !b.edge_treatments.is_empty()
                        || b.component_edits.iter().any(|e| !e.elements.is_empty())
                }))
        {
            return Err(
                "Face modifiers and multiple selection require .osk schema/modeling version 5."
                    .into(),
            );
        }
        self.camera.validate()?;
        let mut ids = HashSet::new();
        let mut triangles = 0;
        let scene = evaluate_scene(&self.bodies)?;
        for (body, (_, geometry)) in self.bodies.iter().zip(&scene) {
            if body.id == 0 || body.id == u64::MAX || !ids.insert(body.id) {
                return Err("Invalid or duplicate object ID.".into());
            }
            if self.selected == Some(body.id)
                && self.selection_mode != SelectionMode::Volume
                && self
                    .selected_element
                    .is_none_or(|element| element >= self.selection_mode.count(geometry))
            {
                return Err("Selected component does not exist.".into());
            }
            triangles += geometry.triangles.len();
            if triangles > 1_000_000 {
                return Err("Project exceeds the current geometry budget.".into());
            }
        }
        let mut picks = HashSet::new();
        if self.selection.len() > 32768
            || self.picks().iter().any(|p| {
                !picks.insert(*p) || !crate::selection::valid(*p, &scene, self.selection_mode)
            })
        {
            return Err("Invalid, duplicate, or excessive selection references.".into());
        }
        if !self.selection.is_empty()
            && self.selection.last().copied() != self.selected.map(|id| (id, self.selected_element))
        {
            return Err("Primary selection must be the last selected item.".into());
        }
        if self.selected.is_some_and(|id| !ids.contains(&id)) {
            return Err("Selected object reference does not exist.".into());
        }
        if (self.selected.is_none() || self.selection_mode == SelectionMode::Volume)
            && self.selected_element.is_some()
        {
            return Err("Invalid component selection.".into());
        }
        if self.schema_version == 1
            && (self.selection_mode != SelectionMode::Volume || self.selected_element.is_some())
        {
            return Err("Schema 1 cannot contain component selection.".into());
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
        let mut project: Self =
            serde_json::from_slice(&bytes).map_err(|e| format!("Invalid .osk document: {e}"))?;
        if project.modeling_version == 1 && project.schema_version <= 2 {
            // Partitioning changes topology slots; preserve the owner, discard stale slots.
            project.selection_mode = SelectionMode::Volume;
            project.selected_element = None;
        }
        if (1..=3).contains(&project.modeling_version) {
            // Modeling v1 had independent profiles; v2/v3 automatically partitioned.
            // Preserve the source evaluator's geometry rather than imposing a new default.
            let cuts = matches!(project.modeling_version, 2 | 3);
            for body in &mut project.bodies {
                body.cuts_profiles = cuts;
            }
        }
        project.validate()?;
        project.schema_version = 13;
        project.modeling_version = 13;
        Ok(project)
    }
    pub fn save(&self, path: &Path) -> Result<(), String> {
        self.validate()?;
        let bytes = serde_json::to_vec_pretty(self)
            .map_err(|e| format!("Cannot serialize project: {e}"))?;
        if bytes.len() as u64 > MAX_BYTES {
            return Err("Project exceeds the 16 MiB file limit.".into());
        }
        write_atomic(path, &bytes)
    }
}

/// Replace only after the complete output has been written and synced.
pub fn write_atomic(path: &Path, bytes: &[u8]) -> Result<(), String> {
    // Same-directory temporary files permit atomic replacement on the destination filesystem.
    let parent = path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    let mut temporary = tempfile::NamedTempFile::new_in(parent)
        .map_err(|e| format!("Cannot create temporary file: {e}"))?;
    temporary
        .write_all(bytes)
        .and_then(|()| temporary.as_file().sync_all())
        .map_err(|e| format!("Cannot write file: {e}"))?;
    temporary
        .persist(path)
        .map_err(|e| format!("Cannot replace file; previous file preserved: {}", e.error))?;
    Ok(())
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
        for mode in [
            SelectionMode::Face,
            SelectionMode::Edge,
            SelectionMode::Point,
        ] {
            let mut component = project.clone();
            component.selection_mode = mode;
            component.selected_element = Some(0);
            component.save(&path).unwrap();
            assert_eq!(Project::load(&path).unwrap(), component);
            component.selected_element = Some(usize::MAX);
            assert!(component.validate().is_err());
        }
        let mut legacy = serde_json::to_value(&project).unwrap();
        legacy["schema_version"] = 1.into();
        legacy["modeling_version"] = 1.into();
        legacy.as_object_mut().unwrap().remove("drawing");
        for body in legacy["bodies"].as_array_mut().unwrap() {
            body.as_object_mut().unwrap().remove("cuts_profiles");
        }
        legacy.as_object_mut().unwrap().remove("selection_mode");
        legacy.as_object_mut().unwrap().remove("selected_element");
        std::fs::write(&path, serde_json::to_vec(&legacy).unwrap()).unwrap();
        assert_eq!(Project::load(&path).unwrap(), project);
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
