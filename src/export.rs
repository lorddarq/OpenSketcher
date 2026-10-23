//! Interchange copies of evaluated geometry. Editable history stays in .osk.
use crate::model::{Body, Geometry, Shape, evaluate_scene};
use serde_json::json;
use std::{fmt::Write, path::Path};
use truck_modeling::{InnerSpace, Solid};

#[derive(Clone, Copy, PartialEq)]
pub enum Format {
    Stl,
    Glb,
    Step,
    Dxf,
}
impl Format {
    pub const ALL: [Self; 4] = [Self::Stl, Self::Glb, Self::Step, Self::Dxf];
    pub fn extension(self) -> &'static str {
        match self {
            Self::Stl => "stl",
            Self::Glb => "glb",
            Self::Step => "step",
            Self::Dxf => "dxf",
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Stl => "STL · printable solids",
            Self::Glb => "GLB · Blender / 3D tools",
            Self::Step => "STEP · faceted CAD",
            Self::Dxf => "DXF · CAD faces and lines",
        }
    }
}
pub fn save(bodies: &[Body], format: Format, path: &Path) -> Result<(), String> {
    let bytes = encode(bodies, format)?;
    crate::project::write_atomic(path, &bytes)
}
fn encode(bodies: &[Body], format: Format) -> Result<Vec<u8>, String> {
    let mut scene = evaluate_scene(bodies)?;
    // Reuse component editing's boundary reconciliation before writing a connected mesh.
    for (_, geometry) in &mut scene {
        crate::components::weld_geometry(geometry);
    }
    if !scene
        .iter()
        .any(|(_, g)| !g.triangles.is_empty() || !g.edges.is_empty())
    {
        return Err("There is no geometry to export.".into());
    }
    match format {
        Format::Stl => {
            if bodies
                .iter()
                .any(|b| b.extrusion == 0. || b.shape == Shape::Line)
            {
                return Err("STL requires closed solids. Extrude profiles and remove standalone lines or sheets first; GLB and DXF support them.".into());
            }
            for (_, g) in &scene {
                crate::components::boundary_shell(g, true)?;
            }
            let count: usize = scene.iter().map(|(_, g)| g.triangles.len()).sum();
            let mut out = vec![0; 80];
            let title = b"OpenSketcher; coordinates in millimeters";
            out[..title.len()].copy_from_slice(title);
            out.extend_from_slice(
                &u32::try_from(count)
                    .map_err(|_| "Too many STL triangles")?
                    .to_le_bytes(),
            );
            for (_, g) in &scene {
                for t in &g.triangles {
                    let n = (t[1] - t[0]).cross(t[2] - t[0]).normalize();
                    for v in [n.x, n.y, n.z]
                        .into_iter()
                        .chain(t.iter().flat_map(|p| [p.x, p.y, p.z]))
                    {
                        out.extend_from_slice(&(v as f32).to_le_bytes());
                    }
                    out.extend_from_slice(&0u16.to_le_bytes());
                }
            }
            Ok(out)
        }
        Format::Glb => glb(&scene),
        Format::Dxf => {
            let mut s = String::from(
                "0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1015\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n",
            );
            for (id, g) in &scene {
                for t in &g.triangles {
                    writeln!(s, "0\n3DFACE\n8\nBody_{id}").unwrap();
                    for (i, p) in [t[0], t[1], t[2], t[2]].iter().enumerate() {
                        writeln!(
                            s,
                            "{}\n{}\n{}\n{}\n{}\n{}",
                            10 + i,
                            p.x,
                            20 + i,
                            p.y,
                            30 + i,
                            p.z
                        )
                        .unwrap();
                    }
                }
                for e in &g.edges {
                    writeln!(s, "0\nLINE\n8\nBody_{id}").unwrap();
                    for (i, p) in e.iter().enumerate() {
                        writeln!(
                            s,
                            "{}\n{}\n{}\n{}\n{}\n{}",
                            10 + i,
                            p.x,
                            20 + i,
                            p.y,
                            30 + i,
                            p.z
                        )
                        .unwrap();
                    }
                }
            }
            s.push_str("0\nENDSEC\n0\nEOF\n");
            Ok(s.into_bytes())
        }
        Format::Step => {
            if scene
                .iter()
                .any(|(_, g)| g.triangles.is_empty() && !g.edges.is_empty())
            {
                return Err("STEP currently exports surfaces and solids; use DXF or GLB for standalone lines.".into());
            }
            let mut solids = Vec::new();
            let mut shells = Vec::new();
            for (b, (_, g)) in bodies.iter().zip(&scene) {
                if g.triangles.is_empty() {
                    continue;
                }
                let closed = b.extrusion > 0. && b.shape != Shape::Line;
                let shell = crate::components::boundary_shell(g, closed)?;
                if closed {
                    for part in shell.connected_components() {
                        solids.push(
                            Solid::try_new(vec![part])
                                .map_err(|e| e.to_string())?
                                .compress(),
                        );
                    }
                } else {
                    shells.push(shell.compress());
                }
            }
            let mut models = truck_stepio::out::StepModels::default();
            for solid in &solids {
                models.push_solid(solid);
            }
            for shell in &shells {
                models.push_shell(shell);
            }
            Ok(truck_stepio::out::CompleteStepDisplay::new(
                models,
                truck_stepio::out::StepHeaderDescriptor {
                    organization_system: "OpenSketcher".into(),
                    ..Default::default()
                },
            )
            .to_string()
            .into_bytes())
        }
    }
}
fn glb(scene: &[(u64, Geometry)]) -> Result<Vec<u8>, String> {
    let mut bin = Vec::new();
    let mut views = Vec::new();
    let mut accessors = Vec::new();
    let mut meshes = Vec::new();
    let mut nodes = Vec::new();
    for (id, g) in scene {
        let mut primitives = Vec::new();
        for (mode, points) in [
            (4, g.triangles.iter().flatten().copied().collect::<Vec<_>>()),
            (1, g.edges.iter().flatten().copied().collect()),
        ] {
            if points.is_empty() {
                continue;
            }
            let offset = bin.len();
            let mut min = [f32::INFINITY; 3];
            let mut max = [f32::NEG_INFINITY; 3];
            for p in &points {
                // Z-up scene millimeters -> glTF Y-up meters (right handed).
                for (i, v) in [p.x, p.z, -p.y]
                    .map(|v| (v * 0.001) as f32)
                    .into_iter()
                    .enumerate()
                {
                    min[i] = min[i].min(v);
                    max[i] = max[i].max(v);
                    bin.extend_from_slice(&v.to_le_bytes());
                }
            }
            let accessor = accessors.len();
            let view = views.len();
            views.push(json!({"buffer":0,"byteOffset":offset,"byteLength":bin.len()-offset,"target":34962}));
            accessors.push(json!({"bufferView":view,"componentType":5126,"count":points.len(),"type":"VEC3","min":min,"max":max}));
            primitives.push(json!({"attributes":{"POSITION":accessor},"mode":mode,"material":0}));
        }
        if !primitives.is_empty() {
            nodes.push(json!({"mesh":meshes.len(),"name":format!("Body {id}")}));
            meshes.push(json!({"primitives":primitives}));
        }
    }
    let doc = json!({"asset":{"version":"2.0","generator":"OpenSketcher"},"scene":0,"scenes":[{"nodes":(0..nodes.len()).collect::<Vec<_>>()}],"nodes":nodes,"meshes":meshes,"materials":[{"doubleSided":true,"pbrMetallicRoughness":{"baseColorFactor":[0.65,0.7,0.75,1.0],"metallicFactor":0.0,"roughnessFactor":0.8}}],"buffers":[{"byteLength":bin.len()}],"bufferViews":views,"accessors":accessors});
    let mut json = serde_json::to_vec(&doc).map_err(|e| e.to_string())?;
    while json.len() % 4 != 0 {
        json.push(b' ');
    }
    let length =
        u32::try_from(12 + 8 + json.len() + 8 + bin.len()).map_err(|_| "GLB exceeds 4 GiB")?;
    let mut out = Vec::new();
    for v in [0x46546c67, 2, length, json.len() as u32, 0x4e4f534a] {
        out.extend_from_slice(&v.to_le_bytes());
    }
    out.extend(json);
    out.extend_from_slice(&(bin.len() as u32).to_le_bytes());
    out.extend_from_slice(&0x004e4942u32.to_le_bytes());
    out.extend(bin);
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn exports_evaluated_solids_and_rejects_lossy_print_inputs() {
        let mut b = Body::new(Shape::Rectangle);
        b.id = 1;
        b.extrusion = 3.;
        let scene = [b.clone()];
        let geometry = evaluate_scene(&scene).unwrap();
        let stl = encode(&scene, Format::Stl).unwrap();
        let count = u32::from_le_bytes(stl[80..84].try_into().unwrap()) as usize;
        assert_eq!(count, geometry[0].1.triangles.len());
        assert_eq!(stl.len(), 84 + 50 * count);
        let glb = encode(&scene, Format::Glb).unwrap();
        assert_eq!(&glb[..4], b"glTF");
        assert_eq!(
            u32::from_le_bytes(glb[8..12].try_into().unwrap()) as usize,
            glb.len()
        );
        let n = u32::from_le_bytes(glb[12..16].try_into().unwrap()) as usize;
        let doc: serde_json::Value = serde_json::from_slice(&glb[20..20 + n]).unwrap();
        assert_eq!(doc["meshes"].as_array().unwrap().len(), 1);
        assert_eq!(doc["accessors"][0]["max"][1], json!(0.003f32));
        let step = String::from_utf8(encode(&scene, Format::Step).unwrap()).unwrap();
        assert!(step.starts_with("ISO-10303-21;"));
        assert!(step.contains("MANIFOLD_SOLID_BREP"));
        assert!(step.contains(".MILLI.,.METRE."));
        let dxf = String::from_utf8(encode(&scene, Format::Dxf).unwrap()).unwrap();
        assert_eq!(dxf.matches("\n3DFACE\n").count(), count);
        assert!(dxf.ends_with("0\nEOF\n"));
        b.extrusion = 0.;
        assert!(encode(&[b.clone()], Format::Stl).is_err());
        assert!(encode(&[b], Format::Step).is_ok());
        let line = Body::new(Shape::Line);
        assert!(encode(std::slice::from_ref(&line), Format::Step).is_err());
        assert!(encode(&[line], Format::Glb).is_ok());
        assert!(encode(&[], Format::Glb).is_err());
        for shape in [Shape::Circle, Shape::Semicircle] {
            let mut curved = Body::new(shape);
            curved.id = 1;
            curved.extrusion = 2.;
            encode(std::slice::from_ref(&curved), Format::Stl).unwrap();
            encode(&[curved], Format::Step).unwrap();
        }
        let mut inner = Body::new(Shape::Rectangle);
        inner.id = 2;
        inner.width = 1.;
        inner.depth = 1.;
        inner.cuts_profiles = true;
        let mut cut_scene = scene.to_vec();
        cut_scene.push(inner);
        let split = String::from_utf8(encode(&cut_scene, Format::Step).unwrap()).unwrap();
        let mut ids = std::collections::HashSet::new();
        for row in split.lines().filter(|r| r.starts_with('#')) {
            let (id, _) = row.split_once('=').unwrap();
            assert!(ids.insert(id.trim().to_owned()));
        }
        for reference in split.split('#').skip(1) {
            let number: String = reference.chars().take_while(char::is_ascii_digit).collect();
            assert!(
                ids.contains(&format!("#{number}")),
                "dangling STEP reference {number}"
            );
        }
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("solid.stl");
        save(&scene, Format::Stl, &path).unwrap();
        assert_eq!(std::fs::read(&path).unwrap(), stl);
        assert!(save(&[Body::new(Shape::Line)], Format::Stl, &path).is_err());
        assert_eq!(
            std::fs::read(&path).unwrap(),
            stl,
            "failed export must preserve the previous destination"
        );
    }
}
