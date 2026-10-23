//! Coplanar closed profiles partition older profiles before Truck builds their faces.
use crate::model::{Body, Shape};
use truck_modeling::{Deg, Matrix3, Matrix4, Point3, SquareMatrix, Transform, Vector3};
type P = [f64; 2];

pub fn transform(b: &Body) -> Matrix4 {
    Matrix4::from_translation(Vector3::from(b.position))
        * Matrix4::from_angle_z(Deg(b.rotation[2]))
        * Matrix4::from_angle_y(Deg(b.rotation[1]))
        * Matrix4::from_angle_x(Deg(b.rotation[0]))
        * Matrix4::from_nonuniform_scale(b.scale[0], b.scale[1], b.scale[2])
}
pub fn rotation_angles(rotation: Matrix3) -> [f64; 3] {
    let y = (-rotation.x.z).clamp(-1., 1.).asin();
    let (x, z) = if y.cos().abs() > 1e-8 {
        (
            rotation.y.z.atan2(rotation.z.z),
            rotation.x.y.atan2(rotation.x.x),
        )
    } else {
        (0., (-rotation.y.x).atan2(rotation.y.y))
    };
    [x.to_degrees(), y.to_degrees(), z.to_degrees()]
}
fn profile(b: &Body) -> Vec<P> {
    let r = b.width / 2.;
    if b.shape == Shape::Rectangle {
        return vec![
            [-r, -b.depth / 2.],
            [r, -b.depth / 2.],
            [r, b.depth / 2.],
            [-r, b.depth / 2.],
        ];
    }
    let count =
        ((std::f64::consts::PI / (1. - (0.005 / r).min(1.)).acos()).ceil() as usize).clamp(16, 512);
    let angle = if b.shape == Shape::Semicircle {
        std::f64::consts::PI
    } else {
        std::f64::consts::TAU
    };
    let n = if b.shape == Shape::Semicircle {
        count / 2
    } else {
        count
    };
    (0..if b.shape == Shape::Semicircle {
        n + 1
    } else {
        n
    })
        .map(|i| {
            let a = angle * i as f64 / n as f64;
            [r * a.cos(), r * a.sin()]
        })
        .collect()
}
fn sub(a: P, b: P) -> P {
    [a[0] - b[0], a[1] - b[1]]
}
fn cross(a: P, b: P) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn dot(a: P, b: P) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn lerp(a: P, b: P, t: f64) -> P {
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
}
fn near(a: P, b: P) -> bool {
    dot(sub(a, b), sub(a, b)) < 1e-14
}
fn inside(p: P, polygon: &[P]) -> bool {
    (0..polygon.len()).all(|i| {
        cross(
            sub(polygon[(i + 1) % polygon.len()], polygon[i]),
            sub(p, polygon[i]),
        ) >= -1e-10
    })
}
pub fn area(p: &[P]) -> f64 {
    (0..p.len())
        .map(|i| cross(p[i], p[(i + 1) % p.len()]))
        .sum::<f64>()
        / 2.
}

pub fn boundaries(body: &Body, bodies: &[Body]) -> Result<Option<Vec<Vec<P>>>, String> {
    if body.shape == Shape::Line {
        return Ok(None);
    }
    let outer = profile(body);
    let inverse = transform(body)
        .invert()
        .ok_or("Invalid profile transform.")?;
    let mut polygons = vec![outer];
    for other in bodies
        .iter()
        .filter(|b| b.id > body.id && b.shape != Shape::Line && b.cuts_profiles)
    {
        let relative = inverse * transform(other);
        let points: Vec<_> = profile(other)
            .into_iter()
            .map(|p| relative.transform_point(Point3::new(p[0], p[1], 0.)))
            .collect();
        if points.iter().any(|p| p.z.abs() > 1e-7) {
            continue;
        }
        let mut polygon: Vec<_> = points.into_iter().map(|p| [p.x, p.y]).collect();
        if area(&polygon) < 0. {
            polygon.reverse();
        }
        // Ignore profiles whose bounding boxes do not overlap with positive area.
        if (0..2).any(|i| {
            let bounds = |p: &Vec<P>| {
                (
                    p.iter().map(|p| p[i]).fold(f64::INFINITY, f64::min),
                    p.iter().map(|p| p[i]).fold(f64::NEG_INFINITY, f64::max),
                )
            };
            let (a, b) = bounds(&polygons[0]);
            let (c, d) = bounds(&polygon);
            b.min(d) - a.max(c) <= 1e-7
        }) {
            continue;
        }
        if polygons[0].iter().all(|p| inside(*p, &polygon)) {
            return Ok(Some(Vec::new()));
        }
        polygons.push(polygon);
    }
    if polygons.len() == 1 {
        return Ok(None);
    }
    let contains = |p| inside(p, &polygons[0]) && !polygons[1..].iter().any(|poly| inside(p, poly));
    let segments: Vec<_> = polygons
        .iter()
        .flat_map(|p| (0..p.len()).map(|i| [p[i], p[(i + 1) % p.len()]]))
        .collect();
    if segments.len() > 8192 {
        return Err("Split profile exceeds the 8,192-segment evaluation budget.".into());
    }
    let mut edges: Vec<[P; 2]> = Vec::new();
    for &[a, b] in &segments {
        let ab = sub(b, a);
        let length = dot(ab, ab).sqrt();
        if length < 1e-8 {
            continue;
        }
        let mut splits = vec![0., 1.];
        for &[c, d] in &segments {
            let cd = sub(d, c);
            let den = cross(ab, cd);
            if den.abs() > 1e-12 {
                let t = cross(sub(c, a), cd) / den;
                let u = cross(sub(c, a), ab) / den;
                if (-1e-9..=1. + 1e-9).contains(&t) && (-1e-9..=1. + 1e-9).contains(&u) {
                    splits.push(t.clamp(0., 1.));
                }
            } else if cross(sub(c, a), ab).abs() < 1e-9 {
                for p in [c, d] {
                    let t = dot(sub(p, a), ab) / dot(ab, ab);
                    if (0.0..=1.).contains(&t) {
                        splits.push(t);
                    }
                }
            }
        }
        splits.sort_by(f64::total_cmp);
        splits.dedup_by(|a, b| (*a - *b).abs() < 1e-10);
        for pair in splits.windows(2) {
            let p = lerp(a, b, pair[0]);
            let q = lerp(a, b, pair[1]);
            if near(p, q) {
                continue;
            }
            let m = lerp(p, q, 0.5);
            let offset = [-ab[1] / length * 1e-6, ab[0] / length * 1e-6];
            let left = contains([m[0] + offset[0], m[1] + offset[1]]);
            let right = contains(sub(m, offset));
            if left == right {
                continue;
            }
            let edge = if left { [p, q] } else { [q, p] };
            if !edges
                .iter()
                .any(|e| near(e[0], edge[0]) && near(e[1], edge[1]))
            {
                edges.push(edge);
            }
        }
    }
    let mut loops = Vec::new();
    while let Some([a, b]) = edges.pop() {
        let mut polygon = vec![a];
        let mut end = b;
        while !near(end, a) {
            polygon.push(end);
            let Some(index) = edges.iter().position(|e| near(e[0], end)) else {
                return Err("Could not close a split profile boundary.".into());
            };
            end = edges.swap_remove(index)[1];
        }
        if polygon.len() >= 3 && area(&polygon).abs() > 1e-9 {
            loops.push(polygon);
        }
    }
    Ok(Some(loops))
}

pub fn contains(p: P, polygon: &[P]) -> bool {
    // General winding test for assigning clockwise hole loops to concave outer loops.
    let mut result = false;
    for i in 0..polygon.len() {
        let a = polygon[i];
        let b = polygon[(i + 1) % polygon.len()];
        if (a[1] > p[1]) != (b[1] > p[1])
            && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
        {
            result = !result;
        }
    }
    result
}
