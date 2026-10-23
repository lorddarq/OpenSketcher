struct Camera {
    right: vec4f,
    up: vec4f,
    back: vec4f,
    target_span: vec4f,
    aspect: vec4f,
}
@group(0) @binding(0) var<uniform> camera: Camera;
struct Output { @builtin(position) position: vec4f, @location(0) color: vec3f }
@vertex fn vertex(@location(0) position: vec3f, @location(1) color: vec3f) -> Output {
    let p = position - camera.target_span.xyz;
    let scale = 2.0 / camera.target_span.w;
    var out: Output;
    out.position = vec4f(dot(p, camera.right.xyz) * scale / camera.aspect.x,
                        dot(p, camera.up.xyz) * scale,
                        0.5 - dot(p, camera.back.xyz) / (100.0 * camera.target_span.w), 1.0);
    out.color = color;
    return out;
}
@fragment fn fragment(in: Output) -> @location(0) vec4f { return vec4f(in.color, 1.0); }
