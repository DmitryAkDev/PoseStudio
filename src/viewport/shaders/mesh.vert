#version 450

// Lit mesh vertex shader with DUAL-QUATERNION skinning. Each vertex blends its (up to 4) joints'
// unit dual quaternions (set 2 storage buffer; real = rotation (xyz,w), dual = 0.5·(0,t)·real),
// normalizes, and transforms by the result, then by the per-draw model matrix and the per-frame
// camera view-projection. DQS, not linear matrix blending, because the figure format authors its
// skin weights AND its pose correctives against dual quaternions (skin_settings.general_map_mode
// = DualQuat): LBS collapses deep bends (a 155° knee folded into a shapeless blob) and the JCMs
// — sculpted as corrections on top of DQS — made the collapse worse instead of fixing it.
// Static meshes bind a single identity joint and default to weight (1,0,0,0), so this same path
// leaves them untransformed. Passes a world-space normal, UV, and world position (the last feeds
// the specular view vector) to the fragment.

// Shared set-0 camera/lighting UBO. The fragment stage declares the full block (view + lighting rig +
// SH + params — see mesh.frag / scene.cpp's CameraUbo); the vertex stage only needs the view-projection,
// so it declares just that first member (std140 offset 0, so no other fields need to be spelled out).
layout(set = 0, binding = 0) uniform CameraUbo {
    mat4 viewProj;
} cam;

layout(push_constant) uniform Push {
    mat4 model;
    vec4 baseColor; // rgb tint, a = opacity
    vec4 material;  // x = roughness, y = normalMode
} pc;

// Per-model skinning dual quaternions (from the rigid poseGlobal * inverseBind per joint).
struct DualQuat {
    vec4 real; // rotation quaternion, stored (x, y, z, w)
    vec4 dual; // 0.5 * (0, translation) * real
};
layout(std430, set = 2, binding = 0) readonly buffer Joints {
    DualQuat dq[];
} joints;

// Rotates v by unit quaternion r ((xyz, w) layout).
vec3 quatRotate(vec4 r, vec3 v) {
    return v + 2.0 * cross(r.xyz, cross(r.xyz, v) + r.w * v);
}

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUv;
layout(location = 3) in uvec4 inJoints;
layout(location = 4) in vec4 inWeights;
layout(location = 5) in float inAo; // baked per-vertex ambient occlusion (1 = open)
layout(location = 6) in vec4 inTangent; // UV tangent + handedness (w = 0 -> none baked)

layout(location = 0) out vec3 vWorldNormal;
layout(location = 1) out vec2 vUv;
layout(location = 2) out vec3 vWorldPos;
layout(location = 3) out float vAo;
layout(location = 4) out vec4 vTangent; // world-space tangent, w = handedness (0 = absent)

void main() {
    // Blend the influencing joints' dual quaternions by weight, sign-aligning each against the
    // first joint's hemisphere (q and -q encode the same rotation; blending across the seam
    // cancels instead of averaging). Weights sum to 1, so at bind pose (identity DQs) the
    // normalized blend is exactly the identity — the mesh renders as its rest geometry.
    vec4 r0 = joints.dq[inJoints.x].real;
    vec4 rAcc = inWeights.x * r0;
    vec4 dAcc = inWeights.x * joints.dq[inJoints.x].dual;
    float s1 = (dot(joints.dq[inJoints.y].real, r0) < 0.0) ? -1.0 : 1.0;
    rAcc += (inWeights.y * s1) * joints.dq[inJoints.y].real;
    dAcc += (inWeights.y * s1) * joints.dq[inJoints.y].dual;
    float s2 = (dot(joints.dq[inJoints.z].real, r0) < 0.0) ? -1.0 : 1.0;
    rAcc += (inWeights.z * s2) * joints.dq[inJoints.z].real;
    dAcc += (inWeights.z * s2) * joints.dq[inJoints.z].dual;
    float s3 = (dot(joints.dq[inJoints.w].real, r0) < 0.0) ? -1.0 : 1.0;
    rAcc += (inWeights.w * s3) * joints.dq[inJoints.w].real;
    dAcc += (inWeights.w * s3) * joints.dq[inJoints.w].dual;
    float invLen = 1.0 / max(length(rAcc), 1e-6);
    rAcc *= invLen;
    dAcc *= invLen;

    // Rigid transform of the normalized blend: rotate by the real part, translate by
    // 2 * (dual * conjugate(real)).vector.
    vec3 skinnedPos = quatRotate(rAcc, inPos) +
                      2.0 * (rAcc.w * dAcc.xyz - dAcc.w * rAcc.xyz + cross(rAcc.xyz, dAcc.xyz));
    vec4 worldPos = pc.model * vec4(skinnedPos, 1.0);
    // mat3(model) is correct for the rigid/uniform-scale model transforms in play; swap to a
    // proper normal matrix if non-uniform scale ever appears.
    vWorldNormal = mat3(pc.model) * quatRotate(rAcc, inNormal);
    vUv = inUv;
    vWorldPos = worldPos.xyz;
    vAo = inAo;
    vTangent = vec4(mat3(pc.model) * quatRotate(rAcc, inTangent.xyz), inTangent.w);
    gl_Position = cam.viewProj * worldPos;
}
