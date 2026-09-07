#version 450

// Position-only vertex shader for the projected passes that need no materials: the key light's
// shadow map (paired with shadow.frag, transformed by the light's ortho view-projection) and
// the selection-outline mask (paired with outlinemask.frag, transformed by the CAMERA's
// view-projection). Same DUAL-QUATERNION skinning as mesh.vert (so a posed figure casts its
// posed shadow and outlines its posed silhouette — the blends MUST match or the figure
// self-shadows against a differently-deformed caster). In both pipelines the joint storage
// buffer is bound at SET 0 (the pipeline's only set — same set object the main pass binds at
// set 2; the layouts match). Normal/UV attributes exist in the vertex layout but are simply not
// consumed.

layout(push_constant) uniform Push {
    mat4 viewProj; // the pass's projection: the fitted light matrix (shadow) or the camera's (outline)
    mat4 model;
} pc;

struct DualQuat {
    vec4 real; // rotation quaternion, stored (x, y, z, w)
    vec4 dual; // 0.5 * (0, translation) * real
};
layout(std430, set = 0, binding = 0) readonly buffer Joints {
    DualQuat dq[];
} joints;

vec3 quatRotate(vec4 r, vec3 v) {
    return v + 2.0 * cross(r.xyz, cross(r.xyz, v) + r.w * v);
}

layout(location = 0) in vec3 inPos;
layout(location = 3) in uvec4 inJoints;
layout(location = 4) in vec4 inWeights;

void main() {
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
    vec3 skinnedPos = quatRotate(rAcc, inPos) +
                      2.0 * (rAcc.w * dAcc.xyz - dAcc.w * rAcc.xyz + cross(rAcc.xyz, dAcc.xyz));
    gl_Position = pc.viewProj * pc.model * vec4(skinnedPos, 1.0);
}
