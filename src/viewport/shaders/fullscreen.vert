#version 450

// Shared fullscreen-triangle vertex stage for every post-processing pass: the SSS blur (H and
// V), the bloom bright-extract and blur (H and V), and the composite — six passes, one vertex
// shader. One oversized triangle covers the screen; vUv is the [0,1] screen UV.
//
// Pass:    every PostProcess pass (the bloom render pass and the swapchain composite).
// Inputs:  none (gl_VertexIndex into the fullscreen triangle).
// Outputs: vUv, the screen UV for the fragment stage.

#extension GL_GOOGLE_include_directive : enable
#include "fullscreen.glsl"

layout(location = 0) out vec2 vUv;

void main() {
    vec2 p = kTri[gl_VertexIndex];
    vUv = p * 0.5 + 0.5;
    gl_Position = vec4(p, 0.0, 1.0);
}
