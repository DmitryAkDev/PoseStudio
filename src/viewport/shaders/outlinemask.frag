#version 450

// Selection-outline mask: every fragment of the selected object writes full coverage into the
// single-channel mask (see OutlineMask). The MSAA resolve turns the silhouette's partially
// covered edge pixels into fractional values, which the composite (composite.frag) dilates by
// the outline width and blends against — so the outline's inner edge matches the object's own
// anti-aliased fringe. Skinned + camera-projected by shadow.vert.

layout(location = 0) out float outCoverage;

void main() {
    outCoverage = 1.0;
}
