/**
 * @file scenematerials.h
 * @brief Applies a preset document's materials onto a figure's assembled zone meshes.
 *
 * materialparser turns one `scene.materials` list into per-zone MaterialRefs with UNRESOLVED
 * texture URIs; this layer does the rest — resolving each zone's URIs to on-disk paths through
 * the content roots and writing the result into FigureMesh::material — and it knows the SECOND
 * form a materials preset comes in. Ordinary presets carry `scene.materials`
 * (applySceneMaterials); hierarchical-material presets carry their values as single-key
 * `scene.animations` tracks instead (applyAnimationMaterials), and a stylized character's follower
 * addons only turn their lens/iris shells transparent through that form. Both appliers no-op when
 * their section is absent, so a non-material document passes through harmlessly and the addon
 * loop can hand every preset to both. Cross-file material bases ("/data/…#<SkinShaderBase>") are
 * pre-resolved here for the parser, which only knows same-file "#id" references.
 * Pure std + GLM + nlohmann (+ UriResolver) — no Qt, no Vulkan.
 */

#ifndef SCENEMATERIALS_H
#define SCENEMATERIALS_H

#include <nlohmann/json_fwd.hpp>

#include <string>

namespace pose {

struct FigureData;
class UriResolver;

/// Applies @p root's `scene.materials` onto @p fig: each zone the scene names gets its resolved
/// colours/maps/opacity, other zones keep what they have. @p referringDir resolves relative
/// texture references (the directory of the document @p root came from). Shared by the main
/// figure path (the preset's own materials) and the addon path (a follower's materials preset).
void applySceneMaterials(FigureData& fig, const nlohmann::json& root, UriResolver& resolver,
                         const std::string& referringDir);

/// Applies a hierarchical-materials preset (asset_info type "preset_hierarchical_material") onto
/// @p fig. Its values ride in single-key `scene.animations` tracks, each url addressing one
/// material property of one zone; only the properties the renderer consumes are read. Zones are
/// matched by name alone (the node prefix is ignored). No-op without a `scene.animations` array.
void applyAnimationMaterials(FigureData& fig, const nlohmann::json& root, UriResolver& resolver,
                             const std::string& referringDir);

} // namespace pose

#endif // SCENEMATERIALS_H
