/**
 * @file figureimporter.cpp
 * @brief Implementation of FigureImporter — the figure import pipeline, step by step. See
 *        figureimporter.h.
 *
 * This file is the readable spine of the figure importer: loadFigureFile() runs the stages in
 * order and every stage lives in its own module — geometryparser (the cage), morphresolver +
 * morphparser (the character's shape), nodeparser + bonefixups (the skeleton), skinparser (the
 * weights), correctivescan (the pose correctives), uvparser + subdivision (the render mesh),
 * scenematerials (the materials), followeraddons (the followers). Only the dialed-morph bake and
 * the per-addon loop stay here: the bake is the one place the cage is mutated in place, and the
 * addon loop recurses into loadFigureFile itself. One UriResolver serves the whole import,
 * addons included (same roots; the document cache is what keeps a shared base file from being
 * inflated once per addon).
 */

#include "figureimporter.h"

#include "bonefixups.h"
#include "correctivescan.h"
#include "figuredocument.h"
#include "figureutils.h"
#include "followeraddons.h"
#include "geometryparser.h"
#include "morphparser.h"
#include "morphresolver.h"
#include "nodeparser.h"
#include "scenematerials.h"
#include "skinparser.h"
#include "subdivision.h"
#include "uriresolver.h"
#include "uvparser.h"

#include <nlohmann/json.hpp>

#include <exception>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace pose {

namespace {

// Applies the preset's dialed shape morphs to the base vertices — this is what turns the generic
// base figure into the specific character. A character rarely dials the shape morphs directly; it
// dials a *control* whose driver formulas drive the real head/body morphs, so resolveDialedMorphs()
// walks that graph and returns each reached morph file + weight. We then bake its sparse deltas
// (position += weight · delta). Controls/scales resolve to no deltas and no-op; a morph that
// fails to load is skipped rather than aborting the whole import. Returns the dialed morphs (the
// corrective scan needs both their folders and their weights) and fills the figure scale and the
// joint-centre offsets the same formulas drive.
std::vector<DialedMorph> bakeDialedMorphs(const nlohmann::json& presetRoot, UriResolver& resolver,
                                          const std::string& presetDir, GeometryData& geo,
                                          float& figureScale, JointCenterOffsets& jointCenterOffsets) {
    const std::vector<DialedMorph> dialedMorphs =
        resolveDialedMorphs(presetRoot, resolver, presetDir, figureScale, jointCenterOffsets);
    // Load every reached morph document up front, across cores — a character preset can reach
    // hundreds of morph files, and their serial read+inflate+parse was a large slice of import
    // time. The bake loop below then hits the resolver's cache.
    {
        std::vector<std::string> morphUris;
        morphUris.reserve(dialedMorphs.size());
        for (const DialedMorph& dialed : dialedMorphs) {
            morphUris.push_back(dialed.url);
        }
        resolver.prefetchDocuments(morphUris, presetDir);
    }
    for (const DialedMorph& dialed : dialedMorphs) {
        const ResolvedUri ru = resolver.resolve(dialed.url, presetDir);
        if (!ru.resolved()) {
            continue;
        }
        std::shared_ptr<const FigureDocument> morphDoc;
        try {
            morphDoc = resolver.loadDocument(ru);
        } catch (const std::exception&) {
            continue;
        }
        for (const auto& [vertexIndex, delta] : parseMorphDeltas(morphDoc->root(), ru.fragment)) {
            if (vertexIndex < geo.positions.size()) {
                geo.positions[vertexIndex] += dialed.weight * delta;
            }
        }
    }
    return dialedMorphs;
}

// The full loader. `includeAddons` is true for the user-facing entry point and false for the
// follower-addon recursion (an addon must not pull in further addons — one level is what the
// format's addon loader does, and it also bounds the recursion). The same flag also means "this is
// the top-level figure": pose correctives are only discovered and carried for it, since a merged
// follower's correctives are dropped anyway (their delta indices mean nothing in the merged
// base-vertex space). @p resolver is shared with the recursion — same roots, one document cache.
FigureData loadFigureFile(const std::string& path, UriResolver& resolver, bool includeAddons) {
    const std::string presetDir = directoryOf(path);

    FigureDocument presetDoc = FigureDocument::loadFromFile(path);
    const nlohmann::json& root = presetDoc.root();

    // Locate the base geometry. Either this file *is* a base .dsf (has geometry_library), or it's a
    // preset that references one via a scene node's geometries[0].url.
    const nlohmann::json* geomEntry = nullptr;
    std::shared_ptr<const FigureDocument> baseDoc; // keeps the referenced base file alive
    std::string baseDir = presetDir;
    std::string baseFile = path; // the base figure .dsf itself (sibling-base corrective matching)

    if (root.contains("geometry_library")) {
        geomEntry = findGeometryEntry(root, std::string());
    } else {
        const std::string geomUri = findGeometryUri(root);
        if (geomUri.empty()) {
            throw std::runtime_error("figure import: no geometry reference found in " + path);
        }
        const ResolvedUri ru = resolver.resolve(geomUri, presetDir);
        if (!ru.resolved()) {
            throw std::runtime_error("figure import: cannot resolve base geometry '" + geomUri + "'");
        }
        baseDoc = resolver.loadDocument(ru);
        baseDir = directoryOf(ru.path);
        baseFile = ru.path;
        geomEntry = findGeometryEntry(baseDoc->root(), ru.fragment);
    }
    if (!geomEntry) {
        throw std::runtime_error("figure import: base geometry library is empty");
    }

    GeometryData geo = parseGeometry(*geomEntry);

    // Pre-morph copy of the base cage: follower addons are authored against THIS shape, so their
    // shape-follow projection (followParentShape) needs base positions + the morph deltas.
    std::vector<glm::vec3> baseCagePositions;
    if (includeAddons) {
        baseCagePositions = geo.positions;
    }

    // The character's shape: bake the dialed morphs into the cage.
    float figureScale = 1.0f;
    JointCenterOffsets jointCenterOffsets; // per-bone rest-origin shifts the same morphs drive
    const std::vector<DialedMorph> dialedMorphs =
        bakeDialedMorphs(root, resolver, presetDir, geo, figureScale, jointCenterOffsets);

    // UV set (referenced by the geometry, resolved relative to the base file).
    UvSet uv;
    if (!geo.defaultUvSetUri.empty()) {
        const ResolvedUri uru = resolver.resolve(geo.defaultUvSetUri, baseDir);
        if (uru.resolved()) {
            std::shared_ptr<const FigureDocument> uvDoc = resolver.loadDocument(uru);
            uv = parseUvSet(uvDoc->root(), uru.fragment);
        }
    }

    FigureData out;
    out.figureScale = figureScale;
    out.graftPopulated = geo.graftPopulated;

    // Skeleton + skin weights (from the base file's node_library + its SkinBinding modifier), parsed
    // *before* assembly so each de-indexed render vertex can carry its base vertex's joint weights.
    const nlohmann::json& baseRoot = baseDoc ? baseDoc->root() : root;
    if (const auto nodeLib = baseRoot.find("node_library"); nodeLib != baseRoot.end()) {
        out.bones = parseSkeleton(*nodeLib);
    }

    // Shift each joint's rest origin by the morph-driven center_point adjustments, so the skeleton
    // matches the character's morphed proportions (a base-figure rig around a morphed mesh would pivot
    // limbs at the wrong places, and the joint overlay would float off the body).
    applyJointCenterOffsets(out.bones, jointCenterOffsets);

    if (!out.bones.empty()) {
        // Twist bones sit mid-limb where there's no real joint; keep their axial twist but forbid the
        // mid-bone bend the figure would otherwise allow. Done after the origins (incl. morph-driven
        // offsets) are final, so each bone's length direction reflects the character's proportions.
        lockTwistBoneBendAxes(out.bones);

        std::unordered_map<std::string, int> boneNameToIndex;
        for (int i = 0; i < static_cast<int>(out.bones.size()); ++i) {
            boneNameToIndex.emplace(out.bones[i].name, i);
        }
        if (const auto modLib = baseRoot.find("modifier_library"); modLib != baseRoot.end()) {
            if (const nlohmann::json* skin = findSkinBinding(*modLib)) {
                // The binding's own vertex_count wins over the cage size; if it is smaller, the
                // tail cage vertices get no entry and subdivision pads them as zero-weight
                // (unskinned) vertices rather than failing.
                const int vc = skin->value("vertex_count", static_cast<int>(geo.positions.size()));
                out.vertexSkins = parseSkinWeights(*skin, vc, boneNameToIndex);
            }
        }

        // Pose correctives: auto-applied morphs driven by these joints' rotations. Discovered
        // structurally (see correctivescan), so they work across figure generations that name
        // their corrective files differently. Only meaningful once we have a skeleton to drive them,
        // and only for the top-level figure — a follower's are dropped at the merge.
        if (includeAddons) {
            std::unordered_set<std::string> boneNames;
            boneNames.reserve(out.bones.size());
            for (const FigureBone& b : out.bones) {
                boneNames.insert(b.name);
            }
            out.correctives = discoverCorrectives(resolver, baseDir, boneNames, dialedMorphs,
                                                  baseFile, geo.positions.size());
        }
    }

    // Catmull-Clark subdivision: smooth the low-resolution base cage into the render mesh. Positions
    // follow the smooth rules while UVs (per corner, so seams stay put) and skin weights refine
    // linearly, then the result is assembled exactly like the un-subdivided path — so skinning and pose
    // correctives run unchanged, just on more vertices. Because Catmull-Clark is a *linear* operator,
    // each corrective's sparse base-cage deltas are carried onto the subdivided cage by the SAME
    // stencils (carryCorrectivesToSubdivided), so the GPU's live corrective blend needs no change.
    // Level 1 (4x faces) is a good editing-time smoothness; raise for finer at a vertex-count cost.
    constexpr int kSubdivisionLevels = 1;
    SubdivisionResult sub = subdivideFigure(geo, uv, out.vertexSkins, kSubdivisionLevels);
    out.meshes = std::move(sub.meshes);
    out.baseVertexCount = sub.vertexCount;
    if (!out.correctives.empty()) {
        carryCorrectivesToSubdivided(sub, geo.positions.size(), out.correctives);
    }

    // Materials come from the preset's scene (a base .dsf on its own carries none -> zones keep the
    // default base color). Texture URIs are resolved to on-disk paths inside.
    applySceneMaterials(out, root, resolver, presetDir);

    // Fitted follower addons (anime eyeballs, lashes, …) the preset declares in its post-load block.
    // Characters built this way deliberately hide the standard surfaces the follower replaces (e.g.
    // every stock eye surface dialed to cutout 0), so WITHOUT this step such a figure imports with
    // empty eye sockets. Each addon is a self-describing wearable figure: load it through this same
    // pipeline, apply its materials preset, and merge it into the figure. Failures skip just that
    // addon — a missing follower must not sink the character import.
    if (includeAddons) {
        const AddonProjection projection =
            buildAddonProjection(geo, baseCagePositions, out.meshes);

        for (const AddonRef& addonRef : findPostLoadAddons(root)) {
            try {
                const ResolvedUri location = resolver.resolve(addonRef.assetUri, presetDir);
                if (!location.resolved()) {
                    continue;
                }
                FigureData addon = loadFigureFile(location.path, resolver, false);
                for (const std::string& presetUri : addonRef.presetUris) {
                    try {
                        const ResolvedUri presetLoc = resolver.resolve(presetUri, presetDir);
                        const std::shared_ptr<const FigureDocument> addonPresetDoc =
                            resolver.loadDocument(presetLoc);
                        // Relative (non-root-relative) references inside the preset resolve against
                        // the preset document's OWN directory — not the character preset's — or a
                        // relative image_file path would silently miss and drop the map.
                        const std::string addonPresetDir = directoryOf(presetLoc.path);
                        // A materials preset comes in either form; each applier no-ops when its
                        // section is absent (the anime-eye presets use the animation-track form),
                        // so a non-material preset document just passes through harmlessly.
                        applySceneMaterials(addon, addonPresetDoc->root(), resolver, addonPresetDir);
                        applyAnimationMaterials(addon, addonPresetDoc->root(), resolver, addonPresetDir);
                    } catch (const std::exception&) {
                        // Unresolvable preset: the follower keeps its own materials.
                    }
                }
                // Shape-follow only the followers authored against the BASE figure. A populated
                // graft marks a vendor-fitted in-place replacement (e.g. an anime character's
                // custom eyeballs) — projecting the morph onto those double-shifts them off the
                // face, verified both ways on real content.
                if (projection.anyDelta && !addon.graftPopulated) {
                    followParentShape(addon, baseCagePositions, projection.cageDeltas,
                                      projection.eligibleCageVerts);
                }
                mergeAddonFigure(out, std::move(addon));
            } catch (const std::exception&) {
                continue;
            }
        }
    }

    return out;
}

} // namespace

FigureData FigureImporter::load(const std::string& path,
                                const std::vector<std::string>& contentRoots) const {
    UriResolver resolver(contentRoots);
    return loadFigureFile(path, resolver, /*includeAddons=*/true);
}

} // namespace pose
