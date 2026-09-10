/**
 * @file scene.cpp
 * @brief Scene construction/teardown, the environment upload, model management + selection, the
 *        per-frame record sequence, and the active-figure logic. The posing forwarders live in
 *        sceneposing.cpp. See scene.h.
 */

#include "scene.h"

#include "bonepicker.h"
#include "camera.h"
#include "cameraubo.h"
#include "iblmaps.h"
#include "lineoverlay.h"
#include "model.h"
#include "modeldata.h"
#include "outlinepass.h"
#include "scenepipelines.h"
#include "shadowpass.h"
#include "vulkancontext.h"
#include "vulkanpipeline.h"

#include <glm/glm.hpp>

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <limits>
#include <vector>

namespace pose {

Scene::Scene(VulkanContext& context, VkRenderPass renderPass, VkRenderPass outlineMaskPass,
             const std::vector<char>& vertSpirv, const std::vector<char>& fragSpirv,
             const std::vector<char>& skeletonVertSpirv, const std::vector<char>& skeletonFragSpirv,
             const std::vector<char>& shadowVertSpirv, const std::vector<char>& shadowFragSpirv,
             const std::vector<char>& backgroundVertSpirv,
             const std::vector<char>& backgroundFragSpirv,
             const std::vector<char>& outlineMaskFragSpirv)
    : m_context(context) {
    // The shared plumbing first: every pass below builds against its layouts.
    m_pipelines = std::make_unique<ScenePipelines>(m_context, renderPass, vertSpirv, fragSpirv,
                                                   backgroundVertSpirv, backgroundFragSpirv);

    // The shadow pass (map + pipeline) needs the pose set layout; the scene-wide set 3 needs the
    // map — so the pass is built right after the layouts and its map written into set 3 next,
    // all before the first frame can sample it.
    m_shadowPass = std::make_unique<ShadowPass>(m_context, m_pipelines->poseSetLayout(),
                                                shadowVertSpirv, shadowFragSpirv);
    m_pipelines->writeShadowMap(m_shadowPass->shadowMap());

    // The selection-outline mask pass (shadow.vert again, projected by the camera) and the line
    // overlay (skeleton, pin markers, the orthographic floor line).
    m_outlinePass = std::make_unique<OutlinePass>(m_context, outlineMaskPass,
                                                  m_pipelines->poseSetLayout(), shadowVertSpirv,
                                                  outlineMaskFragSpirv);
    m_lineOverlay = std::make_unique<LineOverlay>(m_context, renderPass,
                                                  m_pipelines->cameraSetLayout(),
                                                  skeletonVertSpirv, skeletonFragSpirv);

    // Bake the default procedural studio environment synchronously so descriptor set 3 is valid before
    // the first frame. A real HDRI is baked off the render thread (VulkanWindow) and swapped in when
    // ready via applyBakedEnvironment().
    applyBakedEnvironment(bakeEnvironment(generateStudioEnvironment()));
}

Scene::~Scene() {
    // Nothing in flight may reference the pipeline/descriptors when we tear them down. The
    // models go first (their sets come from their own pools but were written against the
    // shared layouts); every other member is an RAII owner released in reverse declaration
    // order.
    vkDeviceWaitIdle(m_context.device());
    m_models.clear();
}

void Scene::applyBakedEnvironment(const BakedEnvironment& baked) {
    // The mesh pipeline samples the specular cubemap + LUT this is about to replace; ensure no in-flight
    // frame is still reading the old IblMaps before it's destroyed. (The expensive CPU bake — SH +
    // prefiltered specular — already happened in bakeEnvironment(), off the render thread for switches.)
    vkDeviceWaitIdle(m_context.device());
    m_environmentSH = baked.sh;
    // The BRDF LUT is environment-independent (roughness + NdotV only), so integrate it once and reuse
    // — re-running the ~8M-sample integration on every HDRI swap would be pure waste.
    if (m_brdfLut.data.empty()) {
        m_brdfLut = integrateBrdfLut();
    }
    m_iblMaps = std::make_unique<IblMaps>(m_context, baked.specular, m_brdfLut);
    m_pipelines->writeEnvironment(*m_iblMaps);
}

VkDescriptorSetLayout Scene::iblSetLayout() const { return m_pipelines->iblSetLayout(); }

VkDescriptorSet Scene::iblSet() const { return m_pipelines->iblSet(); }

const glm::mat4& Scene::lightViewProj() const { return m_shadowPass->lightViewProj(); }

void Scene::setShowSkeleton(bool on) { m_lineOverlay->setShowSkeleton(on); }

bool Scene::showSkeleton() const { return m_lineOverlay->showSkeleton(); }

void Scene::addModel(const ModelData& data) {
    m_models.push_back(std::make_unique<Model>(m_context, data, m_pipelines->materialSetLayout(),
                                               m_pipelines->poseSetLayout(),
                                               m_pipelines->fallbackDiffuse(),
                                               m_pipelines->fallbackNormal()));
    setSelectedModel(static_cast<int>(m_models.size()) - 1); // the new arrival is the selection
}

int Scene::selectedModelIndex() const {
    return (m_selectedModel >= 0 && static_cast<std::size_t>(m_selectedModel) < m_models.size())
               ? m_selectedModel
               : -1;
}

void Scene::setSelectedModel(int index) {
    if (index < 0 || static_cast<std::size_t>(index) >= m_models.size()) {
        index = -1;
    }
    const int previous = selectedModelIndex();
    if (index == previous) {
        return;
    }
    // The outgoing selection's joint selection goes away — a joint
    // selection on a model that isn't the selection would contradict the outline.
    if (previous >= 0 && m_models[static_cast<std::size_t>(previous)]->hasSkeleton()) {
        m_models[static_cast<std::size_t>(previous)]->setSelectedBone(-1);
    }
    m_selectedModel = index;
    if (index >= 0 && m_models[static_cast<std::size_t>(index)]->hasSkeleton()) {
        m_activeFigure = index; // the selected figure is the posing target
    }
}

int Scene::pickModel(const Ray& ray) const {
    int best = -1;
    float bestT = std::numeric_limits<float>::max();
    for (std::size_t i = 0; i < m_models.size(); ++i) {
        float t = 0.0f;
        if (m_models[i]->intersectRay(ray, t) && t < bestT) {
            bestT = t;
            best = static_cast<int>(i);
        }
    }
    return best;
}

bool Scene::framingBounds(glm::vec3& outMin, glm::vec3& outMax) const {
    const int selected = selectedModelIndex();
    if (selected >= 0) {
        return m_models[static_cast<std::size_t>(selected)]->worldBounds(outMin, outMax);
    }
    bool any = false;
    outMin = glm::vec3(std::numeric_limits<float>::max());
    outMax = glm::vec3(std::numeric_limits<float>::lowest());
    for (const std::unique_ptr<Model>& model : m_models) {
        glm::vec3 a;
        glm::vec3 b;
        if (model->worldBounds(a, b)) {
            outMin = glm::min(outMin, a);
            outMax = glm::max(outMax, b);
            any = true;
        }
    }
    return any;
}

void Scene::removeModel(std::size_t index) {
    if (index < m_models.size()) {
        m_models.erase(m_models.begin() + static_cast<std::ptrdiff_t>(index));
        // Keep the active figure pointing at the same model (indices above shift down); the
        // deleted figure itself falls back to the first remaining one. The selection likewise
        // follows its model, and a deleted selection leaves nothing selected.
        if (m_activeFigure == static_cast<int>(index)) {
            m_activeFigure = -1;
        } else if (m_activeFigure > static_cast<int>(index)) {
            --m_activeFigure;
        }
        if (m_selectedModel == static_cast<int>(index)) {
            m_selectedModel = -1;
        } else if (m_selectedModel > static_cast<int>(index)) {
            --m_selectedModel;
        }
    }
}

void Scene::recordShadowPass(VkCommandBuffer cmd, uint32_t frameIndex) {
    // This frame's key direction, shared with record()'s UBO fill (computed once per frame here,
    // since this pass always runs first).
    m_keyLightDir = keyLightDirection(m_lighting, isPbr());
    if (m_shadowPass->fit(m_models, m_keyLightDir, m_lighting.shadowsEnabled)) {
        m_shadowPass->record(cmd, m_models, frameIndex);
    }
}

bool Scene::recordOutlinePass(VkCommandBuffer cmd, const Camera& camera, uint32_t frameIndex,
                              const OutlineMask& mask) {
    const int selected = selectedModelIndex();
    if (selected < 0) {
        return false; // nothing selected: skip the pass; the composite draws no outline
    }
    return m_outlinePass->record(cmd, camera, frameIndex, mask,
                                 *m_models[static_cast<std::size_t>(selected)]);
}

void Scene::record(VkCommandBuffer cmd, const Camera& camera, uint32_t frameIndex) {
    const VulkanPipeline* wire = m_pipelines->wire();

    // The mode's row, with one substitution: on a device without fillModeNonSolid there is no
    // wire pipeline, and a row whose drawing IS the wires (Wireframe, Hidden Line) would leave
    // the figure invisible — draw its surface as Clay instead so the model stays on screen.
    ShadeMode spec = shadeModeSpec();
    if (spec.wireframe && wire == nullptr) {
        spec.wireframe = false;
        if (spec.fill != FillKind::Shaded) {
            spec.fill = FillKind::Shaded;
            spec.fragMode = kFragModeClay;
        }
    }

    // Update this frame's camera + lighting UBO (see fillCameraUbo for what feeds which mode).
    CameraUbo ubo{};
    fillCameraUbo(ubo, camera, m_lighting, spec, m_environmentSH, m_shadowPass->lightViewProj(),
                  m_keyLightDir);
    std::memcpy(m_pipelines->cameraUboData(frameIndex), &ubo, sizeof(ubo));
    const VkDescriptorSet cameraSet = m_pipelines->cameraSet(frameIndex);
    const VkDescriptorSet iblSet = m_pipelines->iblSet();

    // HDRI backdrop first (PBR mode only — the stylized modes keep the flat viewport clear): the
    // environment that lights the figure, visible behind it. Depth test/write are off in its
    // pipeline, so everything after simply draws over it. Runs even with no models — an empty
    // PBR viewport still shows the environment. Backdrop mode 0 ("Off", an Environment-panel
    // dial) skips it, leaving the flat viewport grey.
    if (isPbr() && m_lighting.backdropMode != 0) {
        const VulkanPipeline& background = m_pipelines->background();
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, background.handle());
        const VkDescriptorSet bgSets[2] = {cameraSet, iblSet};
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, background.layout(), 0, 2,
                                bgSets, 0, nullptr);
        vkCmdDraw(cmd, 3, 1, 0, 0);
    }

    if (!m_models.empty()) {
        const VulkanPipeline& opaque = m_pipelines->opaque();
        // Every mesh pipeline shares one layout: the camera set (0) and the scene-global IBL/shadow
        // set (3) are bound once here and stay bound through the surface and wire passes below;
        // set 1 (material) is bound per mesh and set 2 (pose) per model.
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, opaque.layout(), 0, 1,
                                &cameraSet, 0, nullptr);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, opaque.layout(), 3, 1,
                                &iblSet, 0, nullptr);

        // --- The surface: the lit passes, a hidden-line depth fill, or nothing (see ShadeMode). ---
        if (spec.fill == FillKind::Shaded) {
            // Opaque pass first.
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, opaque.handle());
            for (const std::unique_ptr<Model>& model : m_models) {
                model->recordOpaque(cmd, opaque.layout(), frameIndex);
            }
            // Transparent pass: alpha-blended, depth-write off, drawn after all opaque geometry so
            // it blends over what's behind it (e.g. the eye's moisture/cornea over the iris).
            // EVERY model's transparent meshes are sorted back-to-front TOGETHER, by their posed
            // centroids (Model::posedCentroid) — a per-model sort in import order drew a
            // translucent object imported first before a figure's eye behind it, and a bind-pose
            // key mis-ordered the shells of a figure lying on its back.
            const glm::vec3 camPos = camera.position();
            m_transparentScratch.clear();
            for (const std::unique_ptr<Model>& model : m_models) {
                for (const Mesh* mesh : model->transparentMeshes()) {
                    const glm::vec3 d = model->posedCentroid(*mesh) - camPos;
                    m_transparentScratch.push_back({glm::dot(d, d), model.get(), mesh});
                }
            }
            std::sort(m_transparentScratch.begin(), m_transparentScratch.end(),
                      [](const TransparentDraw& a, const TransparentDraw& b) {
                          return a.distSq > b.distSq; // farthest first
                      });
            const VulkanPipeline& transparent = m_pipelines->transparent();
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, transparent.handle());
            const VkPipelineLayout layout = transparent.layout();
            Model* bound = nullptr; // whose pose set (2) is currently bound
            for (const TransparentDraw& draw : m_transparentScratch) {
                if (draw.model != bound) {
                    draw.model->bindPose(cmd, layout, 2, frameIndex);
                    bound = draw.model;
                }
                draw.mesh->record(cmd, layout, draw.model->transform(), MeshDrawKind::Transparent,
                                  draw.model->selectedBone(), draw.model->selectedHighlightTwin());
            }
        } else if (spec.fill == FillKind::HiddenLine) {
            // Depth only (colour writes masked): the surface hides what's behind it and shows the
            // viewport's clear colour; the wire pass then draws only the edges facing the camera.
            const VulkanPipeline& hiddenLine = m_pipelines->hiddenLine();
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, hiddenLine.handle());
            for (const std::unique_ptr<Model>& model : m_models) {
                model->recordDepthFill(cmd, hiddenLine.layout(), frameIndex);
            }
        }

        // --- Wireframe: every mesh's triangle edges, over the surface or on their own. ---
        if (spec.wireframe && wire != nullptr) {
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, wire->handle());
            for (const std::unique_ptr<Model>& model : m_models) {
                model->recordWire(cmd, wire->layout(), frameIndex);
            }
        }
    }

    // --- Line overlays (skeleton, pins, the orthographic floor line), models or not. ---
    // The skeleton follows the ACTIVE figure (the posing target), not figureModel(): Show
    // Skeleton is a view toggle, so selecting a plain model beside a figure must not blank the
    // figure's bones — only the posing utilities go quiet in that state (see figureModel()).
    const int overlayFigure = activeFigureIndex();
    m_lineOverlay->record(cmd, camera, cameraSet, m_models,
                          overlayFigure >= 0 ? m_models[static_cast<std::size_t>(overlayFigure)].get()
                                             : nullptr,
                          frameIndex);
}

const Model* Scene::figureModel() const {
    const int active = activeFigureIndex();
    if (active < 0) {
        return nullptr;
    }
    const int selected = selectedModelIndex();
    if (selected >= 0 && selected != active) {
        return nullptr; // a different model is the selection: nothing to pose
    }
    return m_models[static_cast<std::size_t>(active)].get();
}

Model* Scene::figureModel() {
    return const_cast<Model*>(static_cast<const Scene*>(this)->figureModel());
}

int Scene::activeFigureIndex() const {
    if (m_activeFigure >= 0 && static_cast<std::size_t>(m_activeFigure) < m_models.size() &&
        m_models[static_cast<std::size_t>(m_activeFigure)]->hasSkeleton()) {
        return m_activeFigure;
    }
    for (std::size_t i = 0; i < m_models.size(); ++i) {
        if (m_models[i]->hasSkeleton()) {
            return static_cast<int>(i); // the first figure until one is clicked
        }
    }
    return -1;
}

void Scene::setActiveFigure(int index) {
    if (index >= 0 && static_cast<std::size_t>(index) < m_models.size() &&
        m_models[static_cast<std::size_t>(index)]->hasSkeleton()) {
        m_activeFigure = index;
        setSelectedModel(index); // the posing target is the selection (and the outline)
    }
}

bool Scene::hasPosableFigure() const { return figureModel() != nullptr; }

bool Scene::hasSelectedBone() const {
    const Model* fig = figureModel();
    return fig != nullptr && fig->selectedBone() >= 0;
}

int Scene::selectBoneByName(const std::string& name) {
    Model* fig = figureModel();
    return fig ? fig->selectBoneByName(name) : -1;
}

int Scene::selectBoneAt(float px, float py, float vpW, float vpH, const Camera& camera) {
    // Every figure's joints compete (pickBone): the nearest of ANY figure wins, and its figure
    // becomes the active one (the posing target). Without this a second figure in the scene could
    // never be posed — every call went to the first skeleton. On a miss return -1 (so the caller
    // orbits / click-selects) but keep the current selection — the figure can be orbited while a
    // joint is selected.
    const BonePick pick = pickBone(m_models, px, py, vpW, vpH, camera);
    if (pick.bone < 0) {
        return -1;
    }
    if (pick.model != activeFigureIndex()) {
        // Switching figures: the previous one's selection goes away.
        if (Model* previous = figureModel()) {
            previous->setSelectedBone(-1);
        }
    }
    setActiveFigure(pick.model);
    m_models[static_cast<std::size_t>(pick.model)]->setSelectedBone(pick.bone);
    return pick.bone;
}

} // namespace pose
