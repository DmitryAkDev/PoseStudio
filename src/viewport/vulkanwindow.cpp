/**
 * @file vulkanwindow.cpp
 * @brief Implementation of the Vulkan-hosting QWindow. See vulkanwindow.h.
 */

#include "vulkanwindow.h"

#include "environmentsource.h"
#include "figureimportservice.h"
#include "librarypaths.h"
#include "modelimportservice.h"
#include "rendering/vulkancommon.h"
#include "rendering/vulkancontext.h"
#include "rendering/vulkanrenderer.h"

#include <QAction>
#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QFutureWatcher>
#include <QMenu>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QTimer>
#include <QVulkanInstance>
#include <QWheelEvent>
#include <QtConcurrent>
#include <QtGui/qevent.h> // QPlatformSurfaceEvent lives here (no separate qpa header in binary Qt)

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>

namespace pose {

namespace {
// Input tuning. Kept local for now; promote to Constants / a preference once the
// navigation-preferences panel grows a "viewport" section.
constexpr float kOrbitRadiansPerPixel = 0.008f;
constexpr float kPanPerPixel = 0.0015f;
constexpr float kDollyPerWheelStep = 0.12f; // per 120-unit wheel notch
} // namespace

VulkanWindow::VulkanWindow(QVulkanInstance* instance, uint32_t apiVersion, QString shaderDir,
                           QWindow* parent)
    : QWindow(parent), m_instance(instance), m_apiVersion(apiVersion),
      m_shaderDir(std::move(shaderDir)) {
    setSurfaceType(QSurface::VulkanSurface);
    setVulkanInstance(m_instance);

    // FBIK drag ticker: the IK solve is rate-limited per call, so while the drag button is held
    // the last target is re-issued at ~60 Hz — the pose keeps catching up (and settles) even when
    // the mouse stops moving and mouse-move events stop with it.
    m_ikTimer = new QTimer(this);
    m_ikTimer->setInterval(16);
    if (qEnvironmentVariableIsSet("POSESTUDIO_IK_PRECISE_TIMER")) {
        m_ikTimer->setTimerType(Qt::PreciseTimer); // diagnostics A/B against the coarse default
    }
    // The ground button's fall (see groundFigure): free fall from rest, evaluated against real
    // elapsed time so the landing takes the same 0.45s per metre whatever the frame cadence.
    m_fallTimer = new QTimer(this);
    m_fallTimer->setInterval(16);
    connect(m_fallTimer, &QTimer::timeout, this, [this]() {
        if (!m_renderer || m_fallHeight <= 0.0f) {
            m_fallTimer->stop();
            return;
        }
        constexpr float kGravity = 9.81f; // m/s², world units are metres
        const float t = static_cast<float>(m_fallClock.nsecsElapsed()) * 1e-9f;
        const float dropped = std::min(m_fallHeight, 0.5f * kGravity * t * t);
        const float dy = dropped - m_fallDropped;
        if (dy > 0.0f) {
            m_renderer->translateFigureY(-dy);
            m_fallDropped = dropped;
            requestUpdate();
        }
        if (dropped >= m_fallHeight) {
            m_fallTimer->stop(); // landed
            m_fallHeight = 0.0f;
        }
    });
    m_benchSpec = qEnvironmentVariable("POSESTUDIO_IK_BENCH");
    m_ikPerf = qEnvironmentVariableIsSet("POSESTUDIO_IK_PERF") || !m_benchSpec.isEmpty();
    m_perfClock.start();
    connect(m_ikTimer, &QTimer::timeout, this, [this]() {
        if (!m_renderer) {
            m_ikTimer->stop();
            m_ikSettling = false;
            return;
        }
        const qint64 tickStart = m_ikPerf ? m_perfClock.nsecsElapsed() : 0;
        if (m_ikPerf) {
            if (m_perfLastTickNs >= 0) {
                m_perfTickInterval.add(static_cast<double>(tickStart - m_perfLastTickNs) * 1e-6);
            }
            m_perfLastTickNs = tickStart;
        }
        if (m_benchActive && m_ikDragging) {
            benchAdvance(); // the scripted cursor: moves m_ikLastTarget along the bench path
        }
        if (m_ikSettling) {
            // Animated release settle: one capped round per tick until the feet land.
            if (!m_renderer->settleBoneIkTick()) {
                finishIkSettle();
                if (m_benchActive) {
                    benchFinish();
                }
            }
            requestUpdate();
        } else if (m_ikDragging && m_ikHasTarget) {
            // Redraw only when the solve actually moved the pose: during a held-still drag the
            // settle-freeze stops solving entirely, and re-rendering an unchanged frame at 60 Hz
            // would burn GPU/battery for nothing (rendering is event-driven everywhere else too).
            if (issueIkTarget()) {
                m_ikPoseChanged = true; // a real edit: the release may settle + commit undo
                requestUpdate();
            }
        }
        if (m_ikPerf) {
            m_perfTickWork.add(static_cast<double>(m_perfClock.nsecsElapsed() - tickStart) * 1e-6);
            glm::vec3 eff;
            if (m_ikDragging && m_ikHasTarget && m_renderer->selectedBoneWorldPosition(eff)) {
                m_perfLag.add(static_cast<double>(glm::length(eff - m_ikLastTarget)) * 1000.0);
                if (m_perfHasPrevEff) {
                    const glm::vec3 step = eff - m_perfPrevEff;
                    const float pl = glm::length(m_perfPrevStep);
                    if (pl > 1e-6f && glm::length(step) > 1e-6f) {
                        const float against = -glm::dot(step, m_perfPrevStep) / pl;
                        if (against > 0.0f) {
                            m_perfOsc += static_cast<double>(std::min(against, pl));
                        }
                    }
                    m_perfPrevStep = step;
                }
                m_perfPrevEff = eff;
                m_perfHasPrevEff = true;
            } else {
                m_perfHasPrevEff = false;
            }
            ++m_perfTicks;
            if (!m_benchActive && m_perfTicks % 60 == 0) {
                perfReport("drag"); // the interactive case: a report per second of dragging
            }
        }
    });
}

bool VulkanWindow::issueIkTarget() {
    // ADAPTIVE low-pass on the raw cursor target (see m_ikSmoothedTarget): the smoothing
    // strength follows how far the smoothed target LAGS the raw one — near-still cursor (the
    // lag is pixel noise), heavy smoothing kills jitter before it reaches the solver, which
    // AMPLIFIES it; fast deliberate gesture (the lag is centimeters), the filter opens up and
    // follows tightly, so smoothing costs almost no lag exactly when the user moves fast. The
    // old fixed 0.35 charged ~40ms of lag on flicks while passing ~2x the noise on precise
    // holds. Mirrored in the IK loop harness — keep the constants in sync.
    // Retuned with the exact drag refinement (Model::refinePins with the drag target): the
    // whole-body solve no longer amplifies target noise into the hand (the limb fit that
    // places the hand is locally linear, so noise passes 1:1 - a pixel), so the filter only
    // has to keep a NEAR-STILL cursor's jitter out (alpha 0.30 at zero lag) and can open all
    // the way (alpha 1.0 from ~7mm of lag): its steady-state lag at 0.45 m/s drops from ~11mm
    // to ~3mm, and a fast flick pays nothing.
    constexpr float kIkAlphaMin = 0.30f;
    constexpr float kIkAlphaMax = 1.0f;
    constexpr float kIkAlphaGain = 100.0f; // per meter of lag
    const float lag = glm::length(m_ikLastTarget - m_ikSmoothedTarget);
    const float alpha = glm::clamp(kIkAlphaMin + lag * kIkAlphaGain, kIkAlphaMin, kIkAlphaMax);
    m_ikSmoothedTarget = glm::mix(m_ikSmoothedTarget, m_ikLastTarget, alpha);
    return m_renderer->dragBoneIkTo(m_ikSmoothedTarget);
}

void VulkanWindow::finishIkSettle() {
    if (!m_ikSettling) {
        return;
    }
    m_ikSettling = false;
    m_ikTimer->stop();
    if (m_renderer) {
        m_renderer->endBoneIkDrag();
        m_renderer->finalizePose(); // correctives were deferred through drag AND settle
        commitPoseUndo();
        requestUpdate(); // finalizePose re-morphs corrective geometry - always show it (several
                         // callers reach here on paths with no request of their own)
    }
}

void VulkanWindow::perfReport(const char* label) {
    std::fprintf(stderr,
                 "[ikperf] %-8s ticks %4d | tick interval mean %6.2f max %6.2f ms | tick work mean "
                 "%5.2f max %5.2f ms | frame interval mean %6.2f max %6.2f ms (%d) | draw mean "
                 "%5.2f max %5.2f ms | lag mean %6.1f max %6.1f mm | osc %6.1f mm\n",
                 label, m_perfTickInterval.n, m_perfTickInterval.mean(), m_perfTickInterval.max,
                 m_perfTickWork.mean(), m_perfTickWork.max, m_perfFrameInterval.mean(),
                 m_perfFrameInterval.max, m_perfFrameInterval.n, m_perfDraw.mean(), m_perfDraw.max,
                 m_perfLag.mean(), m_perfLag.max, m_perfOsc * 1000.0);
    std::fflush(stderr);
    m_perfTickInterval.reset();
    m_perfTickWork.reset();
    m_perfFrameInterval.reset();
    m_perfDraw.reset();
    m_perfLag.reset();
    m_perfOsc = 0.0;
}

void VulkanWindow::startBench() {
    const std::string bone = m_benchSpec.section(QLatin1Char(':'), 0, 0).toStdString();
    const int boneIndex = m_renderer ? m_renderer->selectBoneByName(bone) : -2;
    if (boneIndex == -1 && m_benchRetries++ < 120) {
        // No figure yet: a command-line figure imports through the progress dialog, whose
        // processEvents() fires this timer mid-import. Poll until the figure exists.
        QTimer::singleShot(500, this, &VulkanWindow::startBench);
        return;
    }
    const bool began = boneIndex >= 0 && m_renderer->beginBoneIkDrag();
    const bool placed = began && m_renderer->selectedBoneWorldPosition(m_ikPlanePoint);
    if (!placed) {
        std::fprintf(stderr, "[ikbench] cannot start: bone '%s' index %d, beginDrag %d, position %d\n",
                     bone.c_str(), boneIndex, began ? 1 : 0, placed ? 1 : 0);
        std::fflush(stderr);
        std::_Exit(2); // diagnostic mode: no teardown
        return;
    }
    std::fprintf(stderr, "[ikbench] dragging %s from (%.3f %.3f %.3f)\n", bone.c_str(),
                 m_ikPlanePoint.x, m_ikPlanePoint.y, m_ikPlanePoint.z);
    {
        // Pick diagnostic: a ray through the grabbed joint's screen position must hit its model.
        const glm::mat4 vp = m_renderer->camera().viewProjection();
        const glm::vec4 clip = vp * glm::vec4(m_ikPlanePoint, 1.0f);
        if (clip.w > 1e-4f) {
            const glm::vec3 ndc = glm::vec3(clip) / clip.w;
            const float sx = (ndc.x * 0.5f + 0.5f) * static_cast<float>(width());
            const float sy = (ndc.y * 0.5f + 0.5f) * static_cast<float>(height());
            const Ray ray = m_renderer->camera().screenPointToRay(
                sx, sy, static_cast<float>(width()), static_cast<float>(height()));
            std::fprintf(stderr, "[ikbench] pick through joint at px(%.0f %.0f): model %d; ray o(%.2f %.2f %.2f) d(%.2f %.2f %.2f)\n",
                         sx, sy, m_renderer->pickModel(ray), ray.origin.x, ray.origin.y, ray.origin.z,
                         ray.direction.x, ray.direction.y, ray.direction.z);
        }
    }
    m_preEditPose = m_renderer->capturePose();
    m_benchStart = m_ikPlanePoint;
    m_ikLastTarget = m_benchStart;
    m_ikSmoothedTarget = m_benchStart;
    m_ikHasTarget = true;
    m_ikDragging = true;
    m_ikPoseChanged = false;
    m_benchActive = true;
    m_benchTick = 0;
    m_perfLastTickNs = -1;
    m_perfLastFrameNs = -1;
    perfReport("reset");
    m_ikTimer->start();
}

void VulkanWindow::benchAdvance() {
    // The scripted cursor path, in ticks: a moderate 30cm gesture (0.3 m/s at 60 Hz), a hold,
    // the return, a hold, a fast 36cm flick (1.2 m/s), a hold, then release.
    struct Waypoint {
        int         tick;
        glm::vec3   offset;
        const char* phase; // reported when this waypoint is reached
    };
    static const Waypoint kPath[] = {
        {0, glm::vec3(0.0f), "start"},
        {60, glm::vec3(0.25f, 0.15f, 0.05f), "move-med"},
        {120, glm::vec3(0.25f, 0.15f, 0.05f), "hold-1"},
        {180, glm::vec3(0.0f), "return"},
        {240, glm::vec3(0.0f), "hold-2"},
        {270, glm::vec3(-0.30f, 0.20f, 0.0f), "move-fast"},
        {330, glm::vec3(-0.30f, 0.20f, 0.0f), "hold-3"},
    };
    constexpr int kCount = static_cast<int>(sizeof(kPath) / sizeof(kPath[0]));
    const int t = ++m_benchTick;
    for (int i = 1; i < kCount; ++i) {
        if (t == kPath[i].tick) {
            perfReport(kPath[i].phase);
        }
        if (t <= kPath[i].tick) {
            const float f = static_cast<float>(t - kPath[i - 1].tick) /
                            static_cast<float>(kPath[i].tick - kPath[i - 1].tick);
            m_ikLastTarget = m_benchStart + glm::mix(kPath[i - 1].offset, kPath[i].offset, f);
            return;
        }
    }
    // Past the last waypoint: release (mirrors mouseReleaseEvent's IK branch).
    m_ikHasTarget = false;
    m_ikDragging = false;
    m_ikSettling = true;
}

void VulkanWindow::benchFinish() {
    perfReport("settle");
    std::fprintf(stderr, "[ikbench] done\n");
    std::fflush(stderr);
    m_benchActive = false;
    std::_Exit(0); // diagnostic mode: no teardown
}

void VulkanWindow::commitPoseUndo() {
    if (m_renderer && m_renderer->capturePose() != m_preEditPose) {
        UndoEntry entry;
        entry.kind = UndoEntry::Kind::Pose;
        entry.pose = m_preEditPose;
        entry.figure = m_renderer->activeFigureIndex();
        m_undoStack.push_back(std::move(entry));
        m_redoStack.clear();
    }
}

VulkanWindow::~VulkanWindow() {
    releaseVulkan();
}

VkExtent2D VulkanWindow::pixelExtent() const {
    const qreal dpr = devicePixelRatio();
    return VkExtent2D{
        static_cast<uint32_t>(std::lround(width() * dpr)),
        static_cast<uint32_t>(std::lround(height() * dpr)),
    };
}

void VulkanWindow::initializeVulkan() {
    if (m_initialized || m_deviceFailed) {
        return;
    }
    try {
        VkSurfaceKHR surface = m_instance->surfaceForWindow(this);
        if (surface == VK_NULL_HANDLE) {
            throw VulkanError("QVulkanInstance::surfaceForWindow returned a null surface.");
        }
        m_context = std::make_unique<VulkanContext>(m_instance->vkInstance(), surface, m_apiVersion);
        m_renderer = std::make_unique<VulkanRenderer>(*m_context, pixelExtent(),
                                                      m_shaderDir.toStdString());
        m_renderer->setShadeMode(m_shadeMode); // apply a mode chosen before first expose
        m_renderer->setShowSkeleton(m_showSkeleton); // apply a skeleton toggle chosen before first expose
        m_renderer->setLightingSettings(m_lighting); // apply any dials set before first expose

        // Image-based lighting: bake a real HDR panorama over the renderer's built-in procedural studio
        // (the environment the user picked before first expose, else the default). The bake runs off the
        // UI thread; the first frames render the procedural studio (from the Scene ctor) until it lands.
        beginEnvironmentBake(m_environmentPath.isEmpty() ? defaultEnvironmentPath() : m_environmentPath,
                             /*autoAimKey=*/false); // startup: the authored default dials win
        m_initialized = true;

        // Drain any imports requested before the renderer existed (e.g. user hit Import while
        // the viewport was still hidden). A bad file only drops that one model, not the window.
        for (const QString& pending : m_pendingModels) {
            // No progress dialog here: we're inside the expose/init path, where a modal
            // dialog's processEvents() could re-enter exposeEvent()/initializeVulkan().
            if (ModelImportService::importInto(*m_renderer, pending, /*showProgress=*/false)) {
                requestUpdate();
            }
        }
        m_pendingModels.clear();
        for (const QString& pending : m_pendingFigures) {
            if (FigureImportService::importInto(*m_renderer, pending, /*showProgress=*/false)) {
                // The skeleton overlay stays hidden; a click on a joint still selects it (picking is
                // independent of the overlay) and brings up the rotate gizmo.
                requestUpdate();
            }
        }
        if (!m_benchSpec.isEmpty()) {
            std::fprintf(stderr, "[ikbench] drained %zu queued figure(s)\n", m_pendingFigures.size());
            std::fflush(stderr);
        }
        m_pendingFigures.clear();
        if (!m_pendingPose.isEmpty()) {
            m_renderer->loadPose(m_pendingPose.toStdString()); // onto the just-drained figure
            m_pendingPose.clear();
            requestUpdate();
        }
        if (!m_benchSpec.isEmpty()) {
            QTimer::singleShot(1500, this, &VulkanWindow::startBench); // after the first frames
        }
    } catch (const VulkanError& e) {
        // Leave the surface intact (Qt owns it) but mark the device as failed so we
        // don't retry every expose. The container just shows the clear-colour-less window.
        m_renderer.reset();
        m_context.reset();
        m_deviceFailed = true;
        qCritical() << "[Vulkan] Viewport initialisation failed:" << e.what();
    }
}

QString VulkanWindow::defaultEnvironmentPath() const {
    // An explicit <appDir>/environment.hdr (or .exr) override wins; otherwise the stock/first
    // panorama from the user library's hdri/ tree via LibraryPaths::defaultHdri() — the same
    // recursive scan backing the Environment panel's menu, so a collection categorized into
    // subfolders resolves identically here. An empty return leaves the procedural studio
    // environment (beginEnvironmentBake no-ops on it).
    for (const char* name : {"environment.hdr", "environment.exr"}) {
        const QString override =
            QCoreApplication::applicationDirPath() + QLatin1Char('/') + QLatin1String(name);
        if (QFileInfo::exists(override)) {
            return override;
        }
    }
    return LibraryPaths::defaultHdri();
}

void VulkanWindow::beginEnvironmentBake(const QString& hdrPath, bool autoAimKey) {
    if (!m_renderer || hdrPath.isEmpty() || !QFileInfo::exists(hdrPath)) {
        return; // no renderer yet, or nothing to load — the procedural default stays
    }
    // Decode + bake the HDRI off the UI thread: the CPU prefilter is hundreds of ms and would freeze the
    // app on every switch. The viewport keeps rendering the current environment until the result lands.
    // A per-request id discards a slower earlier bake whose selection a newer one has since superseded.
    const quint64 requestId = ++m_environmentRequestId;
    auto* watcher = new QFutureWatcher<std::shared_ptr<BakedEnvironment>>(this);
    connect(watcher, &QFutureWatcherBase::finished, this, [this, watcher, requestId, autoAimKey]() {
        const std::shared_ptr<BakedEnvironment> baked = watcher->result();
        watcher->deleteLater();
        // The GPU upload runs here on the GUI thread; apply only the newest request that still has a
        // live renderer and a successful bake.
        if (requestId == m_environmentRequestId && m_renderer && baked) {
            try {
                m_renderer->applyBakedEnvironment(*baked);
            } catch (const VulkanError& e) {
                // The upload allocates GPU objects, so its VK_CHECKs can throw inside this queued
                // slot — the same no-boundary hole renderFrame() closes. Disable the viewport
                // rather than aborting the app.
                m_renderer.reset();
                m_context.reset();
                m_deviceFailed = true;
                qCritical() << "[Vulkan] Environment upload failed; viewport disabled:" << e.what();
                return;
            }
            // Auto-aim the key light at the environment's dominant light (its "sun"): in the PBR
            // mode the figure is visibly lit by the HDRI, so a key/shadow pointing anywhere else
            // reads as broken — the ground shadow must fall away from where the light comes from.
            // Only for USER-initiated HDRI switches (the startup bake keeps the authored default
            // dials — a fresh launch must show the LightingSettings defaults) and meaningfully
            // directional environments (an overcast sky has no sun); the azimuth/elevation dials
            // stay fully live for manual re-aiming afterwards, and the next HDRI switch re-aims
            // again. Not an undo entry — HDRI selection itself isn't one.
            if (autoAimKey && baked->dominantStrength >= 0.1f) {
                constexpr float kRadToDeg = 57.29577951f;
                const float elevation =
                    std::asin(std::clamp(baked->dominantDir.y, -1.0f, 1.0f)) * kRadToDeg;
                LightingSettings aimed = m_lighting;
                aimed.keyAzimuthDeg = std::atan2(baked->dominantDir.x, baked->dominantDir.z) * kRadToDeg;
                // Keep the key above the horizon (a below-horizon "sun" — window-lit interiors —
                // can't cast a ground shadow) and below ~60°: a near-zenith light drops the shadow
                // straight under the figure where it reads as a smudge, not a shadow. 60° keeps a
                // legible cast direction while staying close to the environment's real sun.
                aimed.keyElevationDeg = std::clamp(elevation, 15.0f, 60.0f);
                setLightingSettings(aimed);
                emit lightingRestored(aimed); // the Environment panel syncs its az/el rows
                qDebug().nospace() << "[viewport] key light auto-aimed to environment sun (azimuth "
                                   << aimed.keyAzimuthDeg << ", elevation " << aimed.keyElevationDeg
                                   << ", directionality " << baked->dominantStrength << ")";
            }
            requestUpdate();
        }
    });
    // The task captures only the path (by value); it touches no window/renderer state, so it stays safe
    // even if the window is torn down before it finishes (the watcher, a child of `this`, won't fire).
    watcher->setFuture(QtConcurrent::run([hdrPath]() -> std::shared_ptr<BakedEnvironment> {
        std::optional<EnvironmentImage> env = loadEnvironmentImage(hdrPath.toStdString());
        if (!env) {
            qWarning() << "[viewport] Failed to decode environment panorama:" << hdrPath;
            return nullptr;
        }
        return std::make_shared<BakedEnvironment>(bakeEnvironment(std::move(*env)));
    }));
}

void VulkanWindow::setEnvironmentFile(const QString& hdrPath) {
    m_environmentPath = hdrPath; // remembered so it survives a device loss / applies before first expose
    beginEnvironmentBake(hdrPath, /*autoAimKey=*/true); // no-op until the renderer exists (init kicks off the first bake)
}

void VulkanWindow::setLightingSettings(const LightingSettings& settings) {
    m_lighting = settings; // remembered so it applies before first expose / after a device loss
    if (m_renderer) {
        m_renderer->setLightingSettings(settings);
        requestUpdate();
    }
}

void VulkanWindow::releaseVulkan() {
    // A pending IK drag or release settle dies with the renderer (nothing left to settle or
    // commit) - the DRAG flags must clear too, or a rebuilt renderer inherits a phantom drag
    // whose eventual release would settle-and-commit against a stale pre-teardown pose.
    m_ikSettling = false;
    m_ikDragging = false;
    m_ikHasTarget = false;
    m_ikPoseChanged = false;
    if (m_fallTimer) {
        m_fallTimer->stop(); // the figure is going away with the renderer
        m_fallHeight = 0.0f;
        m_fallDropped = 0.0f;
    }
    if (m_ikTimer) {
        m_ikTimer->stop();
    }
    // Order matters: the renderer waits for the device to idle in its destructor, so it
    // must go before the context (which owns the device).
    m_renderer.reset();
    m_context.reset();
    m_initialized = false;
}

void VulkanWindow::importObj(const QString& path) {
    if (m_deviceFailed) {
        qWarning() << "[viewport] Ignoring OBJ import; Vulkan is unavailable:" << path;
        return;
    }
    if (!m_renderer) {
        m_pendingModels.push_back(path); // loaded once initializeVulkan() builds the renderer
        return;
    }
    // All the parse/decode/upload orchestration (and its progress UI) lives in the Qt-facing
    // ModelImportService, keeping both the importers and the renderer core Qt-free.
    if (ModelImportService::importInto(*m_renderer, path, /*showProgress=*/true)) {
        requestUpdate();
    }
}

void VulkanWindow::importFigure(const QString& path) {
    if (m_deviceFailed) {
        qWarning() << "[viewport] Ignoring figure import; Vulkan is unavailable:" << path;
        return;
    }
    if (!m_renderer) {
        m_pendingFigures.push_back(path); // loaded once initializeVulkan() builds the renderer
        return;
    }
    if (FigureImportService::importInto(*m_renderer, path, /*showProgress=*/true)) {
        // The skeleton overlay stays hidden; a click on a joint still selects it and shows the gizmo.
        requestUpdate();
    }
}

bool VulkanWindow::hasPosableFigure() const {
    return m_renderer && m_renderer->hasPosableFigure();
}

void VulkanWindow::setShadeMode(int mode) {
    m_shadeMode = mode; // remembered so a mode chosen before first expose still applies
    if (m_renderer) {
        m_renderer->setShadeMode(mode);
        requestUpdate();
    }
}

void VulkanWindow::setShowSkeleton(bool on) {
    if (m_showSkeleton == on) return;
    m_showSkeleton = on; // remembered so a toggle before first expose still applies
    if (m_renderer) {
        m_renderer->setShowSkeleton(on);
        requestUpdate();
    }
}

bool VulkanWindow::showSkeleton() const { return m_showSkeleton; }

void VulkanWindow::resetView() {
    if (m_renderer) {
        m_renderer->camera().reset();
        requestUpdate(); // rendering is on demand — nothing redraws unless we ask
    }
}

void VulkanWindow::groundFigure() {
    if (!m_renderer || m_ikDragging || m_fallTimer->isActive()) {
        return; // no renderer yet, a drag owns the pose, or a fall is already in flight
    }
    finishIkSettle(); // ground the LANDED pose, not a transient mid-settle frame
    float lowestY = 0.0f;
    if (!m_renderer->figureGroundGap(lowestY) || std::abs(lowestY) < 1e-4f) {
        return; // nothing to ground, or already resting on the floor
    }
    if (lowestY < 0.0f) {
        // Sunk into the floor: nothing falls upward — lift it out in one step.
        m_renderer->translateFigureY(-lowestY);
        requestUpdate();
        return;
    }
    // Hovering: FALL. The timer applies the free-fall curve's increments (see the constructor).
    m_fallHeight = lowestY;
    m_fallDropped = 0.0f;
    m_fallClock.start();
    m_fallTimer->start();
}

void VulkanWindow::finishGroundFall() {
    if (!m_fallTimer || !m_fallTimer->isActive()) {
        return;
    }
    m_fallTimer->stop();
    if (m_renderer && m_fallHeight > m_fallDropped) {
        m_renderer->translateFigureY(-(m_fallHeight - m_fallDropped));
        requestUpdate();
    }
    m_fallHeight = 0.0f;
    m_fallDropped = 0.0f;
}

bool VulkanWindow::savePose(const QString& path) {
    finishIkSettle(); // serialize the LANDED pose, not a transient mid-settle frame
    finishGroundFall();
    return m_renderer && m_renderer->savePose(path.toStdString());
}

bool VulkanWindow::loadPose(const QString& path) {
    finishIkSettle(); // don't let a still-animating release settle fight the loaded pose
    if (!m_renderer) {
        m_pendingPose = path; // applied after the queued figure is drained (open-with a .pose)
        return true;
    }
    if (!m_renderer->loadPose(path.toStdString())) {
        return false;
    }
    requestUpdate(); // repaint with the restored pose
    return true;
}

void VulkanWindow::exposeEvent(QExposeEvent*) {
    if (isExposed()) {
        initializeVulkan();
        if (m_renderer) {
            m_renderer->notifyResize(pixelExtent());
            requestUpdate();
        }
    }
}

void VulkanWindow::resizeEvent(QResizeEvent*) {
    if (m_renderer) {
        m_renderer->notifyResize(pixelExtent());
        requestUpdate();
    }
}

bool VulkanWindow::event(QEvent* e) {
    switch (e->type()) {
    case QEvent::UpdateRequest:
        renderFrame();
        return true;
    case QEvent::PlatformSurface:
        // The surface is about to disappear (window closing/reparenting). Tear the
        // Vulkan objects down now, while the surface they reference is still valid.
        if (static_cast<QPlatformSurfaceEvent*>(e)->surfaceEventType() ==
            QPlatformSurfaceEvent::SurfaceAboutToBeDestroyed) {
            releaseVulkan();
        }
        break;
    default:
        break;
    }
    return QWindow::event(e);
}

void VulkanWindow::renderFrame() {
    if (!m_renderer || !isExposed()) {
        return;
    }
    // Event-driven rendering: a frame is drawn only when something requested one (input, imports,
    // pose edits, shade-mode/lighting changes, expose/resize — every state change already calls
    // requestUpdate()). The old continuous requestUpdate() here redrew at full swap rate even when
    // idle, keeping the GPU busy and (with FIFO/vsync present) throttling the whole GUI thread.
    // drawFrame() returns true only when the swapchain was just rebuilt (or the frame skipped
    // mid-rebuild) and one follow-up frame is needed to reflect the new size.
    try {
        const bool perfFrame = m_ikPerf && (m_ikDragging || m_ikSettling);
        const qint64 f0 = perfFrame ? m_perfClock.nsecsElapsed() : 0;
        if (perfFrame) {
            if (m_perfLastFrameNs >= 0) {
                m_perfFrameInterval.add(static_cast<double>(f0 - m_perfLastFrameNs) * 1e-6);
            }
            m_perfLastFrameNs = f0;
        }
        if (m_renderer->drawFrame()) {
            requestUpdate();
        }
        if (perfFrame) {
            m_perfDraw.add(static_cast<double>(m_perfClock.nsecsElapsed() - f0) * 1e-6);
        }
    } catch (const VulkanError& e) {
        // Runtime VK_CHECK failure (e.g. VK_ERROR_DEVICE_LOST after a driver reset/TDR): tear the
        // renderer down and stop scheduling frames, rather than letting the exception escape
        // Qt's event loop and abort the whole app. Mirrors the initializeVulkan() catch.
        m_renderer.reset();
        m_context.reset();
        m_deviceFailed = true;
        qCritical() << "[Vulkan] Rendering failed; viewport disabled:" << e.what();
    }
}

void VulkanWindow::mousePressEvent(QMouseEvent* event) {
    finishIkSettle(); // a new interaction must not overlap a still-animating release settle
    finishGroundFall(); // nor a figure still falling (a drag captures the floor at its start)
    m_lastMousePos = event->position();
    m_activeDragButtons |= event->button(); // a drag with this button started in the viewport

    // Left-press priority: (0) Ctrl held + a joint -> full-body-IK drag of that joint; (1) a gizmo
    // ring of the selected joint -> axis-constrained rotate; (2) a joint -> select it + free-rotate
    // this drag (FK); (3) empty space -> orbit the camera.
    if (event->button() == Qt::LeftButton && m_renderer) {
        const float x = static_cast<float>(event->position().x());
        const float y = static_cast<float>(event->position().y());
        const float w = static_cast<float>(width());
        const float h = static_cast<float>(height());
        if (m_ikDragging) {
            // A stale drag whose release never reached this window (a modal/shortcut swallowed
            // the mouse-up - the m_activeDragButtons event-leak class): close it out, or the
            // 60 Hz timer runs forever and this press hijacks into IK-dragging the old joint.
            m_renderer->endBoneIkDrag();
            m_ikDragging = false;
            m_ikHasTarget = false;
            m_ikTimer->stop();
        }
        if (event->modifiers().testFlag(Qt::ControlModifier)) {
            // FBIK: grab the joint under the cursor and drag it through a camera-parallel plane
            // anchored at its current position; the body follows (feet pinned, auto-balanced).
            m_gizmoAxis = -1;
            m_posingBone = false;
            m_ikDragging = m_renderer->selectBoneAt(x, y, w, h) >= 0 &&
                           m_renderer->beginBoneIkDrag() &&
                           m_renderer->selectedBoneWorldPosition(m_ikPlanePoint);
            if (m_ikDragging) {
                m_ikHasTarget = false;
                m_ikPoseChanged = false;
                m_ikTimer->start();
            }
        } else {
            m_gizmoAxis = m_renderer->hasSelectedBone() ? m_renderer->gizmoAxisAt(x, y, w, h) : -1;
            if (m_gizmoAxis >= 0) {
                m_posingBone = false;
            } else {
                m_posingBone = (m_renderer->selectBoneAt(x, y, w, h) >= 0);
            }
        }
        if (m_gizmoAxis >= 0 || m_posingBone || m_ikDragging) {
            m_preEditPose = m_renderer->capturePose(); // snapshot for undo (committed on release)
        }
        requestUpdate(); // reflect the new selection highlight / gizmo
    }
}

void VulkanWindow::mouseReleaseEvent(QMouseEvent* event) {
    m_activeDragButtons &= ~event->button();
    if (event->button() == Qt::LeftButton) {
        if ((m_posingBone || m_gizmoAxis >= 0 || m_ikDragging) && m_renderer) {
            if (m_ikDragging && !m_ikPoseChanged) {
                // Ctrl+CLICK: the joint was selected but no solve ever moved the pose — end the
                // drag with no settle and no undo (the settle would still walk a hovering
                // figure onto its ground-healed pins, turning a mere selection into an edit).
                m_renderer->endBoneIkDrag();
                m_ikTimer->stop();
            } else if (m_ikDragging) {
                // IK release: hand off to the ANIMATED settle — the timer keeps ticking, each
                // tick relaxing the body onto its pins so hovering feet visibly land instead of
                // popping in one frame. finalizePose + the undo commit run when it finishes
                // (finishIkSettle), keeping the drag-deferred correctives deferred through it.
                m_ikHasTarget = false;
                m_ikSettling = true;
            } else {
                // FK/gizmo edit settled: apply the pose correctives (deferred during the drag
                // because they re-upload geometry) and commit an undo entry if the pose changed.
                m_renderer->finalizePose();
                commitPoseUndo();
            }
            requestUpdate();
        }
        m_posingBone = false;
        m_gizmoAxis = -1;
        m_ikDragging = false;
    }
    // Right-click (on release, the desktop convention) opens the object context menu. The right
    // button drives no camera motion, so there's nothing to disambiguate from a drag here.
    if (event->button() == Qt::RightButton) {
        showObjectContextMenu(event->position(), event->globalPosition().toPoint());
    }
}

void VulkanWindow::showObjectContextMenu(const QPointF& localPos, const QPoint& globalPos) {
    if (!m_renderer) {
        return;
    }
    const float x = static_cast<float>(localPos.x());
    const float y = static_cast<float>(localPos.y());
    const float w = static_cast<float>(width());
    const float h = static_cast<float>(height());
    // A joint under the cursor gets the pin actions (selecting it, so the gizmo shows which
    // joint the menu acts on); the model under the cursor gets Delete. Both can apply.
    const int joint = (m_ikDragging || m_ikSettling) ? -1 : m_renderer->selectBoneAt(x, y, w, h);
    const Ray ray = m_renderer->camera().screenPointToRay(x, y, w, h);
    const int picked = m_renderer->pickModel(ray);
    if (joint < 0 && picked < 0) {
        return; // empty space — no menu (for now)
    }

    QMenu menu;
    QAction* pinAction = nullptr;
    QAction* unpinAllAction = nullptr;
    if (joint >= 0) {
        pinAction = menu.addAction(m_renderer->selectedBonePinned()
                                       ? QStringLiteral("Unpin Joint\tP")
                                       : QStringLiteral("Pin Joint\tP"));
        if (m_renderer->hasPinnedBones()) {
            unpinAllAction = menu.addAction(QStringLiteral("Unpin All Joints"));
        }
        if (picked >= 0) {
            menu.addSeparator();
        }
    }
    QAction* deleteAction = picked >= 0 ? menu.addAction(QStringLiteral("Delete")) : nullptr;
    QAction* chosen = menu.exec(globalPos);
    if (chosen != nullptr && (chosen == pinAction || chosen == unpinAllAction)) {
        // Pins ride in the pose snapshot, so a pin edit is an ordinary undoable pose edit.
        m_preEditPose = m_renderer->capturePose();
        if (chosen == pinAction) {
            m_renderer->togglePinSelectedBone();
        } else {
            m_renderer->unpinAllBones();
        }
        commitPoseUndo();
    } else if (chosen != nullptr && chosen == deleteAction) {
        if (m_ikDragging) {
            // Deleting the dragged figure mid-gesture (left button still held while the menu
            // opened): end the drag cleanly first - its settle/undo would otherwise retarget
            // to whatever figure remains.
            m_renderer->endBoneIkDrag();
            m_ikDragging = false;
            m_ikHasTarget = false;
            m_ikTimer->stop();
        }
        m_renderer->deleteModel(static_cast<std::size_t>(picked));
        // Model indices shift and the deleted figure's poses are meaningless: the pose
        // history goes with it (lighting entries too — one chronological stack).
        m_undoStack.clear();
        m_redoStack.clear();
    }
    requestUpdate(); // selection highlight / pin markers changed even if nothing was chosen
}

void VulkanWindow::mouseMoveEvent(QMouseEvent* event) {
    if (!m_renderer) {
        return;
    }
    const QPointF prev = m_lastMousePos;
    const QPointF delta = event->position() - m_lastMousePos;
    m_lastMousePos = event->position();

    // Only act on buttons that are BOTH held now AND were pressed inside this window. Using
    // event->buttons() alone would let a leaked move (e.g. as the modal Import dialog closes
    // over us with a button still logically down) snap the camera from a stale last-position.
    const Qt::MouseButtons active = m_activeDragButtons & event->buttons();

    Camera& camera = m_renderer->camera();
    if ((active & Qt::LeftButton) && m_ikDragging) {
        // Full-body IK: the grabbed joint tracks the cursor within the camera-parallel plane
        // through its grab point (fixed for the whole drag so the depth can't feed back).
        const glm::mat4 view = camera.view();
        const glm::vec3 planeNormal(view[0][2], view[1][2], view[2][2]); // toward the camera
        const Ray ray = camera.screenPointToRay(
            static_cast<float>(event->position().x()), static_cast<float>(event->position().y()),
            static_cast<float>(width()), static_cast<float>(height()));
        const float denom = glm::dot(ray.direction, planeNormal);
        if (std::abs(denom) > 1e-4f) {
            const float t = glm::dot(m_ikPlanePoint - ray.origin, planeNormal) / denom;
            if (t > 0.0f) {
                m_ikLastTarget = ray.origin + ray.direction * t;
                if (!m_ikHasTarget) {
                    m_ikSmoothedTarget = m_ikLastTarget; // seed the filter at the grab point
                    m_ikHasTarget = true;
                }
                // Deliberately NO solve here: the 60 Hz timer is the SOLE caller of
                // issueIkTarget(). Solving per mouse event stacked extra solves on top of the
                // timer's - with a high-polling-rate mouse the damped-motion dynamics (all
                // tuned in per-tick units) ran several times faster than designed.
            }
        }
    } else if ((active & Qt::LeftButton) && m_gizmoAxis >= 0) {
        // Gizmo: rotate the selected joint about the grabbed ring's axis by the swept screen angle.
        m_renderer->rotateGizmo(m_gizmoAxis, static_cast<float>(prev.x()), static_cast<float>(prev.y()),
                                static_cast<float>(event->position().x()),
                                static_cast<float>(event->position().y()), static_cast<float>(width()),
                                static_cast<float>(height()));
    } else if ((active & Qt::LeftButton) && m_posingBone) {
        // Free posing: drag rotates the selected joint — horizontal about its Y axis, vertical about X.
        constexpr float kDegPerPixel = 0.4f;
        m_renderer->nudgeSelectedBone(glm::vec3(static_cast<float>(delta.y()) * kDegPerPixel,
                                                static_cast<float>(delta.x()) * kDegPerPixel, 0.0f));
    } else if (active & Qt::LeftButton) {
        // Drag right -> orbit right; drag up -> tilt up. Negated to feel like grabbing the scene.
        camera.orbit(-static_cast<float>(delta.x()) * kOrbitRadiansPerPixel,
                     -static_cast<float>(delta.y()) * kOrbitRadiansPerPixel);
    } else if (active & Qt::MiddleButton) {
        camera.pan(static_cast<float>(delta.x()) * kPanPerPixel,
                   static_cast<float>(delta.y()) * kPanPerPixel);
    } else {
        // Plain hover (QWindow gets move events with no buttons held): nothing changed, so don't
        // schedule a frame — an unconditional requestUpdate() here redraws the whole scene at
        // mouse-move rate, exactly the idle churn event-driven rendering exists to avoid.
        return;
    }
    requestUpdate();
}

void VulkanWindow::wheelEvent(QWheelEvent* event) {
    if (!m_renderer) {
        return;
    }
    const float steps = static_cast<float>(event->angleDelta().y()) / 120.0f;
    m_renderer->camera().dolly(steps * kDollyPerWheelStep);
    requestUpdate();
}

void VulkanWindow::keyPressEvent(QKeyEvent* event) {
    // Fallback path: the Edit-menu actions carry the app-wide Ctrl+Z/Ctrl+Y shortcuts, but keep
    // handling the raw keys here too in case a platform delivers them to the native window
    // without the shortcut map consuming them first.
    if (m_renderer && event->modifiers().testFlag(Qt::ControlModifier)) {
        if (event->key() == Qt::Key_Z) {
            undo();
            event->accept();
            return;
        }
        if (event->key() == Qt::Key_Y) {
            redo();
            event->accept();
            return;
        }
    }
    // P toggles the pin on the selected joint (the joint is then held in place through IK drags
    // of other joints). Not mid-gesture: the rig captured its pins at drag start.
    if (m_renderer && event->key() == Qt::Key_P && event->modifiers() == Qt::NoModifier &&
        !m_ikDragging && !m_ikSettling && m_renderer->hasSelectedBone()) {
        m_preEditPose = m_renderer->capturePose(); // a pin toggle is an undoable pose edit
        m_renderer->togglePinSelectedBone();
        commitPoseUndo();
        requestUpdate();
        event->accept();
        return;
    }
    QWindow::keyPressEvent(event);
}

void VulkanWindow::registerLightingUndo(const LightingSettings& preEdit) {
    UndoEntry entry;
    entry.kind = UndoEntry::Kind::Lighting;
    entry.lighting = preEdit;
    m_undoStack.push_back(std::move(entry));
    m_redoStack.clear(); // a fresh edit invalidates the redo branch, same as a pose edit
}

void VulkanWindow::undo() {
    if (m_ikDragging || m_posingBone || m_gizmoAxis >= 0) {
        return; // mid-gesture (Ctrl+Z with the button held): the drag owns the pose right now —
                // an IK drag in particular solves against pins captured at drag start, and
                // re-posing underneath it would leave the solve fighting a stale stance.
    }
    finishIkSettle(); // a pending release settle must commit its own undo entry first
    if (!m_renderer || m_undoStack.empty()) {
        return;
    }
    UndoEntry entry = std::move(m_undoStack.back());
    m_undoStack.pop_back();
    UndoEntry redo;
    redo.kind = entry.kind;
    if (entry.kind == UndoEntry::Kind::Pose) {
        m_renderer->setActiveFigure(entry.figure); // the snapshot belongs to that figure
        redo.figure = entry.figure;
        redo.pose = m_renderer->capturePose(); // current pose becomes redoable
        m_renderer->applyPose(entry.pose);
        m_renderer->finalizePose(); // re-runs correctives, like any pose change
    } else {
        redo.lighting = m_lighting; // current dials become redoable
        setLightingSettings(entry.lighting);
        emit lightingRestored(entry.lighting); // the Environment panel syncs its widgets
    }
    m_redoStack.push_back(std::move(redo));
    requestUpdate();
}

void VulkanWindow::redo() {
    if (m_ikDragging || m_posingBone || m_gizmoAxis >= 0) {
        return; // mid-gesture: see undo()
    }
    finishIkSettle();
    if (!m_renderer || m_redoStack.empty()) {
        return;
    }
    UndoEntry entry = std::move(m_redoStack.back());
    m_redoStack.pop_back();
    UndoEntry undone;
    undone.kind = entry.kind;
    if (entry.kind == UndoEntry::Kind::Pose) {
        m_renderer->setActiveFigure(entry.figure);
        undone.figure = entry.figure;
        undone.pose = m_renderer->capturePose();
        m_renderer->applyPose(entry.pose);
        m_renderer->finalizePose();
    } else {
        undone.lighting = m_lighting;
        setLightingSettings(entry.lighting);
        emit lightingRestored(entry.lighting);
    }
    m_undoStack.push_back(std::move(undone));
    requestUpdate();
}

} // namespace pose
