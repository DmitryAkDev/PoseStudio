/**
 * @file probe.cpp
 * @brief Temporary diagnostic: reproduce the window-level .pss save/load sequence with the real
 *        import/scene classes and print exactly which fields diverge.
 */

#include "figureimportservice.h"
#include "rendering/vulkancontext.h"
#include "rendering/vulkanrenderer.h"
#include "scene/model.h"
#include "scene/posefile.h"
#include "scene/projectfile.h"
#include "scene/scene.h"

#include <QApplication>
#include <QSurface>
#include <QVulkanInstance>
#include <QWindow>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/matrix_decompose.hpp>

#include <cstdio>

using namespace pose;

namespace {

ProjectDocument capture(const Model* m) {
    ProjectFigure fig;
    fig.source = m->sourcePath();

    glm::vec3 rootTranslation(0.0f);
    std::string rootBone;
    for (const auto& [name, value] : m->capturePose()) {
        if (name.compare(0, sizeof(kPosePinPrefix) - 1, kPosePinPrefix) == 0) {
            fig.pins.push_back(name.substr(sizeof(kPosePinPrefix) - 1));
        } else if (name.compare(0, sizeof(kPoseTranslationPrefix) - 1, kPoseTranslationPrefix) == 0) {
            rootBone = name.substr(sizeof(kPoseTranslationPrefix) - 1);
            rootTranslation = value;
        } else {
            fig.pose.emplace_back(name, value);
        }
    }
    if (!rootBone.empty()) {
        fig.rootBone = std::move(rootBone);
        fig.rootTranslation = rootTranslation;
    }

    glm::quat rotation;
    glm::vec3 skew;
    glm::vec4 perspective;
    if (glm::decompose(m->transform(), fig.scale, rotation, fig.translation, skew, perspective)) {
        fig.rotation = glm::vec4(rotation.x, rotation.y, rotation.z, rotation.w);
    }

    ProjectDocument doc;
    doc.figures.push_back(std::move(fig));
    return doc;
}

void printDoc(const char* label, const ProjectDocument& d) {
    if (d.figures.empty()) {
        std::printf("%s: NO FIGURES\n", label);
        return;
    }
    const ProjectFigure& f = d.figures.front();
    std::printf(
        "%s (source='%s'):\n  translation=(%.6f, %.6f, %.6f)\n  rotation=(%.6f, %.6f, %.6f, %.6f)\n"
        "  scale=(%.6f, %.6f, %.6f)\n  rootBone='%s' rootTranslation=(%.6f, %.6f, %.6f)\n",
        label, f.source.c_str(), f.translation.x, f.translation.y, f.translation.z, f.rotation.x,
        f.rotation.y, f.rotation.z, f.rotation.w, f.scale.x, f.scale.y, f.scale.z, f.rootBone.c_str(),
        f.rootTranslation.x, f.rootTranslation.y, f.rootTranslation.z);
    std::printf("  pose rows (%zu):\n", f.pose.size());
    for (const auto& [name, v] : f.pose) {
        std::printf("    %-24s (%.4f, %.4f, %.4f)\n", name.c_str(), v.x, v.y, v.z);
    }
}

bool sameDoc(const ProjectDocument& a, const ProjectDocument& b) {
    if (a.figures.size() != b.figures.size()) return false;
    for (std::size_t i = 0; i < a.figures.size(); ++i) {
        const auto& x = a.figures[i];
        const auto& y = b.figures[i];
        if (x.source != y.source) {
            std::printf("DIFF source: '%s' vs '%s'\n", x.source.c_str(), y.source.c_str());
            return false;
        }
        if ((float)glm::length(x.translation - y.translation) > 1e-4f) {
            std::printf("DIFF translation: (%a,%a,%a) vs (%a,%a,%a) len=%f\n", x.translation.x,
                        x.translation.y, x.translation.z, y.translation.x, y.translation.y,
                        y.translation.z, (float)(float)glm::length(x.translation - y.translation));
            return false;
        }
        if (std::abs(x.rotation.w * y.rotation.w + x.rotation.x * y.rotation.x +
                     x.rotation.y * y.rotation.y + x.rotation.z * y.rotation.z) < 0.99999f) {
            std::printf("DIFF rotation: (%f,%f,%f,%f) vs (%f,%f,%f,%f)\n", x.rotation.x, x.rotation.y,
                        x.rotation.z, x.rotation.w, y.rotation.x, y.rotation.y, y.rotation.z, y.rotation.w);
            return false;
        }
        if ((float)glm::length(x.scale - y.scale) > 1e-4f) { std::printf("DIFF scale\n"); return false; }
        if (x.rootBone != y.rootBone) { std::printf("DIFF rootBone\n"); return false; }
        if ((float)glm::length(x.rootTranslation - y.rootTranslation) > 1e-4f) {
            std::printf("DIFF rootTranslation\n"); return false;
        }
        if (x.pins != y.pins) { std::printf("DIFF pins\n"); return false; }
        if (x.pose.size() != y.pose.size()) { std::printf("DIFF pose size %zu vs %zu\n", x.pose.size(), y.pose.size()); return false; }
        for (std::size_t j = 0; j < x.pose.size(); ++j) {
            if (x.pose[j].first != y.pose[j].first) { std::printf("DIFF pose name %zu\n", j); return false; }
            if ((float)glm::length(x.pose[j].second - y.pose[j].second) > 1e-3f) {
                std::printf("DIFF pose value %zu: (%f,%f,%f) vs (%f,%f,%f)\n", j, x.pose[j].second.x,
                            x.pose[j].second.y, x.pose[j].second.z, y.pose[j].second.x, y.pose[j].second.y,
                            y.pose[j].second.z);
                return false;
            }
        }
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: probe <figure.duf> <out.pss>\n");
        return 2;
    }
    QApplication app(argc, argv);
    const QString figure = argv[1];
    const QString outPss = argv[2];

    QVulkanInstance instance;
    instance.setApiVersion(QVersionNumber(1, 1));
    if (!instance.create()) {
        std::fprintf(stderr, "no vulkan\n");
        return 1;
    }

    QWindow window;
    window.setSurfaceType(QSurface::VulkanSurface);
    window.setVulkanInstance(&instance);
    window.resize(800, 600);
    window.show();

    VulkanContext context(instance.vkInstance(), instance.surfaceForWindow(&window),
                          VK_API_VERSION_1_1);
    VulkanRenderer renderer(context, {800, 600}, "build/shaders");

    if (!FigureImportService::importInto(renderer, figure, false)) {
        std::fprintf(stderr, "import failed\n");
        return 1;
    }
    Model* m = renderer.scene().modelAt(0);
    if (!m) {
        std::fprintf(stderr, "no model after import\n");
        return 1;
    }

    // Pose a few bones the way a user would (name availability printed below).
    const char* candidates[] = {"head", "Head", "chest", "Chest", "leftUpLeg", "LeftUpLeg",
                                "rightArm", "RightArm", "spine", "Spine"};
    for (const char* name : candidates) {
        m->setBoneRotation(name, {12.0f, 7.0f, -5.0f}); // availability shows up in the capture below
    }

    ProjectDocument doc1 = capture(m);
    // Also exercise a non-identity model transform (rotation + translation + scale).
    m->setTransform(glm::translate(glm::mat4(1.0f), glm::vec3(0.3f, 0.0f, -0.2f)) *
                    glm::rotate(glm::mat4(1.0f), glm::radians(37.0f), glm::vec3(0.0f, 1.0f, 0.0f)) *
                    glm::scale(glm::mat4(1.0f), glm::vec3(1.05f)));
    ProjectDocument doc1b = capture(m);
    std::printf("model transform before save: t=(%f,%f,%f) r=(%f,%f,%f,%f) s=(%f,%f,%f)\n",
                doc1b.figures.front().translation.x, doc1b.figures.front().translation.y,
                doc1b.figures.front().translation.z, doc1b.figures.front().rotation.x,
                doc1b.figures.front().rotation.y, doc1b.figures.front().rotation.z,
                doc1b.figures.front().rotation.w, doc1b.figures.front().scale.x,
                doc1b.figures.front().scale.y, doc1b.figures.front().scale.z);
    ProjectDocument& saved = doc1b;
    printDoc("AFTER IMPORT+POSE (capture #1)", doc1);

    if (!writeProjectFile(outPss.toStdString(), saved)) {
        std::fprintf(stderr, "writeProjectFile failed\n");
        return 1;
    }

    ProjectDocument doc2;
    std::string error;
    if (!readProjectFile(outPss.toStdString(), doc2, error)) {
        std::fprintf(stderr, "readProjectFile failed: %s\n", error.c_str());
        return 1;
    }
    printDoc("AFTER FILE ROUND-TRIP (codec)", doc2);
    std::printf("codec round-trip identical: %s\n\n", sameDoc(saved, doc2) ? "yes" : "NO");

    // Re-import fresh and restore exactly like loadProjectFile does.
    renderer.clearModels();
    if (!FigureImportService::importInto(renderer, figure, false)) {
        std::fprintf(stderr, "re-import failed\n");
        return 1;
    }
    Model* m2 = renderer.scene().modelAt(0);
    const ProjectFigure& fig = doc2.figures.front();
    std::printf("fig fields: t=(%f,%f,%f) r=(%f,%f,%f,%f) s=(%f,%f,%f)\n", fig.translation.x,
                fig.translation.y, fig.translation.z, fig.rotation.x, fig.rotation.y, fig.rotation.z,
                fig.rotation.w, fig.scale.x, fig.scale.y, fig.scale.z);
    const glm::quat q(fig.rotation.w, fig.rotation.x, fig.rotation.y, fig.rotation.z);
    std::printf("quat len=%f\n", (float)glm::length(glm::vec4(q.x, q.y, q.z, q.w)));
    const glm::mat4 transform =
        glm::translate(glm::mat4(1.0f), fig.translation) *
        glm::mat4_cast(glm::quat(fig.rotation.w, fig.rotation.x, fig.rotation.y, fig.rotation.z)) *
        glm::scale(glm::mat4(1.0f), fig.scale);
    {
        const glm::mat4 t = glm::translate(glm::mat4(1.0f), fig.translation);
        const glm::mat4 r = glm::mat4_cast(q);
        const glm::mat4 s = glm::scale(glm::mat4(1.0f), fig.scale);
        std::printf("T:\n"); for (int i = 0; i < 4; ++i) std::printf("  [%f %f %f %f]\n", t[i][0], t[i][1], t[i][2], t[i][3]);
        std::printf("R:\n"); for (int i = 0; i < 4; ++i) std::printf("  [%f %f %f %f]\n", r[i][0], r[i][1], r[i][2], r[i][3]);
        std::printf("S:\n"); for (int i = 0; i < 4; ++i) std::printf("  [%f %f %f %f]\n", s[i][0], s[i][1], s[i][2], s[i][3]);
    }
    m2->setTransform(transform);
    std::printf("constructed transform:\n");
    for (int r = 0; r < 4; ++r)
        std::printf("  [ %10.6f %10.6f %10.6f %10.6f ]\n", transform[r][0], transform[r][1],
                    transform[r][2], transform[r][3]);
    std::printf("m2->transform() right after setTransform:\n");
    for (int r = 0; r < 4; ++r)
        std::printf("  [ %10.6f %10.6f %10.6f %10.6f ]\n", m2->transform()[r][0], m2->transform()[r][1],
                    m2->transform()[r][2], m2->transform()[r][3]);

    using PoseSnapshot = std::vector<std::pair<std::string, glm::vec3>>;
    PoseSnapshot rows;
    for (const auto& [name, value] : fig.pose) {
        rows.emplace_back(name, value);
    }
    if (!fig.rootBone.empty()) {
        rows.emplace_back(std::string(kPoseTranslationPrefix) + fig.rootBone, fig.rootTranslation);
    }
    for (const std::string& pin : fig.pins) {
        rows.emplace_back(std::string(kPosePinPrefix) + pin, glm::vec3(1.0f, 0.0f, 0.0f));
    }
    m2->applyPose(rows);

    ProjectDocument doc3 = capture(m2);
    printDoc("AFTER RESTORE (capture #2)", doc3);
    std::printf("\nrestore matches original: %s\n", sameDoc(saved, doc3) ? "yes" : "NO");

    // The model matrix as imported vs restored.
    auto printMatrix = [](const char* label, const glm::mat4& mat) {
        std::printf("%s:\n", label);
        for (int r = 0; r < 4; ++r) {
            std::printf("  [ %10.6f %10.6f %10.6f %10.6f ]\n", mat[r][0], mat[r][1], mat[r][2],
                        mat[r][3]);
        }
    };
    printMatrix("m->transform() AFTER RESTORE", m2->transform());

    return sameDoc(saved, doc3) ? 0 : 3;
}
