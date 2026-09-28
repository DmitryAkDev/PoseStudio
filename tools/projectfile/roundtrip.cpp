/**
 * @file roundtrip.cpp
 * @brief Round-trip and rejection checks for the `.pss` project codec.
 *
 * A standalone guard for the file-format contract: it writes a fully-populated
 * ProjectDocument, reads it back and compares every field bit-for-bit, then
 * feeds the reader the failure cases (bad JSON, wrong marker, higher version,
 * missing file) and a forward-compatibility document with unknown fields. It
 * links only `scene/projectfile.cpp` — no Qt, no Vulkan — so it runs anywhere
 * the app builds, and is registered with CTest (`ctest -R project-file-check`).
 *
 * Exit code = number of failed checks (0 = green).
 */

#include "projectfile.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <glm/glm.hpp>

using pose::ProjectCamera;
using pose::ProjectDocument;
using pose::ProjectEnvironment;
using pose::ProjectFigure;

namespace {

int failures = 0;

/// One scratch directory for every file this check writes: the OS temp dir
/// (the platform's — no hardcoded /tmp), created up front.
const std::string kDir =
    (std::filesystem::temp_directory_path() / "pss-rt").string();

std::string path(const char* name) { return kDir + "/" + name; }

void check(bool ok, const char* name) {
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", name);
    if (!ok) {
        ++failures;
    }
}

bool vec3Equal(const glm::vec3& a, const glm::vec3& b) { return glm::all(glm::equal(a, b)); }
bool vec4Equal(const glm::vec4& a, const glm::vec4& b) { return glm::all(glm::equal(a, b)); }

/// A fully-populated document: two figures (one rich, one at defaults), a
/// non-default environment and camera.
ProjectDocument makeDoc() {
    ProjectDocument doc;

    ProjectFigure f1;
    f1.source = path("hero.duf");
    f1.pose.emplace_back("lShldrBend", glm::vec3(-40.0f, 0.0f, 3.2f));
    f1.pose.emplace_back("rShldrBend", glm::vec3(-40.0f, 0.0f, -3.2f));
    f1.pins.push_back("lHand");
    f1.rootBone = "hip";
    f1.rootTranslation = glm::vec3(0.0f, -0.31f, 0.12f);
    f1.translation = glm::vec3(0.5f, 0.0f, -1.25f);
    f1.rotation = glm::normalize(glm::vec4(0.0f, 0.7071068f, 0.0f, 0.7071068f));
    f1.scale = glm::vec3(1.5f, 2.25f, 0.75f);
    doc.figures.push_back(f1);

    ProjectFigure f2; // minimal: every field at its default
    f2.source = path("model.obj");
    doc.figures.push_back(f2);

    doc.environment.hdri = path("studio.hdr");
    doc.environment.settings.exposure = 0.9f;
    doc.environment.settings.keyIntensity = 1.4f;
    doc.environment.settings.tonemap = false;
    doc.environment.settings.shadowsEnabled = true;

    doc.camera.target = glm::vec3(0.1f, 1.2f, -0.3f);
    doc.camera.yaw = 0.61f;
    doc.camera.pitch = -0.22f;
    doc.camera.distance = 4.25f;
    doc.camera.ortho = true;
    return doc;
}

void writeText(const std::string& path, const std::string& text) {
    std::ofstream out(path);
    out << text;
}

void checkRoundTrip() {
    const ProjectDocument in = makeDoc();
    check(pose::writeProjectFile(path("roundtrip.pss"), in), "write");

    ProjectDocument out;
    std::string error;
    if (!pose::readProjectFile(path("roundtrip.pss"), out, error)) {
        std::printf("  read error: %s\n", error.c_str());
    }
    check(pose::readProjectFile(path("roundtrip.pss"), out, error), "read round-trip");

    check(out.figures.size() == 2, "figure count");
    if (out.figures.size() != 2) {
        return;
    }
    const ProjectFigure& a = in.figures[0];
    const ProjectFigure& b = out.figures[0];
    check(a.source == b.source, "f1 source");
    check(b.pose.size() == 2 && b.pose[0].first == a.pose[0].first &&
              vec3Equal(b.pose[0].second, a.pose[0].second) &&
              b.pose[1].first == a.pose[1].first && vec3Equal(b.pose[1].second, a.pose[1].second),
          "f1 pose rows");
    check(b.pins.size() == 1 && b.pins[0] == "lHand", "f1 pins");
    check(b.rootBone == "hip" && vec3Equal(b.rootTranslation, a.rootTranslation),
          "f1 root bone + translation");
    check(vec3Equal(b.translation, a.translation), "f1 translation");
    check(vec4Equal(b.rotation, a.rotation), "f1 rotation (quaternion)");
    check(vec3Equal(b.scale, a.scale), "f1 scale");

    const ProjectFigure& c = out.figures[1];
    check(c.source == path("model.obj") && c.pose.empty() && c.pins.empty() &&
              c.rootBone.empty() && vec3Equal(c.translation, glm::vec3(0.0f)) &&
              vec3Equal(c.scale, glm::vec3(1.0f)) &&
              vec4Equal(c.rotation, glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)),
          "f2 defaults");

    check(out.environment.hdri == path("studio.hdr"), "environment hdri");
    const pose::LightingSettings& ls = out.environment.settings;
    check(ls.exposure == 0.9f && ls.keyIntensity == 1.4f && !ls.tonemap && ls.shadowsEnabled,
          "environment settings (incl. the bool dials)");

    const ProjectCamera& cam = out.camera;
    check(vec3Equal(cam.target, glm::vec3(0.1f, 1.2f, -0.3f)) && cam.yaw == 0.61f &&
              cam.pitch == -0.22f && cam.distance == 4.25f && cam.ortho,
          "camera framing");
}

void checkRejections() {
    ProjectDocument d;
    std::string error;

    writeText(path("bad.json"), "{ this is not json");
    check(!pose::readProjectFile(path("bad.json"), d, error) && !error.empty(),
          "bad JSON rejected with an error");

    writeText(path("marker.json"),
              R"({"format":"other.app","version":1,"figures":[],"environment":{"settings":{}},"camera":{}})");
    check(!pose::readProjectFile(path("marker.json"), d, error), "wrong marker rejected");

    writeText(path("v2.json"),
              R"({"format":"posestudio.project","version":2,"figures":[],"environment":{"settings":{}},"camera":{}})");
    check(!pose::readProjectFile(path("v2.json"), d, error), "higher version rejected");

    check(!pose::readProjectFile(path("nosuchfile.pss"), d, error), "missing file rejected");

    // Absent top-level sections are optional (defaults apply) — the rejection list is
    // missing file / bad JSON / wrong marker / higher version / non-finite values.
    writeText(path("bare.json"), R"({"format":"posestudio.project","version":1})");
    d = ProjectDocument();
    check(pose::readProjectFile(path("bare.json"), d, error) && d.figures.empty(),
          "absent sections fall back to defaults");
}

void checkForwardCompatibility() {
    writeText(path("future.json"),
              R"({"format":"posestudio.project","version":1,"figures":[{"source":"/x.duf",
                  "pose":[["hip",1.0,2.0,3.0]],"mysteryField":{"a":1},"pins":[]}],
                 "environment":{"hdri":"","settings":{"exposure":0.5,"futureDial":42}},
                 "camera":{"target":[0,1,0],"yaw":0.1,"pitch":0.2,"distance":3.0,"orthographic":false,
                           "fovDeg":60},"topLevelUnknown":[1,2,3]})");
    ProjectDocument d;
    std::string error;
    check(pose::readProjectFile(path("future.json"), d, error), "unknown fields ignored");
    check(d.figures.size() == 1 && d.figures[0].pose.size() == 1 &&
              vec3Equal(d.figures[0].pose[0].second, glm::vec3(1.0f, 2.0f, 3.0f)) &&
              d.environment.settings.exposure == 0.5f && d.camera.distance == 3.0f,
          "known values survive");
}

} // namespace

int main() {
    std::filesystem::create_directories(kDir);
    checkRoundTrip();
    checkRejections();
    checkForwardCompatibility();
    std::printf("\n%d failure(s)\n", failures);
    return failures;
}
