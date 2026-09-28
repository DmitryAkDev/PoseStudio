/**
 * @file projectfile.cpp
 * @brief The `.pss` project file reader/writer. See projectfile.h.
 */

#include "projectfile.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>

namespace pose {

namespace {

using nlohmann::json;

/// The document's format marker and the highest version this build reads (a higher one is
/// rejected: we don't guess what a newer file means).
constexpr const char* kFormatMarker = "posestudio.project";
constexpr int         kVersion      = 1;

/// Wraps an angle into [-360, 360] (fmod keeps the sign) — the posefile.h convention, so a
/// hand-edited file can't feed an absurd angle to the limit clamp downstream.
float wrapDegrees(float v) { return std::fmod(v, 360.0f); }

/// Reads @p j's [x, y, z] into @p out. False (leaving @p out untouched) if it isn't a three-number
/// array or any value is non-finite.
bool readVec3(const json& j, glm::vec3& out) {
    if (!j.is_array() || j.size() != 3) {
        return false;
    }
    float v[3];
    for (int i = 0; i < 3; ++i) {
        if (!j[static_cast<std::size_t>(i)].is_number()) {
            return false;
        }
        v[i] = j[static_cast<std::size_t>(i)].get<float>();
    }
    if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2])) {
        return false;
    }
    out = glm::vec3(v[0], v[1], v[2]);
    return true;
}

/// Reads @p j's [x, y, z] into @p out, wrapping each channel into [-360, 360].
bool readVec3Degrees(const json& j, glm::vec3& out) {
    if (!readVec3(j, out)) {
        return false;
    }
    out = glm::vec3(wrapDegrees(out.x), wrapDegrees(out.y), wrapDegrees(out.z));
    return true;
}

/// Reads one lighting dial by field name. Absent keeps the struct's default (a file written by an
/// older build simply lacks the new dial); present but non-numeric or non-finite rejects the file.
template <class T>
bool readSetting(const json& j, const char* field, T& out) {
    const auto it = j.find(field);
    if (it == j.end()) {
        return true;
    }
    if (!it->is_number()) {
        return false;
    }
    const double v = it->get<double>();
    if (!std::isfinite(v)) {
        return false;
    }
    out = static_cast<T>(v);
    return true;
}

/// The bool analogue (a JSON boolean, not a number).
bool readBoolSetting(const json& j, const char* field, bool& out) {
    const auto it = j.find(field);
    if (it == j.end()) {
        return true;
    }
    if (!it->is_boolean()) {
        return false;
    }
    out = it->get<bool>();
    return true;
}

/// The environment's `settings` object: every LightingSettings field by name (absent = default).
bool readLightingSettings(const json& j, LightingSettings& s) {
    if (!j.is_object()) {
        return false;
    }
    return readSetting(j, "exposure",               s.exposure) &&
           readSetting(j, "diffuseIntensity",       s.diffuseIntensity) &&
           readSetting(j, "specularIntensity",      s.specularIntensity) &&
           readSetting(j, "ambientFill",            s.ambientFill) &&
           readSetting(j, "keyIntensity",           s.keyIntensity) &&
           readSetting(j, "environmentRotationDeg", s.environmentRotationDeg) &&
           readSetting(j, "keyAzimuthDeg",          s.keyAzimuthDeg) &&
           readSetting(j, "keyElevationDeg",        s.keyElevationDeg) &&
           readSetting(j, "subsurface",             s.subsurface) &&
           readSetting(j, "rimIntensity",           s.rimIntensity) &&
           readBoolSetting(j, "tonemap",            s.tonemap) &&
           readSetting(j, "backdropMode",           s.backdropMode) &&
           readSetting(j, "backdropBlur",           s.backdropBlur) &&
           readSetting(j, "backdropBrightness",     s.backdropBrightness) &&
           readSetting(j, "domeRadius",             s.domeRadius) &&
           readSetting(j, "shadowsEnabled",         s.shadowsEnabled) &&
           readSetting(j, "shadowIntensity",        s.shadowIntensity) &&
           readSetting(j, "shadowSoftness",         s.shadowSoftness) &&
           readSetting(j, "shadowReach",            s.shadowReach);
}

/// One figure entry: `source` (a non-empty string), `pose` rows of [name, rx, ry, rz], `pins`
/// names, `rootTranslation`, and the `transform` TRS. A malformed row rejects the WHOLE file.
bool readFigure(const json& j, ProjectFigure& fig) {
    if (!j.is_object()) {
        return false;
    }
    const auto sourceIt = j.find("source");
    if (sourceIt == j.end() || !sourceIt->is_string() || sourceIt->get<std::string>().empty()) {
        return false;
    }
    fig.source = sourceIt->get<std::string>();

    const auto poseIt = j.find("pose");
    if (poseIt != j.end()) {
        if (!poseIt->is_array()) {
            return false;
        }
        for (const json& row : *poseIt) {
            if (!row.is_array() || row.size() != 4 || !row[0].is_string()) {
                return false;
            }
            glm::vec3 v;
            if (!readVec3Degrees(row, v)) {
                return false;
            }
            fig.pose.emplace_back(row[0].get<std::string>(), v);
        }
    }

    const auto pinsIt = j.find("pins");
    if (pinsIt != j.end()) {
        if (!pinsIt->is_array()) {
            return false;
        }
        for (const json& name : *pinsIt) {
            if (!name.is_string()) {
                return false;
            }
            fig.pins.push_back(name.get<std::string>());
        }
    }

    const auto transIt = j.find("rootTranslation");
    if (transIt != j.end() && !readVec3(*transIt, fig.rootTranslation)) {
        return false;
    }

    const auto transformIt = j.find("transform");
    if (transformIt != j.end()) {
        if (!transformIt->is_object()) {
            return false;
        }
        const json& t = *transformIt;
        const auto tr = t.find("translation");
        if (tr != t.end() && !readVec3(*tr, fig.translation)) {
            return false;
        }
        const auto ro = t.find("rotation");
        if (ro != t.end() && !readVec3Degrees(*ro, fig.rotation)) {
            return false;
        }
        const auto sc = t.find("scale");
        if (sc != t.end() && !readVec3(*sc, fig.scale)) {
            return false;
        }
    }

    return true;
}

/// The `environment` object: `hdri` path + the `settings` dials.
bool readEnvironment(const json& j, ProjectEnvironment& env) {
    if (!j.is_object()) {
        return false;
    }
    const auto hdriIt = j.find("hdri");
    if (hdriIt != j.end()) {
        if (!hdriIt->is_string()) {
            return false;
        }
        env.hdri = hdriIt->get<std::string>();
    }
    const auto settingsIt = j.find("settings");
    if (settingsIt != j.end() && !readLightingSettings(*settingsIt, env.settings)) {
        return false;
    }
    return true;
}

/// Reads one camera scalar by field name. Absent keeps the struct's default; present but
/// non-numeric or non-finite rejects the file.
bool readCameraScalar(const json& j, const char* field, float& out) {
    const auto it = j.find(field);
    if (it == j.end()) {
        return true;
    }
    if (!it->is_number() || !std::isfinite(it->get<double>())) {
        return false;
    }
    out = it->get<float>();
    return true;
}

/// The `camera` object: target + yaw/pitch/distance/ortho.
bool readCamera(const json& j, ProjectCamera& cam) {
    if (!j.is_object()) {
        return false;
    }
    const auto targetIt = j.find("target");
    if (targetIt != j.end() && !readVec3(*targetIt, cam.target)) {
        return false;
    }
    return readCameraScalar(j, "yaw", cam.yaw) &&
           readCameraScalar(j, "pitch", cam.pitch) &&
           readCameraScalar(j, "distance", cam.distance) &&
           readBoolSetting(j, "ortho", cam.ortho);
}

} // namespace

bool writeProjectFile(const std::string& path, const ProjectDocument& doc) {
    json root;
    root["format"]  = kFormatMarker;
    root["version"] = kVersion;

    json figures = json::array();
    for (const ProjectFigure& fig : doc.figures) {
        json entry;
        entry["source"] = fig.source;

        json pose = json::array();
        for (const auto& [name, value] : fig.pose) {
            pose.push_back(json::array({name, value.x, value.y, value.z}));
        }
        entry["pose"] = std::move(pose);

        json pins = json::array();
        for (const std::string& name : fig.pins) {
            pins.push_back(name);
        }
        entry["pins"] = std::move(pins);

        entry["rootTranslation"] = json::array({fig.rootTranslation.x, fig.rootTranslation.y,
                                                fig.rootTranslation.z});
        entry["transform"] = {
            {"translation", json::array({fig.translation.x, fig.translation.y, fig.translation.z})},
            {"rotation",    json::array({fig.rotation.x, fig.rotation.y, fig.rotation.z})},
            {"scale",       json::array({fig.scale.x, fig.scale.y, fig.scale.z})},
        };
        figures.push_back(std::move(entry));
    }
    root["figures"] = std::move(figures);

    const LightingSettings& s = doc.environment.settings;
    root["environment"] = {
        {"hdri", doc.environment.hdri},
        {"settings",
         {{"exposure",               s.exposure},
          {"diffuseIntensity",       s.diffuseIntensity},
          {"specularIntensity",      s.specularIntensity},
          {"ambientFill",            s.ambientFill},
          {"keyIntensity",           s.keyIntensity},
          {"environmentRotationDeg", s.environmentRotationDeg},
          {"keyAzimuthDeg",          s.keyAzimuthDeg},
          {"keyElevationDeg",        s.keyElevationDeg},
          {"subsurface",             s.subsurface},
          {"rimIntensity",           s.rimIntensity},
          {"tonemap",                s.tonemap},
          {"backdropMode",           s.backdropMode},
          {"backdropBlur",           s.backdropBlur},
          {"backdropBrightness",     s.backdropBrightness},
          {"domeRadius",             s.domeRadius},
          {"shadowsEnabled",         s.shadowsEnabled},
          {"shadowIntensity",        s.shadowIntensity},
          {"shadowSoftness",         s.shadowSoftness},
          {"shadowReach",            s.shadowReach}}},
    };

    const ProjectCamera& cam = doc.camera;
    root["camera"] = {
        {"target",   json::array({cam.target.x, cam.target.y, cam.target.z})},
        {"yaw",      cam.yaw},
        {"pitch",    cam.pitch},
        {"distance", cam.distance},
        {"ortho",    cam.ortho},
    };

    std::ofstream out(path);
    if (!out) {
        return false;
    }
    out << root.dump(2) << '\n';
    return static_cast<bool>(out);
}

bool readProjectFile(const std::string& path, ProjectDocument& doc, std::string& error) {
    doc = ProjectDocument{};

    std::ifstream in(path);
    if (!in) {
        error = "file not found: " + path;
        return false;
    }
    json root;
    try {
        in >> root;
    } catch (const json::exception& e) {
        error = std::string("not valid JSON: ") + e.what();
        return false;
    }
    if (!root.is_object()) {
        error = "not a project document (top level is not an object)";
        return false;
    }

    const auto formatIt = root.find("format");
    if (formatIt == root.end() || !formatIt->is_string() ||
        formatIt->get<std::string>() != kFormatMarker) {
        error = "not a PoseStudio project file (missing or wrong \"format\" marker)";
        return false;
    }
    const auto versionIt = root.find("version");
    if (versionIt == root.end() || !versionIt->is_number_integer() ||
        versionIt->get<int>() > kVersion) {
        error = "unsupported project file version (this build reads up to " +
                std::to_string(kVersion) + ")";
        return false;
    }

    const auto figuresIt = root.find("figures");
    if (figuresIt != root.end()) {
        if (!figuresIt->is_array()) {
            error = "malformed document: \"figures\" is not an array";
            return false;
        }
        for (const json& entry : *figuresIt) {
            ProjectFigure fig;
            if (!readFigure(entry, fig)) {
                error = "malformed figure entry";
                return false;
            }
            doc.figures.push_back(std::move(fig));
        }
    }

    const auto environmentIt = root.find("environment");
    if (environmentIt != root.end() && !readEnvironment(*environmentIt, doc.environment)) {
        error = "malformed document: \"environment\" is not an object";
        return false;
    }
    const auto cameraIt = root.find("camera");
    if (cameraIt != root.end() && !readCamera(*cameraIt, doc.camera)) {
        error = "malformed document: \"camera\" is not an object";
        return false;
    }

    return true;
}

} // namespace pose
