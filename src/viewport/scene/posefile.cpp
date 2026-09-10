/**
 * @file posefile.cpp
 * @brief The `.pose` file reader/writer. See posefile.h.
 */

#include "posefile.h"

#include <cmath>
#include <fstream>

namespace pose {

bool writePoseFile(const std::string& path, const PoseRows& rows) {
    std::ofstream out(path);
    if (!out) {
        return false;
    }
    // One line per row: "name x y z" — a posed joint's Euler degrees, or an @trans:/@pin: row
    // (see the header).
    for (const auto& [name, value] : rows) {
        out << name << ' ' << value.x << ' ' << value.y << ' ' << value.z << '\n';
    }
    return static_cast<bool>(out);
}

bool readPoseFile(const std::string& path, PoseRows& rows) {
    rows.clear();
    std::ifstream in(path);
    if (!in) {
        return false;
    }
    std::string name;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    while (in >> name >> x >> y >> z) {
        glm::vec3 value(x, y, z);
        if (!std::isfinite(value.x + value.y + value.z)) {
            rows.clear();
            return false;
        }
        if (name.empty() || name[0] != '@') {
            // A rotation row: wrap into [-360, 360] (fmod keeps the sign) — the per-axis limit
            // clamp downstream then sees a sane angle whatever the file said.
            value = glm::vec3(std::fmod(value.x, 360.0f), std::fmod(value.y, 360.0f),
                              std::fmod(value.z, 360.0f));
        }
        rows.emplace_back(name, value);
    }
    // The loop stops on the first extraction failure; only a clean end-of-file is a complete
    // read. A garbled row anywhere rejects the file instead of silently applying its prefix.
    if (!in.eof()) {
        rows.clear();
        return false;
    }
    return true;
}

} // namespace pose
