/**
 * @file morphparser.cpp
 * @brief Implementation of parseMorphDeltas. See morphparser.h.
 */

#include "morphparser.h"

#include "figureutils.h"

#include <nlohmann/json.hpp>

namespace pose {

std::vector<MorphDelta> parseMorphDeltas(const nlohmann::json& morphDoc,
                                         const std::string& fragment) {
    std::vector<MorphDelta> out;

    // The modifier whose id matches the URI fragment (and carries a morph), else the first one
    // carrying a morph at all.
    const nlohmann::json* mod = findMorphModifier(morphDoc, fragment);
    if (!mod) {
        return out; // no shape here (e.g. a formula-only driver modifier)
    }

    const nlohmann::json& morph = (*mod)["morph"];
    const auto deltas = morph.find("deltas");
    if (deltas == morph.end()) {
        return out;
    }

    const nlohmann::json& values = valuesArray(*deltas);
    out.reserve(values.size());
    for (const auto& entry : values) {
        if (entry.is_array() && entry.size() >= 4) {
            out.emplace_back(entry[0].get<uint32_t>(),
                             glm::vec3(entry[1].get<float>(), entry[2].get<float>(),
                                       entry[3].get<float>()));
        }
    }
    return out;
}

} // namespace pose
