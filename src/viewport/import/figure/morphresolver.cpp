/**
 * @file morphresolver.cpp
 * @brief Implementation of resolveDialedMorphs. See morphresolver.h.
 */

#include "morphresolver.h"

#include "figuredocument.h"
#include "figureutils.h"
#include "uriresolver.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pose {

namespace {

// Channel references here are split by figureutils' parseChannelRef. This resolver keys its value
// map by the RAW (as-written, still percent-encoded) id — `rawKey` — on both sides: the seeds come
// from the scene's modifier urls and the driven channels from formula output urls, and both are
// written the same encoded way, so raw-vs-raw compares exactly. `fileUrl` ("/path.dsf#frag", or
// "#frag" for a same-file reference) is what the resolver loads.

double valueOf(const std::unordered_map<std::string, double>& values, const std::string& key) {
    const auto it = values.find(key);
    return it != values.end() ? it->second : 0.0;
}

// One pre-compiled formula operation. The evaluation below runs up to eight fixed-point rounds
// over every reachable formula, and re-reading each op's JSON (and re-parsing each push url) per
// round was pure repetition — so a formula's operations are translated ONCE at discovery into
// this form and the rounds just walk it.
struct CompiledOp {
    enum class Kind { PushValue, PushChannel, Mult, Add, Sub, Div, Other };
    Kind        kind = Kind::Other;
    double      value = 0.0; ///< PushValue: the constant.
    std::string key;         ///< PushChannel: the channel key (rawKey) to read.
};

// Translates a formula's `operations` list. Mirrors the evaluator's old per-op reads exactly: a
// "push" carries a numeric `val`, else a string `url` (a channel reference), else pushes 0; any
// op name outside the vocabulary becomes Other (a no-op at evaluation).
std::vector<CompiledOp> compileOperations(const nlohmann::json& operations) {
    std::vector<CompiledOp> out;
    for (const auto& op : operations) {
        const std::string o = op.value("op", std::string());
        CompiledOp c;
        if (o == "push") {
            if (const auto v = op.find("val"); v != op.end() && v->is_number()) {
                c.kind = CompiledOp::Kind::PushValue;
                c.value = v->get<double>();
            } else if (const auto u = op.find("url"); u != op.end() && u->is_string()) {
                c.kind = CompiledOp::Kind::PushChannel;
                c.key = parseChannelRef(u->get<std::string>()).rawKey;
            } else {
                c.kind = CompiledOp::Kind::PushValue;
                c.value = 0.0;
            }
        } else if (o == "mult") {
            c.kind = CompiledOp::Kind::Mult;
        } else if (o == "add") {
            c.kind = CompiledOp::Kind::Add;
        } else if (o == "sub") {
            c.kind = CompiledOp::Kind::Sub;
        } else if (o == "div") {
            c.kind = CompiledOp::Kind::Div;
        }
        out.push_back(std::move(c));
    }
    return out;
}

// Evaluates a compiled formula — a small stack machine — against the current channel values.
double evalOperations(const std::vector<CompiledOp>& operations,
                      const std::unordered_map<std::string, double>& values) {
    std::vector<double> stack;
    auto pop = [&stack]() -> double {
        if (stack.empty()) {
            return 0.0;
        }
        const double v = stack.back();
        stack.pop_back();
        return v;
    };
    for (const CompiledOp& op : operations) {
        switch (op.kind) {
            case CompiledOp::Kind::PushValue:
                stack.push_back(op.value);
                break;
            case CompiledOp::Kind::PushChannel:
                stack.push_back(valueOf(values, op.key));
                break;
            case CompiledOp::Kind::Mult: {
                const double b = pop();
                stack.push_back(pop() * b);
                break;
            }
            case CompiledOp::Kind::Add:
                stack.push_back(pop() + pop());
                break;
            case CompiledOp::Kind::Sub: {
                const double b = pop();
                stack.push_back(pop() - b);
                break;
            }
            case CompiledOp::Kind::Div: {
                const double b = pop();
                stack.push_back(b != 0.0 ? pop() / b : (pop(), 0.0));
                break;
            }
            case CompiledOp::Kind::Other:
                // Rarer ops (spline, etc.) are uncommon for identity morphs and approximated as
                // no-ops (leave the stack) — a known gap of this identity-morph path only. The
                // pose-corrective evaluator (correctiveparser.cpp) implements the spline
                // vocabulary for real.
                break;
        }
    }
    return stack.empty() ? 0.0 : stack.back();
}

// One reachable formula, compiled: where its result goes plus its operation program. Formulas
// without an `operations` list are kept for their output (discovery enqueues it) but never
// evaluated, as before.
struct CompiledFormula {
    ChannelRef              output;
    bool                    hasOperations = false;
    std::vector<CompiledOp> ops;
};

// Maps a channel property's trailing axis ("center_point/x" -> 0, ".../y" -> 1, ".../z" -> 2).
int axisIndexOf(const std::string& property) {
    if (property.empty()) {
        return -1;
    }
    switch (property.back()) {
        case 'x': return 0;
        case 'y': return 1;
        case 'z': return 2;
        default:  return -1;
    }
}

} // namespace

std::vector<DialedMorph> resolveDialedMorphs(const nlohmann::json& presetRoot, UriResolver& resolver,
                                             const std::string& presetDir, float& outFigureScale,
                                             JointCenterOffsets& outJointCenterOffsets) {
    std::vector<DialedMorph> result;
    double scaleAccum = 0.0; // formula contributions to the figure's uniform scale/general
    outFigureScale = 1.0f;
    outJointCenterOffsets.clear();

    const auto scene = presetRoot.find("scene");
    if (scene == presetRoot.end() || !scene->contains("modifiers")) {
        return result;
    }

    // The figure's uniform scale only comes from formulas targeting the FIGURE node itself, so
    // identify it up front: the scene node carrying the geometry (the same criterion the importer
    // uses to locate the base mesh), keyed by its asset-url fragment — the id formula outputs
    // reference. Bones' scale channels share the same "scale/general" property name, so without
    // this the node check below has nothing to compare against.
    std::string figureNodeKey;
    if (const auto nodes = scene->find("nodes"); nodes != scene->end() && nodes->is_array()) {
        for (const auto& node : *nodes) {
            const auto geos = node.find("geometries");
            if (geos != node.end() && geos->is_array() && !geos->empty()) {
                figureNodeKey = parseChannelRef(node.value("url", std::string())).rawKey;
                break;
            }
        }
    }

    std::unordered_map<std::string, double> values;      // channel key -> accumulated value
    std::unordered_map<std::string, std::string> urlOf;  // channel key -> loadable "/file.dsf#frag"
    std::deque<std::string> queue;
    std::unordered_set<std::string> queued;

    auto enqueue = [&](const std::string& key, const std::string& url) {
        if (!url.empty() && url[0] == '/') {
            urlOf[key] = url; // only file-backed channels can be loaded (for formulas + deltas)
        }
        if (!queued.count(key)) {
            queued.insert(key);
            queue.push_back(key);
        }
    };

    // Seed from the scene's dialed modifiers.
    for (const auto& mod : (*scene)["modifiers"]) {
        const auto ch = mod.find("channel");
        if (ch == mod.end()) {
            continue;
        }
        const double cv = channelDouble(*ch, 0.0); // the DIALED value (current_value over value)
        const std::string url = mod.value("url", std::string());
        if (url.empty()) {
            continue;
        }
        const std::string key = parseChannelRef(url).rawKey;
        values[key] += cv;
        enqueue(key, url);
    }

    // A "routed" output is consumed outside channel propagation (joint centers, figure scale) or
    // deliberately dropped (end_point/orientation, per-bone scale) — see the evaluation loop for
    // the rationale on each. Routed outputs never extend the propagation graph.
    auto isRoutedOutput = [](const std::string& property) {
        return property.rfind("center_point/", 0) == 0 ||
               property.rfind("end_point/", 0) == 0 ||
               property.rfind("orientation/", 0) == 0 ||
               property.find("scale/general") != std::string::npos ||
               property.find("general_scale") != std::string::npos;
    };

    // --- Discovery: walk the driver graph from the seeds, loading each reachable file-backed
    // channel's modifier once and compiling its formula list. Reachability is value-independent
    // (a formula's outputs are enumerated whether or not it currently evaluates to zero), so
    // this graph is fixed before any value is computed. The visited set bounds the walk. The
    // compiled formulas own everything the rounds need, so the documents needn't be kept.
    std::vector<std::vector<CompiledFormula>> loaded; // one formula list per reached modifier
    std::unordered_set<std::string> processed;
    while (!queue.empty()) {
        const std::string key = queue.front();
        queue.pop_front();
        if (processed.count(key)) {
            continue;
        }
        processed.insert(key);

        const auto uit = urlOf.find(key);
        if (uit == urlOf.end()) {
            continue; // same-file-only reference, nothing to load
        }
        std::shared_ptr<const FigureDocument> doc;
        try {
            doc = resolver.loadDocument(uit->second, presetDir);
        } catch (...) {
            continue;
        }
        const nlohmann::json* mod = findModifierById(doc->root(), parseChannelRef(uit->second).rawKey);
        if (!mod) {
            continue;
        }
        const auto formulas = mod->find("formulas");
        if (formulas == mod->end() || !formulas->is_array()) {
            continue;
        }
        std::vector<CompiledFormula> compiled;
        compiled.reserve(formulas->size());
        for (const auto& formula : *formulas) {
            CompiledFormula cf;
            cf.output = parseChannelRef(formula.value("output", std::string()));
            if (const auto ops = formula.find("operations"); ops != formula.end()) {
                cf.hasOperations = true;
                cf.ops = compileOperations(*ops);
            }
            if (!cf.output.rawKey.empty() && !isRoutedOutput(cf.output.property)) {
                enqueue(cf.output.rawKey, cf.output.fileUrl);
            }
            compiled.push_back(std::move(cf));
        }
        loaded.push_back(std::move(compiled));
    }

    // --- Evaluation, iterated to a fixed point. Discovery order is a BFS, which is NOT a
    // topological order: a channel can be reached before all of its own driver inputs have
    // accumulated, and since formula contributions are additive, a single sweep could bake a
    // partial input into a downstream morph. Each round therefore re-derives every driven value
    // from the seeds plus the previous round's values; for a DAG this converges in at most
    // graph-depth rounds (real driver graphs — character CTRL → head/body controls → component
    // morphs — are 2-3 deep, so the cap is generous and cycles still terminate).
    const std::unordered_map<std::string, double> seeds = values;
    for (int round = 0; round < 8; ++round) {
        std::unordered_map<std::string, double> next = seeds;
        double roundScale = 0.0;
        JointCenterOffsets roundOffsets;

        for (const std::vector<CompiledFormula>& formulas : loaded) {
            for (const CompiledFormula& formula : formulas) {
                if (!formula.hasOperations) {
                    continue;
                }
                const ChannelRef& out = formula.output;
                if (out.rawKey.empty()) {
                    continue;
                }
                const double r = evalOperations(formula.ops, values); // the formula's default stage is additive

                // Joint-center adjustment: a full-body/head morph also drives each bone's center_point
                // (the joint's rest origin) so the skeleton follows the character's new proportions.
                // Route these to the bone-offset map — a bone is not a morph channel to propagate.
                if (out.property.rfind("center_point/", 0) == 0) {
                    if (const int a = axisIndexOf(out.property); a >= 0) {
                        roundOffsets[out.rawKey][a] += static_cast<float>(r);
                    }
                    continue;
                }
                // end_point/orientation don't affect our translation-only bind (orientation is read
                // straight from the node), so skip them rather than let them masquerade as channels.
                if (out.property.rfind("end_point/", 0) == 0 ||
                    out.property.rfind("orientation/", 0) == 0) {
                    continue;
                }

                // Scale outputs are routed out of channel propagation (like center_point above): a
                // node's scale channel is not a morph channel. Only the FIGURE node's scale/general
                // sets character height (e.g. a teen figure dialed shorter). A bone-targeted output
                // is a propagating-scale rig instead — a head-scale control drives ~80 per-bone
                // scale/general channels — and summing those into the figure scale imported one teen
                // character at 3x size. Per-bone scale isn't modeled (the bind is translation-only),
                // so those outputs are dropped here.
                if (out.property.find("scale/general") != std::string::npos ||
                    out.property.find("general_scale") != std::string::npos) {
                    if (out.rawKey == figureNodeKey) {
                        roundScale += r;
                    }
                    continue;
                }

                next[out.rawKey] += r;
            }
        }

        scaleAccum = roundScale;
        outJointCenterOffsets = std::move(roundOffsets);

        // Converged once no channel's value moved since the previous round.
        bool stable = next.size() == values.size();
        if (stable) {
            for (const auto& [k, v] : next) {
                const auto it = values.find(k);
                if (it == values.end() || std::abs(it->second - v) > 1e-9) {
                    stable = false;
                    break;
                }
            }
        }
        values = std::move(next);
        if (stable) {
            break;
        }
    }
    outFigureScale = static_cast<float>(1.0 + scaleAccum);

    // Emit every reached, file-backed channel with a meaningful weight; the caller loads each and
    // applies its deltas (controls/scales resolve to no deltas and thus contribute nothing).
    for (const auto& [key, value] : values) {
        if (std::abs(value) < 1e-4) {
            continue;
        }
        if (const auto uit = urlOf.find(key); uit != urlOf.end()) {
            result.push_back({uit->second, static_cast<float>(value)});
        }
    }
    return result;
}

} // namespace pose
