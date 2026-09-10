/**
 * @file correctivescan.cpp
 * @brief Implementation of discoverCorrectives. See correctivescan.h.
 */

#include "correctivescan.h"

#include "correctiveparser.h"
#include "figuredocument.h"
#include "figureutils.h"
#include "parallelfor.h"
#include "uriresolver.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string_view>
#include <thread>
#include <unordered_map>

namespace pose {

namespace {

namespace fs = std::filesystem;

// Appends every directory under @p morphsRoot (recursively) whose name marks it as a corrective /
// flexion pack. Directory NAMES are the one naming convention relied on, and only as a bound on
// what to inflate — the modifiers inside are still confirmed structurally by parseCorrective.
void collectCorrectiveDirs(const fs::path& morphsRoot, std::vector<std::string>& dirs) {
    std::error_code ec;
    if (!fs::exists(morphsRoot, ec)) {
        return;
    }
    for (fs::recursive_directory_iterator it(morphsRoot, ec), end; it != end; it.increment(ec)) {
        if (ec) {
            break;
        }
        if (!it->is_directory(ec)) {
            continue;
        }
        const std::string name = toLowerAscii(it->path().filename().string());
        if (name.find("corrective") != std::string::npos || name.find("flexion") != std::string::npos) {
            dirs.push_back(it->path().string());
        }
    }
}

// SIBLING-BASE fallback: some figure generations ship NO corrective packs of their own — a
// "point-release" variant figure (its Morphs tree holds only head/FACS content) inherits the
// previous generation's correctives through the source app's content database. That link is
// re-created STRUCTURALLY here: a sibling figure directory (same parent as @p baseDir) whose
// base file's digit-stripped name matches ours (a "…8_1Female" and its "…8Female" sibling both
// strip to the same stem — the male sibling does not) AND whose base cage has the SAME
// vertex count is morph-compatible by construction — identical topology means its corrective
// delta indices land on our vertices verbatim, and the shared bone names drive them. The check
// parses the sibling's base file (~a second), which is why the caller only attempts it when the
// figure's own tree yielded nothing. Appends the first matching sibling's pack directories.
void siblingBaseMorphsDirs(const std::string& baseDir, const std::string& baseGeometryFile,
                           std::size_t cageVertexCount, std::vector<std::string>& dirs) {
    const auto strippedStem = [](const fs::path& p) {
        std::string out;
        for (const char c : p.stem().string()) {
            if (!std::isdigit(static_cast<unsigned char>(c)) && c != '_' && c != ' ' &&
                c != '.') {
                out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
            }
        }
        return out;
    };
    std::error_code ec;
    const std::string ownStem = strippedStem(fs::path(baseGeometryFile));
    const fs::path parent = fs::path(baseDir).parent_path();
    bool matched = false;
    for (fs::directory_iterator sib(parent, ec), sibEnd; !matched && sib != sibEnd;
         sib.increment(ec)) {
        if (ec || !sib->is_directory(ec) || sib->path() == fs::path(baseDir)) {
            continue;
        }
        for (fs::directory_iterator f(sib->path(), ec), fEnd; !matched && f != fEnd;
             f.increment(ec)) {
            // Extension compared case-insensitively: a pack saved as ".DSF" is the same file.
            if (ec || !f->is_regular_file(ec) ||
                toLowerAscii(f->path().extension().string()) != ".dsf" ||
                strippedStem(f->path()) != ownStem) {
                continue;
            }
            try {
                const FigureDocument sibDoc = FigureDocument::loadFromFile(f->path().string());
                if (cageVertexCount == 0 || geometryVertexCount(sibDoc.root()) != cageVertexCount) {
                    continue; // different topology: its deltas would land on wrong vertices
                }
            } catch (const std::exception&) {
                continue;
            }
            matched = true;
            collectCorrectiveDirs(sib->path() / "Morphs", dirs);
        }
    }
}

} // namespace

std::vector<PoseCorrective> discoverCorrectives(UriResolver& resolver, const std::string& baseDir,
                                                const std::unordered_set<std::string>& boneNames,
                                                const std::vector<DialedMorph>& dialed,
                                                const std::string& baseGeometryFile,
                                                std::size_t cageVertexCount) {
    // Gate resolver: a value-channel driver resolves to its dialed weight if the preset dials it,
    // else to the channel's own default (loaded once and cached). This makes base correctives (gated
    // by a default-on toggle) fire and a non-loaded character's correctives (gated by that character's
    // shape morph, default 0) drop — uniformly, without any figure-specific knowledge.
    //
    // Ids are compared URL-DECODED: vendor morph ids commonly contain spaces
    // ("Some%20Character%20Body%201") and a corrective's gate url must match the dialed morph's
    // id, which arrives decoded — an encoded mismatch silently resolved the gate to the channel's
    // default (0 for a character shape) and dropped every one of that character's own correctives.
    std::unordered_map<std::string, float> dialedWeights;
    for (const DialedMorph& d : dialed) {
        dialedWeights[parseChannelRef(d.url).decodedKey] = d.weight;
    }
    // Keyed by channel id ALONE (not id + document): a gate id names one channel across the
    // figure's files in practice, so the first default found stands for every later reference.
    std::unordered_map<std::string, float> gateCache;

    CorrectiveContext ctx;
    ctx.boneNames = &boneNames;
    ctx.resolveValue = [&](const std::string& url) -> float {
        const ChannelRef ref = parseChannelRef(url);
        const std::string& id = ref.decodedKey;
        if (const auto it = dialedWeights.find(id); it != dialedWeights.end()) {
            return it->second;
        }
        if (const auto it = gateCache.find(id); it != gateCache.end()) {
            return it->second;
        }
        float def = 0.0f;
        try {
            // A same-file gate ("#Toggle?value", no file part) names a channel in the document
            // being parsed — the resolver has nothing to load, so read its default from the owner
            // (ctx.ownerDoc, set per document below). Any other gate loads its file (cached).
            std::shared_ptr<const FigureDocument> loadedDoc; // keeps a cross-file gate's document alive
            const nlohmann::json* gateRoot = nullptr;
            if (ref.fileUrl.empty() || ref.fileUrl[0] == '#') {
                gateRoot = ctx.ownerDoc;
            } else {
                loadedDoc = resolver.loadDocument(ref.fileUrl, baseDir);
                gateRoot = &loadedDoc->root();
            }
            if (gateRoot) {
                if (const nlohmann::json* m = findModifierById(*gateRoot, id)) {
                    if (const auto ch = m->find("channel"); ch != m->end()) {
                        def = ch->value("value", 0.0f);
                    }
                }
            }
        } catch (const std::exception&) {
            def = 0.0f;
        }
        gateCache[id] = def;
        return def;
    };

    // Candidate directories: corrective/flexion packs under <baseDir>/Morphs, plus each dialed morph's
    // own directory.
    std::vector<std::string> dirs;
    std::error_code ec;
    collectCorrectiveDirs(fs::path(baseDir) / "Morphs", dirs);

    // Only when our own tree yielded nothing: a topology-compatible sibling figure's packs (the
    // check parses the sibling's base file — ~a second — so same-generation figures with their
    // own packs never pay it).
    if (dirs.empty()) {
        siblingBaseMorphsDirs(baseDir, baseGeometryFile, cageVertexCount, dirs);
    }

    for (const DialedMorph& d : dialed) {
        const ResolvedUri ru = resolver.resolve(d.url, baseDir);
        if (ru.resolved()) {
            dirs.push_back(fs::path(ru.path).parent_path().string());
        }
    }
    std::sort(dirs.begin(), dirs.end());
    dirs.erase(std::unique(dirs.begin(), dirs.end()), dirs.end());

    // Gather the candidate files first, so the expensive part — file read + gzip inflate + JSON
    // parse, across hundreds of morph files — can run on all cores. The modifier walk stays serial:
    // parseCorrective itself is cheap, and its gate resolution goes through the resolver/caches
    // above, which aren't thread-safe.
    std::vector<std::string> files;
    for (const std::string& dir : dirs) {
        for (fs::directory_iterator it(dir, ec), end; it != end; it.increment(ec)) {
            if (ec) {
                break;
            }
            if (!it->is_regular_file(ec) || toLowerAscii(it->path().extension().string()) != ".dsf") {
                continue;
            }
            files.push_back(it->path().string());
        }
    }

    std::vector<PoseCorrective> correctives;
    std::unordered_set<std::string> seenIds;
    // Bounded chunks keep peak memory at ~a couple of parsed documents per core rather than the
    // whole candidate set at once (a corrective pack's parsed JSON is megabytes each).
    const unsigned hw = std::thread::hardware_concurrency();
    const std::size_t chunk = std::max<std::size_t>(1, static_cast<std::size_t>(hw > 1 ? hw : 1) * 2);
    std::vector<std::shared_ptr<const FigureDocument>> docs;
    for (std::size_t base = 0; base < files.size(); base += chunk) {
        const std::size_t n = std::min(chunk, files.size() - base);
        docs.clear();
        docs.resize(n);
        // The dialed morphs' own folders are among the candidate dirs, and every dialed morph
        // document was just prefetched into the resolver's cache — reuse those instead of
        // re-reading and re-inflating them. (Serial: the cache isn't touched from the workers.)
        for (std::size_t i = 0; i < n; ++i) {
            docs[i] = resolver.cached(files[base + i]);
        }
        parallelFor(static_cast<int>(n), [&](int i) {
            if (docs[static_cast<std::size_t>(i)]) {
                return; // already parsed by the prefetch
            }
            try {
                const std::string& file = files[base + static_cast<std::size_t>(i)];
                // Pre-filter before the (expensive) DOM parse: a corrective's driver formula always
                // references a joint rotation channel as the literal text "?rotation/" (the format
                // writes these urls unencoded — every parser here splits on the raw '?'). The
                // candidate dirs include each dialed morph's own folder, which can hold hundreds of
                // plain shape morphs; inflating is cheap, but DOM-parsing them all dominated the
                // corrective scan, and none can ever pass parseCorrective. (A cached document
                // skips the filter — parseCorrective rejects it just the same, only slower.)
                std::ifstream in(file, std::ios::binary | std::ios::ate);
                if (!in) {
                    return;
                }
                const std::streamoff size = in.tellg();
                if (size <= 0) {
                    return;
                }
                std::vector<uint8_t> bytes(static_cast<std::size_t>(size));
                in.seekg(0, std::ios::beg);
                if (!in.read(reinterpret_cast<char*>(bytes.data()), size)) {
                    return;
                }
                if (isGzip(bytes)) {
                    bytes = gunzip(bytes.data(), bytes.size());
                }
                const std::string_view text(reinterpret_cast<const char*>(bytes.data()),
                                            bytes.size());
                if (text.find("?rotation/") == std::string_view::npos) {
                    return; // no joint-rotation driver anywhere — cannot be a corrective
                }
                docs[static_cast<std::size_t>(i)] = std::make_shared<FigureDocument>(
                    FigureDocument::loadFromBytes(std::move(bytes), file));
            } catch (const std::exception&) {
                // unreadable/!gzip/!json — left null, skipped below
            }
        });
        for (const std::shared_ptr<const FigureDocument>& doc : docs) {
            if (!doc) {
                continue;
            }
            const auto ml = doc->root().find("modifier_library");
            if (ml == doc->root().end() || !ml->is_array()) {
                continue;
            }
            ctx.ownerDoc = &doc->root(); // for same-file gate references
            for (const auto& mod : *ml) {
                PoseCorrective pc;
                try {
                    if (parseCorrective(mod, ctx, pc) && seenIds.insert(pc.id).second) {
                        correctives.push_back(std::move(pc));
                    }
                } catch (const std::exception&) {
                    // A malformed modifier (non-numeric knot/delta, a numeric "clamped", string
                    // limits) throws out of the JSON accessors — skip just that modifier, matching
                    // the tolerance of the parallel load above.
                    continue;
                }
            }
        }
    }
    ctx.ownerDoc = nullptr;
    return correctives;
}

} // namespace pose
