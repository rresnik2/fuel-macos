#include "ui/tsc.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace fuel::ui {
namespace {

std::string lower(std::string s) {
    for (char& c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

}  // namespace

// 0x7c2890 (PK10): scanning starts after the last '"' of the line; every %<digits> there with 1 <= N <= the frame's
// parameter count is replaced by parameter N. The result holds only the text from that point on (what precedes
// the last quote is dropped, as in the game). A '%' without digits makes the whole line run unexpanded (returns
// false), as does a line without '%'. The game loops forever on an out-of-range %N (0x7c2890 re-finds the same
// '%'); the port keeps the text "%N" instead.
bool expandScriptParams(const std::string& line, const std::vector<std::string>& params, std::string* out) {
    const size_t q = line.rfind('"');
    size_t at = q == std::string::npos ? 0 : q + 1;
    size_t pct = line.find('%', at);
    if (pct == std::string::npos) return false;
    std::string res;
    for (;;) {
        res.append(line, at, pct - at);
        size_t d = pct + 1;
        while (d < line.size() && line[d] >= '0' && line[d] <= '9') ++d;
        if (d == pct + 1) return false;
        const long n = std::atol(line.substr(pct + 1, d - pct - 1).c_str());
        if (n > 0 && size_t(n) <= params.size()) {
            res += params[size_t(n - 1)];
            at = d;
        } else {
            res.append(line, pct, d - pct);  // out of range: the game hangs here
            at = d;
        }
        pct = line.find('%', at);
        if (pct == std::string::npos) {
            res.append(line, at, std::string::npos);
            *out = res;
            return true;
        }
    }
}

std::string findPathNoCase(const std::string& root, const std::string& rel) {
    std::string r = rel;
    std::replace(r.begin(), r.end(), '\\', '/');
    fs::path cur(root);
    std::stringstream ss(r);
    std::string part;
    while (std::getline(ss, part, '/')) {
        if (part.empty() || part == ".") continue;
        std::error_code ec;
        if (fs::exists(cur / part, ec)) {
            cur /= part;
            continue;
        }
        bool found = false;
        for (const auto& e : fs::directory_iterator(cur, ec)) {
            if (lower(e.path().filename().string()) == lower(part)) {
                cur = e.path();
                found = true;
                break;
            }
        }
        if (!found) return {};
    }
    return cur.string();
}

std::vector<std::string> splitTscLine(const std::string& line, bool* inBlock) {
    std::vector<std::string> out;
    std::string cur;
    bool quoted = false, have = false;
    for (size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (*inBlock) {
            if (c == '*' && i + 1 < line.size() && line[i + 1] == '/') *inBlock = false, ++i;
            continue;
        }
        if (quoted) {
            if (c == '"') quoted = false;
            else cur += c;
            continue;
        }
        if (c == '"') {
            quoted = have = true;
            continue;
        }
        if (c == '/' && i + 1 < line.size() && line[i + 1] == '/') break;
        if (c == '/' && i + 1 < line.size() && line[i + 1] == '*') {
            *inBlock = true;
            ++i;
            continue;
        }
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            if (have) out.push_back(cur);
            cur.clear();
            have = false;
            continue;
        }
        cur += c;
        have = true;
    }
    if (have) out.push_back(cur);
    return out;
}

void TscReader::run(const std::string& rel, const std::vector<std::string>& params, const std::function<void(const TscLine&)>& fn) {
    const std::string path = findPathNoCase(root_, rel);
    if (path.empty()) {
        problems.push_back("script not found: " + rel);
        return;
    }
    runFile(path, params, fn, 0);
}

void TscReader::runFile(const std::string& path, const std::vector<std::string>& params, const std::function<void(const TscLine&)>& fn, int depth) {
    if (depth > 16) {
        problems.push_back("BSource nesting too deep at " + path);
        return;
    }
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        problems.push_back("cannot read " + path);
        return;
    }
    std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    std::vector<bool> active{true};  // #if stack
    std::vector<bool> taken{true};
    bool inBlock = false;
    std::stringstream ss(text);
    std::string raw;
    int lineNo = 0;
    while (std::getline(ss, raw)) {
        ++lineNo;
        // %N expansion (PK10, 0x7c2890): see expandScriptParams.
        std::string line = raw;
        if (!params.empty()) {
            const std::string body = raw.substr(0, raw.find("//"));  // 0x7c2340 cuts the comment first
            std::string out;
            if (expandScriptParams(body, params, &out)) line = out;
        }
        std::vector<std::string> a = splitTscLine(line, &inBlock);
        if (a.empty()) continue;
        const std::string cmd = lower(a[0]);
        if (cmd[0] == '#') {
            auto defined = [&](const std::string& s) { return defines.count(s) > 0; };
            if (cmd == "#set" && a.size() > 1) {
                if (active.back()) defines.insert(a[1]);
            } else if (cmd == "#ifdef" || cmd == "#ifndef" || cmd == "#if") {
                bool v = false;
                if (cmd == "#if") {
                    for (size_t k = 1; k < a.size(); ++k)
                        if (a[k] != "||" && defined(a[k])) v = true;
                } else {
                    v = a.size() > 1 && defined(a[1]);
                    if (cmd == "#ifndef") v = !v;
                }
                const bool parent = active.back();
                active.push_back(parent && v);
                taken.push_back(v);
            } else if (cmd == "#else" && active.size() > 1) {
                const bool parent = active[active.size() - 2];
                active.back() = parent && !taken.back();
                taken.back() = true;
            } else if (cmd == "#endif" && active.size() > 1) {
                active.pop_back();
                taken.pop_back();
            }
            continue;
        }
        if (!active.back()) continue;
        TscLine tl{a, path, lineNo};
        fn(tl);
        if (cmd == "bsource" && a.size() > 1) {
            const std::string inc = findPathNoCase(root_, a[1]);
            if (inc.empty()) {
                problems.push_back(path + ":" + std::to_string(lineNo) + ": BSource not found: " + a[1]);
                continue;
            }
            // 0x7c1fb0: the frame holds 32 parameter slots (0x40 bytes each, +0x29c).
            std::vector<std::string> pp(a.begin() + 2, a.end());
            if (pp.size() > 32) pp.resize(32);
            runFile(inc, pp, fn, depth + 1);
        }
    }
}

}  // namespace fuel::ui
