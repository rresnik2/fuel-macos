// Reader for FUEL's console scripts (.tsc, trtext/*.pc): one command per line,
// C and C++ comments, quoted arguments, #set/#if/#ifdef/#ifndef/#else/#endif and
// `BSource FILE ARGS...` includes whose %1.. are replaced by ARGS (see
// ~/re-work/fuel/notes/p4-side-formats.md section 4). Paths use backslashes and
// any case; they resolve against the LocalizedComponent folder.
#pragma once

#include <functional>
#include <set>
#include <string>
#include <vector>

namespace fuel::ui {

struct TscLine {
    std::vector<std::string> args;  // args[0] is the command
    std::string file;
    int line = 0;
};

// `rel` (backslashes or slashes, any case) under `root`; empty when nothing matches.
std::string findPathNoCase(const std::string& root, const std::string& rel);

class TscReader {
public:
    explicit TscReader(std::string root) : root_(std::move(root)) {}

    std::set<std::string> defines{"_PC", "CD_VERSION"};
    std::vector<std::string> problems;

    // Runs `rel` (relative to the root) with %1.. = params, calling `fn` for every
    // command line; BSource lines are followed (and also passed to `fn`).
    void run(const std::string& rel, const std::vector<std::string>& params, const std::function<void(const TscLine&)>& fn);

private:
    void runFile(const std::string& path, const std::vector<std::string>& params, const std::function<void(const TscLine&)>& fn, int depth);
    std::string root_;
};

// Splits one line into arguments (quotes removed, comments dropped).
std::vector<std::string> splitTscLine(const std::string& line, bool* inBlockComment);

}  // namespace fuel::ui
