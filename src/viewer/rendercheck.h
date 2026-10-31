// fuelview --rendercheck: the P26 renderer decode's test vectors (p26-renderer.md L2-L5 and the worked
// ground pixel, post.md P6, shadows.md S4; emulated on FUEL.exe or derived exactly from the decoded maths)
// run through the port's CPU maths (core/atmosphere.hpp), and the same maths evaluated by the Metal
// shaders compared with the CPU. Prints every check; returns 0 when all pass.
#pragma once

#include <string>

namespace fuel {

int runRenderCheck(const std::string& dataDir);

}  // namespace fuel
