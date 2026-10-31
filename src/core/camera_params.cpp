#include "camera_params.hpp"

#include <algorithm>

namespace fuel {

float cameraParam(const std::vector<float>& P, int id, int view) {
    const int v = std::min(view, 5);
    const long n = long(id) + 1 + 33L * (v - 1);
    return n > 0 && size_t(n) < P.size() ? P[size_t(n)] : 0.0f;
}

CameraParams cameraParams(const std::vector<float>& P, int view) {
    CameraParams c;
    for (int i = 0; i < 33; ++i) c.k[i] = cameraParam(P, 406 + i, view);
    return c;
}

}  // namespace fuel
