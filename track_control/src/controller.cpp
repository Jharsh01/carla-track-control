#include "track_control/controller.hpp"

namespace tc {

PathState path_state(const Path& path, const State& x) {
    const auto [s, e_y] = path.project(x[IX], x[IY]);
    return {s, e_y, wrap_angle(x[IPSI] - path.pose(s).psi)};
}

}  // namespace tc
