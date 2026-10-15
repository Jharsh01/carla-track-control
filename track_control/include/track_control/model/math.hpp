// Math on doubles under the names CasADi uses, so the model equations can be
// written once and evaluated on numbers (simulation) and on symbols (NMPC).
#pragma once

#include <cmath>

namespace tc::ops {

using std::atan;
using std::cos;
using std::fabs;
using std::fmax;
using std::sin;
using std::sqrt;
using std::tan;

inline double sign(double x) { return (x > 0.0) - (x < 0.0); }
inline double if_else(bool condition, double if_true, double if_false) { return condition ? if_true : if_false; }

}  // namespace tc::ops
