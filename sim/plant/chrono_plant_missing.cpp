// Stand-in for the Chrono plant when the project is built without Project Chrono.
#include <stdexcept>

#include "sim/plant/chrono_plant.hpp"

namespace sim {

struct ChronoPlant::Impl {};

ChronoPlant::ChronoPlant(const VehicleParams&, double, double) {
    throw std::runtime_error("built without Project Chrono; run scripts/install_chrono.sh, or configure with -DChrono_DIR=<install>/lib/cmake/Chrono");
}
ChronoPlant::~ChronoPlant() = default;
void ChronoPlant::reset(const State&) {}
State ChronoPlant::observe() const { return State::Zero(); }
Input ChronoPlant::saturate(const Input& u) const { return u; }
State ChronoPlant::step(const Input&, double) { return State::Zero(); }
void ChronoPlant::set_friction(double) {}
ChronoPlant::Facts ChronoPlant::facts() const { return {}; }
ChronoPlant::Attitude ChronoPlant::attitude() const { return {}; }

}  // namespace sim
