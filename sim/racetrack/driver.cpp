#include "sim/racetrack/driver.hpp"

#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "track_control/nmpc.hpp"
#include "track_control/pure_pursuit.hpp"

namespace sim {

Tuning load_tuning(const std::string& dynamics, const std::string& file) {
    std::ifstream in(file);
    if (!in) throw std::runtime_error("cannot open " + file);
    const nlohmann::json data = nlohmann::json::parse(in);
    Tuning t;
    if (!data.contains(dynamics) || !data.at(dynamics).contains("nmpc")) return t;
    const nlohmann::json& n = data.at(dynamics).at("nmpc");
    t.mu_use = n.value("mu_use", t.mu_use);
    t.brake_use = n.value("brake_use", t.brake_use);
    t.drive_use = n.value("drive_use", t.drive_use);
    if (n.contains("sideslip_max_deg")) t.sideslip_max = n.at("sideslip_max_deg").get<double>() * M_PI / 180.0;
    return t;
}

std::unique_ptr<Controller> make_controller(const std::string& name, const Circuit& circuit,
                                            const VehicleParams& params, double road_range, const Tuning& tuning) {
    const double home = circuit.lane_offset(circuit.ego_lane);
    if (name == "pure_pursuit") {
        PurePursuit::Options o;
        o.lane_offset = home;
        o.pass_offset = -home;
        o.road_range = road_range;
        return std::make_unique<PurePursuit>(circuit.track, circuit.road(), params, circuit.body, o);
    }
    if (name == "nmpc") {
        NMPC::Options o;
        o.lane_offset = home;
        o.road_range = road_range;
        o.sideslip_max = tuning.sideslip_max;
        o.max_wall_time = tuning.solve_time_limit;
        if (tuning.nmpc_dt > 0.0) o.h = tuning.nmpc_dt;
        if (tuning.nmpc_horizon > 0.0) o.horizon = tuning.nmpc_horizon;
        o.h_fine = tuning.nmpc_fine_dt;
        o.fine = tuning.nmpc_fine;
        VehicleParams planned = params;
        planned.mu *= tuning.mu_use;
        planned.F_brake_max *= tuning.brake_use;
        planned.F_drive_max *= tuning.drive_use;
        return std::make_unique<NMPC>(circuit.track, circuit.road(), planned, circuit.body, o);
    }
    throw std::invalid_argument("unknown controller '" + name + "' (nmpc or pure_pursuit)");
}

Driver::Driver(std::unique_ptr<Controller> controller, Perception perception_)
    : perception(std::move(perception_)), controller_(std::move(controller)) {}

Input Driver::operator()(double t, const State& x) {
    last.t = t;
    last.x = perception.ego(x);
    const double s = perception.c.track.project(last.x[IX], last.x[IY]).first;
    last.obstacles = perception.observe(t, s);
    return controller_->compute(last);
}

}  // namespace sim
