// Offline lap of the race track: a controller and a plant, no CARLA. For
// checking a change quickly, and for comparing with what CARLA shows.
//
//   simulate                                  both controllers on the bicycle plant
//   simulate --plant chrono                   on the Project Chrono sedan; the controllers then plan
//                                             with config/vehicles/chrono_sedan.json
//   simulate --controller nmpc                or pure_pursuit
//   simulate --perfect                        exact traffic and ego state, whole road known
//   simulate --road-range 20 --ego-noise 0 --noise 2 --range 40 --seed 3
//   simulate --params FILE                    vehicle parameters for the controllers and the plant's limits
//   simulate --mu-use 0.9 --brake-use 0.5 --drive-use 0.5 --sideslip 4
//                                             NMPC planning margins (sim/racetrack/driver.hpp); the
//                                             defaults for each plant are in config/tuning.json
//   simulate --track FILE                     another track (default config/racetrack.json)
//   simulate --speed-limit 60                 the ego's speed limit [km/h] instead of the track's
//   simulate --out DIR                        where the logs go (default results/offline)
//
// Each run writes <out>/<controller>_<plant>.csv (one row per 10 ms) and a .json with its result; the
// NMPC also writes <controller>_<plant>_solves.csv, one row per solve (README.md, "Solve-time logs").
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "sim/plant/bicycle_plant.hpp"
#include "sim/plant/chrono_plant.hpp"
#include "sim/racetrack/driver.hpp"
#include "sim/racetrack/run_lap.hpp"
#include "track_control/nmpc.hpp"

using namespace sim;
using nlohmann::json;
namespace fs = std::filesystem;

namespace {

constexpr double CONTROL_PERIOD = 0.05;  // run_lap's default dt_ctrl [s]

struct Args {
    std::string controller = "both";
    std::string plant = "bicycle";
    std::string out = "results/offline";
    std::string params;  // file of vehicle parameters; empty: the one that goes with the plant
    std::string track = Circuit::default_file();
    bool perfect = false;
    double range = 60.0, noise = 1.0, ego_noise = 1.0, road_range = 40.0;
    unsigned seed = 0;
    double speed_limit = 0.0;  // [km/h]; 0: the track's
    // NMPC margins given on the command line; the rest come from config/tuning.json for the plant
    std::optional<double> mu_use, brake_use, drive_use, sideslip_deg;
    Tuning tuning;
};

const char* USAGE =
    "usage: simulate [--controller both|nmpc|pure_pursuit] [--plant bicycle|chrono] [--params FILE] [--track FILE]\n"
    "                [--out DIR] [--perfect] [--range M] [--noise K] [--ego-noise K] [--road-range M] [--seed N]\n"
    "                [--speed-limit KMH] [--mu-use K] [--brake-use K] [--drive-use K] [--sideslip DEG]\n";

Args parse(int argc, char** argv) {
    Args a;
    std::vector<std::string> words(argv + 1, argv + argc);
    for (size_t i = 0; i < words.size(); ++i) {
        const std::string& w = words[i];
        auto value = [&]() -> const std::string& {
            if (i + 1 >= words.size()) throw std::invalid_argument(w + " needs a value");
            return words[++i];
        };
        if (w == "-h" || w == "--help") {
            std::cout << USAGE;
            std::exit(0);
        } else if (w == "--controller") a.controller = value();
        else if (w == "--plant") a.plant = value();
        else if (w == "--out") a.out = value();
        else if (w == "--params") a.params = value();
        else if (w == "--track") a.track = value();
        else if (w == "--perfect") a.perfect = true;
        else if (w == "--range") a.range = std::stod(value());
        else if (w == "--noise") a.noise = std::stod(value());
        else if (w == "--ego-noise") a.ego_noise = std::stod(value());
        else if (w == "--road-range") a.road_range = std::stod(value());
        else if (w == "--seed") a.seed = static_cast<unsigned>(std::stoul(value()));
        else if (w == "--mu-use") a.mu_use = std::stod(value());
        else if (w == "--brake-use") a.brake_use = std::stod(value());
        else if (w == "--drive-use") a.drive_use = std::stod(value());
        else if (w == "--speed-limit") a.speed_limit = std::stod(value());
        else if (w == "--sideslip") a.sideslip_deg = std::stod(value());
        else throw std::invalid_argument("unknown option " + w);
    }
    if (a.controller != "both" && a.controller != "nmpc" && a.controller != "pure_pursuit")
        throw std::invalid_argument("unknown controller " + a.controller);
    if (a.plant != "bicycle" && a.plant != "chrono") throw std::invalid_argument("unknown plant " + a.plant);
    a.tuning = load_tuning(a.plant);
    if (a.mu_use) a.tuning.mu_use = *a.mu_use;
    if (a.brake_use) a.tuning.brake_use = *a.brake_use;
    if (a.drive_use) a.tuning.drive_use = *a.drive_use;
    if (a.sideslip_deg) a.tuning.sideslip_max = *a.sideslip_deg * M_PI / 180.0;
    return a;
}

std::string vehicle_file(const Args& a) {
    if (!a.params.empty()) return a.params;
    return std::string(SIM_CONFIG_DIR "/vehicles/") + (a.plant == "chrono" ? "chrono_sedan.json" : "model3.json");
}

std::unique_ptr<PlantModel> make_plant(const std::string& name, const VehicleParams& params) {
    if (name == "chrono") return std::make_unique<ChronoPlant>(params);
    return std::make_unique<BicyclePlant>(params);
}

json optional(const std::optional<double>& value) { return value ? json(*value) : json(nullptr); }

// One row per NMPC solve, the columns of carla_bridge/run.py's SOLVE_COLUMNS.
void write_solves(const fs::path& file, const std::vector<NMPC::SolveStats>& stats) {
    fs::create_directories(file.parent_path());
    std::ofstream out(file);
    out << "t,s0,ux0,t_wall,t_total,t_proc,t_hess,t_jac,t_grad,t_f,t_g,iterations,success,status,warm,n_obstacles,obj,"
           "slack_obstacle,slack_edge,slack_sideslip,slack_speed,x0_gap\n"
        << std::setprecision(9);
    for (const auto& s : stats) {
        out << s.t << ',' << s.s0 << ',' << s.ux0 << ',' << s.t_wall << ',' << s.t_total << ',' << s.t_proc << ','
            << s.t_hess << ',' << s.t_jac << ',' << s.t_grad << ',' << s.t_f << ',' << s.t_g << ',' << s.iterations
            << ',' << (s.success ? "True" : "False") << ',' << s.status << ',' << (s.warm ? "True" : "False") << ','
            << s.n_obstacles << ',' << s.obj;
        for (double slack : s.slack_max) out << ',' << slack;
        out << ',' << s.x0_gap << '\n';
    }
}

// Where the solve time went over all solves of a lap: the keys carla_bridge/run.py's solve_breakdown writes.
json solve_breakdown(const std::vector<NMPC::SolveStats>& stats, double dt) {
    if (stats.empty()) return json::object();
    std::vector<double> wall;
    double total = 0.0, proc = 0.0, hess = 0.0, jac = 0.0, grad = 0.0, f = 0.0, g = 0.0, iters = 0.0;
    int iter_max = 0, overruns = 0;
    std::map<std::string, int> statuses;
    for (const auto& s : stats) {
        wall.push_back(s.t_wall);
        total += s.t_total, proc += s.t_proc;
        hess += s.t_hess, jac += s.t_jac, grad += s.t_grad, f += s.t_f, g += s.t_g;
        iters += s.iterations;
        iter_max = std::max(iter_max, s.iterations);
        overruns += s.t_wall > dt;
        ++statuses[s.status];
    }
    std::vector<double> sorted = wall;
    std::sort(sorted.begin(), sorted.end());
    auto percentile = [&](double q) {  // linear interpolation, as numpy's default
        const double pos = q / 100.0 * (sorted.size() - 1);
        const size_t lo = static_cast<size_t>(std::floor(pos)), hi = std::min(lo + 1, sorted.size() - 1);
        return sorted[lo] + (pos - lo) * (sorted[hi] - sorted[lo]);
    };
    auto share = [&](double part) { return total > 0.0 ? part / total : 0.0; };
    const json shares = {{"hess", share(hess)}, {"jac", share(jac)}, {"grad", share(grad)}, {"f", share(f)},
                         {"g", share(g)},  // the rest: linear solves and IPOPT's own work
                         {"ipopt_internal", total > 0.0 ? 1.0 - share(hess + jac + grad + f + g) : 0.0}};
    return {{"solve_p50", percentile(50)}, {"solve_p95", percentile(95)},
            {"overrun_fraction", double(overruns) / stats.size()}, {"iter_mean", iters / stats.size()},
            {"iter_max", iter_max}, {"time_share", shares}, {"proc_over_wall", share(proc)},
            {"status_counts", statuses}, {"first_solve", wall.front()}};
}

void write_log(const fs::path& file, const LapLog& log) {
    fs::create_directories(file.parent_path());
    std::ofstream out(file);
    out << "t,X,Y,psi,Ux,Uy,r,delta,delta_rate,Fx,s,e_y,mu\n" << std::setprecision(12);
    for (size_t k = 0; k < log.t.size(); ++k) {
        out << log.t[k];
        for (int i = 0; i < NX; ++i) out << ',' << log.x[k][i];
        for (int i = 0; i < NU; ++i) out << ',' << log.u[k][i];
        out << ',' << log.s[k] << ',' << log.e_y[k] << ',' << log.mu[k] << '\n';
    }
}

void run(const Args& a, const std::string& name, const Circuit& circuit, const VehicleParams& params) {
    std::printf("driving the lap with %s on the %s plant ...\n", name.c_str(), a.plant.c_str());
    std::fflush(stdout);
    Perception perception = a.perfect ? Perception(circuit)
                                      : Perception(circuit, a.range, 20.0, 0.5 * a.noise, 0.2 * a.noise,
                                                   0.5 * a.noise, a.seed, State(a.ego_noise * EGO_SIGMA), a.road_range);
    const double road_range = perception.road_range;
    Driver driver(make_controller(name, circuit, params, road_range, a.tuning), std::move(perception));
    auto plant = make_plant(a.plant, params);
    const auto [log, r] = run_lap(circuit, std::ref(driver), *plant);

    std::string solves, solve_detail;
    json solve_summary = json::object();
    auto* nmpc = dynamic_cast<NMPC*>(&driver.controller());
    if (nmpc) {
        double sum = 0.0, max = 0.0;
        for (double t : nmpc->solve_times) sum += t, max = std::max(max, t);
        char text[128];
        std::snprintf(text, sizeof text, "   | solve mean %.0f ms, max %.0f ms, %d not converged",
                      1e3 * sum / nmpc->solve_times.size(), 1e3 * max, nmpc->failures);
        solves = text;
        solve_summary = {{"solve_mean", sum / nmpc->solve_times.size()}, {"solve_max", max},
                         {"not_converged", nmpc->failures}};
        solve_summary.update(solve_breakdown(nmpc->solve_stats, CONTROL_PERIOD));
        if (solve_summary.contains("time_share")) {
            const json& b = solve_summary;
            const json& share = b["time_share"];
            char detail[256];
            std::snprintf(detail, sizeof detail,
                          "%-13s solve p50 %.0f ms, p95 %.0f ms, over dt %.0f%%   iterations mean %.1f, max %d"
                          "   time in hessian %.0f%%, jacobian %.0f%%, ipopt %.0f%%"
                          "   cpu/wall %.2f   first solve %.0f ms\n",
                          "", 1e3 * b["solve_p50"].get<double>(), 1e3 * b["solve_p95"].get<double>(),
                          100 * b["overrun_fraction"].get<double>(), b["iter_mean"].get<double>(),
                          b["iter_max"].get<int>(), 100 * share["hess"].get<double>(), 100 * share["jac"].get<double>(),
                          100 * share["ipopt_internal"].get<double>(), b["proc_over_wall"].get<double>(),
                          1e3 * b["first_solve"].get<double>());
            solve_detail = detail;
        }
    }
    char outcome[32] = "   none";
    if (r.completed) std::snprintf(outcome, sizeof outcome, "%6.1f s", *r.lap_time);
    const int passed = cars_passed(circuit, log);
    std::printf("%-13s lap %s   speed %2.0f-%2.0f km/h   passed %d/%zu   clearance %5.2f m   asphalt margin %5.2f m"
                "   on grass %3.1f s   sideslip %4.1f deg%s",
                name.c_str(), outcome, 3.6 * r.min_speed, 3.6 * r.max_speed, passed, circuit.traffic.size(),
                r.min_clearance, r.road_margin, r.time_on_grass, r.peak_sideslip * 180.0 / M_PI, solves.c_str());
    if (r.collided || r.left_track) std::printf("   %s at %.1f s", r.collided ? "collision" : "left the track", log.t.back());
    std::printf("\n%s", solve_detail.c_str());

    const fs::path file = fs::path(a.out) / (name + "_" + a.plant + ".csv");
    write_log(file, log);
    if (nmpc && !nmpc->solve_stats.empty())
        write_solves(fs::path(a.out) / (name + "_" + a.plant + "_solves.csv"), nmpc->solve_stats);
    json result = {{"controller", name}, {"plant", a.plant}, {"completed", r.completed},
                   {"lap_time", optional(r.lap_time)}, {"collided", r.collided}, {"left_track", r.left_track},
                   {"min_clearance", r.min_clearance}, {"road_margin", r.road_margin},
                   {"time_on_grass", r.time_on_grass}, {"peak_sideslip", r.peak_sideslip},
                   {"min_speed", r.min_speed}, {"max_speed", r.max_speed}, {"passed", passed}};
    result.update(solve_summary);
    std::ofstream(fs::path(file).replace_extension(".json")) << result.dump(2) << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args a = parse(argc, argv);
        Circuit circuit(a.track);
        if (a.speed_limit > 0.0) circuit.v_max = a.speed_limit / 3.6;
        const VehicleParams params = VehicleParams::from_file(vehicle_file(a));
        if (a.controller != "pure_pursuit") run(a, "nmpc", circuit, params);
        if (a.controller != "nmpc") run(a, "pure_pursuit", circuit, params);
    } catch (const std::exception& error) {
        std::cerr << "simulate: " << error.what() << '\n' << USAGE;
        return 1;
    }
    return 0;
}
