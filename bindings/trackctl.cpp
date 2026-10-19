// Python module `trackctl`: the controllers, the race track and the plants, for the CARLA bridge.
//
// States and inputs are numpy arrays in the library's convention
// (track_control/types.hpp): right-handed world frame, centre of gravity.
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "sim/plant/bicycle_plant.hpp"
#include "sim/plant/chrono_plant.hpp"
#include "sim/racetrack/driver.hpp"
#include "sim/racetrack/fallback_control.hpp"
#include "track_control/nmpc.hpp"
#include "track_control/pure_pursuit.hpp"

namespace py = pybind11;
using namespace sim;

PYBIND11_MODULE(trackctl, m) {
    m.doc() = "track_control controllers, the race track and the plants (bicycle, Project Chrono)";
    m.attr("have_chrono") = static_cast<bool>(SIM_HAVE_CHRONO);
    m.attr("config_dir") = SIM_CONFIG_DIR;

    py::class_<VehicleParams> params(m, "VehicleParams");
    params.def(py::init<>()).def_static("from_file", &VehicleParams::from_file).def("L", &VehicleParams::L);
    for (const auto& field : VehicleParams::fields()) params.def_readwrite(field.name, field.member);

    py::class_<Pose>(m, "Pose").def_readonly("x", &Pose::x).def_readonly("y", &Pose::y).def_readonly("psi", &Pose::psi);

    py::class_<Path>(m, "Path")
        .def_static("from_waypoints", &Path::from_waypoints, py::arg("points"), py::arg("closed"))
        .def_property_readonly("length", &Path::length)
        .def_property_readonly("closed", &Path::closed)
        .def("curvature", &Path::curvature)
        .def("pose", &Path::pose, py::arg("s"), py::arg("e_y") = 0.0)
        .def("project", &Path::project)
        .def("wrap", &Path::wrap)
        .def("difference", &Path::difference)
        .def_property_readonly("piece_lengths", &Path::piece_lengths)
        .def_property_readonly("piece_curvatures", &Path::piece_curvatures)
        .def_property_readonly("piece_poses", &Path::piece_poses);

    py::class_<Body>(m, "Body")
        .def_readonly("length", &Body::length)
        .def_readonly("width", &Body::width)
        .def_readonly("cg_to_center", &Body::cg_to_center);

    py::class_<TrafficPose>(m, "TrafficPose")
        .def_readonly("x", &TrafficPose::x)
        .def_readonly("y", &TrafficPose::y)
        .def_readonly("psi", &TrafficPose::psi)
        .def_readonly("s", &TrafficPose::s);

    py::class_<Circuit>(m, "Circuit")
        .def(py::init<const std::string&>(), py::arg("file") = Circuit::default_file())
        .def_readonly("track", &Circuit::track)
        .def_readonly("lane_width", &Circuit::lane_width)
        .def_readonly("grass_width", &Circuit::grass_width)
        .def_readonly("mu_road", &Circuit::mu_road)
        .def_readonly("mu_grass", &Circuit::mu_grass)
        .def_readonly("v_max", &Circuit::v_max)
        .def_readonly("ego_lane", &Circuit::ego_lane)
        .def_readonly("body", &Circuit::body)
        .def_property_readonly("n_traffic", [](const Circuit& c) { return c.traffic.size(); })
        .def("lane_offset", &Circuit::lane_offset)
        .def("friction", &Circuit::friction)
        .def("traffic_pose", [](const Circuit& c, size_t i, double t) { return c.traffic_pose(c.traffic.at(i), t); })
        .def("initial_state", &Circuit::initial_state)
        .def("track_state",
             [](const Circuit& c, const State& x) {
                 const PathState ps = c.track_state(x);
                 return py::make_tuple(ps.s, ps.e_y, ps.e_psi);
             })
        .def("clearance", &Circuit::clearance)
        .def("road_margin", &Circuit::road_margin);

    py::class_<Perception>(m, "Perception")
        .def_static("exact", [](const Circuit& c) { return Perception(c); })
        .def_static(
            "noisy",
            [](const Circuit& c, double range, double noise, double ego_noise, double road_range, unsigned seed) {
                return Perception(c, range, 20.0, 0.5 * noise, 0.2 * noise, 0.5 * noise, seed,
                                  State(ego_noise * EGO_SIGMA), road_range);
            },
            py::arg("circuit"), py::arg("range") = 60.0, py::arg("noise") = 1.0, py::arg("ego_noise") = 1.0,
            py::arg("road_range") = 40.0, py::arg("seed") = 0u,
            "Traffic `range` m ahead and 20 m behind, errors scaled by `noise` and `ego_noise`, the road known "
            "`road_range` m ahead (the offline simulation's defaults)")
        .def_readonly("road_range", &Perception::road_range);

    py::class_<Tuning>(m, "Tuning")
        .def(py::init<>())
        .def_readwrite("mu_use", &Tuning::mu_use)
        .def_readwrite("brake_use", &Tuning::brake_use)
        .def_readwrite("drive_use", &Tuning::drive_use)
        .def_readwrite("sideslip_max", &Tuning::sideslip_max)
        .def_readwrite("solve_time_limit", &Tuning::solve_time_limit);
    m.def("load_tuning", [](const std::string& dynamics) { return load_tuning(dynamics); },
          "NMPC margins for 'bicycle', 'chrono' or 'carla' from config/tuning.json");

    py::class_<Driver>(m, "Driver")
        .def(py::init([](const std::string& controller, const Circuit& circuit, const VehicleParams& params,
                         Perception perception, const Tuning& tuning) {
                 const double road_range = perception.road_range;
                 return Driver(make_controller(controller, circuit, params, road_range, tuning), std::move(perception));
             }),
             py::arg("controller"), py::arg("circuit"), py::arg("params"), py::arg("perception"),
             py::arg("tuning") = Tuning(), "controller: 'nmpc' or 'pure_pursuit'")
        .def("__call__", &Driver::operator(), py::arg("t"), py::arg("x"), py::call_guard<py::gil_scoped_release>(),
             "input [steering rate, Fx] for the true state x at time t (other Python threads run meanwhile)")
        .def_property_readonly("name", [](Driver& d) { return d.controller().name(); })
        .def_property_readonly("measured_state", [](const Driver& d) { return d.last.x; })
        .def_property_readonly("n_detected", [](const Driver& d) { return d.last.obstacles.size(); })
        .def_property_readonly("last_solve_time",
                               [](Driver& d) -> py::object {
                                   auto* nmpc = dynamic_cast<NMPC*>(&d.controller());
                                   if (!nmpc || nmpc->solve_times.empty()) return py::none();
                                   return py::float_(nmpc->solve_times.back());
                               })
        .def_property_readonly("solve_times",
                               [](Driver& d) {
                                   auto* nmpc = dynamic_cast<NMPC*>(&d.controller());
                                   return nmpc ? nmpc->solve_times : std::vector<double>();
                               })
        .def_property_readonly("solve_stats",
                               [](Driver& d) {
                                   py::list out;
                                   if (auto* nmpc = dynamic_cast<NMPC*>(&d.controller()))
                                       for (const auto& s : nmpc->solve_stats) {
                                           py::dict r;
                                           r["t"] = s.t, r["s0"] = s.s0, r["ux0"] = s.ux0;
                                           r["t_wall"] = s.t_wall, r["t_total"] = s.t_total, r["t_proc"] = s.t_proc;
                                           r["t_hess"] = s.t_hess, r["t_jac"] = s.t_jac, r["t_grad"] = s.t_grad;
                                           r["t_f"] = s.t_f, r["t_g"] = s.t_g;
                                           r["iterations"] = s.iterations, r["success"] = s.success;
                                           r["status"] = s.status, r["warm"] = s.warm;
                                           r["n_obstacles"] = s.n_obstacles, r["obj"] = s.obj;
                                           r["slack_obstacle"] = s.slack_max[0], r["slack_edge"] = s.slack_max[1];
                                           r["slack_sideslip"] = s.slack_max[2], r["slack_speed"] = s.slack_max[3];
                                           r["x0_gap"] = s.x0_gap;
                                           out.append(r);
                                       }
                                   return out;
                               },
                               "per solve: a dict of NMPC::SolveStats (times in seconds)")
        .def_property_readonly("failures",
                               [](Driver& d) {
                                   auto* nmpc = dynamic_cast<NMPC*>(&d.controller());
                                   return nmpc ? nmpc->failures : 0;
                               })
        .def_property_readonly(
            "plan",
            [](Driver& d) {
                // the NMPC's predicted (s, e_y); pure pursuit's lookahead line has no plan
                std::vector<std::pair<double, double>> out;
                if (auto* nmpc = dynamic_cast<NMPC*>(&d.controller()))
                    for (int k = 0; k < nmpc->plan.rows(); ++k) out.emplace_back(nmpc->plan(k, IX), nmpc->plan(k, IY));
                return out;
            },
            "predicted path of the NMPC as (s, e_y) pairs; empty for pure pursuit");

    py::class_<FallbackControl>(m, "FallbackControl",
                                "the NMPC with pure pursuit as its fallback, on their own threads against the running "
                                "bicycle plant (sim/racetrack/fallback_control.hpp)")
        .def(py::init([](Driver& nmpc, Driver& pure_pursuit, double rate, double deadline) {
                 return std::make_unique<FallbackControl>(nmpc, pure_pursuit, FallbackControl::Options{rate, deadline});
             }),
             py::arg("nmpc"), py::arg("pure_pursuit"), py::arg("rate") = 100.0, py::arg("deadline") = 0.1,
             py::keep_alive<1, 2>(), py::keep_alive<1, 3>())
        .def("start", &FallbackControl::start)
        .def("stop", &FallbackControl::stop, py::call_guard<py::gil_scoped_release>())
        .def("status",
             [](const FallbackControl& c) {
                 const auto s = c.status();
                 py::dict d;
                 d["source"] = source_name(s.source), d["reason"] = s.reason, d["plan_age"] = s.plan_age;
                 d["last_solve"] = s.last_solve, d["last_converged"] = s.last_converged;
                 d["ticks"] = s.ticks, d["nmpc_ticks"] = s.nmpc_ticks, d["pure_pursuit_ticks"] = s.pure_pursuit_ticks;
                 d["solves"] = s.solves, d["solves_on_time"] = s.solves_on_time;
                 d["pure_pursuit_updates"] = s.pure_pursuit_updates;
                 return d;
             },
             "source ('nmpc', 'pure_pursuit' or 'none') of the last command, the reason for pure pursuit, the "
             "plan's age [s], the last solve's wall time [s] and convergence, and counts")
        .def_property_readonly("plan", &FallbackControl::plan, "(s, e_y) of the NMPC plan in use; empty if none");

    py::class_<PlantModel>(m, "PlantModel")
        .def("reset", &PlantModel::reset)
        .def("observe", &PlantModel::observe)
        .def("saturate", &PlantModel::saturate)
        .def("step", &PlantModel::step, py::arg("u"), py::arg("duration"))
        .def("set_friction", &PlantModel::set_friction);

    py::class_<BicyclePlant, PlantModel>(m, "BicyclePlant")
        .def(py::init<const VehicleParams&>())
        .def(
            "start",
            [](BicyclePlant& plant, const Circuit* circuit, double rate, int priority, int cpu) {
                plant.surface_friction = nullptr;
                if (circuit)
                    plant.surface_friction = [circuit](const State& x) {
                        return circuit->friction(circuit->track_state(x).e_y);
                    };
                plant.start(BicyclePlant::Realtime{rate, priority, cpu});
            },
            py::arg("circuit") = nullptr, py::arg("rate") = 1000.0, py::arg("priority") = 80, py::arg("cpu") = -1,
            py::keep_alive<1, 2>(),
            "step on its own thread at `rate` Hz in real time, through trackctl.bicycle_state; with `circuit`, "
            "the friction follows the surface under the car")
        .def("stop", &BicyclePlant::stop, py::call_guard<py::gil_scoped_release>())
        .def_property_readonly("running", &BicyclePlant::running)
        .def(
            "timing",
            [](const BicyclePlant& plant) {
                const auto s = plant.timing();
                py::dict d;
                d["ticks"] = s.ticks, d["overruns"] = s.overruns, d["latency_mean"] = s.latency_mean;
                d["latency_max"] = s.latency_max, d["step_max"] = s.step_max, d["realtime"] = s.realtime;
                return d;
            },
            "the thread's timing: ticks, overruns, latency_mean, latency_max, step_max [s], realtime (SCHED_FIFO)");

    py::class_<BicycleBus>(m, "BicycleBus", "the running bicycle plant's state and input (sim/plant/bicycle_plant.hpp)")
        .def(
            "state",
            [](const BicycleBus& bus) {
                double t;
                const State x = bus.state(&t);
                return py::make_tuple(t, x);
            },
            "(t, x): plant time and state after its latest tick")
        .def("command", &BicycleBus::command, py::arg("u"), "input held by the plant from its next tick on")
        .def_property_readonly("commanded", &BicycleBus::commanded)
        .def_property_readonly("applied", &BicycleBus::applied_input, "the input after the plant's limits")
        .def_property(
            "mu", [](const BicycleBus& bus) { return bus.mu.load(); },
            [](BicycleBus& bus, double mu) { bus.mu.store(mu); });
    m.attr("bicycle_state") = py::cast(&bicycle_state, py::return_value_policy::reference);

    py::class_<ChronoPlant::Attitude>(m, "Attitude")
        .def_readonly("roll", &ChronoPlant::Attitude::roll)
        .def_readonly("pitch", &ChronoPlant::Attitude::pitch)
        .def_readonly("heave", &ChronoPlant::Attitude::heave);
    py::class_<ChronoPlant, PlantModel>(m, "ChronoPlant")
        .def(py::init<const VehicleParams&, double, double>(), py::arg("params"), py::arg("dt") = 1e-3,
             py::arg("tau_steer") = 0.05)
        .def("attitude", &ChronoPlant::attitude);
}
