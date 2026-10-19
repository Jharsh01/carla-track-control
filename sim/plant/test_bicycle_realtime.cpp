// The bicycle plant on its real-time thread: rate, plant time, the shared state, and
// that it steps exactly as the offline plant does.
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <thread>

#include "sim/plant/bicycle_plant.hpp"
#include "track_control/tests/check.hpp"

using namespace sim;

int main() {
    const VehicleParams params = VehicleParams::from_file(SIM_CONFIG_DIR "/vehicles/model3.json");
    State x0 = State::Zero();
    x0[IUX] = 15.0;
    const Input u(0.05, 1500.0);

    BicyclePlant plant(params);
    plant.reset(x0);
    bicycle_state.mu = params.mu;
    bicycle_state.command(u);
    plant.start();
    CHECK(plant.running());
    CHECK(bicycle_state.time() < 0.05);  // published at 0 by start; the thread may have ticked since

    bool threw = false;
    try {
        plant.step(u, 1e-3);
    } catch (const std::logic_error&) {
        threw = true;
    }
    CHECK(threw);
    BicyclePlant other(params);
    threw = false;
    try {
        other.start();
    } catch (const std::runtime_error&) {
        threw = true;
    }
    CHECK(threw);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    plant.stop();
    CHECK(!plant.running());

    const auto timing = plant.timing();
    std::printf("ticks %llu, overruns %llu, latency mean %.1f us, max %.1f us, longest tick %.1f us, %s\n",
                static_cast<unsigned long long>(timing.ticks), static_cast<unsigned long long>(timing.overruns),
                1e6 * timing.latency_mean, 1e6 * timing.latency_max, 1e6 * timing.step_max,
                timing.realtime ? "SCHED_FIFO" : "normal scheduling");
    CHECK_NEAR(double(timing.ticks), 500.0, 25.0);
    CHECK_NEAR(bicycle_state.time(), timing.ticks * 1e-3, 1e-9);
    CHECK_NEAR(bicycle_state[IUX], plant.observe()[IUX], 0.0);

    // the same number of time steps offline gives the same state
    BicyclePlant offline(params);
    offline.reset(x0);
    offline.set_friction(params.mu);
    for (uint64_t k = 0; k < timing.ticks; ++k) offline.step(u, 1e-3);
    CHECK((offline.observe() - bicycle_state.state()).cwiseAbs().maxCoeff() <= 1e-9);
    CHECK((bicycle_state.applied_input() - offline.saturate(u)).cwiseAbs().maxCoeff() <= 1e-6);

    // a stopped plant steps offline again, and another one may run now
    plant.step(u, 1e-3);
    other.reset(x0);
    other.start();
    other.stop();
    return check_report("test_bicycle_realtime");
}
