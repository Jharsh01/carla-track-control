#include "track_control/vehicle_params.hpp"

#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace tc {

const std::vector<VehicleParams::Field>& VehicleParams::fields() {
    using P = VehicleParams;
    static const std::vector<Field> all = {
        {"m", &P::m},
        {"Izz", &P::Izz},
        {"a", &P::a},
        {"b", &P::b},
        {"h", &P::h},
        {"C_alpha_f", &P::C_alpha_f},
        {"C_alpha_r", &P::C_alpha_r},
        {"mu", &P::mu},
        {"grip_f", &P::grip_f, true},
        {"grip_r", &P::grip_r, true},
        {"drive_front", &P::drive_front},
        {"brake_front", &P::brake_front},
        {"F_drive_max", &P::F_drive_max},
        {"F_brake_max", &P::F_brake_max},
        {"P_max", &P::P_max},
        {"CdA", &P::CdA},
        {"Crr", &P::Crr},
        {"rho", &P::rho},
        {"delta_max", &P::delta_max},
        {"delta_rate_max", &P::delta_rate_max},
        {"Ux_min", &P::Ux_min},
        {"g", &P::g},
    };
    return all;
}

VehicleParams VehicleParams::from_file(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open " + path);
    const nlohmann::json data = nlohmann::json::parse(file);

    // the model and the controllers are written for this state and input order
    const std::vector<std::string> state = {"X", "Y", "psi", "Ux", "Uy", "r", "delta"};
    const std::vector<std::string> input = {"delta_rate", "Fx"};
    auto names = [&](const char* key) {
        std::vector<std::string> out;
        for (const auto& entry : data.at(key)) out.push_back(entry.at("name"));
        return out;
    };
    if ((data.contains("state") && names("state") != state) || (data.contains("input") && names("input") != input))
        throw std::runtime_error(path + ": the state or input order differs from the one the library is written for");

    VehicleParams p;
    const auto& values = data.at("vehicle").at("parameters");
    for (const auto& field : fields()) {
        if (!values.contains(field.name)) {
            if (field.optional) continue;
            throw std::runtime_error(path + ": no value for " + field.name);
        }
        p.*field.member = values.at(field.name).at("value");
    }
    return p;
}

double VehicleParams::understeer_gradient() const {
    const double Wf = m * g * b / L();
    const double Wr = m * g * a / L();
    return (Wf / C_alpha_f - Wr / C_alpha_r) / g;
}

}  // namespace tc
