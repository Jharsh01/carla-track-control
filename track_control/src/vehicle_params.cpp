#include "track_control/vehicle_params.hpp"

#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

#include "track_control/model/tire.hpp"

namespace tc {

double LoadTable::operator()(double load) const { return cornering_stiffness(*this, 0.0, load); }

namespace {

LoadTable read_load_table(const nlohmann::json& entry, const std::string& where) {
    LoadTable t;
    if (!entry.contains("load_table")) return t;
    t.Fz = entry.at("load_table").at("Fz").get<std::vector<double>>();
    t.C_alpha = entry.at("load_table").at("C_alpha").get<std::vector<double>>();
    if (t.Fz.empty() || t.Fz.size() != t.C_alpha.size())
        throw std::runtime_error(where + ": load_table needs as many C_alpha as Fz values, at least one");
    for (size_t i = 0; i < t.Fz.size(); ++i) {
        if (i > 0 && !(t.Fz[i] > t.Fz[i - 1])) throw std::runtime_error(where + ": load_table Fz must increase");
        if (!(t.C_alpha[i] > 0.0)) throw std::runtime_error(where + ": load_table C_alpha must be positive");
    }
    return t;
}

}  // namespace

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
    p.C_alpha_f_load = read_load_table(values.at("C_alpha_f"), path + ": C_alpha_f");
    p.C_alpha_r_load = read_load_table(values.at("C_alpha_r"), path + ": C_alpha_r");
    return p;
}

double VehicleParams::understeer_gradient(double Fx) const {
    // the cornering force each axle needs follows the weight distribution; the
    // stiffness it has follows its load under the longitudinal load transfer (model/bicycle.hpp)
    const double Fzf = (m * g * b - h * Fx) / L();
    const double Fzr = (m * g * a + h * Fx) / L();
    return m / L() * (b / C_alpha_f_at(Fzf) - a / C_alpha_r_at(Fzr));
}

}  // namespace tc
