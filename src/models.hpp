// Subsystem models: solar cells, battery, panel thermal, RF link, pointing laws.
#pragma once
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "astro.hpp"
#include "vecmath.hpp"

namespace cs {

inline Vec3 parse_axis(const std::string& s) {
    if (s.size() == 2) {
        double sg = s[0] == '-' ? -1.0 : 1.0;
        switch (s[1]) {
            case 'X': case 'x': return {sg, 0, 0};
            case 'Y': case 'y': return {0, sg, 0};
            case 'Z': case 'z': return {0, 0, sg};
        }
    }
    throw std::runtime_error("bad axis '" + s + "', expected one of +X -X +Y -Y +Z -Z");
}

// ---------------------------------------------------------------- solar cells
struct SolarCellModel {
    double eff_ref = 0.295;       // BOL efficiency at T_ref, AM0
    double t_ref_c = 28.0;
    double temp_coeff = -0.0023;  // relative change of Pmp per K
    double iam_ar = 0.16;         // Martin & Ruiz angular-loss coefficient
    double iam_diffuse = 0.90;    // hemispherical-average IAM for albedo
    double derate = 0.95;         // wiring, mismatch, cover-glass, UV
    double degradation = 1.00;    // radiation / ageing (1 = BOL)

    double efficiency(double t_c) const {
        return std::max(0.0, eff_ref * (1.0 + temp_coeff * (t_c - t_ref_c)));
    }
    // Incidence angle modifier, normalised to 1 at normal incidence.
    double iam(double cos_inc) const {
        if (cos_inc <= 0) return 0.0;
        return (1.0 - std::exp(-cos_inc / iam_ar)) / (1.0 - std::exp(-1.0 / iam_ar));
    }
};

// -------------------------------------------------------------------- battery
struct Battery {
    double capacity_wh = 40.0;
    double energy_wh = 32.0;
    double charge_eff = 0.95, discharge_eff = 0.95;
    double max_charge_w = 20.0, max_discharge_w = 40.0;

    double soc() const { return energy_wh / capacity_wh; }

    // net_bus_w = solar delivered to the bus minus load. Returns the power that
    // could not be used (array curtailed by the shunt regulator), and sets
    // `unmet_w` to load that the battery could not supply.
    double step(double net_bus_w, double dt, double& unmet_w) {
        unmet_w = 0.0;
        double curtailed = 0.0;
        if (net_bus_w >= 0) {
            double p_in = std::min(net_bus_w, max_charge_w);
            double room_wh = capacity_wh - energy_wh;
            double e_in = std::min(p_in * charge_eff * dt / 3600.0, room_wh);
            energy_wh += e_in;
            curtailed = net_bus_w - e_in * 3600.0 / (charge_eff * dt);
        } else {
            double p_out = std::min(-net_bus_w, max_discharge_w);
            double e_out = std::min(p_out / discharge_eff * dt / 3600.0, energy_wh);
            energy_wh -= e_out;
            unmet_w = -net_bus_w - e_out * discharge_eff * 3600.0 / dt;
        }
        return std::max(0.0, curtailed);
    }
};

// ------------------------------------------------------------- panel thermal
// Single-node model of a deployable panel (both sides radiate to space).
struct PanelThermal {
    double mass_kg = 0.12, cp = 900.0;
    double alpha_front = 0.92, eps_front = 0.85;
    double alpha_back = 0.92, eps_back = 0.85;
    double t_k = 293.15;

    // q_solar_*: direct+albedo irradiance absorbed-by-area (W/m^2) on each side,
    // q_ir_*: Earth IR irradiance on each side, p_elec: electrical power removed.
    void step(double area, double q_sol_f, double q_ir_f, double q_sol_b, double q_ir_b,
              double p_elec, double dt) {
        const int sub = std::max(1, int(std::ceil(dt / 2.0)));
        const double h = dt / sub;
        for (int i = 0; i < sub; ++i) {
            double q_in = area * (alpha_front * q_sol_f + eps_front * q_ir_f + alpha_back * q_sol_b +
                                  eps_back * q_ir_b) - p_elec;
            double q_out = area * (eps_front + eps_back) * SIGMA_SB * std::pow(t_k, 4);
            t_k += (q_in - q_out) * h / (mass_kg * cp);
        }
    }
};

// ------------------------------------------------------------------ RF link
struct LinkModel {
    double freq_hz = 2.245e9;
    double tx_power_w = 2.0;
    double ant_gain_dbi = 6.5;
    double ant_hpbw_deg = 70.0;
    double tx_losses_db = 2.0;
    double gs_g_over_t = 10.0;  // dB/K
    double path_losses_db = 2.0;  // atmosphere + polarisation + pointing at GS
    double req_ebn0_db = 4.5;     // e.g. QPSK + rate-1/2 conv/RS, BER 1e-6
    double impl_loss_db = 2.0;
    double margin_db = 3.0;
    double max_off_boresight_deg = 60.0;
    std::vector<double> rates_mbps = {0.5, 1, 2, 4, 8};
    double code_rate = 0.5;          // information bits per coded bit
    double bits_per_symbol = 2;      // QPSK
    double max_symbol_rate = 5e6;    // symbols/s allowed by the spectrum assignment
    double frame_efficiency = 0.97;  // ASM + frame headers + CRC

    // Required Eb/N0 (dB) at ~1e-6 BER / 1e-4 FER, read off CCSDS 130.1-G curves (approximate).
    void set_coding(const std::string& c) {
        if (c == "uncoded") { code_rate = 1.0; req_ebn0_db = 10.5; }
        else if (c == "conv_rs") { code_rate = 0.5 * 223.0 / 255.0; req_ebn0_db = 2.5; }
        else if (c == "turbo_1_2") { code_rate = 0.5; req_ebn0_db = 1.0; }
        else if (c == "turbo_1_3") { code_rate = 1.0 / 3.0; req_ebn0_db = 0.5; }
        else if (c == "ldpc_7_8") { code_rate = 7136.0 / 8160.0; req_ebn0_db = 4.0; }
        else throw std::runtime_error("unknown link.coding '" + c + "' (uncoded, conv_rs, turbo_1_2, turbo_1_3, ldpc_7_8)");
    }
    double max_info_rate() const { return max_symbol_rate * bits_per_symbol * code_rate * frame_efficiency; }

    double tx_gain(double off_rad) const {
        double r = off_rad * RAD / ant_hpbw_deg;
        return std::max(ant_gain_dbi - 12.0 * r * r, ant_gain_dbi - 25.0);
    }
    double cn0(double range_km, double off_rad) const {
        double eirp = 10 * std::log10(tx_power_w) + tx_gain(off_rad) - tx_losses_db;
        double fspl = 20 * std::log10(4 * PI * range_km * 1e3 * freq_hz / C_LIGHT);
        return eirp - fspl - path_losses_db + gs_g_over_t - K_BOLTZ_DB;
    }
    // Highest ladder rate that closes with the required margin (0 if none).
    double select_rate(double cn0_dbhz, double& margin_out) const {
        double best = 0;
        margin_out = -99;
        for (double r : rates_mbps) {
            if (r * 1e6 > max_info_rate() + 1) continue;  // does not fit the allowed bandwidth
            double m = cn0_dbhz - 10 * std::log10(r * 1e6) - req_ebn0_db - impl_loss_db;
            if (m >= margin_db && r > best) {
                best = r;
                margin_out = m;
            }
        }
        return best;
    }
};

// ------------------------------------------------------------- payload
// Cross-track roll that brings a target just inside the swath. roll_centre is
// the roll that would centre it; half_fov is half the cross-track field of view.
// Zero when the target already lies in the nadir swath (the imager ICD requires nadir).
inline double roll_to_swath(double roll_centre, double half_fov) {
    double r = std::fabs(roll_centre) - half_fov;
    return r <= 0 ? 0.0 : std::copysign(r, roll_centre);
}

// Imager storage model (imager control ICD): capture writes sessions to the camera's flash;
// READ OUT SESSION later copies the selected part (band / line / packet filters)
// to OBC storage over LVDS at a fixed rate; downlink drains OBC storage.
struct PayloadStore {
    double cam_bits = 0, cam_cap = 128 * 8e9;   // camera flash
    double obc_bits = 0, obc_cap = 32 * 8e9;    // OBC SD / eMMC
    double readout_bps = 100e6;                 // HSDIF, fixed
    double select = 1.0;                        // fraction of captured data kept at readout
    double discarded = 0;

    // One readout step of dt seconds. Returns bits written to OBC storage.
    double readout(double dt) {
        double room = obc_cap - obc_bits;
        if (cam_bits <= 0 || room <= 0 || select <= 0) return 0;
        double kept = std::min({readout_bps * dt, cam_bits * select, room});
        double scanned = kept / select;
        cam_bits -= scanned;
        obc_bits += kept;
        discarded += scanned - kept;
        return kept;
    }
};

// ------------------------------------------------------------ pointing laws
// Attitude that puts body axis b1 exactly on target t1 and body axis b2 as
// close as possible to t2 (the "align-constrain" or TRIAD construction).
inline Quat align_constrain(Vec3 b1, Vec3 t1, Vec3 b2, Vec3 t2) {
    b1 = unit(b1);
    t1 = unit(t1);
    if (norm(cross(b1, b2)) < 1e-6) b2 = any_perp(b1);
    if (norm(cross(t1, t2)) < 1e-6) t2 = any_perp(t1);
    Vec3 bx = unit(cross(b1, b2)), tx = unit(cross(t1, t2));
    Mat3 B = Mat3::from_cols(b1, bx, cross(b1, bx));
    Mat3 T = Mat3::from_cols(t1, tx, cross(t1, tx));
    return Quat::from_matrix(T * B.T());
}

// Secondary body axis that is not parallel to the primary.
inline Vec3 secondary_axis(const Vec3& primary, const Vec3& preferred) {
    if (norm(cross(primary, preferred)) > 0.1) return preferred;
    return std::fabs(primary.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
}

}  // namespace cs
