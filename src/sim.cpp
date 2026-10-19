// 3U hyperspectral CubeSat mission simulator.
//
//   Pass 1  propagate the orbit (J2..J6 + drag, RK4) and compute Sun, eclipse,
//           ground-station geometry and imaging-target visibility.
//   Pass 2  plan imaging and downlink windows, then run the onboard executor:
//           attitude mode logic with rate/acceleration-limited slews, solar
//           array power with ray-cast self-shadowing and Earth albedo, panel
//           thermal, battery, payload data and an adaptive-rate RF link.
//
// Usage:
//   cubesat_sim [config.cfg] [--out DIR] [--viewer-dir DIR] [--quiet]
//   cubesat_sim [config.cfg] --serve [PORT]     web UI at http://127.0.0.1:PORT
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "adcs.hpp"
#include "astro.hpp"
#include "config.hpp"
#include "igrf.hpp"
#include "earth_radiation.hpp"
#include "geometry.hpp"
#include "models.hpp"
#include "orbit.hpp"
#include "vecmath.hpp"

using namespace cs;
namespace fs = std::filesystem;

// ------------------------------------------------------------------ calendar
struct Civil { int y, mo, d, h, mi; double s; };

static Civil parse_iso(const std::string& s) {
    Civil c{};
    if (std::sscanf(s.c_str(), "%d-%d-%dT%d:%d:%lf", &c.y, &c.mo, &c.d, &c.h, &c.mi, &c.s) < 5)
        throw std::runtime_error("bad start_utc '" + s + "', expected YYYY-MM-DDTHH:MM:SS");
    return c;
}

// Howard Hinnant's civil_from_days.
static void civil_from_days(long z, int& y, int& m, int& d) {
    z += 719468;
    long era = (z >= 0 ? z : z - 146096) / 146097;
    unsigned doe = unsigned(z - era * 146097);
    unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    long yy = long(yoe) + era * 400;
    unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    unsigned mp = (5 * doy + 2) / 153;
    d = int(doy - (153 * mp + 2) / 5 + 1);
    m = int(mp < 10 ? mp + 3 : mp - 9);
    y = int(yy + (m <= 2));
}

static std::string jd_to_iso(double jd) {
    double unix_s = (jd - 2440587.5) * 86400.0;
    long secs = long(std::floor(unix_s + 0.5));
    long days = secs >= 0 ? secs / 86400 : (secs - 86399) / 86400;
    long rem = secs - days * 86400;
    int y, m, d;
    civil_from_days(days, y, m, d);
    char buf[32];
    std::snprintf(buf, sizeof buf, "%04d-%02d-%02dT%02d:%02d:%02ld", y, m, d, int(rem / 3600),
                  int(rem % 3600 / 60), rem % 60);
    return buf;
}

static std::string hms(double sec) {
    char buf[32];
    int s = int(std::lround(sec));
    std::snprintf(buf, sizeof buf, "%02d:%02d:%02d", s / 3600, s % 3600 / 60, s % 60);
    return buf;
}

// ------------------------------------------------------------------- types
enum Mode { SUN = 0, IMAGING = 1, DOWNLINK = 2, SAFE = 3, NADIR_IDLE = 4, DETUMBLE = 5, TUMBLE = 6 };
static const char* MODE_NAME[] = {"SUN-POINT", "IMAGING", "DOWNLINK", "SAFE", "NADIR-IDLE", "DETUMBLE", "TUMBLING"};
constexpr int N_MODES = 7;
enum Phase { PH_INHIBIT = 0, PH_DETUMBLE = 1, PH_MISSION = 2 };

struct Target { std::string name; double lat, lon, radius_km; };

enum TaskStatus { PENDING, ACTIVE, DONE, SKIPPED };
static const char* STATUS_NAME[] = {"pending", "done", "done", "skipped"};

struct Task {
    Mode type = SUN;
    int k0 = 0, k1 = 0;
    int target = -1;
    TaskStatus status = PENDING;
    std::string reason;
    double bits = 0;       // collected (imaging) or downlinked (downlink)
    double active_s = 0;   // time actually imaging / transmitting
    double max_el = 0;
    double roll = 0;  // cross-track roll for imaging, rad
    bool preempted = false;
    int slew_start_k = -1;
};

struct Event { double t; std::string kind, detail; };

struct OrbitStat {
    int n = 0;
    double t0 = 0, t1 = 0;
    double eclipse_s = 0, sunlit_s = 0;
    double e_avail_wh = 0, e_gen_wh = 0, e_load_wh = 0, e_curt_wh = 0;
    double p_sunlit_sum = 0;
    double soc_min = 1, soc_max = 0;
    double beta_deg = 0;
    double imaging_s = 0, downlink_s = 0;
};

// --------------------------------------------------------------- output fmt
struct Column {
    std::string name;
    int decimals;
    std::vector<double> v;
};

struct Table {
    std::vector<Column> cols;
    int add(const std::string& n, int dec) { cols.push_back({n, dec, {}}); return int(cols.size()) - 1; }
    void push(int i, double x) { cols[i].v.push_back(x); }
};

static void fmt_num(std::string& out, double x, int dec) {
    char buf[40];
    if (!std::isfinite(x)) { out += "null"; return; }
    int n = std::snprintf(buf, sizeof buf, "%.*f", dec, x);
    // trim trailing zeros for compactness
    if (dec > 0) {
        while (n > 1 && buf[n - 1] == '0') --n;
        if (buf[n - 1] == '.') --n;
    }
    buf[n] = 0;
    if (std::strcmp(buf, "-0") == 0) { out += "0"; return; }
    out += buf;
}

static std::string json_str(const std::string& s) {
    std::string o = "\"";
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += char(c); }
        else if (c == '\n') o += "\\n";
        else if (c == '\t') o += "\\t";
        else if (c < 0x20) { char b[8]; std::snprintf(b, sizeof b, "\\u%04x", c); o += b; }
        else o += char(c);
    }
    return o + "\"";
}

static std::string vec_json(const Vec3& v, int dec = 5) {
    std::string o = "[";
    fmt_num(o, v.x, dec); o += ",";
    fmt_num(o, v.y, dec); o += ",";
    fmt_num(o, v.z, dec);
    return o + "]";
}

struct RunResult { std::string json, console; };

static void appendf(std::string& out, const char* fmt, ...) {
    char buf[1024];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    out += buf;
}

// Runs one simulation from a parsed configuration. Returns the viewer JSON and
// the console report; also writes CSV products when out_dir is non-empty.
static RunResult run_simulation(Ini& ini, const std::string& out_dir) {
    RunResult res;
    auto wall0 = std::chrono::steady_clock::now();
    // ------------------------------------------------------------ settings
    Civil ep = parse_iso(ini.str("simulation.start_utc", "2027-03-21T00:00:00"));
    const double jd0 = julian_date(ep.y, ep.mo, ep.d, ep.h, ep.mi, ep.s);
    const double duration_s = ini.num("simulation.duration_hours", 24) * 3600.0;
    const double dt = ini.num("simulation.dt_s", 2.0);
    const double out_every_s = ini.num("simulation.output_every_s", 10.0);
    const double orbit_sub = ini.num("simulation.orbit_substep_s", 1.0);
    const int N = int(std::ceil(duration_s / dt));
    const int out_every = std::max(1, int(std::lround(out_every_s / dt)));

    // Orbit
    const double alt0 = ini.num("orbit.altitude_km", 500);
    const double ecc = ini.num("orbit.eccentricity", 0.0);
    const bool sso = ini.flag("orbit.sun_synchronous", true);
    Elements el0;
    el0.a = R_EARTH + alt0;
    el0.e = ecc;
    el0.i = sso ? sso_inclination(el0.a, ecc) : ini.num("orbit.inclination_deg", 97.4) * DEG;
    const double ltan = ini.num("orbit.ltan_hours", 10.5);
    el0.raan = ini.has("orbit.raan_deg") ? ini.num("orbit.raan_deg", 0) * DEG : raan_for_ltan(jd0, ltan);
    ini.used.insert("orbit.raan_deg");
    el0.argp = ini.num("orbit.arg_perigee_deg", 0) * DEG;
    el0.nu = ini.num("orbit.arg_latitude_deg", 0) * DEG - el0.argp;

    // Geometry
    GeometryConfig gc;
    gc.body_w = ini.num("geometry.body_width_m", gc.body_w);
    gc.body_l = ini.num("geometry.body_length_m", gc.body_l);
    gc.panel_w = ini.num("geometry.panel_width_m", gc.panel_w);
    gc.panel_l = ini.num("geometry.panel_length_m", gc.panel_l);
    gc.hinge_end = ini.str("geometry.hinge_end", "+Z") == "-Z" ? -1 : +1;
    int n_failed = 0;
    {
        auto ang = ini.list("geometry.panel_angles_deg", {135, 135, 135, 135});
        if (ang.size() == 1) ang.assign(4, ang[0]);
        if (ang.size() != 4) throw std::runtime_error("geometry.panel_angles_deg needs 1 or 4 values");
        for (int k = 0; k < 4; ++k) gc.angles_deg[k] = ang[k];
        auto bc = ini.list("geometry.body_cells", {0, 0, 0, 0, 0, 0});
        if (bc.size() != 6) throw std::runtime_error("geometry.body_cells needs 6 values (+X,-X,+Y,-Y,+Z,-Z)");
        for (int k = 0; k < 6; ++k) gc.body_cells[k] = bc[k];
        // Deployment-failure case (TPP OI-30): a failed panel never leaves the stowed position.
        auto fp = ini.list("geometry.failed_panels", {0, 0, 0, 0});
        if (fp.size() != 4) throw std::runtime_error("geometry.failed_panels needs 4 values (+X,+Y,-X,-Y), 1 = never deploys");
        for (int k = 0; k < 4; ++k)
            if (fp[k] != 0) { gc.angles_deg[k] = 0; ++n_failed; }
    }
    gc.cells_front = ini.num("geometry.cells_front", gc.cells_front);
    gc.cells_back = ini.num("geometry.cells_back", gc.cells_back);
    gc.cell_area_cm2 = ini.num("geometry.cell_area_cm2", gc.cell_area_cm2);
    gc.samples_u = int(ini.num("geometry.shadow_samples_u", gc.samples_u));
    gc.samples_v = int(ini.num("geometry.shadow_samples_v", gc.samples_v));
    const Geometry geo(gc);
    GeometryConfig gc_stow = gc;
    for (double& a : gc_stow.angles_deg) a = 0;
    const Geometry geo_stow(gc_stow);
    const size_t NF = geo.faces.size();

    // Mass budget -> total mass, centre of mass, inertia (deployed and stowed)
    const double panel_kg = ini.num("thermal.panel_mass_kg", 0.12);
    std::vector<MassItem> mass_items;
    for (auto& [name, val] : ini.sections["mass_budget"]) {
        auto v = parse_list(val);
        if (v.size() < 7) throw std::runtime_error("mass_budget '" + name + "' needs mass, cx, cy, cz, sx, sy, sz [, shell]");
        mass_items.push_back({name, v[0], {v[1], v[2], v[3]}, {v[4], v[5], v[6]}, v.size() > 7 && v[7] != 0});
    }
    Vec3 com_off;
    { auto v = ini.list("adcs.com_offset_m", {0.0, 0.0, 0.0}); com_off = {v.at(0), v.at(1), v.at(2)}; }
    const double mass_cfg = ini.num("orbit.mass_kg", 4.0);
    const MassProps mp_dep = mass_items.empty() ? mass_properties(geo, mass_cfg, panel_kg, com_off) : mass_properties(geo, mass_items, panel_kg);
    const MassProps mp_stow = mass_items.empty() ? mass_properties(geo_stow, mass_cfg, panel_kg, com_off) : mass_properties(geo_stow, mass_items, panel_kg);

    // Forces
    ForceModel fm;
    fm.jmax = std::clamp(int(ini.num("orbit.zonal_degree", 6)), 1, 6);
    fm.drag = ini.flag("orbit.drag", true);
    fm.cd = ini.num("orbit.drag_cd", 2.2);
    fm.mass_kg = mp_dep.mass;
    fm.density_scale = ini.num("orbit.density_scale", 1.0);
    {
        double tot = 0;
        for (size_t k = 0; k < geo.plates.size(); ++k)
            tot += 4 * geo.plates[k].hu * geo.plates[k].hv * (k < 6 ? 1.0 : 2.0);
        fm.area_m2 = ini.num("orbit.drag_area_m2", tot / 4.0);  // mean projected area
    }

    // Solar / thermal / battery
    SolarCellModel cell;
    cell.eff_ref = ini.num("solar.cell_efficiency", cell.eff_ref);
    cell.t_ref_c = ini.num("solar.ref_temp_c", cell.t_ref_c);
    cell.temp_coeff = ini.num("solar.temp_coeff_per_k", cell.temp_coeff);
    cell.iam_ar = ini.num("solar.iam_ar", cell.iam_ar);
    cell.iam_diffuse = ini.num("solar.iam_diffuse", cell.iam_diffuse);
    cell.derate = ini.num("solar.derate", cell.derate);
    cell.degradation = ini.num("solar.degradation", cell.degradation);
    const double mppt_eff = ini.num("solar.mppt_efficiency", 0.93);
    EarthRadiation erad;
    erad.albedo = ini.num("solar.earth_albedo", erad.albedo);
    erad.olr = ini.num("solar.earth_ir_wm2", erad.olr);
    erad.n_lambda = int(ini.num("solar.earth_grid_rings", erad.n_lambda));
    erad.n_phi = int(ini.num("solar.earth_grid_sectors", erad.n_phi));
    erad.init();
    const bool use_albedo = ini.flag("solar.include_albedo", true);
    const double albedo_update_s = ini.num("solar.albedo_update_s", 10.0);

    PanelThermal pth;
    pth.mass_kg = ini.num("thermal.panel_mass_kg", pth.mass_kg);
    pth.cp = ini.num("thermal.panel_cp", pth.cp);
    pth.alpha_front = ini.num("thermal.alpha_front", pth.alpha_front);
    pth.eps_front = ini.num("thermal.eps_front", pth.eps_front);
    pth.alpha_back = ini.num("thermal.alpha_back", pth.alpha_back);
    pth.eps_back = ini.num("thermal.eps_back", pth.eps_back);
    pth.t_k = ini.num("thermal.initial_temp_c", 20) + 273.15;
    const double body_temp_c = ini.num("thermal.body_temp_c", 20);
    PanelThermal panel_th[4] = {pth, pth, pth, pth};

    Battery bat;
    bat.capacity_wh = ini.num("battery.capacity_wh", 40);
    bat.energy_wh = bat.capacity_wh * ini.num("battery.initial_soc", 0.8);
    bat.charge_eff = ini.num("battery.charge_eff", 0.95);
    bat.discharge_eff = ini.num("battery.discharge_eff", 0.95);
    bat.max_charge_w = ini.num("battery.max_charge_w", 20);
    bat.max_discharge_w = ini.num("battery.max_discharge_w", 40);
    const double soc_min_img = ini.num("battery.min_soc_imaging", 0.50);
    const double soc_min_dl = ini.num("battery.min_soc_downlink", 0.35);
    const double soc_safe = ini.num("battery.safe_soc", 0.25);
    const double soc_safe_exit = ini.num("battery.safe_exit_soc", 0.40);

    // Loads (subsystem level)
    // EPS board: SAMD21 microcontroller + regulators; runs deployment timers, beacon and detumbling.
    const double p_eps_mcu = ini.num("loads.eps_mcu_w", 0.03);
    const double p_eps_board = ini.num("loads.eps_board_w", 0.15);
    // UHF beacon: RFM96W (SX1276) + turnstile antenna
    const double p_bcn_tx = ini.num("loads.beacon_tx_w", 0.40);
    const double p_bcn_rx = ini.num("loads.beacon_rx_w", 0.034);
    const double bcn_period = ini.num("loads.beacon_period_s", 30);
    const double bcn_air = ini.num("loads.beacon_airtime_s", 0.5);
    const double bcn_rx_duty = ini.num("loads.beacon_rx_duty", 0.2);
    const double p_beacon = p_bcn_tx * bcn_air / bcn_period + p_bcn_rx * bcn_rx_duty;
    // OBC + TTC: ADRV9364-Z7020 SOM (Zynq-7020 + AD9364) with an external S-band PA
    const double p_som = ini.num("loads.som_base_w", 3.6);
    const double p_obc_acc = ini.num("loads.obc_accelerator_extra_w", 1.0);
    const double p_ad_rx = ini.num("loads.ad9364_rx_w", 0.36);
    const double p_ad_tx = ini.num("loads.ad9364_tx_w", 0.64);
    const bool ttc_fdd = ini.flag("loads.ttc_fdd", false);
    const double pa_eff = ini.num("loads.pa_efficiency", 0.31);
    const double p_other = ini.num("loads.other_w", 0.1);
    const double p_img = ini.num("loads.imager_imaging_w", 7.75);
    const double p_img_idle = ini.num("loads.imager_idle_w", 4.50);
    const double p_img_ro = ini.num("loads.imager_readout_w", 4.50);
    const double warmup_s = ini.num("loads.imager_warmup_s", 60);
    const double p_heater = ini.num("loads.eclipse_heater_w", 0.5);

    // ADCS hardware (CubeSpace)
    const WheelSpec wheel = wheel_spec(ini.str("adcs.wheel_model", "CW0057"));
    const TorquerSpec tq[3] = {torquer_spec(ini.str("adcs.torquer_x", "CR0002")),
                               torquer_spec(ini.str("adcs.torquer_y", "CR0003")),
                               torquer_spec(ini.str("adcs.torquer_z", "CR0003"))};
    const double tq_v = ini.num("adcs.torquer_voltage", 5.0);
    const double mtq_duty_max = ini.num("adcs.mtq_max_duty", 0.8);
    const double p_adcs_cpu = ini.num("adcs.computer_w", 0.0);
    const double p_gyro = ini.num("adcs.gyro_w", 0.05);
    const double p_mag = ini.num("adcs.magnetometer_w", 0.03);
    const double p_fss = ini.num("adcs.sun_sensor_w", 0.10);
    const bool use_str = ini.flag("adcs.star_tracker", false);
    const double p_str = ini.num("adcs.star_tracker_w", 0.192);
    Vec3 m_res;
    { auto v = ini.list("adcs.residual_dipole_am2", {0.005, 0.005, 0.01}); m_res = {v.at(0), v.at(1), v.at(2)}; }
    const double k_dump = ini.num("adcs.dump_gain", 0.002);
    const double dyn_sub = ini.num("adcs.dyn_substep_s", 0.1);

    // Flight plan
    const bool launch = ini.str("flightplan.start", "mission") == "launch";
    const double t_deploy = ini.num("flightplan.deploy_inhibit_s", 1800);
    const double t_rf = ini.num("flightplan.rf_inhibit_s", 2700);
    const double t_adcs = t_deploy + ini.num("flightplan.adcs_start_delay_s", 60);
    const double p_pdeploy = ini.num("flightplan.panel_deploy_w", 2.0), s_pdeploy = ini.num("flightplan.panel_deploy_s", 10);
    const double p_mdeploy = ini.num("flightplan.mag_deploy_w", 0.0), s_mdeploy = ini.num("flightplan.mag_deploy_s", 0);
    const double p_adeploy = ini.num("flightplan.antenna_deploy_w", 2.0), s_adeploy = ini.num("flightplan.antenna_deploy_s", 10);
    const bool obc_in_leop = ini.flag("flightplan.obc_on_during_detumble", false);
    const double obc_boot_s = ini.num("flightplan.obc_boot_s", 60);
    const int release_seed = int(ini.num("flightplan.release_seed", 1));
    Vec3 w0;
    { auto v = ini.list("flightplan.initial_omega_deg_s", {2, -2, 2}); w0 = Vec3{v.at(0), v.at(1), v.at(2)} * DEG; }
    const double det_exit = ini.num("flightplan.detumble_exit_deg_s", 0.5) * DEG;
    const double det_hold = ini.num("flightplan.detumble_hold_s", 300);
    const double bdot_T = ini.num("flightplan.bdot_period_s", 1.0);
    const double commissioning_s = ini.num("flightplan.commissioning_hours", 0) * 3600;

    // Attitude
    const Vec3 ax_sun = parse_axis(ini.str("attitude.sun_axis", "+Z"));
    const Vec3 ax_img = parse_axis(ini.str("attitude.imager_axis", "-Z"));
    const Vec3 ax_ant = parse_axis(ini.str("attitude.antenna_axis", "-Z"));
    const Vec3 ax_track = parse_axis(ini.str("attitude.imager_along_track_axis", "+Y"));
    // "auto": the fastest slew the wheels can support with 20 % momentum/torque reserve
    const double i_max = std::max({mp_dep.inertia.m[0][0], mp_dep.inertia.m[1][1], mp_dep.inertia.m[2][2]});
    const bool rate_auto = ini.str("attitude.max_rate_deg_s", "auto") == "auto";
    const bool acc_auto = ini.str("attitude.max_accel_deg_s2", "auto") == "auto";
    const double w_max = rate_auto ? 0.8 * wheel.h_rated / i_max : ini.num("attitude.max_rate_deg_s", 1.0) * DEG;
    const double a_max = acc_auto ? 0.8 * wheel.torque / i_max : ini.num("attitude.max_accel_deg_s2", 0.1) * DEG;
    const double settle_s = ini.num("attitude.settle_s", 20);
    const double tol = ini.num("attitude.pointing_tolerance_deg", 2.0) * DEG;
    const bool eclipse_nadir = ini.str("attitude.eclipse_mode", "sun") == "nadir";
    const bool idle_nadir = ini.str("attitude.idle_mode", "sun") == "nadir";

    // Payload
    const double px = ini.num("payload.pixels", 4096);
    const double bands = ini.num("payload.bands", 32);
    const double bits = ini.num("payload.bits", 12);
    const double gsd_ref = ini.num("payload.gsd_m", 30), gsd_alt = ini.num("payload.gsd_ref_altitude_km", 500);
    const double swath_ref = ini.num("payload.swath_km", 120);
    const double binning = ini.num("payload.spatial_binning", 1);
    const double cr = ini.num("payload.compression_ratio", 1.8);
    const double storage_bits = ini.num("payload.storage_gb", 128) * 8e9;
    const double min_sun_el = ini.num("payload.min_sun_elevation_deg", 20) * DEG;
    const double min_img_s = ini.num("payload.min_window_s", 4);
    const double max_roll = ini.num("payload.max_off_nadir_deg", 30) * DEG;
    const double scene_km = ini.num("payload.scene_length_km", 100);
    const double stored0 = ini.num("payload.initial_storage_gb", 0) * 8e9;
    const bool img_continuous = ini.str("payload.imaging_mode", "targets") == "continuous";
    const double half_fov = std::atan(swath_ref / 2 / gsd_alt);  // swath scales with altitude: constant angle
    PayloadStore ps;
    ps.cam_cap = storage_bits;
    ps.obc_cap = ini.num("payload.obc_storage_gb", 32) * 8e9;
    ps.obc_bits = std::min(stored0, ps.obc_cap);
    ps.readout_bps = ini.num("payload.readout_mbps", 100) * 1e6;
    ps.select = std::clamp(ini.num("payload.readout_select_fraction", 1.0), 0.01, 1.0);
    const double sun_excl = ini.num("payload.sun_exclusion_deg", 11) * DEG;
    const double sun_fov_max_s = ini.num("payload.max_sun_in_fov_s", 900);
    const double v_bus = ini.num("battery.nominal_v", 7.4);

    // Link / ground station
    LinkModel link;
    link.freq_hz = ini.num("link.freq_ghz", 2.245) * 1e9;
    link.tx_power_w = ini.num("link.tx_power_w", link.tx_power_w);
    link.ant_gain_dbi = ini.num("link.antenna_gain_dbi", link.ant_gain_dbi);
    link.ant_hpbw_deg = ini.num("link.antenna_hpbw_deg", link.ant_hpbw_deg);
    link.tx_losses_db = ini.num("link.tx_losses_db", link.tx_losses_db);
    link.gs_g_over_t = ini.num("link.gs_g_over_t_dbk", link.gs_g_over_t);
    link.path_losses_db = ini.num("link.path_losses_db", link.path_losses_db);
    link.req_ebn0_db = ini.num("link.required_ebn0_db", link.req_ebn0_db);
    link.impl_loss_db = ini.num("link.implementation_loss_db", link.impl_loss_db);
    link.margin_db = ini.num("link.required_margin_db", link.margin_db);
    link.rates_mbps = ini.list("link.rates_mbps", link.rates_mbps);
    link.set_coding(ini.str("link.coding", "turbo_1_2"));
    if (ini.has("link.required_ebn0_db")) link.req_ebn0_db = ini.num("link.required_ebn0_db", link.req_ebn0_db);
    link.max_symbol_rate = ini.num("link.max_symbol_rate_msps", 5.0) * 1e6;
    link.frame_efficiency = ini.num("link.frame_efficiency", 0.97);
    const double gs_mask = ini.num("link.min_elevation_deg", 10) * DEG;
    const bool prio_imaging = ini.str("link.priority", "imaging") == "imaging";

    Geodetic gs{ini.num("ground_station.lat_deg", 28.3639) * DEG, ini.num("ground_station.lon_deg", 75.5870) * DEG,
                ini.num("ground_station.alt_m", 300) / 1000.0};
    const std::string gs_name = ini.str("ground_station.name", "Ground station");
    const Vec3 gs_ecef = geodetic_to_ecef(gs);

    std::vector<Target> targets;
    for (auto& [name, val] : ini.sections["targets"]) {
        auto v = parse_list(val);
        if (v.size() != 3) throw std::runtime_error("target '" + name + "' needs lat, lon, radius_km");
        targets.push_back({name, v[0] * DEG, v[1] * DEG, v[2]});
    }
    if (img_continuous) targets.push_back({"Continuous nadir strip", 0, 0, 0});
    res.console += ini.warn_unused({"targets"});
    if (duration_s > 30 * 86400.0) throw std::runtime_error("duration_hours limited to 30 days");
    if (dt < 0.1 || dt > 60) throw std::runtime_error("dt_s must be within 0.1 .. 60 s");

    // ============================================================ pass 1
    std::vector<State> st(N + 1);
    std::vector<double> gmst_v(N + 1), sunfrac(N + 1), flux(N + 1);
    std::vector<Vec3> sun_hat(N + 1);
    std::vector<Geodetic> sub(N + 1);
    std::vector<Topo> gst(N + 1);
    std::vector<int> tgt_in(N + 1, -1);
    std::vector<double> tgt_roll(N + 1, 0), tgt_along(N + 1, 1e9);
    std::vector<double> sub_sun_el(N + 1);
    std::vector<Vec3> b_eci(N + 1);  // geomagnetic field, tesla
    const double dec_year = 2000.0 + (jd0 - 2451545.0) / 365.25;
    const Igrf igrf(dec_year);
    std::vector<Vec3> tgt_ecef;
    for (auto& tg : targets) tgt_ecef.push_back(geodetic_to_ecef({tg.lat, tg.lon, 0}));

    st[0] = elements_to_state(el0);
    const int nsub = std::max(1, int(std::ceil(dt / orbit_sub)));
    for (int k = 0; k <= N; ++k) {
        if (k > 0) {
            State s = st[k - 1];
            for (int j = 0; j < nsub; ++j) s = rk4_step(s, dt / nsub, fm);
            st[k] = s;
        }
        double jd = jd0 + k * dt / 86400.0;
        gmst_v[k] = gmst(jd);
        Vec3 rs = sun_position_eci(jd);
        sun_hat[k] = unit(rs);
        sunfrac[k] = sunlight_fraction(st[k].r, rs);
        double d_au = norm(rs - st[k].r) / AU_KM;
        flux[k] = SOLAR_CONST_1AU / (d_au * d_au);
        Vec3 r_ecef = eci_to_ecef(st[k].r, gmst_v[k]);
        sub[k] = ecef_to_geodetic(r_ecef);
        gst[k] = look_angles(gs, gs_ecef, r_ecef);
        b_eci[k] = ecef_to_eci(igrf.field_ecef(r_ecef), gmst_v[k]);
        // solar elevation at the sub-satellite point
        Vec3 e, n, u;
        enu_basis({sub[k].lat, sub[k].lon, 0}, e, n, u);
        sub_sun_el[k] = std::asin(std::clamp(dot(eci_to_ecef(sun_hat[k], gmst_v[k]), u), -1.0, 1.0));
        // Pushbroom imaging with a cross-track roll: a target is being imaged while
        // it lies within half a scene of the satellite's cross-track plane, and the
        // roll needed to centre it in the swath is within the off-nadir limit.
        if (img_continuous) {
            if (sunfrac[k] > 0.99 && sub_sun_el[k] >= min_sun_el) { tgt_in[k] = int(targets.size()) - 1; tgt_roll[k] = 0; tgt_along[k] = 0; }
        } else if (sunfrac[k] > 0.99) {
            const double rn = norm(st[k].r);
            Vec3 rh = st[k].r / rn, hh = unit(cross(st[k].r, st[k].v)), wh = cross(hh, rh);
            double best = 1e9;
            for (size_t t = 0; t < targets.size(); ++t) {
                Vec3 th = unit(ecef_to_eci(tgt_ecef[t], gmst_v[k]));
                double along = std::atan2(dot(th, wh), dot(th, rh)) * R_EARTH;  // km, signed
                double half = scene_km / 2;  // one strip of scene_length_km centred on closest approach
                if (std::fabs(along) > half) continue;
                double lam_c = std::asin(std::clamp(dot(th, hh), -1.0, 1.0));  // cross-track central angle
                // nearest edge of the target area, as a ground central angle, then the look angle to it
                double lam_e = std::max(0.0, std::fabs(lam_c) - targets[t].radius_km / R_EARTH);
                double look = std::copysign(std::atan2(std::sin(lam_e), rn / R_EARTH - std::cos(lam_e)), lam_c);
                double roll = roll_to_swath(look, half_fov);  // 0 when any part of the area is in the nadir swath
                if (std::fabs(roll) > max_roll + 1e-9) continue;
                Vec3 e, n, u;
                enu_basis({targets[t].lat, targets[t].lon, 0}, e, n, u);
                double sun_el_t = std::asin(std::clamp(dot(eci_to_ecef(sun_hat[k], gmst_v[k]), u), -1.0, 1.0));
                if (sun_el_t < min_sun_el) continue;
                const double score = std::fabs(roll) + 1e-6 * targets[t].radius_km;  // ties: the specific (smaller) target wins
                if (score < best) {
                    best = score;
                    tgt_in[k] = int(t);
                    tgt_roll[k] = roll;
                    tgt_along[k] = along;
                }
            }
        }
    }

    // ============================================================ planning
    std::vector<Task> tasks;
    auto build_windows = [&](auto pred, Mode type, auto label) {
        int k = 0;
        while (k <= N) {
            if (!pred(k)) { ++k; continue; }
            int a = k;
            int lab = label(k);
            while (k <= N && pred(k) && label(k) == lab) ++k;
            Task t;
            t.type = type;
            t.k0 = a;
            t.k1 = k - 1;
            t.target = lab;
            tasks.push_back(t);
        }
    };
    build_windows([&](int k) { return tgt_in[k] >= 0; }, IMAGING, [&](int k) { return tgt_in[k]; });
    tasks.erase(std::remove_if(tasks.begin(), tasks.end(),
                               [&](const Task& t) { return (t.k1 - t.k0 + 1) * dt < min_img_s; }),
                tasks.end());
    std::vector<Task> img_tasks = tasks;
    std::vector<Task> dl_tasks;
    {
        std::vector<Task> tmp;
        std::swap(tasks, tmp);
        build_windows([&](int k) { return gst[k].el >= gs_mask; }, DOWNLINK, [](int) { return -1; });
        dl_tasks = tasks;
        tasks = tmp;
    }
    // Conflict resolution: the higher-priority window wins; the other is clipped/split.
    auto clip = [&](std::vector<Task>& low, const std::vector<Task>& high) {
        std::vector<Task> out;
        for (Task t : low) {
            std::vector<Task> pieces{t};
            for (const Task& h : high) {
                std::vector<Task> next;
                for (Task p : pieces) {
                    if (h.k1 < p.k0 || h.k0 > p.k1) { next.push_back(p); continue; }
                    if (p.k0 < h.k0) { Task a = p; a.k1 = h.k0 - 1; next.push_back(a); }
                    if (p.k1 > h.k1) { Task b = p; b.k0 = h.k1 + 1; next.push_back(b); }
                }
                pieces = next;
            }
            for (auto& p : pieces)
                if ((p.k1 - p.k0 + 1) * dt >= 10) out.push_back(p);
        }
        low = out;
    };
    if (prio_imaging) clip(dl_tasks, img_tasks); else clip(img_tasks, dl_tasks);
    tasks = img_tasks;
    tasks.insert(tasks.end(), dl_tasks.begin(), dl_tasks.end());
    std::sort(tasks.begin(), tasks.end(), [](const Task& a, const Task& b) { return a.k0 < b.k0; });
    for (auto& t : tasks) {
        if (t.type == DOWNLINK)
            for (int k = t.k0; k <= t.k1; ++k) t.max_el = std::max(t.max_el, gst[k].el);
        if (t.type == IMAGING) {  // roll at closest along-track approach
            int kb = t.k0;
            for (int k = t.k0; k <= t.k1; ++k)
                if (std::fabs(tgt_along[k]) < std::fabs(tgt_along[kb])) kb = k;
            t.roll = tgt_roll[kb];
        }
    }

    // ============================================================ pass 2
    auto gs_eci = [&](int k) { return ecef_to_eci(gs_ecef, gmst_v[k]); };
    auto law = [&](Mode m, int k, double roll = 0) -> Quat {
        const Vec3& r = st[k].r;
        Vec3 nadir = -unit(r);
        switch (m) {
            case IMAGING:
            case NADIR_IDLE: {
                Vec3 hh = unit(cross(r, st[k].v));
                Vec3 look = std::cos(roll) * nadir + std::sin(roll) * hh;
                return align_constrain(ax_img, look, secondary_axis(ax_img, ax_track), cross(hh, -nadir));
            }
            case DOWNLINK:
                return align_constrain(ax_ant, gs_eci(k) - r, secondary_axis(ax_ant, ax_sun), sun_hat[k]);
            default:
                return align_constrain(ax_sun, sun_hat[k], secondary_axis(ax_sun, {1, 0, 0}), nadir);
        }
    };

    // Output table
    Table tb;
    const int C_t = tb.add("t", 0), C_x = tb.add("x", 1), C_y = tb.add("y", 1), C_z = tb.add("z", 1);
    const int C_qw = tb.add("qw", 5), C_qx = tb.add("qx", 5), C_qy = tb.add("qy", 5), C_qz = tb.add("qz", 5);
    const int C_sx = tb.add("sx", 5), C_sy = tb.add("sy", 5), C_sz = tb.add("sz", 5);
    const int C_gm = tb.add("gmst", 6), C_lat = tb.add("lat", 3), C_lon = tb.add("lon", 3), C_alt = tb.add("alt", 2);
    const int C_mode = tb.add("mode", 0), C_slew = tb.add("slew", 0), C_perr = tb.add("perr", 2);
    const int C_sf = tb.add("sunfrac", 3), C_beta = tb.add("beta", 2);
    const int C_ps = tb.add("p_solar", 2), C_pl = tb.add("p_load", 2), C_pn = tb.add("p_net", 2);
    const int C_pc = tb.add("p_curtail", 2), C_pa = tb.add("p_albedo", 2), C_soc = tb.add("soc", 4);
    const int C_bwh = tb.add("battery_wh", 2);
    std::vector<int> C_fp, C_fi, C_fl;
    for (size_t f = 0; f < NF; ++f) {
        C_fp.push_back(tb.add("face_p_" + std::to_string(f), 2));
        C_fi.push_back(tb.add("face_inc_" + std::to_string(f), 1));
        C_fl.push_back(tb.add("face_lit_" + std::to_string(f), 2));
    }
    int C_temp[4];
    for (int p = 0; p < 4; ++p) C_temp[p] = tb.add("panel_temp_" + std::to_string(p), 1);
    const int C_snel = tb.add("sun_el_nadir", 1);
    const int C_gel = tb.add("gs_el", 2), C_gaz = tb.add("gs_az", 1), C_grg = tb.add("gs_range", 0);
    const int C_rate = tb.add("link_rate", 2), C_marg = tb.add("link_margin", 1), C_cn0 = tb.add("cn0", 1);
    const int C_store = tb.add("stored_gb", 4), C_down = tb.add("downlinked_gb", 4), C_imgd = tb.add("imaged_gb", 4);
    const int C_imgr = tb.add("imager", 0), C_task = tb.add("task", 0);
    const int C_cam = tb.add("camera_gb", 4), C_obcs = tb.add("obc_gb", 4), C_sunimg = tb.add("sun_imager_angle", 1);
    const int C_asun = tb.add("sun_axis_angle", 1), C_aant = tb.add("antenna_gs_angle", 1), C_anad = tb.add("imager_nadir_angle", 1);
    const int C_load_obc = tb.add("load_obc", 2), C_load_eps = tb.add("load_eps", 2), C_load_ttc = tb.add("load_ttc", 2);
    const int C_load_adcs = tb.add("load_adcs", 3), C_load_img = tb.add("load_imager", 2), C_load_htr = tb.add("load_heater", 2);
    const int C_phase = tb.add("phase", 0);
    const int C_wx = tb.add("wx", 3), C_wy = tb.add("wy", 3), C_wz = tb.add("wz", 3), C_wm = tb.add("wmag", 3);
    const int C_mx = tb.add("mtq_x", 4), C_my = tb.add("mtq_y", 4), C_mz = tb.add("mtq_z", 4), C_mw = tb.add("mtq_w", 3);
    const int C_hx = tb.add("hw_x", 4), C_hy = tb.add("hw_y", 4), C_hz = tb.add("hw_z", 4);
    const int C_rx = tb.add("rpm_x", 0), C_ry = tb.add("rpm_y", 0), C_rz = tb.add("rpm_z", 0), C_ww = tb.add("wheel_w", 3);
    const int C_bm = tb.add("bmag", 2), C_tgg = tb.add("tau_gg", 3), C_taero = tb.add("tau_aero", 3);

    std::vector<Event> events;
    std::vector<OrbitStat> orbits;
    OrbitStat cur_orbit;
    cur_orbit.t0 = 0;

    Quat q_act = law(SUN, 0);
    Vec3 w_act{0, 0, 0};

    // Rigid-body / ADCS state
    Phase phase = launch ? PH_INHIBIT : PH_MISSION;
    bool deployed = !launch;
    Mat3 Iner = deployed ? mp_dep.inertia : mp_stow.inertia, Iinv = inverse(Iner);
    Vec3 com = deployed ? mp_dep.com : mp_stow.com;
    // Release attitude: pseudo-random but reproducible from flightplan.release_seed
    auto lcg = [st_ = uint32_t(release_seed * 2654435761u + 12345u)]() mutable { st_ = st_ * 1664525u + 1013904223u; return (st_ >> 8) / 16777216.0; };
    Vec3 rel_axis{lcg() * 2 - 1, lcg() * 2 - 1, lcg() * 2 - 1};
    const double rel_ang = lcg() * TWO_PI;
    RigidState rs{(Quat::from_axis_angle(norm(rel_axis) > 1e-3 ? rel_axis : Vec3{1, 0, 0}, rel_ang) * q_act).normalized(), w0};
    if (!launch) rs = RigidState{q_act, {0, 0, 0}};
    Vec3 H_I{0, 0, 0};           // total angular momentum (body + wheels), inertial
    Vec3 hw{0, 0, 0}, tau_w{0, 0, 0};
    Vec3 m_cmd{0, 0, 0};          // magnetorquer dipole, body
    Vec3 b_prev{0, 0, 0};
    double t_bdot = -1e9, t_below = 0, t_mission = launch ? -1 : 0, t_det_start = -1, t_det_end = -1;
    double e_pa = 0, e_bcn = 0, mtq_e = 0, e_adcs = 0, e_obc = 0, e_ttc = 0, e_eps = 0, e_img = 0, e_htr = 0;
    bool wheel_sat_flag = false;
    double hw_max = 0;
    // B-dot gain (Avanzini & Giulietti 2012): k = 2 n (1 + sin xi_m) J_min
    const double n_orb = std::sqrt(MU_EARTH / std::pow(norm(st[0].r), 3));
    double k_bdot;
    {
        Vec3 hn = unit(cross(st[0].r, st[0].v));
        Vec3 dip = ecef_to_eci(igrf.dipole_axis_ecef(), gmst_v[0]);
        double xi = angle_between(hn, dip);
        double jmin = std::min({mp_dep.inertia.m[0][0], mp_dep.inertia.m[1][1], mp_dep.inertia.m[2][2]});
        k_bdot = 2 * n_orb * (1 + std::fabs(std::sin(xi))) * jmin;
    }
    auto sat_dipole = [&](Vec3 m) {  // scale so no axis exceeds its duty-limited capacity
        double s = 1;
        const double c[3] = {m.x, m.y, m.z};
        for (int i = 0; i < 3; ++i) {
            double lim = tq[i].m_max * mtq_duty_max;
            if (std::fabs(c[i]) > lim) s = std::min(s, lim / std::fabs(c[i]));
        }
        return m * s;
    };
    auto mtq_power = [&](const Vec3& m) {
        const double c[3] = {m.x, m.y, m.z};
        double p = 0;
        for (int i = 0; i < 3; ++i) p += std::fabs(c[i]) / tq[i].m_max * tq_v * tq_v / tq[i].ohm;
        return p;
    };
    auto ext_torque = [&](const Quat& q, int k, const Vec3& b_i, const Vec3& m) {
        Vec3 rb = q.conj().rotate(unit(st[k].r));
        Vec3 bb = q.conj().rotate(b_i);
        Vec3 vrel = (st[k].v - cross(Vec3{0, 0, OMEGA_EARTH}, st[k].r)) * 1000.0;
        double rho = atmos_density(norm(st[k].r) - R_EARTH) * fm.density_scale;
        const Geometry& g = deployed ? geo : geo_stow;
        Vec3 tgg = gravity_gradient_torque(rb, norm(st[k].r), Iner);
        Vec3 taero = aero_torque(g, com, q.conj().rotate(vrel), rho, fm.cd);
        return std::array<Vec3, 3>{tgg, taero, cross(m_res + m, bb)};
    };
    double down_total = 0, imaged_total = 0, ro_total = 0, t_ro = 0;
    double t_sun_fov = 0, sun_fov_run = 0, sun_fov_max = 0, sun_img_min = PI;
    bool sun_fov_flag = false;
    // Rails (TPP v03): switched 5V_PAY, 5V_TTC, 3V3_OBC, 3V3_ADCS, Vsys_RW; always-on 3V3_AON and
    // 5V_AON (magnetorquer drivers, which need 5 V and cannot run from Vsys); Vsys direct (heater, burn wires)
    constexpr int NR = 8;
    const char* RAIL_NAME[NR] = {"5V_PAY   (imager)", "5V_TTC   (S-band PA)", "3V3_OBC  (Zynq SOM, GNSS)", "3V3_ADCS (gyro, sun sensors)",
                                 "Vsys_RW  (wheels)", "3V3_AON  (EPS MCU, beacon, mag)", "5V_AON   (magnetorquer drivers)",
                                 "Vsys     (heater, burn wires)"};
    const double RAIL_V[NR] = {5.0, 5.0, 3.3, 3.3, v_bus, 3.3, 5.0, v_bus};
    double rail_e[NR] = {}, rail_pk[NR] = {};
    double e_avail = 0, e_gen = 0, e_load = 0, e_curt = 0, e_unmet = 0, e_alb = 0, soc_min = 1, soc_max = 0;
    double t_eclipse = 0, t_img = 0, t_dl = 0, t_slew = 0;
    bool in_safe = false;
    size_t next_task = 0;
    std::string prev_state;
    bool prev_ecl = sunfrac[0] < 0.5, prev_vis = gst[0].el >= gs_mask;
    int prev_task_active = -1;
    std::vector<Vec3> nrm_eci(NF);
    std::vector<double> ealb, eir;
    int k_alb = -1;
    Quat q_alb;

    for (int k = 0; k <= N; ++k) {
        const double t = k * dt;
        const State& s = st[k];
        const bool eclipse = sunfrac[k] < 0.5;

        // ---- flight-plan phase transitions
        if (phase == PH_INHIBIT && !deployed && t >= t_deploy) {
            Vec3 Hb = Iner * rs.w;
            deployed = true;
            Iner = mp_dep.inertia; Iinv = inverse(Iner); com = mp_dep.com;
            rs.w = Iinv * Hb;  // angular momentum is conserved through deployment
            events.push_back({t, "DEPLOY", n_failed ? "solar panels released; " + std::to_string(n_failed) + " panel(s) failed to deploy (stay stowed)"
                                                    : std::string("solar panels deployed (inhibit timer expired)")});
        }
        if (phase == PH_INHIBIT && t >= t_adcs) {
            phase = PH_DETUMBLE;
            t_det_start = t;
            char buf[128];
            std::snprintf(buf, sizeof buf, "EPS MCU starts B-dot detumbling (magnetometer + magnetorquers only), |w| = %.2f deg/s", norm(rs.w) * RAD);
            events.push_back({t, "ADCS", buf});
        }
        if (phase == PH_DETUMBLE) {
            t_below = norm(rs.w) < det_exit ? t_below + dt : 0;
            if (t_below >= det_hold) {
                phase = PH_MISSION;
                t_mission = t_det_end = t;
                q_act = rs.q;
                w_act = rs.q.rotate(rs.w);
                H_I = rs.q.rotate(Iner * rs.w);
                hw = tau_w = m_cmd = Vec3{};
                char buf[160];
                std::snprintf(buf, sizeof buf, "detumble complete in %.1f min (|w| = %.2f deg/s) -> OBC boots, wheels on, Sun acquisition",
                              (t - t_det_start) / 60, norm(rs.w) * RAD);
                events.push_back({t, "ADCS", buf});
            }
        }
        const bool tasks_allowed = phase == PH_MISSION && t >= t_mission + std::max(commissioning_s, launch ? obc_boot_s : 0.0);
        const bool rf_allowed = !launch || t >= t_rf;
        if (launch && k > 0 && t >= t_rf && t - dt < t_rf) events.push_back({t, "TTC", "RF inhibit expired: UHF beacon on"});

        // ---- safe-mode hysteresis
        if (!in_safe && bat.soc() < soc_safe) {
            in_safe = true;
            events.push_back({t, "SAFE", "battery below " + std::to_string(int(soc_safe * 100)) + "% -> safe mode"});
        } else if (in_safe && bat.soc() > soc_safe_exit) {
            in_safe = false;
            events.push_back({t, "SAFE", "battery recovered -> nominal"});
        }

        // ---- task selection
        while (next_task < tasks.size() && tasks[next_task].k1 < k) {
            Task& tk = tasks[next_task];
            if (tk.status == ACTIVE) tk.status = DONE;
            if (tk.status == PENDING) { tk.status = SKIPPED; if (tk.reason.empty()) tk.reason = "not reached"; }
            ++next_task;
        }
        int active = -1;
        for (size_t j = next_task; j < tasks.size(); ++j) {
            Task& tk = tasks[j];
            if (tk.k0 > k + int(400 / dt)) break;
            if (tk.status == SKIPPED || tk.status == DONE) continue;
            bool in_win = k >= tk.k0 && k <= tk.k1;
            bool start = in_win;
            if (!in_win && k < tk.k0) {
                double theta = quat_angle(q_act, law(tk.type, tk.k0, tk.roll));
                double lead = theta / w_max + w_max / a_max + settle_s;
                start = (tk.k0 - k) * dt <= lead;
            }
            if (!start) continue;
            if (tk.status == PENDING) {
                double need = tk.type == IMAGING ? soc_min_img : soc_min_dl;
                if (!tasks_allowed) { tk.status = SKIPPED; tk.reason = phase != PH_MISSION ? "LEOP: detumbling" : "commissioning"; }
                else if (in_safe) { tk.status = SKIPPED; tk.reason = "safe mode"; }
                else if (bat.soc() < need) {
                    tk.status = SKIPPED;
                    char buf[64];
                    std::snprintf(buf, sizeof buf, "battery %.0f%% < %.0f%%", bat.soc() * 100, need * 100);
                    tk.reason = buf;
                } else if (tk.type == IMAGING && ps.cam_bits >= ps.cam_cap) {
                    tk.status = SKIPPED; tk.reason = "storage full";
                } else {
                    tk.status = ACTIVE;
                    tk.slew_start_k = k;
                }
                std::string lbl = tk.type == IMAGING ? "imaging " + targets[tk.target].name : "downlink pass";
                events.push_back({t, tk.status == ACTIVE ? "TASK" : "SKIP",
                                  lbl + (tk.status == ACTIVE ? " accepted" : " skipped (" + tk.reason + ")")});
            }
            if (tk.status != ACTIVE) continue;
            // A higher-priority task whose slew must start now preempts the current one.
            Mode hi = prio_imaging ? IMAGING : DOWNLINK;
            if (active < 0 || (tk.type == hi && tasks[active].type != hi)) active = int(j);
        }
        if (in_safe || phase != PH_MISSION) active = -1;
        for (size_t j = next_task; j < tasks.size() && tasks[j].k0 <= k; ++j)
            if (int(j) != active && tasks[j].status == ACTIVE && k <= tasks[j].k1) tasks[j].preempted = true;

        Mode mode = SUN;
        if (phase == PH_INHIBIT) mode = TUMBLE;
        else if (phase == PH_DETUMBLE) mode = DETUMBLE;
        else if (in_safe) mode = SAFE;
        else if (active >= 0) mode = tasks[active].type;
        else if (idle_nadir || (eclipse && eclipse_nadir)) mode = NADIR_IDLE;
        const bool leop = phase != PH_MISSION;
        const bool in_window = active >= 0 && k >= tasks[active].k0 && k <= tasks[active].k1;

        const double roll_cmd = mode == IMAGING ? tasks[active].roll : 0.0;
        Quat q_cmd = law(mode, k, roll_cmd);
        Vec3 w_ff{0, 0, 0};
        if (k < N) w_ff = (law(mode, k + 1, roll_cmd) * q_cmd.conj()).to_rotvec() / dt;
        const double perr = leop ? 0.0 : quat_angle(q_act, q_cmd);
        const bool pointed = !leop && perr <= tol;
        const bool slewing = !leop && !pointed;
        const Quat q_att = leop ? rs.q : q_act;  // attitude actually flown this step
        const Geometry& GEO = deployed ? geo : geo_stow;

        // ---- geometry in body frame
        const Quat qi = q_att.conj();
        const Vec3 sun_b = qi.rotate(sun_hat[k]);
        const double G = flux[k] * sunfrac[k];
        for (size_t f = 0; f < NF; ++f) nrm_eci[f] = q_att.rotate(GEO.faces[f].n);
        // Earth light changes slowly: reuse it unless 10 s passed or the attitude moved > 3 deg.
        if (use_albedo) {
            if (k_alb < 0 || (k - k_alb) * dt >= albedo_update_s || quat_angle(q_att, q_alb) > 3 * DEG || ealb.size() != NF) {
                erad.irradiance(s.r, sun_hat[k], flux[k], nrm_eci, ealb, eir);
                k_alb = k;
                q_alb = q_att;
            }
        } else { ealb.assign(NF, 0); eir.assign(NF, 0); }

        // ---- solar array
        double p_raw = 0, p_alb_raw = 0;
        std::vector<double> fp(NF), finc(NF), flit(NF);
        double panel_elec[4] = {0, 0, 0, 0};
        double q_sol[4][2] = {}, q_ir[4][2] = {};
        for (size_t f = 0; f < NF; ++f) {
            const CellFace& cf = GEO.faces[f];
            double c = dot(cf.n, sun_b);
            finc[f] = std::acos(std::clamp(c, -1.0, 1.0)) * RAD;
            flit[f] = G > 0 ? GEO.lit_fraction(cf, sun_b) : 0.0;
            double e_dir = G * std::max(0.0, c) * flit[f];
            double t_c = cf.panel >= 0 ? panel_th[cf.panel].t_k - 273.15 : body_temp_c;
            double eta = cell.efficiency(t_c) * cell.derate * cell.degradation;
            double p_dir = cf.cell_area * eta * e_dir * cell.iam(c);
            double p_alb = cf.cell_area * eta * ealb[f] * cell.iam_diffuse;
            fp[f] = (p_dir + p_alb) * mppt_eff;
            p_raw += p_dir + p_alb;
            p_alb_raw += p_alb;
            if (cf.panel >= 0) {
                bool back = cf.name.find("back") != std::string::npos;
                panel_elec[cf.panel] += p_dir + p_alb;
                q_sol[cf.panel][back] = e_dir + ealb[f];
                q_ir[cf.panel][back] = eir[f];
            }
        }
        // Panel sides without cells still absorb/radiate: compute their irradiance.
        for (int p = 0; p < 4; ++p) {
            if (gc.cells_back > 0) continue;
            const Plate& pl = GEO.plates[6 + p];
            Vec3 nb = -pl.n;
            double c = dot(nb, sun_b);
            // back side: shadowing is not sampled (no cell face) - use unoccluded cosine
            std::vector<Vec3> one{q_att.rotate(nb)};
            std::vector<double> a1, i1;
            if (use_albedo) erad.irradiance(s.r, sun_hat[k], flux[k], one, a1, i1);
            else { a1 = {0}; i1 = {0}; }
            q_sol[p][1] = G * std::max(0.0, c) + a1[0];
            q_ir[p][1] = i1[0];
        }
        const double p_solar = p_raw * mppt_eff;

        // ---- payload state
        bool img_collect = mode == IMAGING && in_window && pointed && ps.cam_bits < ps.cam_cap;
        int imager_state = 0;  // 0 off, 1 idle/warm, 2 imaging
        if (img_collect) imager_state = 2;
        else if (!in_safe) {
            for (size_t j = next_task; j < tasks.size(); ++j) {
                const Task& tk = tasks[j];
                if (tk.k0 * dt > t + warmup_s + 400) break;
                if (tk.type != IMAGING || tk.status == SKIPPED || tk.status == DONE) continue;
                if (tk.k0 * dt - t <= warmup_s && k <= tk.k1) { imager_state = 1; break; }
            }
        }
        // READ OUT SESSION (imager control ICD): copy captured sessions to OBC storage whenever the camera is
        // not capturing; the camera draws its readout power meanwhile.
        if (imager_state != 2 && tasks_allowed && !in_safe && ps.cam_bits > 0 && ps.obc_bits < ps.obc_cap &&
            bat.soc() >= soc_min_dl)
            imager_state = 3;

        // ---- link
        double rate = 0, margin = -99, cn0 = -99;
        double ant_ang = angle_between(q_att.rotate(ax_ant), gs_eci(k) - s.r);
        if (gst[k].el >= gs_mask) {
            cn0 = link.cn0(gst[k].range, ant_ang);
            if (ant_ang <= link.max_off_boresight_deg * DEG) rate = link.select_rate(cn0, margin);
        }
        const bool transmitting = mode == DOWNLINK && in_window && rate > 0 && ps.obc_bits > 0;
        const bool radio_on = mode == DOWNLINK && in_window;

        // ---- ADCS state and loads
        const Vec3 w_body = leop ? rs.w : q_att.conj().rotate(w_act);
        double p_wheels = 0;
        if (!leop) {
            const double c[3] = {tau_w.x, tau_w.y, tau_w.z};
            for (int i = 0; i < 3; ++i)
                p_wheels += wheel.p_avg2000 + (wheel.p_peak - wheel.p_avg2000) * std::min(1.0, std::fabs(c[i]) / wheel.torque);
        }
        const double p_mtq = mtq_power(m_cmd);
        // Power sequencing: during detumbling only the EPS MCU (B-dot), magnetometer and
        // magnetorquers run; the OBC/TTC SOM boots after detumbling unless configured otherwise.
        const bool obc_on = !launch || phase == PH_MISSION || obc_in_leop;
        double l_adcs = 0;
        if (phase == PH_DETUMBLE) l_adcs = p_mag + p_mtq;
        if (phase == PH_MISSION) l_adcs = p_adcs_cpu + p_gyro + p_mag + p_fss + p_mtq + p_wheels + (use_str ? p_str : 0);
        if (launch && phase == PH_DETUMBLE && t < t_adcs + s_mdeploy) l_adcs += p_mdeploy;
        double l_obc = 0;
        if (obc_on) {
            l_obc = p_som + (imager_state == 2 ? p_obc_acc : 0);
            if (radio_on) l_obc += p_ad_tx + (ttc_fdd ? p_ad_rx : 0);
        }
        const double l_pa = radio_on ? link.tx_power_w / pa_eff : 0;
        const double l_bcn = rf_allowed ? p_beacon : 0;
        double l_eps = p_eps_mcu + p_eps_board + p_other;
        if (launch && t >= t_deploy && t < t_deploy + s_pdeploy) l_eps += p_pdeploy;
        // Burn wires fire one at a time to limit current: panels, then turnstile.
        if (launch && t >= t_deploy + s_pdeploy && t < t_deploy + s_pdeploy + s_adeploy) l_eps += p_adeploy;
        const double l_ttc = l_pa + l_bcn;
        const double l_img = imager_state == 2 ? p_img : imager_state == 1 ? p_img_idle : imager_state == 3 ? p_img_ro : 0;
        const double l_htr = eclipse ? p_heater : 0;
        const double p_load = l_obc + l_eps + l_ttc + l_adcs + l_img + l_htr;
        const double r_adcs = phase == PH_MISSION ? p_adcs_cpu + p_gyro + p_fss + (use_str ? p_str : 0) : 0;
        const bool mtq_live = !(leop && phase != PH_DETUMBLE);
        const double r_aon = p_eps_mcu + p_eps_board + p_other + l_bcn + (mtq_live ? p_mag : 0);
        const double r_aon5 = mtq_live ? p_mtq : 0;
        const double rail_p[NR] = {l_img, l_pa, l_obc, r_adcs, p_wheels, r_aon, r_aon5,
                                   p_load - (l_img + l_pa + l_obc + r_adcs + p_wheels + r_aon + r_aon5)};

        // ---- Sun in the imager field of view (imager ICD: keep > 11 deg off axis; <= 15 min direct Sun)
        const double sun_img = angle_between(q_att.rotate(ax_img), sun_hat[k]);
        const bool sun_in_fov = !eclipse && sun_img < sun_excl;

        // ---- data
        double img_bits = 0, dl_bits = 0;
        if (img_collect) {
            Vec3 vrel = s.v - cross(Vec3{0, 0, OMEGA_EARTH}, s.r);
            Vec3 up = unit(s.r);
            double v_ground = norm(vrel - dot(vrel, up) * up) * R_EARTH / norm(s.r) * 1000.0;  // m/s
            double gsd = gsd_ref * sub[k].alt / gsd_alt * binning;
            double line_rate = v_ground / gsd;
            double raw_bps = (px / binning) * bands * bits * line_rate;
            img_bits = std::min(raw_bps / cr * dt, ps.cam_cap - ps.cam_bits);
        }
        if (transmitting) dl_bits = std::min(rate * 1e6 * dt, ps.obc_bits);

        // ---- record (state at time t, before integrating over [t, t+dt])
        const double net = p_solar - p_load;
        const double soc_now = bat.soc();
        double unmet = 0;
        Battery bat_next = bat;
        const double curt = bat_next.step(net, dt, unmet);
        if (k % out_every == 0) {
            tb.push(C_t, t); tb.push(C_x, s.r.x); tb.push(C_y, s.r.y); tb.push(C_z, s.r.z);
            tb.push(C_qw, q_att.w); tb.push(C_qx, q_att.x); tb.push(C_qy, q_att.y); tb.push(C_qz, q_att.z);
            tb.push(C_sx, sun_hat[k].x); tb.push(C_sy, sun_hat[k].y); tb.push(C_sz, sun_hat[k].z);
            tb.push(C_gm, gmst_v[k]); tb.push(C_lat, sub[k].lat * RAD); tb.push(C_lon, sub[k].lon * RAD);
            tb.push(C_alt, sub[k].alt);
            tb.push(C_mode, mode); tb.push(C_slew, slewing); tb.push(C_perr, perr * RAD);
            tb.push(C_sf, sunfrac[k]);
            Vec3 h = unit(cross(s.r, s.v));
            tb.push(C_beta, std::asin(std::clamp(dot(sun_hat[k], h), -1.0, 1.0)) * RAD);
            tb.push(C_ps, p_solar); tb.push(C_pl, p_load); tb.push(C_pn, net);
            tb.push(C_pc, curt); tb.push(C_pa, p_alb_raw * mppt_eff); tb.push(C_soc, soc_now); tb.push(C_bwh, bat.energy_wh);
            for (size_t f = 0; f < NF; ++f) { tb.push(C_fp[f], fp[f]); tb.push(C_fi[f], finc[f]); tb.push(C_fl[f], flit[f]); }
            for (int p = 0; p < 4; ++p) tb.push(C_temp[p], panel_th[p].t_k - 273.15);
            tb.push(C_snel, sub_sun_el[k] * RAD);
            tb.push(C_gel, gst[k].el * RAD); tb.push(C_gaz, gst[k].az * RAD); tb.push(C_grg, gst[k].range);
            tb.push(C_rate, transmitting ? rate : 0); tb.push(C_marg, gst[k].el >= gs_mask ? margin : -99);
            tb.push(C_cn0, cn0);
            tb.push(C_store, (ps.cam_bits + ps.obc_bits) / 8e9); tb.push(C_cam, ps.cam_bits / 8e9); tb.push(C_obcs, ps.obc_bits / 8e9);
            tb.push(C_sunimg, sun_img * RAD); tb.push(C_down, down_total / 8e9); tb.push(C_imgd, imaged_total / 8e9);
            tb.push(C_imgr, imager_state); tb.push(C_task, active);
            tb.push(C_asun, angle_between(q_att.rotate(ax_sun), sun_hat[k]) * RAD);
            tb.push(C_aant, ant_ang * RAD);
            tb.push(C_anad, angle_between(q_att.rotate(ax_img), -s.r) * RAD);
            tb.push(C_load_obc, l_obc); tb.push(C_load_eps, l_eps); tb.push(C_load_ttc, l_ttc);
            tb.push(C_load_adcs, l_adcs); tb.push(C_load_img, l_img); tb.push(C_load_htr, l_htr);
            tb.push(C_phase, phase);
            tb.push(C_wx, w_body.x * RAD); tb.push(C_wy, w_body.y * RAD); tb.push(C_wz, w_body.z * RAD);
            tb.push(C_wm, norm(w_body) * RAD);
            tb.push(C_mx, m_cmd.x); tb.push(C_my, m_cmd.y); tb.push(C_mz, m_cmd.z); tb.push(C_mw, p_mtq);
            const double jw = wheel.h_rated / (6000 * TWO_PI / 60);
            tb.push(C_hx, hw.x * 1e3); tb.push(C_hy, hw.y * 1e3); tb.push(C_hz, hw.z * 1e3);
            tb.push(C_rx, hw.x / jw * 60 / TWO_PI); tb.push(C_ry, hw.y / jw * 60 / TWO_PI); tb.push(C_rz, hw.z / jw * 60 / TWO_PI);
            tb.push(C_ww, p_wheels);
            tb.push(C_bm, norm(b_eci[k]) * 1e6);
            auto T3 = ext_torque(q_att, k, b_eci[k], Vec3{});
            tb.push(C_tgg, norm(T3[0]) * 1e6); tb.push(C_taero, norm(T3[1]) * 1e6);
        }

        // ---- events
        std::string state = std::string(slewing ? "SLEW -> " : "") + MODE_NAME[mode];
        if (state != prev_state) {
            events.push_back({t, "MODE", state + (active >= 0 && tasks[active].type == IMAGING
                                                      ? " (" + targets[tasks[active].target].name + ")" : "")});
            prev_state = state;
        }
        if (eclipse != prev_ecl) { events.push_back({t, eclipse ? "ECLIPSE" : "SUN", eclipse ? "eclipse entry" : "eclipse exit"}); prev_ecl = eclipse; }
        bool vis = gst[k].el >= gs_mask;
        if (vis != prev_vis) { events.push_back({t, vis ? "AOS" : "LOS", std::string(vis ? "AOS " : "LOS ") + gs_name}); prev_vis = vis; }
        int ta = img_collect ? active : -1;
        if (ta != prev_task_active) {
            if (ta >= 0) events.push_back({t, "IMG", "imaging start: " + targets[tasks[ta].target].name});
            else if (prev_task_active >= 0) events.push_back({t, "IMG", "imaging stop: " + targets[tasks[prev_task_active].target].name});
            prev_task_active = ta;
        }

        if (k == N) break;

        // ---- integrate [t, t+dt]
        bat = bat_next;
        for (int p = 0; p < 4; ++p) {
            const Plate& pl = geo.plates[6 + p];
            panel_th[p].step(4 * pl.hu * pl.hv, q_sol[p][0], q_ir[p][0], q_sol[p][1], q_ir[p][1], panel_elec[p], dt);
        }
        ps.cam_bits = std::min(ps.cam_bits + img_bits, ps.cam_cap);
        ps.obc_bits = std::max(0.0, ps.obc_bits - dl_bits);
        if (imager_state == 3) { ro_total += ps.readout(dt); t_ro += dt; }
        for (int r = 0; r < NR; ++r) { rail_e[r] += rail_p[r] * dt / 3600.0; rail_pk[r] = std::max(rail_pk[r], rail_p[r]); }
        if (sun_in_fov) {
            t_sun_fov += dt;
            sun_fov_run += dt;
            sun_fov_max = std::max(sun_fov_max, sun_fov_run);
            if (sun_fov_run > sun_fov_max_s && !sun_fov_flag) {
                events.push_back({t, "IMG-SUN", "Sun inside the imager exclusion cone for more than " + std::to_string(int(sun_fov_max_s / 60)) + " min (imager ICD limit)"});
                sun_fov_flag = true;
            }
        } else { sun_fov_run = 0; sun_fov_flag = false; }
        if (img_collect) sun_img_min = std::min(sun_img_min, sun_img);
        imaged_total += img_bits;
        down_total += dl_bits;
        if (active >= 0) {
            tasks[active].bits += tasks[active].type == IMAGING ? img_bits : dl_bits;
            if (img_collect || transmitting) tasks[active].active_s += dt;
        }
        const double hr = dt / 3600.0;
        e_avail += p_solar * hr;
        e_gen += (p_solar - curt) * hr;
        e_load += p_load * hr;
        e_adcs += l_adcs * hr; e_obc += l_obc * hr; e_ttc += l_ttc * hr; e_eps += l_eps * hr;
        e_img += l_img * hr; e_htr += l_htr * hr; mtq_e += p_mtq * hr;
        e_pa += l_pa * hr; e_bcn += l_bcn * hr;
        e_curt += curt * hr;
        e_unmet += unmet * hr;
        e_alb += p_alb_raw * mppt_eff * hr;
        soc_min = std::min(soc_min, soc_now);
        soc_max = std::max(soc_max, soc_now);
        if (eclipse) t_eclipse += dt;
        if (img_collect) t_img += dt;
        if (transmitting) t_dl += dt;
        if (slewing) t_slew += dt;

        // orbit bookkeeping (ascending node)
        cur_orbit.e_avail_wh += p_solar * hr;
        cur_orbit.e_gen_wh += (p_solar - curt) * hr;
        cur_orbit.e_load_wh += p_load * hr;
        cur_orbit.e_curt_wh += curt * hr;
        (eclipse ? cur_orbit.eclipse_s : cur_orbit.sunlit_s) += dt;
        if (!eclipse) cur_orbit.p_sunlit_sum += p_solar * dt;
        cur_orbit.soc_min = std::min(cur_orbit.soc_min, soc_now);
        cur_orbit.soc_max = std::max(cur_orbit.soc_max, soc_now);
        if (img_collect) cur_orbit.imaging_s += dt;
        if (transmitting) cur_orbit.downlink_s += dt;
        if (st[k].r.z < 0 && st[k + 1].r.z >= 0) {
            cur_orbit.t1 = t + dt;
            Vec3 h = unit(cross(s.r, s.v));
            cur_orbit.beta_deg = std::asin(dot(sun_hat[k], h)) * RAD;
            cur_orbit.n = int(orbits.size()) + 1;
            orbits.push_back(cur_orbit);
            cur_orbit = OrbitStat{};
            cur_orbit.t0 = t + dt;
        }

        if (phase == PH_MISSION) {
            // ---- 3-axis wheel control: rate- and acceleration-limited eigenaxis tracking.
            // The attitude profile is kinematic; the wheels absorb whatever the body does,
            // h_w = R^T H_I - I w, while external torques (gravity gradient, aero, residual
            // dipole, magnetorquer dumping) change the total momentum H_I.
            const int nat = std::max(1, int(std::ceil(dt / 0.25)));
            const double h = dt / nat;
            for (int j = 0; j < nat; ++j) {
                Quat qc = Quat::from_rotvec(w_ff * (j * h)) * q_cmd;
                Vec3 ev = (qc * q_act.conj()).to_rotvec();
                double th = norm(ev);
                Vec3 w_des = w_ff;
                if (th > 1e-9) {
                    double wm = std::min({w_max, std::sqrt(2 * a_max * th), th / h});
                    w_des += ev / th * wm;
                }
                Vec3 dw = w_des - w_act;
                double dwn = norm(dw);
                if (dwn > a_max * h) dw = dw * (a_max * h / dwn);
                w_act += dw;
                q_act = (Quat::from_rotvec(w_act * h) * q_act).normalized();
                Vec3 bi = b_eci[k] + (b_eci[k + 1] - b_eci[k]) * ((j + 0.5) / nat);
                auto T3 = ext_torque(q_act, k, bi, m_cmd);
                H_I += q_act.rotate(T3[0] + T3[1] + T3[2]) * h;
            }
            Vec3 wb = q_act.conj().rotate(w_act);
            Vec3 hw_new = q_act.conj().rotate(H_I) - Iner * wb;
            tau_w = (hw_new - hw) / dt;
            hw = hw_new;
            // Cross-product momentum dumping with the magnetorquers: m = k (h x B)/|B|^2
            Vec3 bb = q_act.conj().rotate(b_eci[k + 1]);
            double b2 = dot(bb, bb);
            m_cmd = b2 > 0 ? sat_dipole(cross(hw, bb) * (k_dump / b2)) : Vec3{};
            bool sat = std::fabs(hw.x) > wheel.h_rated || std::fabs(hw.y) > wheel.h_rated || std::fabs(hw.z) > wheel.h_rated;
            if (sat && !wheel_sat_flag) {
                char buf[128];
                std::snprintf(buf, sizeof buf, "wheel momentum above %s rating (%.2f mNms): |h| = %.2f mNms",
                              wheel.model.c_str(), wheel.h_rated * 1e3, norm(hw) * 1e3);
                events.push_back({t, "WHEEL", buf});
            }
            wheel_sat_flag = sat;
            hw_max = std::max({hw_max, std::fabs(hw.x), std::fabs(hw.y), std::fabs(hw.z)});
            rs = RigidState{q_act, wb};
        } else {
            // ---- free / magnetically controlled rigid body (wheels off)
            const int nd = std::max(1, int(std::ceil(dt / dyn_sub)));
            const double hd = dt / nd;
            for (int j = 0; j < nd; ++j) {
                const double tj = t + j * hd;
                Vec3 bi = b_eci[k] + (b_eci[k + 1] - b_eci[k]) * (double(j) / nd);
                if (phase == PH_DETUMBLE) {
                    if (tj - t_bdot >= bdot_T - 1e-9) {  // B-dot: m = -(k/|B|^2) dB/dt (body frame)
                        Vec3 bb = rs.q.conj().rotate(bi);
                        if (t_bdot > -1e8) {
                            Vec3 bdot = (bb - b_prev) / (tj - t_bdot);
                            m_cmd = sat_dipole(bdot * (-k_bdot / dot(bb, bb)));
                        }
                        b_prev = bb;
                        t_bdot = tj;
                    }
                } else {
                    m_cmd = Vec3{};
                }
                auto T3 = ext_torque(rs.q, k, bi, m_cmd);
                rs = rigid_step(rs, Iner, Iinv, T3[0] + T3[1] + T3[2], hd);
            }
            q_act = rs.q;
        }
    }
    // Finish any tasks that ran to the end, and flag windows that produced nothing.
    for (auto& tk : tasks) {
        if (tk.status == ACTIVE) tk.status = DONE;
        if (tk.status == PENDING) { tk.status = SKIPPED; tk.reason = "beyond simulation end"; }
        if (tk.status == DONE && tk.active_s == 0) {
            tk.status = SKIPPED;
            tk.reason = tk.preempted ? "preempted by higher-priority task"
                        : tk.type == IMAGING ? "slew did not finish inside the window" : "no data on board";
        }
    }

    // ============================================================ reports
    Elements elm = state_to_elements(st[0]);
    const double period = TWO_PI * std::sqrt(el0.a * el0.a * el0.a / MU_EARTH);
    const double sim_h = N * dt / 3600.0;

    // Peak sun-pointed power (Sun exactly on the sun axis, 1 AU, reference temperature, no albedo).
    double p_peak = 0;
    {
        for (const auto& cf : geo.faces) {
            double c = dot(cf.n, ax_sun);
            double lit = geo.lit_fraction(cf, ax_sun);
            p_peak += cf.cell_area * cell.eff_ref * cell.derate * cell.degradation * SOLAR_CONST_1AU *
                      std::max(0.0, c) * lit * cell.iam(c);
        }
        p_peak *= mppt_eff;
    }

    const bool write_files = !out_dir.empty();
    if (write_files) fs::create_directories(out_dir);

    // ---- timeline.csv
    if (write_files) {
        std::ofstream f(out_dir + "/timeline.csv");
        for (size_t c = 0; c < tb.cols.size(); ++c) f << (c ? "," : "") << tb.cols[c].name;
        f << "\n";
        size_t rows = tb.cols[0].v.size();
        std::string line;
        for (size_t r = 0; r < rows; ++r) {
            line.clear();
            for (size_t c = 0; c < tb.cols.size(); ++c) {
                if (c) line += ",";
                fmt_num(line, tb.cols[c].v[r], tb.cols[c].decimals);
            }
            f << line << "\n";
        }
    }
    // ---- events.csv, orbits.csv, tasks.csv
    if (write_files) {
        std::ofstream f(out_dir + "/events.csv");
        f << "t_s,utc,kind,detail\n";
        for (auto& e : events) f << e.t << "," << jd_to_iso(jd0 + e.t / 86400) << "," << e.kind << ",\"" << e.detail << "\"\n";
    }
    if (write_files) {
        std::ofstream f(out_dir + "/orbits.csv");
        f << "orbit,start_utc,duration_s,beta_deg,eclipse_s,sunlit_avg_w,orbit_avg_gen_w,orbit_avg_load_w,curtailed_wh,soc_min,soc_max,imaging_s,downlink_s\n";
        for (auto& o : orbits) {
            double dur = o.t1 - o.t0;
            f << o.n << "," << jd_to_iso(jd0 + o.t0 / 86400) << "," << dur << "," << o.beta_deg << "," << o.eclipse_s << ","
              << (o.sunlit_s > 0 ? o.p_sunlit_sum / o.sunlit_s : 0) << "," << o.e_gen_wh * 3600 / dur << ","
              << o.e_load_wh * 3600 / dur << "," << o.e_curt_wh << "," << o.soc_min << "," << o.soc_max << ","
              << o.imaging_s << "," << o.downlink_s << "\n";
        }
    }
    if (write_files) {
        std::ofstream f(out_dir + "/tasks.csv");
        f << "type,label,start_utc,end_utc,window_s,status,reason,active_s,gbytes,max_el_deg\n";
        for (auto& tk : tasks) {
            f << (tk.type == IMAGING ? "imaging" : "downlink") << ","
              << (tk.type == IMAGING ? targets[tk.target].name : gs_name) << "," << jd_to_iso(jd0 + tk.k0 * dt / 86400) << ","
              << jd_to_iso(jd0 + tk.k1 * dt / 86400) << "," << (tk.k1 - tk.k0 + 1) * dt << "," << STATUS_NAME[tk.status] << ","
              << tk.reason << "," << tk.active_s << "," << tk.bits / 8e9 << "," << tk.max_el * RAD << "\n";
        }
    }

    // ---- viewer data (JS so it loads from file:// without a server)
    {
        std::string o;
        o.reserve(8 << 20);
        o += "{\n\"meta\": {";
        o += "\"epoch_utc\":" + json_str(jd_to_iso(jd0)) + ",\"jd0\":";
        fmt_num(o, jd0, 6);
        o += ",\"dt_out\":"; fmt_num(o, out_every * dt, 3);
        o += ",\"duration_s\":"; fmt_num(o, N * dt, 0);
        o += ",\"period_s\":"; fmt_num(o, period, 2);
        o += ",\"altitude_km\":"; fmt_num(o, alt0, 2);
        o += ",\"inclination_deg\":"; fmt_num(o, el0.i * RAD, 4);
        o += ",\"raan_deg\":"; fmt_num(o, el0.raan * RAD, 4);
        o += ",\"ltan_h\":"; fmt_num(o, ltan, 3);
        o += ",\"battery_wh\":"; fmt_num(o, bat.capacity_wh, 2);
        o += ",\"storage_gb\":"; fmt_num(o, storage_bits / 8e9, 1);
        o += ",\"obc_storage_gb\":"; fmt_num(o, ps.obc_cap / 8e9, 1);
        o += ",\"peak_power_w\":"; fmt_num(o, p_peak, 2);
        o += ",\"gs_mask_deg\":"; fmt_num(o, gs_mask * RAD, 1);
        o += ",\"tol_deg\":"; fmt_num(o, tol * RAD, 2);
        o += ",\"swath_km\":"; fmt_num(o, swath_ref, 1);
        o += ",\"launch\":" + std::string(launch ? "true" : "false");
        o += ",\"t_deploy\":"; fmt_num(o, t_deploy, 0);
        o += ",\"t_rf\":"; fmt_num(o, t_rf, 0);
        o += ",\"t_adcs\":"; fmt_num(o, t_adcs, 0);
        o += ",\"det_exit_deg_s\":"; fmt_num(o, det_exit * RAD, 3);
        o += ",\"wheel\":{\"model\":" + json_str(wheel.model) + ",\"h_rated_mnms\":";
        fmt_num(o, wheel.h_rated * 1e3, 3); o += "}";
        o += ",\"inertia\":[";
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) { if (i || j) o += ","; fmt_num(o, mp_dep.inertia.m[i][j], 7); }
        o += "],\"stowed_plates\":[";
        for (size_t p2 = 6; p2 < geo_stow.plates.size(); ++p2) {
            const Plate& pl = geo_stow.plates[p2];
            o += std::string(p2 > 6 ? "," : "") + "{\"c\":" + vec_json(pl.c) + ",\"u\":" + vec_json(pl.u) + ",\"v\":" + vec_json(pl.v) +
                 ",\"n\":" + vec_json(pl.n) + "}";
        }
        o += "]";
        o += ",\"max_roll_deg\":"; fmt_num(o, max_roll * RAD, 1);
        o += ",\"config\":" + json_str(ini.raw);
        o += ",\"wall_s\":"; fmt_num(o, std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count(), 2);
        o += ",\"axes\":{\"sun\":" + vec_json(ax_sun) + ",\"imager\":" + vec_json(ax_img) + ",\"antenna\":" + vec_json(ax_ant) + "}";
        o += ",\"modes\":[";
        for (int m = 0; m < N_MODES; ++m) o += std::string(m ? "," : "") + json_str(MODE_NAME[m]);
        o += "],\"gs\":{\"name\":" + json_str(gs_name) + ",\"lat\":";
        fmt_num(o, gs.lat * RAD, 4); o += ",\"lon\":"; fmt_num(o, gs.lon * RAD, 4); o += "}";
        o += ",\"targets\":[";
        for (size_t t = 0; t < targets.size(); ++t) {
            o += std::string(t ? "," : "") + "{\"name\":" + json_str(targets[t].name) + ",\"lat\":";
            fmt_num(o, targets[t].lat * RAD, 4); o += ",\"lon\":"; fmt_num(o, targets[t].lon * RAD, 4);
            o += ",\"r\":"; fmt_num(o, targets[t].radius_km, 1); o += "}";
        }
        o += "],\"plates\":[";
        for (size_t p = 0; p < geo.plates.size(); ++p) {
            const Plate& pl = geo.plates[p];
            o += std::string(p ? "," : "") + "{\"c\":" + vec_json(pl.c) + ",\"u\":" + vec_json(pl.u) + ",\"v\":" + vec_json(pl.v) +
                 ",\"n\":" + vec_json(pl.n) + ",\"hu\":";
            fmt_num(o, pl.hu, 5); o += ",\"hv\":"; fmt_num(o, pl.hv, 5);
            o += ",\"panel\":" + std::string(p >= 6 ? "true" : "false") + "}";
        }
        o += "],\"faces\":[";
        for (size_t f = 0; f < NF; ++f) {
            o += std::string(f ? "," : "") + "{\"name\":" + json_str(geo.faces[f].name) + ",\"plate\":" +
                 std::to_string(geo.faces[f].plate_id) + ",\"panel\":" + std::to_string(geo.faces[f].panel) +
                 ",\"n\":" + vec_json(geo.faces[f].n) + ",\"cell_area\":";
            fmt_num(o, geo.faces[f].cell_area, 5);
            o += "}";
        }
        o += "],\"panel_angles\":[";
        for (int p = 0; p < 4; ++p) { if (p) o += ","; fmt_num(o, gc.angles_deg[p], 2); }
        o += "],\"tasks\":[";
        for (size_t j = 0; j < tasks.size(); ++j) {
            const Task& tk = tasks[j];
            o += std::string(j ? "," : "") + "{\"type\":" + json_str(tk.type == IMAGING ? "imaging" : "downlink") +
                 ",\"label\":" + json_str(tk.type == IMAGING ? targets[tk.target].name : gs_name) + ",\"t0\":";
            fmt_num(o, tk.k0 * dt, 0); o += ",\"t1\":"; fmt_num(o, tk.k1 * dt, 0);
            o += ",\"status\":" + json_str(STATUS_NAME[tk.status]) + ",\"reason\":" + json_str(tk.reason) + ",\"gb\":";
            fmt_num(o, tk.bits / 8e9, 4); o += ",\"max_el\":"; fmt_num(o, tk.max_el * RAD, 1);
            o += ",\"roll\":"; fmt_num(o, tk.roll * RAD, 2); o += "}";
        }
        o += "],\"events\":[";
        for (size_t j = 0; j < events.size(); ++j) {
            o += std::string(j ? "," : "") + "[";
            fmt_num(o, events[j].t, 0);
            o += "," + json_str(events[j].kind) + "," + json_str(events[j].detail) + "]";
        }
        o += "],\"summary\":{";
        auto kvn = [&](const char* k, double v, int d, bool first = false) {
            o += std::string(first ? "" : ",") + "\"" + k + "\":"; fmt_num(o, v, d);
        };
        kvn("orbits", orbits.size(), 0, true);
        kvn("eclipse_frac", t_eclipse / (N * dt), 4);
        kvn("avg_avail_w", e_avail / sim_h, 3);
        kvn("avg_gen_w", e_gen / sim_h, 3);
        kvn("avg_load_w", e_load / sim_h, 3);
        kvn("curtailed_wh", e_curt, 2);
        kvn("unmet_wh", e_unmet, 3);
        kvn("albedo_wh", e_alb, 2);
        kvn("soc_min", soc_min, 4);
        kvn("imaging_s", t_img, 0);
        kvn("downlink_s", t_dl, 0);
        kvn("imaged_gb", imaged_total / 8e9, 3);
        kvn("downlinked_gb", down_total / 8e9, 3);
        kvn("slew_s", t_slew, 0);
        kvn("e_obc_w", e_obc / sim_h, 3);
        kvn("e_adcs_w", e_adcs / sim_h, 3);
        kvn("e_mtq_w", mtq_e / sim_h, 3);
        kvn("e_ttc_w", e_ttc / sim_h, 3);
        kvn("e_pa_w", e_pa / sim_h, 3);
        kvn("e_bcn_w", e_bcn / sim_h, 3);
        kvn("mass_kg", mp_dep.mass, 3);
        kvn("e_img_w", e_img / sim_h, 3);
        kvn("e_eps_w", e_eps / sim_h, 3);
        kvn("e_htr_w", e_htr / sim_h, 3);
        kvn("t_det_start", t_det_start, 0);
        kvn("t_det_end", t_det_end, 0);
        kvn("hw_max_mnms", hw_max * 1e3, 3);
        kvn("readout_s", t_ro, 0);
        kvn("readout_gb", ro_total / 8e9, 3);
        kvn("discarded_gb", ps.discarded / 8e9, 3);
        kvn("sun_fov_s", t_sun_fov, 0);
        kvn("sun_fov_max_s", sun_fov_max, 0);
        o += "}},\n\"cols\": {\n";
        for (size_t c = 0; c < tb.cols.size(); ++c) {
            o += json_str(tb.cols[c].name) + ":[";
            for (size_t r = 0; r < tb.cols[c].v.size(); ++r) {
                if (r) o += ",";
                fmt_num(o, tb.cols[c].v[r], tb.cols[c].decimals);
            }
            o += c + 1 < tb.cols.size() ? "],\n" : "]\n";
        }
        o += "}}\n";
        res.json = std::move(o);
    }

    auto wall1 = std::chrono::steady_clock::now();
    double wall = std::chrono::duration<double>(wall1 - wall0).count();

    // ---- console summary
    {
        appendf(res.console, "\n=== 3U CubeSat mission simulation =============================================\n");
        appendf(res.console, "Epoch            %s UTC   span %.1f h   step %.1f s   (%d steps, %.2f s wall)\n",
                    jd_to_iso(jd0).c_str(), sim_h, dt, N, wall);
        appendf(res.console, "Orbit            h=%.1f km  a=%.3f km  e=%.5f  i=%.4f deg%s  RAAN=%.3f deg  LTAN=%s\n",
                    alt0, elm.a, elm.e, el0.i * RAD, sso ? " (SSO)" : "", el0.raan * RAD, hms(ltan * 3600).c_str());
        appendf(res.console, "                 period %.2f min   forces J2..J%d%s   drag area %.4f m^2\n", period / 60, fm.jmax,
                    fm.drag ? " + drag" : "", fm.area_m2);
        appendf(res.console, "Array            panels at %.0f/%.0f/%.0f/%.0f deg, hinge %s   %zu cell faces   cells %.0f front + %.0f back per panel\n",
                    gc.angles_deg[0], gc.angles_deg[1], gc.angles_deg[2], gc.angles_deg[3], gc.hinge_end > 0 ? "+Z" : "-Z", NF,
                    gc.cells_front, gc.cells_back);
        if (n_failed) appendf(res.console, "                 DEPLOYMENT-FAILURE CASE: %d panel(s) stowed (geometry.failed_panels)\n", n_failed);
        appendf(res.console, "                 peak sun-pointed power (1 AU, %.0f C, no albedo): %.2f W\n", cell.t_ref_c, p_peak);
        appendf(res.console, "\nFace geometry when sun-pointed (sun on %s):\n", ini.str("attitude.sun_axis", "+Z").c_str());
        appendf(res.console, "  %-18s %10s %10s %10s\n", "face", "incidence", "lit", "cell m^2");
        for (const auto& cf : geo.faces) {
            double c = dot(cf.n, ax_sun);
            appendf(res.console, "  %-18s %9.1f° %9.0f%% %10.4f\n", cf.name.c_str(), std::acos(std::clamp(c, -1.0, 1.0)) * RAD,
                        geo.lit_fraction(cf, ax_sun) * 100, cf.cell_area);
        }

        appendf(res.console, "\nPower and energy (whole run)\n");
        appendf(res.console, "  eclipse fraction      %6.1f %%\n", 100 * t_eclipse / (N * dt));
        appendf(res.console, "  mean array available  %6.2f W   (of which Earth albedo %.2f W)\n", e_avail / sim_h, e_alb / sim_h);
        appendf(res.console, "  mean array used       %6.2f W   (rest curtailed: battery full)\n", e_gen / sim_h);
        appendf(res.console, "  mean load             %6.2f W\n", e_load / sim_h);
        appendf(res.console, "  curtailed (bat full)  %6.1f Wh\n", e_curt);
        appendf(res.console, "  unmet load            %6.3f Wh%s\n", e_unmet, e_unmet > 1e-3 ? "   <-- POWER-NEGATIVE" : "");
        appendf(res.console, "  battery SoC           min %.1f %%   max %.1f %%   end %.1f %%\n", soc_min * 100, soc_max * 100, bat.soc() * 100);

        appendf(res.console, "\nLoad by subsystem (whole-run average)\n");
        auto sub = [&](const char* n, double e) {
            appendf(res.console, "  %-34s %6.3f W  %5.1f %%\n", n, e / sim_h, e_load > 0 ? 100 * e / e_load : 0);
        };
        sub("OBC+TTC SOM (ADRV9364-Z7020)", e_obc);
        sub("ADCS (CW0017, CubeSense, MTQ)", e_adcs);
        sub("  of which magnetorquers", mtq_e);
        sub("S-band PA (passes)", e_pa);
        sub("UHF beacon (RFM96W)", e_bcn);
        sub("Payload (hyperspectral imager)", e_img);
        sub("EPS + housekeeping", e_eps);
        sub("Heater", e_htr);

        appendf(res.console, "\nRails (TPP v03)   average / peak power, peak current at rail voltage\n");
        for (int r = 0; r < NR; ++r)
            appendf(res.console, "  %-38s %6.3f W  %6.2f W  %5.2f A @ %.1f V\n", RAIL_NAME[r], rail_e[r] / sim_h, rail_pk[r],
                    rail_pk[r] / RAIL_V[r], RAIL_V[r]);
        const auto& I0 = mp_dep.inertia.m;
        appendf(res.console, "\nADCS\n");
        appendf(res.console, "  mass %.3f kg   CoM (%.1f, %.1f, %.1f) mm from geometric centre\n", mp_dep.mass, mp_dep.com.x * 1e3,
                mp_dep.com.y * 1e3, mp_dep.com.z * 1e3);
        appendf(res.console, "  slew limits: %.2f deg/s, %.3f deg/s^2 (%s)\n", w_max * RAD, a_max * RAD, rate_auto ? "auto from wheel capacity" : "config");
        appendf(res.console, "  inertia (deployed) kg m^2   Ixx %.5f  Iyy %.5f  Izz %.5f  Ixy %.2e  Ixz %.2e  Iyz %.2e\n",
                I0[0][0], I0[1][1], I0[2][2], I0[0][1], I0[0][2], I0[1][2]);
        appendf(res.console, "  inertia (stowed)   kg m^2   Ixx %.5f  Iyy %.5f  Izz %.5f\n", mp_stow.inertia.m[0][0],
                mp_stow.inertia.m[1][1], mp_stow.inertia.m[2][2]);
        appendf(res.console, "  wheels %s x3 (%.2f mNms rated)   torquers %s/%s/%s   B-dot gain %.3e N m s\n", wheel.model.c_str(),
                wheel.h_rated * 1e3, tq[0].model.c_str(), tq[1].model.c_str(), tq[2].model.c_str(), k_bdot);
        if (launch) {
            if (t_det_end > 0)
                appendf(res.console, "  detumble: start T+%.1f min, complete T+%.1f min  (%.1f min, from |w0| = %.2f deg/s)\n",
                        t_det_start / 60, t_det_end / 60, (t_det_end - t_det_start) / 60, norm(w0) * RAD);
            else
                appendf(res.console, "  detumble: NOT COMPLETE within the run\n");
        }
        {
            const double rpm = hw_max / (wheel.h_rated / (6000 * TWO_PI / 60)) * 60 / TWO_PI;
            appendf(res.console, "  peak wheel momentum (mission) %.3f mNms = %.0f %% of rated (6000 rpm), %.0f rpm = %.0f %% of %.0f rpm max\n",
                    hw_max * 1e3, 100 * hw_max / wheel.h_rated, rpm, 100 * rpm / wheel.max_rpm, wheel.max_rpm);
        }

        appendf(res.console, "\nPer orbit (ascending node to ascending node)\n");
        appendf(res.console, "  %3s %-20s %6s %7s %9s %9s %8s %8s %7s %7s %6s %6s\n", "#", "start UTC", "beta", "eclipse", "sunlit W",
                    "OAP avail", "OAP used", "OAP load", "SoCmin", "SoCmax", "img s", "dl s");
        for (auto& o : orbits) {
            double dur = o.t1 - o.t0;
            appendf(res.console, "  %3d %-20s %5.1f° %6.1fm %9.2f %9.2f %8.2f %8.2f %6.1f%% %6.1f%% %6.0f %6.0f\n", o.n,
                        jd_to_iso(jd0 + o.t0 / 86400).c_str(), o.beta_deg, o.eclipse_s / 60,
                        o.sunlit_s > 0 ? o.p_sunlit_sum / o.sunlit_s : 0, o.e_avail_wh * 3600 / dur, o.e_gen_wh * 3600 / dur, o.e_load_wh * 3600 / dur,
                        o.soc_min * 100, o.soc_max * 100, o.imaging_s, o.downlink_s);
        }

        appendf(res.console, "\nGround-station passes (%s, mask %.0f deg)   coding: rate %.3f, Eb/N0 req %.1f dB, max info rate %.2f Mbps @ %.1f Msps, PA %.2f W RF / %.2f W DC\n",
                gs_name.c_str(), gs_mask * RAD, link.code_rate, link.req_ebn0_db, link.max_info_rate() / 1e6, link.max_symbol_rate / 1e6,
                link.tx_power_w, link.tx_power_w / pa_eff);
        appendf(res.console, "  %-20s %8s %7s %-8s %9s %s\n", "AOS UTC", "window", "max el", "status", "downlink", "note");
        int npass = 0;
        for (auto& tk : tasks) {
            if (tk.type != DOWNLINK) continue;
            ++npass;
            appendf(res.console, "  %-20s %7.0fs %6.1f° %-8s %7.3f GB %s\n", jd_to_iso(jd0 + tk.k0 * dt / 86400).c_str(),
                        (tk.k1 - tk.k0 + 1) * dt, tk.max_el * RAD, STATUS_NAME[tk.status], tk.bits / 8e9, tk.reason.c_str());
        }
        if (!npass) appendf(res.console, "  (none)\n");

        appendf(res.console, "\nImaging windows\n");
        appendf(res.console, "  %-20s %-20s %7s %-8s %9s %s\n", "start UTC", "target", "window", "status", "data", "note");
        int nimg = 0;
        for (auto& tk : tasks) {
            if (tk.type != IMAGING) continue;
            ++nimg;
            appendf(res.console, "  %-20s %-20s %6.0fs %-8s %7.3f GB %s\n", jd_to_iso(jd0 + tk.k0 * dt / 86400).c_str(),
                        targets[tk.target].name.c_str(), (tk.k1 - tk.k0 + 1) * dt, STATUS_NAME[tk.status], tk.bits / 8e9,
                        tk.reason.c_str());
        }
        if (!nimg) appendf(res.console, "  (none)\n");

        appendf(res.console, "\nData\n");
        appendf(res.console, "  imaged       %8.3f GB in %.0f s\n", imaged_total / 8e9, t_img);
        appendf(res.console, "  downlinked   %8.3f GB in %.0f s\n", down_total / 8e9, t_dl);
        appendf(res.console, "  read out     %8.3f GB to OBC in %.0f s at %.0f Mbit/s (keeping %.0f %% of what is scanned; %.3f GB dropped)\n",
                ro_total / 8e9, t_ro, ps.readout_bps / 1e6, ps.select * 100, ps.discarded / 8e9);
        appendf(res.console, "  camera flash %8.3f GB of %.0f GB at end\n", ps.cam_bits / 8e9, ps.cam_cap / 8e9);
        appendf(res.console, "  OBC storage  %8.3f GB of %.0f GB at end\n", ps.obc_bits / 8e9, ps.obc_cap / 8e9);
        appendf(res.console, "\nImager Sun exclusion (ICD: > %.0f deg off axis, <= %.0f min direct Sun)\n", sun_excl * RAD, sun_fov_max_s / 60);
        appendf(res.console, "  Sun inside the cone  %.0f s total, longest %.0f s%s\n", t_sun_fov, sun_fov_max,
                sun_fov_max > sun_fov_max_s ? "   <-- EXCEEDS LIMIT" : "");
        if (t_img > 0) appendf(res.console, "  closest Sun approach while imaging %.1f deg\n", sun_img_min * RAD);
    }
    return res;
}

// ================================================================ web server
// Minimal single-threaded HTTP/1.1 server bound to 127.0.0.1. Serves the
// viewer's static files and runs the simulator on POST /api/run (body: INI).
static std::string read_file(const std::string& path, bool& ok) {
    std::ifstream f(path, std::ios::binary);
    ok = bool(f);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static void send_all(int fd, const std::string& data) {
    size_t off = 0;
    while (off < data.size()) {
        ssize_t n = ::send(fd, data.data() + off, data.size() - off, MSG_NOSIGNAL);
        if (n <= 0) return;
        off += size_t(n);
    }
}

static void respond(int fd, int code, const std::string& type, const std::string& body) {
    const char* msg = code == 200 ? "OK" : code == 404 ? "Not Found" : code == 400 ? "Bad Request" : "Error";
    std::string h = "HTTP/1.1 " + std::to_string(code) + " " + msg + "\r\nContent-Type: " + type +
                    "\r\nContent-Length: " + std::to_string(body.size()) +
                    "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";
    send_all(fd, h + body);
}

static std::string mime(const std::string& p) {
    auto ends = [&](const char* e) { return p.size() >= std::strlen(e) && p.compare(p.size() - std::strlen(e), std::string::npos, e) == 0; };
    if (ends(".html")) return "text/html; charset=utf-8";
    if (ends(".js")) return "text/javascript; charset=utf-8";
    if (ends(".css")) return "text/css";
    if (ends(".json")) return "application/json";
    if (ends(".png")) return "image/png";
    if (ends(".jpg") || ends(".jpeg")) return "image/jpeg";
    if (ends(".svg")) return "image/svg+xml";
    return "text/plain; charset=utf-8";
}

static int serve(int port, const std::string& cfg_path, const std::string& viewer_dir, const std::string& out_dir) {
    int srv = ::socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    ::setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(uint16_t(port));
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(srv, (sockaddr*)&addr, sizeof addr) < 0 || ::listen(srv, 16) < 0) {
        std::perror("bind/listen");
        return 1;
    }
    std::printf("CubeSat simulator UI:  http://127.0.0.1:%d/   (Ctrl+C to stop)\n", port);
    std::fflush(stdout);
    for (;;) {
        int fd = ::accept(srv, nullptr, nullptr);
        if (fd < 0) continue;
        std::string req;
        char buf[65536];
        size_t hdr_end = std::string::npos, need = 0;
        while (true) {
            ssize_t n = ::recv(fd, buf, sizeof buf, 0);
            if (n <= 0) break;
            req.append(buf, size_t(n));
            if (hdr_end == std::string::npos && (hdr_end = req.find("\r\n\r\n")) != std::string::npos) {
                size_t cl = req.find("Content-Length:");
                if (cl == std::string::npos) cl = req.find("content-length:");
                need = hdr_end + 4 + (cl != std::string::npos && cl < hdr_end ? std::stoul(req.substr(cl + 15)) : 0);
            }
            if (hdr_end != std::string::npos && req.size() >= need) break;
            if (req.size() > (4u << 20)) break;
        }
        std::istringstream rl(req);
        std::string method, path;
        rl >> method >> path;
        path = path.substr(0, path.find('?'));
        std::string body = hdr_end == std::string::npos ? "" : req.substr(hdr_end + 4);

        if (method == "GET" && path == "/api/config") {
            bool ok;
            std::string txt = read_file(cfg_path, ok);
            respond(fd, ok ? 200 : 404, "text/plain; charset=utf-8", txt);
        } else if (method == "POST" && path == "/api/run") {
            try {
                Ini ini;
                ini.parse(body, "request");
                RunResult r = run_simulation(ini, out_dir);
                std::printf("run: %zu KB result\n", r.json.size() / 1024);
                std::fflush(stdout);
                respond(fd, 200, "application/json", "{\"ok\":true,\"console\":" + json_str(r.console) + ",\"sim\":" + r.json + "}");
            } catch (std::exception& e) {
                respond(fd, 200, "application/json", std::string("{\"ok\":false,\"error\":") + json_str(e.what()) + "}");
            }
        } else if (method == "GET") {
            if (path == "/") path = "/index.html";
            bool safe = path.find("..") == std::string::npos;
            for (char c : path) safe = safe && (std::isalnum((unsigned char)c) || std::strchr("/._-", c));
            bool ok = false;
            std::string data = safe ? read_file(viewer_dir + path, ok) : "";
            if (ok) respond(fd, 200, mime(path), data);
            else respond(fd, 404, "text/plain", "not found");
        } else {
            respond(fd, 400, "text/plain", "bad request");
        }
        ::close(fd);
    }
}

// ======================================================================= main
int main(int argc, char** argv) {
    std::string cfg_path = "config/sim.cfg", out_dir = "output", viewer_dir = "viewer";
    bool quiet = false;
    int port = 0;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--out" && i + 1 < argc) out_dir = argv[++i];
        else if (a == "--viewer-dir" && i + 1 < argc) viewer_dir = argv[++i];
        else if (a == "--quiet") quiet = true;
        else if (a == "--serve") port = (i + 1 < argc && std::isdigit((unsigned char)argv[i + 1][0])) ? std::atoi(argv[++i]) : 8080;
        else if (a == "-h" || a == "--help") {
            std::puts("usage: cubesat_sim [config.cfg] [--out DIR] [--viewer-dir DIR] [--quiet] [--serve [PORT]]");
            return 0;
        } else cfg_path = a;
    }
    try {
        Ini ini;
        if (!ini.load(cfg_path)) {
            std::fprintf(stderr, "cannot open config '%s' (run from the cubesat_sim directory)\n", cfg_path.c_str());
            return 1;
        }
        RunResult r = run_simulation(ini, out_dir);
        fs::create_directories(viewer_dir);
        std::ofstream(viewer_dir + "/sim_data.js") << "// Generated by cubesat_sim from " << cfg_path << "\nwindow.SIM = " << r.json << ";\nwindow.SIM.meta.console = " << json_str(r.console) << ";\n";
        if (!quiet) {
            std::fputs(r.console.c_str(), stdout);
            std::printf("\nOutputs: %s/{timeline,events,orbits,tasks}.csv   viewer: %s/index.html\n\n", out_dir.c_str(), viewer_dir.c_str());
        }
    } catch (std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    if (port) return serve(port, cfg_path, viewer_dir, out_dir);
    return 0;
}
