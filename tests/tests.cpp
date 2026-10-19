// Physics validation against published reference values and analytic results.
#include <cstdio>
#include <string>

#include "adcs.hpp"
#include "astro.hpp"
#include "igrf.hpp"
#include "earth_radiation.hpp"
#include "geometry.hpp"
#include "models.hpp"
#include "orbit.hpp"

using namespace cs;

static int fails = 0, passes = 0;
static void check(bool ok, const std::string& name, double got, double want, double tol) {
    std::printf("  [%s] %-58s got %.9g  want %.9g  (tol %.2g)\n", ok ? "PASS" : "FAIL", name.c_str(), got, want, tol);
    ok ? ++passes : ++fails;
}
static void near(const std::string& name, double got, double want, double tol) {
    check(std::fabs(got - want) <= tol, name, got, want, tol);
}

int main() {
    std::puts("Time and ephemeris");
    // Vallado Example 3-5: 1992-08-20 12:14:00 UT1 -> GMST = 152.578787810 deg
    near("GMST, Vallado Ex 3-5 (deg)", gmst(julian_date(1992, 8, 20, 12, 14, 0)) * RAD, 152.578787810, 1e-6);
    // Vallado Example 5-1: 2006-04-02 00:00 UT1 -> r_sun = (0.9771945, 0.1924424, 0.0834308) AU
    Vec3 rs = sun_position_eci(julian_date(2006, 4, 2, 0, 0, 0)) / AU_KM;
    near("Sun x, Vallado Ex 5-1 (AU)", rs.x, 0.9771945, 2e-6);
    near("Sun y, Vallado Ex 5-1 (AU)", rs.y, 0.1924424, 2e-6);
    near("Sun z, Vallado Ex 5-1 (AU)", rs.z, 0.0834308, 2e-6);
    near("JD of J2000 epoch", julian_date(2000, 1, 1, 12, 0, 0), 2451545.0, 1e-9);

    std::puts("Geodesy");
    Geodetic g{28.3639 * DEG, 75.587 * DEG, 0.3};
    Geodetic g2 = ecef_to_geodetic(geodetic_to_ecef(g));
    near("geodetic round trip lat (deg)", g2.lat * RAD, 28.3639, 1e-9);
    near("geodetic round trip alt (km)", g2.alt, 0.3, 1e-8);
    near("pole ECEF z = b (km)", geodetic_to_ecef({90 * DEG, 0, 0}).z, R_EARTH * (1 - WGS84_F), 1e-6);

    std::puts("Orbit mechanics");
    Elements el{R_EARTH + 500, 0.001, 97.4 * DEG, 30 * DEG, 40 * DEG, 50 * DEG};
    Elements e2 = state_to_elements(elements_to_state(el));
    near("elements round trip a (km)", e2.a, el.a, 1e-6);
    near("elements round trip i (deg)", e2.i * RAD, 97.4, 1e-9);
    near("elements round trip RAAN (deg)", e2.raan * RAD, 30, 1e-9);
    near("elements round trip nu (deg)", e2.nu * RAD, 50, 1e-6);
    near("SSO inclination at 500 km (deg)", sso_inclination(R_EARTH + 500, 0) * RAD, 97.40, 0.02);

    {   // Two-body energy conservation, 1 day, RK4 1 s
        ForceModel f; f.jmax = 1; f.drag = false;
        State s = elements_to_state(el);
        auto energy = [](const State& x) { return dot(x.v, x.v) / 2 - MU_EARTH / norm(x.r); };
        double e0 = energy(s);
        for (int i = 0; i < 86400; ++i) s = rk4_step(s, 1.0, f);
        near("two-body relative energy drift, 1 day", std::fabs((energy(s) - e0) / e0), 0, 1e-10);
    }
    {   // J2 nodal regression of an SSO orbit must be ~0.9856 deg/day (Sun's mean motion).
        ForceModel f; f.jmax = 2; f.drag = false;
        double a = R_EARTH + 500;
        Elements s0{a, 0, sso_inclination(a, 0), 0, 0, 0};
        State s = elements_to_state(s0);
        double T = TWO_PI * std::sqrt(a * a * a / MU_EARTH);
        int n_orb = 45;  // ~3 days, compare at the same orbital phase
        int steps = int(std::round(n_orb * T));
        double h = n_orb * T / steps;
        for (int i = 0; i < steps; ++i) s = rk4_step(s, h, f);
        double days = n_orb * T / 86400;
        double rate = wrap_pi(state_to_elements(s).raan) * RAD / days;
        near("J2 nodal rate of SSO orbit (deg/day)", rate, 360 / 365.2422, 0.01);
    }
    {   // J2 radial acceleration at the equator: -mu/r^2 (1 + 1.5 J2 (R/r)^2)
        Vec3 r{R_EARTH + 500, 0, 0};
        double rn = norm(r);
        double want = -MU_EARTH / (rn * rn) * (1 + 1.5 * J_ZONAL[2] * std::pow(R_EARTH / rn, 2));
        near("J2 equatorial acceleration (km/s^2)", gravity_accel(r, 2).x, want, 1e-15);
    }
    near("density at 500 km (kg/m^3)", atmos_density(500), 6.967e-13, 1e-18);

    std::puts("Eclipse");
    Vec3 sun{AU_KM, 0, 0};
    near("full sun on day side", sunlight_fraction({R_EARTH + 500, 0, 0}, sun), 1, 0);
    near("umbra behind Earth", sunlight_fraction({-(R_EARTH + 500), 0, 0}, sun), 0, 0);
    {
        // Sweep across the shadow boundary: fraction must be monotonic and pass through 0.5.
        double r = R_EARTH + 500, prev = 0, cross_at = 0;
        bool mono = true;
        for (double z = 6200; z <= 6500; z += 0.5) {
            double x = -std::sqrt(r * r - z * z);
            double f = sunlight_fraction({x, 0, z}, sun);
            if (f + 1e-12 < prev) mono = false;
            if (prev < 0.5 && f >= 0.5) cross_at = z;
            prev = f;
        }
        check(mono, "penumbra monotonic across terminator", mono, 1, 0);
        near("50% light at shadow edge ~ R_earth (km)", cross_at, R_EARTH, 5);
    }

    std::puts("Earth radiation (numerical cap integral vs analytic view factor)");
    {
        EarthRadiation er;
        Vec3 r{R_EARTH + 500, 0, 0};
        std::vector<Vec3> n = {{-1, 0, 0}, {1, 0, 0}, {0, 1, 0}};
        std::vector<double> a, ir;
        er.irradiance(r, {1, 0, 0}, 1361, n, a, ir);
        double sin2 = std::pow(R_EARTH / norm(r), 2);
        near("IR on nadir plate = OLR sin^2(rho) (W/m^2)", ir[0], er.olr * sin2, er.olr * sin2 * 0.003);
        near("IR on zenith plate (W/m^2)", ir[1], 0, 1e-9);
        check(ir[2] > 0 && ir[2] < ir[0] * 0.5, "IR on horizontal plate between 0 and half nadir", ir[2], ir[0] * 0.3, 0);
        check(a[0] < er.albedo * 1361 * sin2 && a[0] > 0.8 * er.albedo * 1361 * sin2,
              "albedo on nadir plate, Sun at zenith, < a S sin^2", a[0], er.albedo * 1361 * sin2, 0);
    }

    std::puts("Spacecraft geometry (135 deg panels)");
    {
        GeometryConfig gc;
        Geometry geo(gc);
        const CellFace* xf = nullptr; const CellFace* mxf = nullptr; const CellFace* xb = nullptr;
        for (auto& f : geo.faces) {
            if (f.name == "+X panel front") xf = &f;
            if (f.name == "-X panel front") mxf = &f;
            if (f.name == "+X panel back") xb = &f;
        }
        near("front normal angle from +Z (deg)", angle_between(xf->n, {0, 0, 1}) * RAD, 45, 1e-9);
        near("+X front lit fraction, Sun on +Z", geo.lit_fraction(*xf, {0, 0, 1}), 1, 0);
        near("+X back lit fraction, Sun on +Z", geo.lit_fraction(*xb, {0, 0, 1}), 0, 0);
        check(geo.lit_fraction(*mxf, {1, 0, 0}) < 0.1, "-X front shadowed by +X panel, Sun on +X",
              geo.lit_fraction(*mxf, {1, 0, 0}), 0, 0.1);
        near("+X back lit fraction, Sun on +X", geo.lit_fraction(*xb, {1, 0, 0}), 1, 0);
        GeometryConfig g90; for (double& a : g90.angles_deg) a = 90;
        Geometry geo90(g90);
        near("90 deg front normal = +Z", dot(geo90.faces[0].n, {0, 0, 1}), 1, 1e-12);
    }

    std::puts("Attitude and link");
    {
        Quat q = align_constrain({0, 0, 1}, unit(Vec3{1, 2, 3}), {1, 0, 0}, {0, 0, 1});
        near("align-constrain primary error (rad)", angle_between(q.rotate({0, 0, 1}), unit(Vec3{1, 2, 3})), 0, 1e-12);
        Quat a = Quat::from_axis_angle({0, 1, 0}, 0.3);
        near("quat angle", quat_angle(Quat{}, a), 0.3, 1e-12);
        LinkModel lk;
        double fspl = 20 * std::log10(4 * PI * 1e6 * 2.245e9 / C_LIGHT);
        near("FSPL at 1000 km, 2.245 GHz (dB)", fspl, 159.47, 0.01);
        near("C/N0 at boresight, 1000 km (dBHz)", lk.cn0(1000, 0),
             10 * std::log10(2.0) + 6.5 - 2 - fspl - 2 + 10 + 228.6, 1e-9);
    }
    {
        Battery b; b.capacity_wh = 10; b.energy_wh = 5;
        double unmet;
        b.step(-9.5, 3600, unmet);  // 9.5 W for 1 h at 95% -> 10 Wh needed, only 5
        near("battery unmet load when empty (W)", unmet, 9.5 - 5 * 0.95, 1e-9);
    }

    std::puts("Geomagnetic field (IGRF-14, 2025.0)");
    {
        Igrf f(2025.0);
        // Scan the surface for the field extremes.
        double fmin = 1e9, fmax = 0, latmin = 0, lonmin = 0;
        for (double la = -89; la <= 89; la += 1)
            for (double lo = -180; lo < 180; lo += 1) {
                Vec3 r = geodetic_to_ecef({la * DEG, lo * DEG, 0});
                double b = norm(f.field_ecef(r)) * 1e9;
                if (b < fmin) { fmin = b; latmin = la; lonmin = lo; }
                fmax = std::max(fmax, b);
            }
        check(fmin > 21000 && fmin < 23500, "South Atlantic Anomaly minimum |B| ~22 uT (nT)", fmin, 22200, 1300);
        check(latmin > -35 && latmin < -15 && lonmin > -75 && lonmin < -35, "SAA minimum located over S. America / S. Atlantic",
              latmin, -27, 10);
        check(fmax > 64000 && fmax < 68000, "maximum |B| ~66 uT near south magnetic pole (nT)", fmax, 66000, 2000);
        // Pure dipole term at the geographic north pole: radial field = 2 g10.
        Igrf d(2025.0, 1);
        Vec3 bp = d.field_ecef({0, 0, Igrf::A_REF});
        near("dipole Bz at north pole = 2 g10 (nT)", bp.z * 1e9, 2 * -29350.0, 1e-6);
        Vec3 pil = f.field_ecef(geodetic_to_ecef({28.36 * DEG, 75.59 * DEG, 0}));
        check(norm(pil) * 1e9 > 44000 && norm(pil) * 1e9 < 50000, "|B| at Pilani 44-50 uT (nT)", norm(pil) * 1e9, 47500, 3000);
        double b500 = norm(f.field_ecef(geodetic_to_ecef({0, 0, 500}))) * 1e9;
        check(b500 > 20000 && b500 < 35000, "|B| at 500 km over equator 20-35 uT (nT)", b500, 27000, 8000);
    }

    std::puts("Attitude dynamics and mass model");
    {
        GeometryConfig gc;
        Geometry geo(gc);
        MassProps mp = mass_properties(geo, 4.0, 0.12, {0, 0, 0});
        near("total mass (kg)", mp.mass, 4.0, 1e-12);
        check(mp.inertia.m[2][2] < mp.inertia.m[0][0], "Izz < Ixx for 3U with 135 deg panels", mp.inertia.m[2][2], mp.inertia.m[0][0], 0);
        near("inertia symmetric Ixy = Iyx", mp.inertia.m[0][1], mp.inertia.m[1][0], 1e-15);
        Mat3 I = mp.inertia, Ii = inverse(I);
        Mat3 id = I * Ii;
        near("I * I^-1 = identity", id.m[0][0] + id.m[1][1] + id.m[2][2], 3, 1e-12);
        RigidState s{Quat{}, Vec3{2 * DEG, -2 * DEG, 2 * DEG}};
        auto H = [&](const RigidState& x) { return x.q.rotate(I * x.w); };
        auto T = [&](const RigidState& x) { return 0.5 * dot(x.w, I * x.w); };
        Vec3 h0 = H(s);
        double t0 = T(s);
        for (int k = 0; k < 36000; ++k) s = rigid_step(s, I, Ii, {}, 0.1);
        near("torque-free: inertial angular momentum conserved (rel)", norm(H(s) - h0) / norm(h0), 0, 1e-9);
        near("torque-free: rotational energy conserved (rel)", std::fabs(T(s) - t0) / t0, 0, 1e-9);
        WheelSpec w = wheel_spec("CW0057");
        near("CW0057 rated momentum (Nms)", w.h_rated, 5.7e-3, 0);
        near("CR0002 power at full dipole = V^2/R (W)", 25.0 / torquer_spec("CR0002").ohm, 0.490, 0.001);
    }

    std::puts("Payload");
    near("roll_to_swath: target inside nadir swath needs no roll", roll_to_swath(0.05, 0.12), 0, 0);
    near("roll_to_swath: target 0.2 rad off, half-FOV 0.12 -> 0.08", roll_to_swath(-0.2, 0.12), -0.08, 1e-12);
    {   // readout at 100 Mbit/s keeping 25 %: 10 s moves 1 Gbit into OBC and scans 4 Gbit of camera data
        PayloadStore ps; ps.cam_bits = 10e9; ps.select = 0.25;
        double kept = ps.readout(10);
        near("readout kept bits (100 Mbit/s x 10 s)", kept, 1e9, 1);
        near("readout camera bits scanned", 10e9 - ps.cam_bits, 4e9, 1);
        near("readout discarded bits", ps.discarded, 3e9, 1);
    }

    std::printf("\n%d passed, %d failed\n", passes, fails);
    return fails ? 1 : 0;
}
