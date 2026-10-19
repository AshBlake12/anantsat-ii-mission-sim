// Orbit propagation: Earth zonal harmonics J2..J6 + atmospheric drag, RK4.
#pragma once
#include <array>
#include <cmath>

#include "astro.hpp"
#include "vecmath.hpp"

namespace cs {

// EGM-96 unnormalised zonal coefficients J2..J6.
constexpr std::array<double, 7> J_ZONAL = {0, 0, 1.08262668355e-3, -2.53265648533e-6,
                                           -1.61962159137e-6, -2.27296082869e-7, 5.40681239107e-7};

struct State {
    Vec3 r;  // km, ECI
    Vec3 v;  // km/s, ECI
};

struct Elements {
    double a = 0, e = 0, i = 0, raan = 0, argp = 0, nu = 0;  // km, -, rad...
};

inline State elements_to_state(const Elements& el) {
    double p = el.a * (1 - el.e * el.e);
    double r = p / (1 + el.e * std::cos(el.nu));
    Vec3 r_pqw{r * std::cos(el.nu), r * std::sin(el.nu), 0};
    double k = std::sqrt(MU_EARTH / p);
    Vec3 v_pqw{-k * std::sin(el.nu), k * (el.e + std::cos(el.nu)), 0};
    double cO = std::cos(el.raan), sO = std::sin(el.raan);
    double cw = std::cos(el.argp), sw = std::sin(el.argp);
    double ci = std::cos(el.i), si = std::sin(el.i);
    Mat3 R = Mat3::from_cols({cO * cw - sO * sw * ci, sO * cw + cO * sw * ci, sw * si},
                             {-cO * sw - sO * cw * ci, -sO * sw + cO * cw * ci, cw * si},
                             {sO * si, -cO * si, ci});
    return {R * r_pqw, R * v_pqw};
}

inline Elements state_to_elements(const State& s) {
    Elements el;
    Vec3 h = cross(s.r, s.v);
    Vec3 n = cross(Vec3{0, 0, 1}, h);
    double r = norm(s.r), v2 = dot(s.v, s.v);
    Vec3 ev = ((v2 - MU_EARTH / r) * s.r - dot(s.r, s.v) * s.v) / MU_EARTH;
    el.e = norm(ev);
    el.a = 1.0 / (2.0 / r - v2 / MU_EARTH);
    el.i = std::acos(std::clamp(h.z / norm(h), -1.0, 1.0));
    el.raan = wrap_2pi(std::atan2(n.y, n.x));
    // Argument of latitude is well defined for near-circular orbits; store it in nu
    // with argp = 0 when e is tiny.
    double u = std::atan2(dot(s.r, cross(h, n)) / norm(h), dot(s.r, n));
    if (el.e > 1e-6) {
        el.argp = wrap_2pi(std::atan2(dot(ev, cross(h, n)) / norm(h), dot(ev, n)));
        el.nu = wrap_2pi(u - el.argp);
    } else {
        el.argp = 0;
        el.nu = wrap_2pi(u);
    }
    return el;
}

// Sun-synchronous inclination for a given semi-major axis / eccentricity.
inline double sso_inclination(double a, double e) {
    const double rate = TWO_PI / (365.2421897 * 86400.0);  // rad/s, mean Sun
    double n = std::sqrt(MU_EARTH / (a * a * a));
    double p = a * (1 - e * e);
    double c = -rate / (1.5 * n * J_ZONAL[2] * (R_EARTH / p) * (R_EARTH / p));
    return std::acos(std::clamp(c, -1.0, 1.0));
}

// RAAN that places the ascending node at the given local mean solar time.
inline double raan_for_ltan(double jd_utc, double ltan_hours) {
    double ut_hours = std::fmod(jd_utc - 0.5, 1.0) * 24.0;
    return wrap_2pi(gmst(jd_utc) + (ltan_hours - ut_hours) * 15.0 * DEG);
}

// Zonal gravity acceleration J2..jmax, from the gradient of
// V_n = -mu J_n R^n / r^(n+1) P_n(u), u = z/r:
//   a_n = mu J_n R^n / r^(n+2) [((n+1) P_n + u P_n') r_hat - P_n' z_hat]
inline Vec3 gravity_accel(const Vec3& r, int jmax) {
    double rn = norm(r);
    Vec3 rh = r / rn;
    Vec3 a = -MU_EARTH / (rn * rn) * rh;
    double u = rh.z;
    double P[8] = {1, u}, dP[8] = {0, 1};
    for (int n = 2; n <= jmax; ++n) {
        P[n] = ((2 * n - 1) * u * P[n - 1] - (n - 1) * P[n - 2]) / n;
        dP[n] = dP[n - 2] + (2 * n - 1) * P[n - 1];
    }
    double Rr = R_EARTH / rn;
    double Rrn = Rr;
    for (int n = 2; n <= jmax; ++n) {
        Rrn *= Rr;
        double k = MU_EARTH * J_ZONAL[n] * Rrn / (rn * rn);
        a += k * (((n + 1) * P[n] + u * dP[n]) * rh - dP[n] * Vec3{0, 0, 1});
    }
    return a;
}

// Exponential atmosphere (Vallado Table 8-4, CIRA-72 based), kg/m^3.
inline double atmos_density(double h_km) {
    static const double tbl[][3] = {
        {0, 1.225, 7.249},          {25, 3.899e-2, 6.349},    {30, 1.774e-2, 6.682},
        {40, 3.972e-3, 7.554},      {50, 1.057e-3, 8.382},    {60, 3.206e-4, 7.714},
        {70, 8.770e-5, 6.549},      {80, 1.905e-5, 5.799},    {90, 3.396e-6, 5.382},
        {100, 5.297e-7, 5.877},     {110, 9.661e-8, 7.263},   {120, 2.438e-8, 9.473},
        {130, 8.484e-9, 12.636},    {140, 3.845e-9, 16.149},  {150, 2.070e-9, 22.523},
        {180, 5.464e-10, 29.740},   {200, 2.789e-10, 37.105}, {250, 7.248e-11, 45.546},
        {300, 2.418e-11, 53.628},   {350, 9.518e-12, 53.298}, {400, 3.725e-12, 58.515},
        {450, 1.585e-12, 60.828},   {500, 6.967e-13, 63.822}, {600, 1.454e-13, 71.835},
        {700, 3.614e-14, 88.667},   {800, 1.170e-14, 124.64}, {900, 5.245e-15, 181.05},
        {1000, 3.019e-15, 268.00}};
    int n = sizeof(tbl) / sizeof(tbl[0]);
    int k = 0;
    while (k + 1 < n && h_km >= tbl[k + 1][0]) ++k;
    return tbl[k][1] * std::exp(-(h_km - tbl[k][0]) / tbl[k][2]);
}

struct ForceModel {
    int jmax = 6;
    bool drag = true;
    double cd = 2.2;
    double area_m2 = 0.05;
    double mass_kg = 4.0;
    double density_scale = 1.0;  // >1 for high solar activity

    Vec3 accel(const Vec3& r, const Vec3& v) const {
        Vec3 a = gravity_accel(r, jmax);
        if (drag) {
            double h = norm(r) - R_EARTH;
            double rho = atmos_density(h) * density_scale;
            Vec3 vrel = v - cross(Vec3{0, 0, OMEGA_EARTH}, r);  // km/s
            double vr = norm(vrel) * 1000.0;                     // m/s
            double k = -0.5 * rho * cd * area_m2 / mass_kg * vr;  // 1/s
            a += k * vrel;  // km/s^2 (vrel in km/s, k in 1/s)
        }
        return a;
    }
};

inline State rk4_step(const State& s, double h, const ForceModel& f) {
    auto deriv = [&](const State& x) { return State{x.v, f.accel(x.r, x.v)}; };
    State k1 = deriv(s);
    State s2{s.r + k1.r * (h / 2), s.v + k1.v * (h / 2)};
    State k2 = deriv(s2);
    State s3{s.r + k2.r * (h / 2), s.v + k2.v * (h / 2)};
    State k3 = deriv(s3);
    State s4{s.r + k3.r * h, s.v + k3.v * h};
    State k4 = deriv(s4);
    return {s.r + (k1.r + 2 * k2.r + 2 * k3.r + k4.r) * (h / 6),
            s.v + (k1.v + 2 * k2.v + 2 * k3.v + k4.v) * (h / 6)};
}

}  // namespace cs
