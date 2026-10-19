// Time systems, Earth rotation, solar ephemeris, geodesy and eclipse geometry.
//
// References:
//   Vallado, "Fundamentals of Astrodynamics and Applications", 4th ed.
//     Alg. 14 (JD), Eq. 3-47 (GMST IAU-82), Alg. 29 (Sun), Alg. 51 (geodetic)
//   Montenbruck & Gill, "Satellite Orbits", Sec. 3.4.2 (conical shadow model)
#pragma once
#include <cmath>

#include "vecmath.hpp"

namespace cs {

// Physical constants (WGS-84 / EGM-96 / IAU values).
constexpr double MU_EARTH = 398600.4418;         // km^3/s^2
constexpr double R_EARTH = 6378.1363;            // km, equatorial
constexpr double R_EARTH_MEAN = 6371.0088;       // km, mean (for great-circle distance)
constexpr double WGS84_F = 1.0 / 298.257223563;  // flattening
constexpr double OMEGA_EARTH = 7.292115146706979e-5;  // rad/s
constexpr double R_SUN = 696000.0;               // km
constexpr double AU_KM = 149597870.7;            // km
constexpr double SOLAR_CONST_1AU = 1361.0;       // W/m^2 (TSI, Kopp & Lean 2011)
constexpr double SIGMA_SB = 5.670374419e-8;      // W/m^2/K^4
constexpr double C_LIGHT = 299792458.0;          // m/s
constexpr double K_BOLTZ_DB = -228.6;            // dBW/K/Hz

// Julian date from UTC calendar date (valid 1900-2100).
inline double julian_date(int yr, int mon, int day, int hr, int min, double sec) {
    return 367.0 * yr - std::floor(7 * (yr + std::floor((mon + 9) / 12.0)) * 0.25) +
           std::floor(275 * mon / 9.0) + day + 1721013.5 +
           ((sec / 60.0 + min) / 60.0 + hr) / 24.0;
}

// Greenwich mean sidereal time, IAU-82, radians. UT1 is taken equal to UTC
// (|UT1-UTC| < 0.9 s, i.e. < 4e-3 deg of Earth rotation).
inline double gmst(double jd_ut1) {
    double T = (jd_ut1 - 2451545.0) / 36525.0;
    double sec = 67310.54841 + (876600.0 * 3600.0 + 8640184.812866) * T + 0.093104 * T * T -
                 6.2e-6 * T * T * T;
    sec = std::fmod(sec, 86400.0);
    return wrap_2pi(sec / 240.0 * DEG);
}

// Geocentric Sun position (km) in the mean-of-date equatorial frame.
// Accuracy ~0.01 deg in direction, well below any power/pointing effect here.
inline Vec3 sun_position_eci(double jd) {
    double T = (jd - 2451545.0) / 36525.0;
    double lam_m = 280.460 + 36000.771 * T;
    double M = (357.5291092 + 35999.05034 * T) * DEG;
    double lam = (lam_m + 1.914666471 * std::sin(M) + 0.019994643 * std::sin(2 * M)) * DEG;
    double r_au = 1.000140612 - 0.016708617 * std::cos(M) - 0.000139589 * std::cos(2 * M);
    double eps = (23.439291 - 0.0130042 * T) * DEG;
    double r = r_au * AU_KM;
    return {r * std::cos(lam), r * std::cos(eps) * std::sin(lam), r * std::sin(eps) * std::sin(lam)};
}

inline Vec3 eci_to_ecef(const Vec3& r, double theta_gmst) { return Mat3::rot_z(-theta_gmst) * r; }
inline Vec3 ecef_to_eci(const Vec3& r, double theta_gmst) { return Mat3::rot_z(theta_gmst) * r; }

struct Geodetic {
    double lat = 0, lon = 0, alt = 0;  // rad, rad, km
};

inline Vec3 geodetic_to_ecef(const Geodetic& g) {
    const double e2 = WGS84_F * (2 - WGS84_F);
    double sl = std::sin(g.lat), cl = std::cos(g.lat);
    double N = R_EARTH / std::sqrt(1 - e2 * sl * sl);
    return {(N + g.alt) * cl * std::cos(g.lon), (N + g.alt) * cl * std::sin(g.lon),
            (N * (1 - e2) + g.alt) * sl};
}

inline Geodetic ecef_to_geodetic(const Vec3& r) {
    const double e2 = WGS84_F * (2 - WGS84_F);
    double p = std::hypot(r.x, r.y);
    Geodetic g;
    g.lon = std::atan2(r.y, r.x);
    double lat = std::atan2(r.z, p * (1 - e2));
    for (int i = 0; i < 6; ++i) {
        double sl = std::sin(lat);
        double N = R_EARTH / std::sqrt(1 - e2 * sl * sl);
        lat = std::atan2(r.z + N * e2 * sl, p);
    }
    double sl = std::sin(lat);
    double N = R_EARTH / std::sqrt(1 - e2 * sl * sl);
    g.lat = lat;
    g.alt = std::fabs(std::cos(lat)) > 1e-8 ? p / std::cos(lat) - N : std::fabs(r.z) - N * (1 - e2);
    return g;
}

// Local East-North-Up unit vectors (ECEF) at a geodetic location.
inline void enu_basis(const Geodetic& g, Vec3& e, Vec3& n, Vec3& u) {
    double sl = std::sin(g.lat), cl = std::cos(g.lat), so = std::sin(g.lon), co = std::cos(g.lon);
    e = {-so, co, 0};
    n = {-sl * co, -sl * so, cl};
    u = {cl * co, cl * so, sl};
}

struct Topo {
    double az = 0, el = 0, range = 0;  // rad, rad, km
};

inline Topo look_angles(const Geodetic& site, const Vec3& site_ecef, const Vec3& target_ecef) {
    Vec3 e, n, u;
    enu_basis(site, e, n, u);
    Vec3 rho = target_ecef - site_ecef;
    Topo t;
    t.range = norm(rho);
    double re = dot(rho, e), rn = dot(rho, n), ru = dot(rho, u);
    t.el = std::asin(ru / t.range);
    t.az = wrap_2pi(std::atan2(re, rn));
    return t;
}

// Great-circle surface distance (km) on the mean sphere.
inline double ground_distance(double lat1, double lon1, double lat2, double lon2) {
    double dlat = lat2 - lat1, dlon = lon2 - lon1;
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2) * std::sin(dlon / 2);
    return 2 * R_EARTH_MEAN * std::asin(std::min(1.0, std::sqrt(a)));
}

// Fraction of the solar disc visible from r (0 = umbra, 1 = full sun), using
// the conical penumbra model with a spherical Earth of radius R_EARTH.
inline double sunlight_fraction(const Vec3& r_sat, const Vec3& r_sun) {
    Vec3 d = r_sun - r_sat;
    double dn = norm(d), rn = norm(r_sat);
    double a = std::asin(std::min(1.0, R_SUN / dn));    // apparent Sun radius
    double b = std::asin(std::min(1.0, R_EARTH / rn));  // apparent Earth radius
    double c = std::acos(std::clamp(dot(-r_sat, d) / (rn * dn), -1.0, 1.0));
    if (c >= a + b) return 1.0;
    if (c < b - a) return 0.0;
    if (c < a - b) return 1.0 - (b * b) / (a * a);
    double x = (c * c + a * a - b * b) / (2 * c);
    double y = std::sqrt(std::max(0.0, a * a - x * x));
    double A = a * a * std::acos(std::clamp(x / a, -1.0, 1.0)) +
               b * b * std::acos(std::clamp((c - x) / b, -1.0, 1.0)) - c * y;
    return std::clamp(1.0 - A / (PI * a * a), 0.0, 1.0);
}

}  // namespace cs
