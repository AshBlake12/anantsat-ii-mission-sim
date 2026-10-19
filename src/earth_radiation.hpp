// Earth-reflected sunlight (albedo) and Earth infrared irradiance on flat
// faces, by numerical integration over the visible spherical cap.
//
// Each cap cell is a Lambertian emitter:
//   albedo radiance  L_a = a * S * max(0, cos(solar zenith)) / pi
//   infrared radiance L_ir = OLR / pi
// Irradiance on a face with unit normal n:
//   E = sum L * max(0, n . d_hat) * cos(theta_earth) * dA / |d|^2
// This handles the terminator, partial views and tilted faces exactly, to the
// resolution of the grid. Spacecraft self-occlusion of Earth light is ignored.
#pragma once
#include <vector>

#include "astro.hpp"
#include "vecmath.hpp"

namespace cs {

struct EarthRadiation {
    double albedo = 0.30;  // global mean Bond albedo (CERES ~0.29-0.30)
    double olr = 237.0;    // W/m^2 outgoing longwave (CERES global mean ~239)
    int n_lambda = 48, n_phi = 96;

    std::vector<double> cphi, sphi;

    EarthRadiation() { init(); }
    void init() {
        cphi.resize(n_phi);
        sphi.resize(n_phi);
        for (int j = 0; j < n_phi; ++j) {
            double ph = (j + 0.5) * TWO_PI / n_phi;
            cphi[j] = std::cos(ph);
            sphi[j] = std::sin(ph);
        }
    }

    // normals: face normals in ECI. Outputs per-face albedo and IR irradiance (W/m^2).
    void irradiance(const Vec3& r_sat, const Vec3& sun_hat, double solar_flux,
                    const std::vector<Vec3>& normals, std::vector<double>& e_alb,
                    std::vector<double>& e_ir) const {
        const size_t nf = normals.size();
        e_alb.assign(nf, 0.0);
        e_ir.assign(nf, 0.0);
        double rn = norm(r_sat);
        Vec3 k = r_sat / rn;
        Vec3 e1 = any_perp(k), e2 = cross(k, e1);
        double lam_max = std::acos(R_EARTH / rn);
        double dl = lam_max / n_lambda, dp = TWO_PI / n_phi;
        const double La0 = albedo * solar_flux / PI, Lir = olr / PI;
        for (int i = 0; i < n_lambda; ++i) {
            double lam = (i + 0.5) * dl;
            double cl = std::cos(lam), sl = std::sin(lam);
            double dA = R_EARTH * R_EARTH * sl * dl * dp * 1e6;  // m^2
            for (int j = 0; j < n_phi; ++j) {
                Vec3 c = cl * k + sl * (cphi[j] * e1 + sphi[j] * e2);
                Vec3 d = c * R_EARTH - r_sat;  // km
                double dist = norm(d);
                Vec3 dh = d / dist;
                double cos_e = -dot(c, dh);
                if (cos_e <= 0) continue;
                double w = cos_e * dA / (dist * dist * 1e6);  // solid-angle weight
                double cos_s = dot(c, sun_hat);
                double La = cos_s > 0 ? La0 * cos_s : 0.0;
                for (size_t f = 0; f < nf; ++f) {
                    double cf = dot(normals[f], dh);
                    if (cf <= 0) continue;
                    e_alb[f] += La * cf * w;
                    e_ir[f] += Lir * cf * w;
                }
            }
        }
    }
};

}  // namespace cs
