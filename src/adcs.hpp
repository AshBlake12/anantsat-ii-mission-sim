// Attitude dynamics and CubeSpace ADCS hardware models.
//
// Hardware figures are from CubeSpace product specification sheets (2023-2025):
//   CubeWheel NanoSat range (CS-DEV... Reaction Wheels spec, Table 3)
//   CubeTorquer rods (Magnetorquer Rods spec) and Turn-Key ADCS guideline Table 5
//   CubeMag, CubeSense Sun, CubeStar product descriptions
#pragma once
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry.hpp"
#include "vecmath.hpp"

namespace cs {

// ------------------------------------------------------------ 3x3 algebra
inline Mat3 zeros3() { Mat3 z; for (auto& r : z.m) for (double& v : r) v = 0; return z; }
inline Mat3 outer(const Vec3& a, const Vec3& b) {
    Mat3 r;
    const double A[3] = {a.x, a.y, a.z}, B[3] = {b.x, b.y, b.z};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.m[i][j] = A[i] * B[j];
    return r;
}
inline Mat3 add(const Mat3& a, const Mat3& b, double s = 1) {
    Mat3 r;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[i][j] + s * b.m[i][j];
    return r;
}
inline Mat3 inverse(const Mat3& M) {
    const auto& m = M.m;
    double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                 m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    Mat3 r;
    r.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / det;
    r.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / det;
    r.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / det;
    r.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) / det;
    r.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / det;
    r.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) / det;
    r.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / det;
    r.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / det;
    r.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / det;
    return r;
}

// ------------------------------------------------------------ mass model
struct MassProps {
    double mass = 0;
    Vec3 com;     // body frame, m
    Mat3 inertia; // about the CoM, kg m^2
};

// Body: uniform box of (total - panels) mass, optionally offset CoM.
// Panels: thin uniform plates at their deployed (or stowed) pose.
inline MassProps mass_properties(const Geometry& g, double total_kg, double panel_kg, const Vec3& body_com_offset) {
    const double w = g.cfg.body_w, l = g.cfg.body_l;
    const double mb = total_kg - 4 * panel_kg;
    struct Part { double m; Vec3 c; Mat3 I; };
    std::vector<Part> parts;
    Mat3 Ib = zeros3();
    Ib.m[0][0] = mb / 12 * (w * w + l * l);
    Ib.m[1][1] = mb / 12 * (w * w + l * l);
    Ib.m[2][2] = mb / 12 * (2 * w * w);
    parts.push_back({mb, body_com_offset, Ib});
    for (size_t k = 6; k < g.plates.size(); ++k) {
        const Plate& p = g.plates[k];
        const double L = 2 * p.hu, W = 2 * p.hv;
        Mat3 I = zeros3();
        I = add(I, outer(p.u, p.u), panel_kg * W * W / 12);
        I = add(I, outer(p.v, p.v), panel_kg * L * L / 12);
        I = add(I, outer(p.n, p.n), panel_kg * (L * L + W * W) / 12);
        parts.push_back({panel_kg, p.c, I});
    }
    MassProps mp;
    for (auto& p : parts) { mp.mass += p.m; mp.com += p.c * p.m; }
    mp.com = mp.com / mp.mass;
    mp.inertia = zeros3();
    for (auto& p : parts) {
        Vec3 d = p.c - mp.com;
        Mat3 par = zeros3();
        for (int i = 0; i < 3; ++i) par.m[i][i] = dot(d, d);
        par = add(par, outer(d, d), -1.0);
        mp.inertia = add(mp.inertia, add(p.I, par, p.m));
    }
    return mp;
}

// Component-level mass budget: each item is a uniform box (or a thin-walled box
// "shell", e.g. the structure) at a body-frame position. Deployable panels are
// added from the geometry as thin plates.
struct MassItem {
    std::string name;
    double m = 0;
    Vec3 c, size;
    bool shell = false;
};

inline Mat3 box_inertia(double m, const Vec3& s) {
    Mat3 I = zeros3();
    I.m[0][0] = m / 12 * (s.y * s.y + s.z * s.z);
    I.m[1][1] = m / 12 * (s.x * s.x + s.z * s.z);
    I.m[2][2] = m / 12 * (s.x * s.x + s.y * s.y);
    return I;
}

inline MassProps mass_properties(const Geometry& g, const std::vector<MassItem>& items, double panel_kg) {
    struct Part { double m; Vec3 c; Mat3 I; };
    std::vector<Part> parts;
    for (const auto& it : items) {
        if (!it.shell) { parts.push_back({it.m, it.c, box_inertia(it.m, it.size)}); continue; }
        // thin-walled box: mass shared by the six faces in proportion to area
        const Vec3 s = it.size;
        const double ax = s.y * s.z, ay = s.x * s.z, az = s.x * s.y, tot = 2 * (ax + ay + az);
        const struct { Vec3 off; Vec3 dim; double a; } f[6] = {
            {{s.x / 2, 0, 0}, {0, s.y, s.z}, ax}, {{-s.x / 2, 0, 0}, {0, s.y, s.z}, ax},
            {{0, s.y / 2, 0}, {s.x, 0, s.z}, ay}, {{0, -s.y / 2, 0}, {s.x, 0, s.z}, ay},
            {{0, 0, s.z / 2}, {s.x, s.y, 0}, az}, {{0, 0, -s.z / 2}, {s.x, s.y, 0}, az}};
        for (const auto& fc : f) {
            double m = it.m * fc.a / tot;
            parts.push_back({m, it.c + fc.off, box_inertia(m, fc.dim)});
        }
    }
    for (size_t k = 6; k < g.plates.size(); ++k) {
        const Plate& p = g.plates[k];
        const double L = 2 * p.hu, W = 2 * p.hv;
        Mat3 I = zeros3();
        I = add(I, outer(p.u, p.u), panel_kg * W * W / 12);
        I = add(I, outer(p.v, p.v), panel_kg * L * L / 12);
        I = add(I, outer(p.n, p.n), panel_kg * (L * L + W * W) / 12);
        parts.push_back({panel_kg, p.c, I});
    }
    MassProps mp;
    for (auto& p : parts) { mp.mass += p.m; mp.com += p.c * p.m; }
    mp.com = mp.com / mp.mass;
    mp.inertia = zeros3();
    for (auto& p : parts) {
        Vec3 d = p.c - mp.com;
        Mat3 par = zeros3();
        for (int i = 0; i < 3; ++i) par.m[i][i] = dot(d, d);
        par = add(par, outer(d, d), -1.0);
        mp.inertia = add(mp.inertia, add(p.I, par, p.m));
    }
    return mp;
}

// ------------------------------------------------------------ hardware
struct WheelSpec {
    std::string model;
    double h_rated;      // Nms at 6000 rpm, min motor voltage
    double torque;       // N m nominal
    double max_rpm;
    double p_avg2000;    // W, average power at 2000 rpm
    double p_peak;       // W, peak at nominal torque (incl. digital)
    double mass;         // kg
};
// CubeSpace Reaction Wheels specification sheet, Table 3 (NanoSat range).
inline WheelSpec wheel_spec(const std::string& m) {
    if (m == "CW0017") return {m, 1.77e-3, 0.23e-3, 10000, 0.180, 0.85, 0.060};
    if (m == "CW0057") return {m, 5.7e-3, 2.0e-3, 10000, 0.336, 2.7, 0.115};
    if (m == "CW0162") return {m, 16.2e-3, 7.0e-3, 10000, 0.336, 7.2, 0.144};
    if (m == "CW0500") return {m, 50e-3, 10e-3, 10000, 0.800, 15.0, 0.322};
    throw std::runtime_error("unknown wheel model '" + m + "' (CW0017, CW0057, CW0162, CW0500)");
}
struct TorquerSpec {
    std::string model;
    double m_max;  // A m^2 at 5 V
    double ohm;    // coil resistance
};
// CubeSpace Magnetorquer Rods specification: dipole @5 V and resistance.
// CR0003 (0.3 A m^2) appears in the Turn-Key ADCS guideline but its resistance is
// not published; 45 ohm is interpolated between CR0002 and CR0004 [A].
inline TorquerSpec torquer_spec(const std::string& m) {
    if (m == "CR0002") return {m, 0.2, 51.0};
    if (m == "CR0003") return {m, 0.3, 45.0};
    if (m == "CR0004") return {m, 0.4, 39.5};
    if (m == "CR0006") return {m, 0.6, 45.0};
    if (m == "CR0010") return {m, 1.0, 37.5};
    throw std::runtime_error("unknown torquer model '" + m + "' (CR0002, CR0003, CR0004, CR0006, CR0010)");
}

// ------------------------------------------------------------ dynamics
struct RigidState {
    Quat q;   // body -> ECI
    Vec3 w;   // body rate, rad/s (body frame)
};

// Euler's equations with torque held over the step; RK4.
inline RigidState rigid_step(const RigidState& s, const Mat3& I, const Mat3& Iinv, const Vec3& tau_b, double h) {
    auto wdot = [&](const Vec3& w) { return Iinv * (tau_b - cross(w, I * w)); };
    auto qdot = [](const Quat& q, const Vec3& w) {
        Quat d = q * Quat{0, w.x, w.y, w.z};
        return Quat{0.5 * d.w, 0.5 * d.x, 0.5 * d.y, 0.5 * d.z};
    };
    auto qadd = [](const Quat& q, const Quat& d, double k) { return Quat{q.w + d.w * k, q.x + d.x * k, q.y + d.y * k, q.z + d.z * k}; };
    Vec3 k1w = wdot(s.w);              Quat k1q = qdot(s.q, s.w);
    Vec3 w2 = s.w + k1w * (h / 2);     Quat q2 = qadd(s.q, k1q, h / 2);
    Vec3 k2w = wdot(w2);               Quat k2q = qdot(q2, w2);
    Vec3 w3 = s.w + k2w * (h / 2);     Quat q3 = qadd(s.q, k2q, h / 2);
    Vec3 k3w = wdot(w3);               Quat k3q = qdot(q3, w3);
    Vec3 w4 = s.w + k3w * h;           Quat q4 = qadd(s.q, k3q, h);
    Vec3 k4w = wdot(w4);               Quat k4q = qdot(q4, w4);
    RigidState o;
    o.w = s.w + (k1w + 2 * k2w + 2 * k3w + k4w) * (h / 6);
    Quat q{s.q.w + (k1q.w + 2 * k2q.w + 2 * k3q.w + k4q.w) * h / 6, s.q.x + (k1q.x + 2 * k2q.x + 2 * k3q.x + k4q.x) * h / 6,
           s.q.y + (k1q.y + 2 * k2q.y + 2 * k3q.y + k4q.y) * h / 6, s.q.z + (k1q.z + 2 * k2q.z + 2 * k3q.z + k4q.z) * h / 6};
    o.q = q.normalized();
    return o;
}

// Gravity-gradient torque (body frame), r_b = position unit vector in body frame.
inline Vec3 gravity_gradient_torque(const Vec3& r_hat_b, double r_km, const Mat3& I) {
    const double n2 = 398600.4418e9 / std::pow(r_km * 1e3, 3);
    return 3 * n2 * cross(r_hat_b, I * r_hat_b);
}

// Aerodynamic force and torque (body frame) from the plate model.
// v_rel_b: velocity of the spacecraft relative to the atmosphere (m/s), body frame.
inline Vec3 aero_torque(const Geometry& g, const Vec3& com, const Vec3& v_rel_b, double rho, double cd) {
    double v = norm(v_rel_b);
    if (v <= 0 || rho <= 0) return {};
    Vec3 vh = v_rel_b / v, tau;
    for (size_t k = 0; k < g.plates.size(); ++k) {
        const Plate& p = g.plates[k];
        double c = dot(p.n, vh);
        if (k < 6 && c <= 0) continue;            // body: only windward faces
        double area = 4 * p.hu * p.hv * std::fabs(c);  // thin panels: either side
        Vec3 f = -0.5 * rho * cd * area * v * v * vh;
        tau += cross(p.c - com, f);
    }
    return tau;
}

}  // namespace cs
