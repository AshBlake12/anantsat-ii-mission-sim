// Spacecraft geometry in the body frame: 3U structure, hinged deployable
// panels, cell-bearing faces, and ray-cast self-shadowing.
//
// Body frame: +Z along the long (3U) axis. Panels hinge on the four long edges
// at the `hinge_end` (+Z or -Z) end of the structure.
//
//   Deploy angle theta, measured from the stowed position (panel flat against
//   the side face, cells facing outward):
//
//        theta = 0     panel lies along the body side
//        theta = 90    panel sticks straight out (cells face the hinge end, +Z)
//        theta = 135   panel swept 45 deg past perpendicular; front cells tilt
//                      45 deg inward, forming a cup that opens toward +Z
//        theta = 180   panel points straight up, cells face inward
//
//   Front normal   n(theta) = s cos(theta) + sigma z sin(theta)
//   Extension dir  d(theta) = s sin(theta) - sigma z cos(theta)
//   where s is the outward side normal and sigma = +1 for a +Z hinge end.
#pragma once
#include <string>
#include <vector>

#include "vecmath.hpp"

namespace cs {

struct Plate {  // opaque rectangle used for occlusion
    Vec3 c, u, v, n;
    double hu = 0, hv = 0;
    int id = -1;
};

struct CellFace {  // one side of a plate that may carry solar cells
    std::string name;
    int plate_id = -1;
    int panel = -1;       // deployable panel index, -1 for body faces
    Vec3 n;               // outward normal of this side
    double area = 0;      // geometric plate area, m^2
    double cell_area = 0; // active cell area, m^2
    std::vector<Vec3> samples;
};

struct GeometryConfig {
    double body_w = 0.100, body_l = 0.3405;  // m
    double panel_w = 0.098, panel_l = 0.311;  // m
    int hinge_end = +1;                       // +1: +Z end, -1: -Z end
    double angles_deg[4] = {135, 135, 135, 135};  // panels on +X, +Y, -X, -Y
    double cells_front = 7, cells_back = 7;       // cells per panel side
    double cell_area_cm2 = 30.18;                 // per cell (e.g. 3G30C)
    double body_cells[6] = {0, 0, 0, 0, 0, 0};    // +X,-X,+Y,-Y,+Z,-Z
    int samples_u = 12, samples_v = 6;            // shadow sampling grid
};

struct Geometry {
    std::vector<Plate> plates;
    std::vector<CellFace> faces;
    GeometryConfig cfg;

    static Plate make_plate(Vec3 c, Vec3 u, Vec3 v, double hu, double hv, int id) {
        Plate p{c, unit(u), unit(v), unit(cross(u, v)), hu, hv, id};
        return p;
    }

    void add_face(const std::string& name, const Plate& p, bool flip, double cells, int panel,
                  double cell_area_m2) {
        CellFace f;
        f.name = name;
        f.plate_id = p.id;
        f.panel = panel;
        f.n = flip ? -p.n : p.n;
        f.area = 4 * p.hu * p.hv;
        f.cell_area = cells * cell_area_m2;
        const int nu = cfg.samples_u, nv = cfg.samples_v;
        for (int i = 0; i < nu; ++i)
            for (int j = 0; j < nv; ++j) {
                double a = ((i + 0.5) / nu * 2 - 1) * p.hu;
                double b = ((j + 0.5) / nv * 2 - 1) * p.hv;
                f.samples.push_back(p.c + a * p.u + b * p.v + f.n * 1e-6);
            }
        faces.push_back(std::move(f));
    }

    explicit Geometry(const GeometryConfig& c) : cfg(c) {
        const double w = c.body_w / 2, l = c.body_l / 2;
        const double cell_m2 = c.cell_area_cm2 * 1e-4;
        const Vec3 X{1, 0, 0}, Y{0, 1, 0}, Z{0, 0, 1};
        // Body box: outward normals via u x v.
        struct B { const char* name; Vec3 c, u, v; double hu, hv; };
        B box[6] = {{"+X body", X * w, Y, Z, w, l},  {"-X body", -X * w, Z, Y, l, w},
                    {"+Y body", Y * w, Z, X, l, w},  {"-Y body", -Y * w, X, Z, w, l},
                    {"+Z body", Z * l, X, Y, w, w},  {"-Z body", -Z * l, Y, X, w, w}};
        for (int k = 0; k < 6; ++k) {
            Plate p = make_plate(box[k].c, box[k].u, box[k].v, box[k].hu, box[k].hv, (int)plates.size());
            plates.push_back(p);
            if (c.body_cells[k] > 0) add_face(box[k].name, p, false, c.body_cells[k], -1, cell_m2);
        }
        // Deployable panels.
        const Vec3 sides[4] = {X, Y, -X, -Y};
        const char* names[4] = {"+X", "+Y", "-X", "-Y"};
        const double sg = c.hinge_end >= 0 ? 1.0 : -1.0;
        for (int k = 0; k < 4; ++k) {
            double th = c.angles_deg[k] * DEG;
            Vec3 s = sides[k];
            Vec3 hinge = s * (w + 0.0015) + Z * (sg * l);  // hinge line just outside the rail
            Vec3 d = s * std::sin(th) - Z * (sg * std::cos(th));
            Vec3 n = s * std::cos(th) + Z * (sg * std::sin(th));
            Vec3 t = unit(cross(n, d));  // along hinge
            Plate p = make_plate(hinge + d * (c.panel_l / 2), d, t, c.panel_l / 2, c.panel_w / 2,
                                 (int)plates.size());
            // make_plate normal is d x t = n (front)
            plates.push_back(p);
            add_face(std::string(names[k]) + " panel front", p, false, c.cells_front, k, cell_m2);
            if (c.cells_back > 0)
                add_face(std::string(names[k]) + " panel back", p, true, c.cells_back, k, cell_m2);
        }
    }

    static bool ray_hits(const Vec3& o, const Vec3& dir, const Plate& p) {
        double den = dot(dir, p.n);
        if (std::fabs(den) < 1e-12) return false;
        double t = dot(p.c - o, p.n) / den;
        if (t <= 1e-9) return false;
        Vec3 q = o + dir * t - p.c;
        return std::fabs(dot(q, p.u)) <= p.hu && std::fabs(dot(q, p.v)) <= p.hv;
    }

    // Fraction of a face's sample points with an unobstructed line to the Sun.
    double lit_fraction(const CellFace& f, const Vec3& sun_b) const {
        if (dot(f.n, sun_b) <= 0) return 0.0;
        int lit = 0;
        for (const Vec3& s : f.samples) {
            bool blocked = false;
            for (const Plate& p : plates) {
                if (p.id == f.plate_id) continue;
                if (ray_hits(s, sun_b, p)) { blocked = true; break; }
            }
            if (!blocked) ++lit;
        }
        return double(lit) / f.samples.size();
    }

    // Approximate projected area along a direction (overlaps not removed).
    double projected_area(const Vec3& d) const {
        double a = 0;
        for (int k = 0; k < 6; ++k) a += std::max(0.0, dot(plates[k].n, d)) * 4 * plates[k].hu * plates[k].hv;
        for (size_t k = 6; k < plates.size(); ++k) a += std::fabs(dot(plates[k].n, d)) * 4 * plates[k].hu * plates[k].hv;
        return a;
    }
};

}  // namespace cs
