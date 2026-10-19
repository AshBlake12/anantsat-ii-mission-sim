# CubeSat Mission Simulator

A C++17 mission simulator for AnantSAT-II, a 3U hyperspectral Earth-observation CubeSat (Team Anant, BITS Pilani), with an
interactive browser front end (orbit, groundtracks, time series, and a live 3D attitude
window of the spacecraft).

```
cmake -S . -B build && cmake --build build -j
./build/cubesat_tests                 # physics validation (50 checks)
./build/cubesat_sim                   # run config/sim.cfg, print report, write output/ + viewer/sim_data.js
./build/cubesat_sim --serve 8080      # web UI at http://127.0.0.1:8080 — edit inputs, "Run Simulation!"
```

The UI is tabbed: **Setup**, **3D Orbit**, **Groundtrack**, **Attitude**, **Plots** (pick the charts you want),
**Telemetry** and **Windows & Report**. Only the open tab renders, and "Open in new window" pops any tab into
its own window that stays synchronised with the main clock.

Opening `viewer/index.html` directly as a file shows the last CLI run. The **Run Simulation!**
button needs `--serve`, which runs the same C++ engine on each request (local only, bound to
127.0.0.1).

## What is modelled

| area | model |
|---|---|
| orbit | RK4, zonal gravity J2–J6 (EGM-96), exponential-atmosphere drag, SSO inclination and RAAN from LTAN |
| time / frames | IAU-82 GMST, WGS-84 geodesy, mean-of-date Sun ephemeris (Vallado Alg. 29) |
| eclipse | conical umbra/penumbra model (Montenbruck & Gill) |
| array geometry | 3U body plus four hinged panels at any deploy angle; front and back cells; optional body cells |
| self-shadowing | ray-cast from a sample grid on every cell face against body and panel plates |
| Earth light | albedo and IR by numerical integration over the visible cap, per face |
| cells | temperature coefficient, Martin–Ruiz incidence losses, derating, MPPT |
| panel thermal | one-node radiative balance per panel (solar, albedo, Earth IR, electrical output) |
| battery | charge/discharge efficiency, charge limit, curtailment when full, unmet-load tracking |
| geomagnetic field | IGRF-14 (NOAA/IAGA coefficients, degree 13, 2025 + secular variation) |
| attitude dynamics | rigid body (Euler equations, RK4), inertia from the body + panel geometry, gravity-gradient, aero and residual-dipole torques |
| LEOP flight plan | separation → 30 min inhibit (free tumble, stowed) → panel deploy (momentum conserved) → ADCS on + CubeMag deploy → B-dot detumble with CubeTorquers → wheels on, Sun acquisition → mission |
| ADCS hardware | CubeSpace CubeWheel CW0017/57/162 (momentum, torque, power from spec sheet), CubeTorquer CR0002/3/4 (dipole, resistance → power), CubeMag, CubeSense Sun, CubeStar |
| wheels | momentum bookkeeping h_w = Rᵀ H − Iω with cross-product magnetic dumping; rating exceedance flagged |
| subsystem loads | OBC (Zynq-7020, Xiphos Q7 typical 2 W), TTC (AD9364 RX/TX + PA), ADCS, payload, EPS |
| attitude | Sun-pointing, nadir target imaging (roll only if `payload.max_off_nadir_deg` > 0; the imager ICD requires nadir), ground-station tracking; rate- and acceleration-limited eigenaxis slews planned with lead time; safe mode with hysteresis |
| payload | 1U pushbroom hyperspectral imager (32 bands, 30 m GSD): data rate from ground speed, GSD, binning, compression; capture into camera flash, READ OUT SESSION to OBC storage at a fixed 100 Mbit/s with a band/line selection fraction, downlink from OBC storage; Sun-exclusion (11°, 15 min) check; per-rail power and peak current for the TPP v03 rails (five switched, 3V3_AON, 5V_AON, Vsys direct) |
| link | S-band C/N0 with antenna pattern and range, adaptive rate ladder with margin |

Every parameter lives in `config/sim.cfg`, tagged [S] sourced, [D] derived or [A] assumption.

## Flight plan switch

In the UI, **Detumbling (LEOP) mode** runs from separation (`flightplan.start = launch`); **Mission mode** starts
already detumbled and Sun-pointing (`start = mission`). `attitude.idle_mode = nadir` keeps the imager on
Earth between tasks; `payload.imaging_mode = continuous` images whenever the Sun is high enough.

## Panel angle convention

The deploy angle is measured from stowed (panel flat against the side face):
90° = straight out, cells facing the hinge end (+Z); 135° = swept 45° further, cells tilted
45° inward. With the Sun on +Z, a 135° panel sees the Sun at 45° incidence (cos 45° = 0.71 of
the 90° power). Change `panel_angles_deg` (four values for +X, +Y, −X, −Y) and `hinge_end` to
match the hardware.

## Outputs

`output/timeline.csv` (every field at `output_every_s`), `events.csv` (mode changes, eclipses,
AOS/LOS, imaging), `orbits.csv` (per-orbit energy balance), `tasks.csv` (every imaging and
downlink window with outcome).

## Known simplifications

- Mean and osculating elements are not distinguished at initialisation (SSO nodal rate off by ~0.4%).
- Drag uses a fixed mean area and a static exponential atmosphere (use `density_scale` for solar maximum).
- Albedo is uniform (0.30); panels do not shadow Earth light; the body is not modelled thermally.
- Attitude is kinematic (rate/acceleration limits), not a wheel-dynamics or controller model.

## Changes for TPP v03 (October 2026)

These follow the team's Technical Progress Plan, issue 03, which reviewed the published flight results of
earlier spectral CubeSats and checked the chosen parts against their datasheets.

| change | where | why |
|---|---|---|
| `geometry.failed_panels` (four flags, `1` = panel never deploys and stays stowed) | `src/sim.cpp`, `config/sim.cfg` | A flown 3U pushbroom mission lost two of four panels to non-deployment; the energy balance must be checked for one and two failed panels (TPP OI-30) |
| New always-on `5V_AON` rail for the magnetorquer drivers, split from `3V3_AON` | `src/sim.cpp` | The coils are sized for 5 V; the DRV8830 accepts 2.75-6.8 V, so it cannot run from Vsys (TPP finding 14) |
| `payload.compression_ratio` 4 -> 2 | `config/sim.cfg` | Lossless CCSDS-123 on a flown imager of this class gave a 50-60 % reduction, about 2:1 |
| Notes on the S-band PA placeholder, beacon spreading factor, battery options, off-nadir roll | `config/sim.cfg` comments | The candidate PA covers 2300-2400 MHz only (OI-25); the UHF link budget needs LoRa SF >= 10 at +20 dBm; 2S4P vs 3S2P (OI-09) |

To run the deployment-failure cases:

```
# edit config/sim.cfg:  failed_panels = 1, 0, 0, 0   (then 1, 0, 1, 0)
./build/cubesat_sim
```

The report then flags `DEPLOYMENT-FAILURE CASE` under *Array*, the DEPLOY event names the failed panels, and
the *Rails* table lists `5V_AON` separately.

**Status:** these code changes were written without a compiler available and have not yet been built or run.
Build and run the tests (`./build/cubesat_tests`) before trusting the results.

`build/`, `output/` and `viewer/sim_data.js` are generated by a run and are not committed. Run the simulator
once before opening `viewer/index.html` directly.

## Built with Claude

The simulator and the TPP v03 changes in this repository were built with
[Claude](https://claude.com/claude-code) (Anthropic's Claude Code), working with Team Anant. Engineering
decisions, parameter choices and verification are the team's responsibility.
