// Bullet NEXT (Stage 1, C1S.6) - Chrono vehicle rigs: the reference curves that Rally FX's tyre
// and driveline are checked against (tools/run_chrono_rigs.sh builds and runs this).
//
//   bn_vehicle_rigs <Sedan_Pac02Tire.tir>   prints one JSON object
//
// Tyres, steady state on tarmac, Chrono's own force code (ChTMeasyTire::Advance,
// ChPac02Tire::CalcFxyMz) driven at set loads and slips:
//   - TMeasy with Chrono's BMW E90 front tyre data (225/45 R17, nominal load 3,089 N)
//   - Pacejka 2002 with Chrono's sedan tyre file (245/40 R18, nominal load 4,850 N)
//   at 2, 3 and 4 kN: pure longitudinal slip 0-0.5, pure slip angle 0-20 degrees, and combined
//   slip (4 degrees with longitudinal slip 0-0.3).
// Driveline: Chrono's shaft elements (gears, planetary differentials, clutches) built as the Evo
// VI's 4WD (Rally FX's data sheet: final 4.875, centre diff 50/50 with a clutch pack, plate
// limited-slip front and rear), on split grip (left wheels 0.3, right 0.9), first gear, full
// throttle: torque at each wheel and the front share.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsGear.h"
#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChShaftsClutch.h"
#include "chrono/physics/ChShaftsTorque.h"
#include "chrono/physics/ChShaftsMotorSpeed.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChPac02Tire.h"

using namespace chrono;
using namespace chrono::vehicle;

namespace {

// TMeasy with Chrono's BMW E90 front tyre parameters (chrono_models/vehicle/bmw, BSD-3).
class RigTMeasy : public ChTMeasyTire {
  public:
    RigTMeasy() : ChTMeasyTire("rig_tmeasy") { SetTMeasyParams(); }
    void SetTMeasyParams() override {
        m_par.pn = 3089.094750;
        m_par.pn_max = 10811.831625;
        m_unloaded_radius = 0.318600;
        m_width = 0.225000;
        m_rim_radius = 0.228600;
        m_bottom_radius = 0.233600;
        m_bottom_stiffness = 931578.958633;
        SetVerticalStiffness(310526.319544);
        SetFrictionCoefficient(0.850000);
        SetRollingResistanceCoefficient(0.010000);
        m_par.dz = 186.491147;
        m_par.dfx0_pn = 115219.880802;
        m_par.dfx0_p2n = 223456.348293;
        m_par.fxm_pn = 3804.775171;
        m_par.fxm_p2n = 7315.557946;
        m_par.fxs_pn = 2883.743455;
        m_par.fxs_p2n = 5026.523713;
        m_par.sxm_pn = 0.106017;
        m_par.sxm_p2n = 0.100500;
        m_par.sxs_pn = 0.950000;
        m_par.sxs_p2n = 0.950000;
        m_par.dfy0_pn = 87137.208230;
        m_par.dfy0_p2n = 157051.536207;
        m_par.fym_pn = 3423.092559;
        m_par.fym_p2n = 6094.402570;
        m_par.fys_pn = 2412.660879;
        m_par.fys_p2n = 3661.371397;
        m_par.sym_pn = 0.126939;
        m_par.sym_p2n = 0.119538;
        m_par.sys_pn = 1.000000;
        m_par.sys_p2n = 1.000000;
        m_par.nL0_pn = 0.106964;
        m_par.nL0_p2n = 0.153714;
        m_par.sq0_pn = 0.134683;
        m_par.sq0_p2n = 0.155617;
        m_par.sqe_pn = 0.134683;
        m_par.sqe_p2n = 0.155617;
    }
    double GetTireMass() const override { return 37.6; }
    ChVector3d GetTireInertia() const override { return ChVector3d(3.84, 6.69, 3.84); }

    // The tyre's own Synchronize steps for a wheel rolling at 20 m/s with the given load and
    // slips (sx longitudinal slip, sy = tan(slip angle)), then its Advance: the steady force.
    void Force(double fz, double sx, double sy, double& fx, double& fy) {
        m_data.in_contact = true;
        m_data.normal_force = fz;
        m_data.vel = ChVector3d(20, 0, 0);
        m_data.depth = 0.01;
        m_states.muscale = 1.0;
        m_states.gamma = 0;
        m_states.q = ChClamp(fz, 0.0, m_par.pn_max) / m_par.pn;
        m_states.omega = 20 / m_unloaded_radius;
        m_states.sx = sx;
        m_states.sy = sy;
        m_states.vsx = -sx * 20;
        m_states.vsy = -sy * 20;
        m_states.brx = m_states.bry = 0;
        m_states.dfx0 = InterpQ(m_par.dfx0_pn, m_par.dfx0_p2n);
        m_states.sxm = InterpL(m_par.sxm_pn, m_par.sxm_p2n);
        m_states.fxm = InterpQ(m_par.fxm_pn, m_par.fxm_p2n);
        m_states.sxs = InterpL(m_par.sxs_pn, m_par.sxs_p2n);
        m_states.fxs = InterpQ(m_par.fxs_pn, m_par.fxs_p2n);
        m_states.dfy0 = InterpQ(m_par.dfy0_pn, m_par.dfy0_p2n);
        m_states.sym = InterpL(m_par.sym_pn, m_par.sym_p2n);
        m_states.fym = InterpQ(m_par.fym_pn, m_par.fym_p2n);
        m_states.sys = InterpL(m_par.sys_pn, m_par.sys_p2n);
        m_states.fys = InterpQ(m_par.fys_pn, m_par.fys_p2n);
        m_states.nL0 = InterpL(m_par.nL0_pn, m_par.nL0_p2n);
        m_states.sq0 = InterpL(m_par.sq0_pn, m_par.sq0_p2n);
        m_states.sqe = InterpL(m_par.sqe_pn, m_par.sqe_p2n);
        m_stepsize = 1e-3;
        Advance(1e-3);
        fx = m_tireforce.force.x();
        fy = m_tireforce.force.y();
    }
};

// Pacejka 2002 with a .tir file (Chrono's sedan tyre).
class RigPac02 : public ChPac02Tire {
  public:
    explicit RigPac02(const std::string& tir) : ChPac02Tire("rig_pac02"), m_tir(tir) { SetMFParams(); }
    void SetMFParams() override { SetMFParamsByFile(m_tir); }
    double GetTireMass() const override { return 11.5; }
    ChVector3d GetTireInertia() const override { return ChVector3d(0.156, 0.679, 0.156); }
    double NominalLoad() const { return m_par.FNOMIN; }

    // The steady combined-slip force at a load, longitudinal slip and slip angle (rad).
    void Force(double fz, double kappa, double alpha, double& fx, double& fy) {
        m_states.mu_scale = 1.0;
        m_states.Fz0_prime = m_par.FNOMIN * m_par.LFZO;
        m_states.dfz0 = (fz - m_states.Fz0_prime) / m_states.Fz0_prime;
        m_states.Pi0_prime = m_par.IP_NOM * m_par.LIP;
        m_states.dpi = (m_par.IP - m_states.Pi0_prime) / m_states.Pi0_prime;
        m_states.vx = 20;
        m_states.kappa = kappa;
        m_states.alpha = alpha;
        m_states.gamma = 0;
        double mz = 0;
        CalcFxyMz(fx, fy, mz, kappa, alpha, fz, 0);
    }

  private:
    std::string m_tir;
};

const double kDeg = M_PI / 180.0;

struct Curve {
    std::vector<double> x, y;
};

template <class F>
void tyreJson(const char* name, const char* data, double nominal, F force) {
    std::printf("\"%s\": {\"data\": \"%s\", \"nominal_load_n\": %.1f, \"loads\": [", name, data, nominal);
    const double loads[3] = {2000, 3000, 4000};
    for (int l = 0; l < 3; ++l) {
        const double fz = loads[l];
        Curve lon, lat, comb;
        for (int i = 0; i <= 50; ++i) {
            const double k = 0.01 * i;
            double fx, fy;
            force(fz, k, 0.0, fx, fy);
            lon.x.push_back(k);
            lon.y.push_back(std::fabs(fx) / fz);
        }
        for (int i = 0; i <= 40; ++i) {
            const double a = 0.5 * i * kDeg;
            double fx, fy;
            force(fz, 0.0, a, fx, fy);
            lat.x.push_back(0.5 * i);
            lat.y.push_back(std::fabs(fy) / fz);
        }
        for (int i = 0; i <= 30; ++i) {
            const double k = 0.01 * i;
            double fx, fy;
            force(fz, k, 4.0 * kDeg, fx, fy);
            comb.x.push_back(k);
            comb.y.push_back(std::fabs(fy) / fz);
        }
        int ix = 0, iy = 0;
        for (size_t i = 0; i < lon.y.size(); ++i) if (lon.y[i] > lon.y[ix]) ix = (int)i;
        for (size_t i = 0; i < lat.y.size(); ++i) if (lat.y[i] > lat.y[iy]) iy = (int)i;
        // cornering stiffness per unit load (1/rad), from the first half degree
        const double cAlpha = lat.y[1] / (0.5 * kDeg);
        const double kSlip = lon.y[1] / 0.01;
        std::printf("%s{\"fz_n\": %.0f, \"mu_x_peak\": %.4f, \"slip_at_mu_x_peak\": %.3f, \"mu_x_at_0_5\": %.4f, "
                    "\"long_stiffness_per_load\": %.2f, \"mu_y_peak\": %.4f, \"angle_at_mu_y_peak_deg\": %.1f, "
                    "\"mu_y_at_20_deg\": %.4f, \"cornering_stiffness_per_load_per_rad\": %.2f, "
                    "\"mu_y_at_4_deg_slip_0\": %.4f, \"mu_y_at_4_deg_slip_0_1\": %.4f, \"mu_y_at_4_deg_slip_0_3\": %.4f",
                    l ? ", " : "", fz, lon.y[ix], lon.x[ix], lon.y.back(), kSlip, lat.y[iy], lat.x[iy], lat.y.back(), cAlpha,
                    comb.y[0], comb.y[10], comb.y[30]);
        std::printf(", \"curves\": {\"mu_x_vs_slip\": [");
        for (size_t i = 0; i < lon.x.size(); ++i) std::printf("%s[%.2f, %.4f]", i ? ", " : "", lon.x[i], lon.y[i]);
        std::printf("], \"mu_y_vs_angle_deg\": [");
        for (size_t i = 0; i < lat.x.size(); ++i) std::printf("%s[%.1f, %.4f]", i ? ", " : "", lat.x[i], lat.y[i]);
        std::printf("], \"mu_y_at_4_deg_vs_slip\": [");
        for (size_t i = 0; i < comb.x.size(); ++i) std::printf("%s[%.2f, %.4f]", i ? ", " : "", comb.x[i], comb.y[i]);
        std::printf("]}}");
    }
    // time per tyre evaluation (B1S.3): 200,000 combined-slip evaluations at 3 kN
    double sink = 0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < 200000; ++i) {
        double fx, fy;
        force(3000.0, 0.001 * (i % 300), 0.0002 * (i % 700), fx, fy);
        sink += fx + fy;
    }
    const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / 200000.0;
    std::printf("], \"microseconds_per_evaluation\": %.3f, \"checksum\": %.3f}", us, sink * 1e-9);
}

// --- driveline --------------------------------------------------------------------------------
struct DrivelineResult {
    double wheelTorque[4];   // FL, FR, RL, RR (Nm, at the wheel)
    double wheelSpeed[4];    // rad/s
    double frontShare;
};

// A wheel on split grip: road torque from a linear-then-saturated slip curve at a fixed car speed.
class RoadTorque : public ChShaftsTorque {
  public:
    RoadTorque(double mu, double fz, double radius, double carSpeed) : m_mu(mu), m_fz(fz), m_r(radius), m_v(carSpeed) {}
    double ComputeTorque() override {
        const double w = GetShaft1()->GetPosDt();
        const double slip = (w * m_r - m_v) / std::max(1.0, m_v);
        const double f = m_mu * m_fz * std::tanh(slip / 0.08);   // peak near 10 % slip
        m_last = f * m_r;
        return -m_last;
    }
    double m_last = 0;

  private:
    double m_mu, m_fz, m_r, m_v;
};

// Locking torques as Rally FX's drivetrain sets them (data sheet, P1.5): the centre clutch pack
// base + per rad/s of front-rear speed difference; each axle's plates preload + ramp x the torque
// the axle passes on (updated every step from the last step's road torques).
struct Locks {
    double centreBase, centrePerRadS, frontPreload, frontRamp, rearPreload, rearRamp;
};

DrivelineResult driveline(const Locks& lk) {
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, 0));
    const double engineTorque = 510;             // Nm (data sheet), full boost
    const double gear1 = 2.92, finalDrive = 4.875;
    const double r = 0.31, fz = 1230 * 9.81 / 4; // wheel radius (m), load per wheel (N)
    const double v = 5.0;                        // car speed held at 5 m/s (a launch)
    const double mu[4] = {0.3, 0.9, 0.3, 0.9};   // left wheels on ice-like grip

    auto ground = chrono_types::make_shared<ChShaft>();
    ground->SetFixed(true);
    sys.AddShaft(ground);
    auto driveshaft = chrono_types::make_shared<ChShaft>();
    driveshaft->SetInertia(0.15 * gear1 * gear1);
    sys.AddShaft(driveshaft);
    struct ConstTorque : public ChShaftsTorque {
        double t;
        explicit ConstTorque(double v) : t(v) {}
        double ComputeTorque() override { return t; }
    };
    auto motor = chrono_types::make_shared<ConstTorque>(engineTorque * gear1);
    motor->Initialize(driveshaft, ground);
    sys.Add(motor);

    auto mk = [&](double inertia) {
        auto s = chrono_types::make_shared<ChShaft>();
        s->SetInertia(inertia);
        sys.AddShaft(s);
        return s;
    };
    // centre differential (open 50/50, planetary with ratio -1) and its clutch pack
    auto frontShaft = mk(0.05), rearShaft = mk(0.05);
    auto centre = chrono_types::make_shared<ChShaftsPlanetary>();
    centre->Initialize(driveshaft, frontShaft, rearShaft);
    centre->SetTransmissionRatioOrdinary(-1.0);
    sys.Add(centre);
    auto centreClutch = chrono_types::make_shared<ChShaftsClutch>();
    centreClutch->Initialize(frontShaft, rearShaft);
    centreClutch->SetTorqueLimit(lk.centreBase);
    sys.Add(centreClutch);

    std::shared_ptr<ChShaft> wheels[4];
    std::shared_ptr<RoadTorque> road[4];
    std::shared_ptr<ChShaftsClutch> axleClutch[2];
    std::shared_ptr<ChShaft> axleIn[2] = {frontShaft, rearShaft};
    for (int a = 0; a < 2; ++a) {
        auto diffBox = mk(0.05);
        auto finalGear = chrono_types::make_shared<ChShaftsGear>();
        finalGear->Initialize(axleIn[a], diffBox);
        finalGear->SetTransmissionRatio(1.0 / finalDrive);  // direction kept (only the ratio matters here)
        sys.Add(finalGear);
        auto left = mk(1.0), right = mk(1.0);
        auto diff = chrono_types::make_shared<ChShaftsPlanetary>();
        diff->Initialize(diffBox, left, right);
        diff->SetTransmissionRatioOrdinary(-1.0);
        sys.Add(diff);
        auto clutch = chrono_types::make_shared<ChShaftsClutch>();
        clutch->Initialize(left, right);
        clutch->SetTorqueLimit(a ? lk.rearPreload : lk.frontPreload);
        sys.Add(clutch);
        axleClutch[a] = clutch;
        wheels[2 * a] = left;
        wheels[2 * a + 1] = right;
    }
    for (int w = 0; w < 4; ++w) {
        wheels[w]->SetPosDt(v / r);
        road[w] = chrono_types::make_shared<RoadTorque>(mu[w], fz, r, v);
        road[w]->Initialize(wheels[w], ground);
        sys.Add(road[w]);
    }
    const double dt = 1e-4;
    for (int i = 0; i < 5000; ++i) {  // 0.5 s: past the clutch transients
        centreClutch->SetTorqueLimit(lk.centreBase + lk.centrePerRadS * std::fabs(frontShaft->GetPosDt() - rearShaft->GetPosDt()));
        axleClutch[0]->SetTorqueLimit(lk.frontPreload + lk.frontRamp * std::fabs(road[0]->m_last + road[1]->m_last));
        axleClutch[1]->SetTorqueLimit(lk.rearPreload + lk.rearRamp * std::fabs(road[2]->m_last + road[3]->m_last));
        sys.DoStepDynamics(dt);
    }
    DrivelineResult res;
    double front = 0, total = 0;
    for (int w = 0; w < 4; ++w) {
        res.wheelTorque[w] = road[w]->m_last;
        res.wheelSpeed[w] = wheels[w]->GetPosDt();
        total += road[w]->m_last;
        if (w < 2) front += road[w]->m_last;
    }
    res.frontShare = total != 0 ? front / total : 0;
    return res;
}

void drivelineJson(const char* name, const Locks& lk, bool last) {
    const DrivelineResult d = driveline(lk);
    // torque bias ratio of each axle: the high-grip wheel's torque over the low-grip wheel's
    const double tbrFront = d.wheelTorque[0] != 0 ? d.wheelTorque[1] / d.wheelTorque[0] : 0;
    const double tbrRear = d.wheelTorque[2] != 0 ? d.wheelTorque[3] / d.wheelTorque[2] : 0;
    std::printf("{\"case\": \"%s\", \"centre_base_nm\": %.0f, \"front_preload_nm\": %.0f, \"front_ramp\": %.2f, \"rear_preload_nm\": %.0f, \"rear_ramp\": %.2f, "
                "\"wheel_torque_nm\": [%.1f, %.1f, %.1f, %.1f], \"wheel_speed_rad_s\": [%.2f, %.2f, %.2f, %.2f], "
                "\"front_share\": %.3f, \"total_drive_nm\": %.1f, \"torque_bias_front\": %.2f, \"torque_bias_rear\": %.2f}%s",
                name, lk.centreBase, lk.frontPreload, lk.frontRamp, lk.rearPreload, lk.rearRamp, d.wheelTorque[0], d.wheelTorque[1], d.wheelTorque[2],
                d.wheelTorque[3], d.wheelSpeed[0], d.wheelSpeed[1], d.wheelSpeed[2], d.wheelSpeed[3], d.frontShare,
                d.wheelTorque[0] + d.wheelTorque[1] + d.wheelTorque[2] + d.wheelTorque[3], tbrFront, tbrRear, last ? "" : ", ");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: bn_vehicle_rigs <Sedan_Pac02Tire.tir>\n");
        return 2;
    }
    RigTMeasy tm;
    RigPac02 pac(argv[1]);
    std::printf("{\"tyres\": {");
    tyreJson("tmeasy_bmw_e90_front", "Chrono BMW E90 front TMeasy (225/45 R17)", 3089.1,
             [&](double fz, double k, double a, double& fx, double& fy) { tm.Force(fz, k, std::tan(a), fx, fy); });
    std::printf(", ");
    tyreJson("pac02_sedan", "Chrono sedan Pacejka 2002 (245/40 R18)", pac.NominalLoad(),
             [&](double fz, double k, double a, double& fx, double& fy) { pac.Force(fz, k, a, fx, fy); });
    std::printf("}, \"driveline\": [");
    // Rally FX's drivetrain at full throttle (P1.5): centre 100 + 900 x throttle Nm + 20 Nm per
    // rad/s; front plates 40 Nm + 0.25 x axle torque, rear 60 Nm + 0.5 x axle torque. The axle
    // torque is what the axle passes to the road (its two wheels' road torques).
    drivelineJson("open diffs", Locks{0, 0, 0, 0, 0, 0}, false);
    drivelineJson("Rally FX settings, full throttle", Locks{1000, 20, 40, 0.25, 60, 0.5}, false);
    drivelineJson("all locked", Locks{1e6, 0, 1e6, 0, 1e6, 0}, true);
    std::printf("]}\n");
    return 0;
}
