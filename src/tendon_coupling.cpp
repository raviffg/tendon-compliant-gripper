// tendon_coupling.cpp
// Excursion coupling solver for the 3-finger underactuated compliant gripper.
//
// Architecture recap:
//   - 3 fingers, 2 phalanges each (proximal + distal) -> 6 joints total.
//   - 3 tendons, one per finger, routed over idler pulleys at each joint.
//   - Each joint carries a torsional return spring (k_j) that resists flexion.
//   - Underactuation: 3 actuators, 6 DOF. Joint torques are resolved by the
//     spring equilibrium, not by the motor directly.
//
// The coupling matrix maps tendon pull (dL, metres) to joint angles (dtheta, rad)
// through the pulley radii. For a single finger with tendons routed on the
// flexion side of both joints:
//
//     dL_i = R_prox * dtheta_prox + R_dist * dtheta_dist
//
// so the full 3x6 matrix is block diagonal, one 1x2 row per finger. We solve
// the least-squares joint displacement for a commanded excursion vector and
// then distribute contact normal forces at the phalanges.

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace gripper {

constexpr std::size_t kNumFingers = 3;
constexpr std::size_t kJointsPerFinger = 2;
constexpr std::size_t kNumJoints = kNumFingers * kJointsPerFinger;

// Pulley geometry, measured on the rev C finger mold. Do not change without
// re-running scripts/grasp_stiffness_eval.py, the slip margins are sensitive.
constexpr double kRadiusProximal = 0.0082;  // m, 8.2 mm idler at MCP
constexpr double kRadiusDistal = 0.0055;    // m, 5.5 mm idler at PIP

// Torsional return spring stiffness per joint, N*m/rad. Proximal springs are
// deliberately softer so the finger curls distal-first under load.
constexpr std::array<double, kNumJoints> kSpringStiffness = {
    0.42, 0.28,   // finger 0: prox, dist
    0.45, 0.30,   // finger 1
    0.41, 0.27    // finger 2
};

// Joint limits from the mechanical hard stops, rad.
constexpr double kMaxFlexion = 1.75;

struct FingerState {
    double dtheta_prox;
    double dtheta_dist;
    double tendon_tension;  // N, resolved pull on this tendon
};

struct ContactForce {
    std::size_t finger;
    std::size_t phalanx;  // 0 = proximal, 1 = distal
    double normal_n;
    double tangential_n;
};

// Build the 3x6 excursion coupling matrix. Row i corresponds to tendon i,
// columns 2i and 2i+1 are the proximal and distal joints of that finger.
std::array<std::array<double, kNumJoints>, kNumFingers> build_coupling() {
    std::array<std::array<double, kNumJoints>, kNumFingers> j{};
    for (std::size_t f = 0; f < kNumFingers; ++f) {
        for (std::size_t c = 0; c < kNumJoints; ++c) {
            j[f][c] = 0.0;
        }
        j[f][2 * f] = kRadiusProximal;
        j[f][2 * f + 1] = kRadiusDistal;
    }
    return j;
}

// Solve joint displacements for a commanded tendon excursion vector.
// Because the coupling is block diagonal and rank deficient (1 equation,
// 2 unknowns per finger), we pick the minimum-norm solution weighted by the
// inverse spring stiffness. That is exactly the spring equilibrium: the
// compliant joint with the softer spring absorbs more of the excursion.
std::vector<FingerState> solve_excursion(
    const std::array<double, kNumFingers>& excursion_m) {
    const auto coupling = build_coupling();
    std::vector<FingerState> out(kNumFingers);

    for (std::size_t f = 0; f < kNumFingers; ++f) {
        const double rp = coupling[f][2 * f];
        const double rd = coupling[f][2 * f + 1];
        const double kp = kSpringStiffness[2 * f];
        const double kd = kSpringStiffness[2 * f + 1];

        // Weight each joint by 1/k, then project the excursion onto the
        // weighted row vector. w_j = (1/k_j) / sum(1/k).
        const double wp = (1.0 / kp) / (1.0 / kp + 1.0 / kd);
        const double wd = (1.0 / kd) / (1.0 / kp + 1.0 / kd);

        const double denom = rp * wp * rp * wp + rd * wd * rd * wd;
        const double scale = (denom > 1e-12) ? (excursion_m[f] / denom) : 0.0;

        double tp = scale * wp * rp;
        double td = scale * wd * rd;

        // Clamp to the hard stops. Saturation means the tendon is still
        // taking up slack, so tension rises faster than angle.
        tp = std::fmin(std::fmax(tp, 0.0), kMaxFlexion);
        td = std::fmin(std::fmax(td, 0.0), kMaxFlexion);

        // Resolved tendon tension = sum of spring torques divided by radius.
        const double torque = kp * tp + kd * td;
        const double tension = torque / (rp + rd);

        out[f] = FingerState{tp, td, tension};
    }
    return out;
}

// Distribute contact normal forces across the phalanges of each finger.
// A phalanx in contact carries load proportional to its joint torque arm.
// Fingers with no contact contribute nothing, which is the whole point of
// underactuation: the remaining fingers keep closing.
std::vector<ContactForce> distribute_contact_forces(
    const std::vector<FingerState>& state,
    const std::array<bool, kNumJoints>& in_contact) {
    std::vector<ContactForce> forces;
    forces.reserve(kNumJoints);

    for (std::size_t f = 0; f < kNumFingers; ++f) {
        for (std::size_t p = 0; p < kJointsPerFinger; ++p) {
            const std::size_t joint = 2 * f + p;
            if (!in_contact[joint]) {
                continue;
            }
            const double theta = (p == 0) ? state[f].dtheta_prox
                                          : state[f].dtheta_dist;
            const double k = kSpringStiffness[joint];
            const double arm = (p == 0) ? kRadiusProximal : kRadiusDistal;

            // Normal force balances the spring torque at that joint.
            const double normal = (k * theta) / arm;
            // Tangential component from Coulomb friction at the pad.
            const double tangential = 0.6 * normal;
            forces.push_back(ContactForce{f, p, normal, tangential});
        }
    }
    return forces;
}

// TODO(ravi): the current solve assumes tendons are perfectly routed on the
// flexion side. Rev D adds a redirect pulley at the fingertip which introduces
// a theta-dependent radius. Re-derive the coupling once the CAD is frozen.

}  // namespace gripper
