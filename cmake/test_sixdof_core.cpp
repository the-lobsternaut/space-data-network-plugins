/**
 * Tests for sixdof_core.h — shared 6DOF rigid body dynamics
 *
 * Validates:
 * 1. Quaternion algebra (multiply, rotate, normalize, Euler conversion)
 * 2. Inertia tensor operations
 * 3. Euler's rotational equations (torque-free precession)
 * 4. RK4 integration of a spinning rigid body
 * 5. Aerodynamic angle computation
 * 6. Free-fall 6DOF (projectile with spin)
 * 7. Conservation of angular momentum (torque-free)
 */

#include "sixdof_core.h"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace sixdof;

static constexpr double TOL = 1e-10;
static constexpr double ATOL = 1e-6; // for integration tests

void assertNear(double a, double b, double tol, const char* msg) {
    if (std::abs(a - b) > tol) {
        std::cerr << "FAIL: " << msg << " | expected " << b << " got " << a
                  << " (diff " << std::abs(a-b) << ")\n";
        assert(false);
    }
}

void assertVec3Near(const Vec3& a, const Vec3& b, double tol, const char* msg) {
    for (int i = 0; i < 3; i++) {
        if (std::abs(a[i] - b[i]) > tol) {
            std::cerr << "FAIL: " << msg << " component " << i
                      << " | expected " << b[i] << " got " << a[i] << "\n";
            assert(false);
        }
    }
}

// ============================================================================
// Test 1: Quaternion Basics
// ============================================================================

void testQuaternionBasics() {
    // Identity rotation
    Quat id = qidentity();
    Vec3 v = {1, 2, 3};
    Vec3 rotated = qrotate(id, v);
    assertVec3Near(rotated, v, TOL, "identity rotation");

    // 90° rotation around Z axis: (1,0,0) → (0,1,0)
    Quat qz90 = qfromAxisAngle({0, 0, 1}, M_PI / 2);
    Vec3 x_axis = {1, 0, 0};
    Vec3 result = qrotate(qz90, x_axis);
    assertVec3Near(result, {0, 1, 0}, TOL, "90° Z rotation");

    // 180° rotation around Y axis: (1,0,0) → (-1,0,0)
    Quat qy180 = qfromAxisAngle({0, 1, 0}, M_PI);
    result = qrotate(qy180, x_axis);
    assertVec3Near(result, {-1, 0, 0}, TOL, "180° Y rotation");

    // Quaternion multiply: two 90° Z rotations = 180° Z rotation
    Quat q180 = qmul(qz90, qz90);
    result = qrotate(q180, x_axis);
    assertVec3Near(result, {-1, 0, 0}, TOL, "double 90° Z rotation");

    // Conjugate inverse: q * q* = identity
    Quat qinv = qconj(qz90);
    Quat prod = qmul(qz90, qinv);
    assertNear(prod[0], 1.0, TOL, "q*q^-1 scalar");
    assertNear(prod[1], 0.0, TOL, "q*q^-1 x");
    assertNear(prod[2], 0.0, TOL, "q*q^-1 y");
    assertNear(prod[3], 0.0, TOL, "q*q^-1 z");

    // Normalize preserves direction
    Quat unnorm = {2, 0, 0, 0};
    Quat normed = qnormalize(unnorm);
    assertNear(qnorm(normed), 1.0, TOL, "normalize");

    std::cout << "  Quaternion basics ✓\n";
}

// ============================================================================
// Test 2: Euler Angle Round-Trip
// ============================================================================

void testEulerRoundTrip() {
    double roll = 0.3, pitch = 0.5, yaw = 1.2;
    Quat q = qfromEulerZYX(roll, pitch, yaw);
    Vec3 euler = qtoEulerZYX(q);
    assertNear(euler[0], roll, TOL, "Euler round-trip roll");
    assertNear(euler[1], pitch, TOL, "Euler round-trip pitch");
    assertNear(euler[2], yaw, TOL, "Euler round-trip yaw");

    // Edge case: gimbal lock (pitch = ±90°)
    Quat q_gimbal = qfromEulerZYX(0, M_PI / 2.0 - 0.001, 0);
    Vec3 e = qtoEulerZYX(q_gimbal);
    assertNear(e[1], M_PI / 2.0 - 0.001, 1e-3, "near-gimbal pitch");

    std::cout << "  Euler round-trip ✓\n";
}

// ============================================================================
// Test 3: DCM ↔ Quaternion Round-Trip
// ============================================================================

void testDCMRoundTrip() {
    Quat q = qfromEulerZYX(0.5, 0.3, 1.0);
    Mat3 dcm = qtoDCM(q);
    Quat q2 = dcmToQuat(dcm);

    // Quaternions may differ by sign (q and -q represent same rotation)
    if (q[0] * q2[0] < 0) {
        q2 = qscale(q2, -1);
    }
    for (int i = 0; i < 4; i++) {
        assertNear(q[i], q2[i], TOL, "DCM round-trip");
    }

    // Verify DCM is orthogonal (R^T * R = I)
    Mat3 RtR;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            double sum = 0;
            for (int k = 0; k < 3; k++)
                sum += dcm[k][i] * dcm[k][j];
            RtR[i][j] = sum;
        }
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            assertNear(RtR[i][j], (i==j) ? 1.0 : 0.0, TOL, "DCM orthogonality");

    std::cout << "  DCM round-trip ✓\n";
}

// ============================================================================
// Test 4: Inertia Tensor Operations
// ============================================================================

void testInertiaTensor() {
    // Diagonal inertia: cylinder
    InertiaTensor I = inertiaDiag(10.0, 10.0, 5.0);
    Vec3 omega = {1, 0, 0};
    Vec3 Iw = inertiaTimesOmega(I, omega);
    assertVec3Near(Iw, {10, 0, 0}, TOL, "I*omega diagonal");

    // I^-1 * I * omega = omega
    Vec3 inv_Iw = inertiaInvTimesVec(I, Iw);
    assertVec3Near(inv_Iw, omega, TOL, "I^-1 * I * omega");

    // Non-diagonal inertia
    InertiaTensor I2 = {10, 8, 5, 1.0, 0.5, 0.2};
    Vec3 omega2 = {1, 2, 3};
    Vec3 Iw2 = inertiaTimesOmega(I2, omega2);
    Vec3 recovered = inertiaInvTimesVec(I2, Iw2);
    assertVec3Near(recovered, omega2, 1e-8, "non-diagonal I^-1 * I * omega");

    std::cout << "  Inertia tensor ✓\n";
}

// ============================================================================
// Test 5: Torque-Free Spinning Body (angular momentum conservation)
// ============================================================================

void testTorqueFreeSpinning() {
    // Symmetric body spinning around principal axis
    InertiaTensor I = inertiaDiag(10.0, 10.0, 5.0); // oblate
    State s;
    s.pos = {0, 0, 100};
    s.vel = {0, 0, 0};
    s.quat = qidentity();
    s.omega = {0, 0, 10.0}; // spin around Z at 10 rad/s
    s.mass = 50;

    // No forces, no torques
    auto forceFn = [](const State&, double) -> ForcesTorques {
        return {};
    };

    // Initial angular momentum in body frame
    Vec3 L0_body = inertiaTimesOmega(I, s.omega);
    // In inertial frame
    Vec3 L0_inertial = qrotate(s.quat, L0_body);
    double L0_mag = v3norm(L0_inertial);

    // Integrate for 10 seconds
    double dt = 0.001;
    double t = 0;
    for (int i = 0; i < 10000; i++) {
        s = rk4Step(s, I, dt, t, forceFn);
        t += dt;
    }

    // Angular momentum should be conserved
    Vec3 L_body = inertiaTimesOmega(I, s.omega);
    Vec3 L_inertial = qrotate(s.quat, L_body);
    double L_mag = v3norm(L_inertial);

    assertNear(L_mag, L0_mag, 1e-6, "angular momentum conservation");

    // For spin around principal axis of symmetric body, omega magnitude should be constant
    assertNear(v3norm(s.omega), 10.0, 1e-6, "omega magnitude constant");

    std::cout << "  Torque-free spinning ✓ (L=" << L0_mag
              << " → " << L_mag << ")\n";
}

// ============================================================================
// Test 6: Torque-Free Precession (asymmetric body)
// ============================================================================

void testTorqueFreePrecession() {
    // Asymmetric body → expect nutation/precession
    InertiaTensor I = inertiaDiag(10.0, 8.0, 5.0);
    State s;
    s.pos = {0, 0, 0};
    s.vel = {0, 0, 0};
    s.quat = qidentity();
    s.omega = {0.1, 0.2, 5.0}; // mostly spin around Z, slight wobble
    s.mass = 50;

    auto forceFn = [](const State&, double) -> ForcesTorques {
        return {};
    };

    Vec3 L0_body = inertiaTimesOmega(I, s.omega);
    Vec3 L0 = qrotate(s.quat, L0_body);
    double L0_mag = v3norm(L0);
    double KE0 = 0.5 * v3dot(s.omega, inertiaTimesOmega(I, s.omega));

    double dt = 0.001;
    double t = 0;
    for (int i = 0; i < 50000; i++) { // 50 seconds
        s = rk4Step(s, I, dt, t, forceFn);
        t += dt;
    }

    Vec3 L_body = inertiaTimesOmega(I, s.omega);
    Vec3 L = qrotate(s.quat, L_body);
    double L_mag = v3norm(L);
    double KE = 0.5 * v3dot(s.omega, inertiaTimesOmega(I, s.omega));

    assertNear(L_mag, L0_mag, 1e-4, "precession: L conservation");
    assertNear(KE, KE0, 1e-4, "precession: KE conservation");

    // Angular momentum direction should be preserved in inertial frame
    Vec3 L0_dir = v3normalized(L0);
    Vec3 L_dir = v3normalized(L);
    double dot = v3dot(L0_dir, L_dir);
    assertNear(dot, 1.0, 1e-4, "precession: L direction conserved");

    std::cout << "  Torque-free precession ✓ (L=" << L0_mag
              << "→" << L_mag << ", KE=" << KE0 << "→" << KE << ")\n";
}

// ============================================================================
// Test 7: Free-Fall Projectile with Spin
// ============================================================================

void testFreeFallWithSpin() {
    constexpr double g = 9.80665;
    InertiaTensor I = inertiaDiag(0.01, 0.01, 0.005); // small projectile
    State s;
    s.pos = {0, 0, 1000};   // 1000m altitude
    s.vel = {100, 0, 0};    // 100 m/s forward
    s.quat = qidentity();
    s.omega = {0, 0, 100};  // 100 rad/s spin (rifling)
    s.mass = 1.0;

    // Gravity only, no aero
    auto forceFn = [g](const State& st, double) -> ForcesTorques {
        ForcesTorques ft;
        ft.force_inertial = {0, 0, -g * st.mass}; // gravity
        return ft;
    };

    double dt = 0.01;
    double t = 0;
    double t_impact = -1;
    for (int i = 0; i < 200000; i++) { // up to 2000s
        s = rk4Step(s, I, dt, t, forceFn);
        t += dt;
        if (s.pos[2] <= 0) {
            t_impact = t;
            break;
        }
    }

    // Analytical: t = sqrt(2h/g) = sqrt(2000/9.80665) ≈ 14.28s
    double t_analytical = std::sqrt(2.0 * 1000.0 / g);
    assertNear(t_impact, t_analytical, 0.02, "free-fall time");

    // X position: v*t = 100 * 14.28 ≈ 1428m
    assertNear(s.pos[0], 100.0 * t_analytical, 2.0, "free-fall x distance");

    // Spin should be preserved (no torques)
    assertNear(s.omega[2], 100.0, 1e-4, "spin conservation");

    std::cout << "  Free-fall with spin ✓ (t=" << t_impact
              << "s, x=" << s.pos[0] << "m, ωz=" << s.omega[2] << ")\n";
}

// ============================================================================
// Test 8: Aero Angles
// ============================================================================

void testAeroAngles() {
    // Body aligned with velocity → alpha=0, beta=0
    Quat q = qidentity();
    Vec3 vel = {100, 0, 0}; // flying along X
    auto [alpha, beta] = aeroAngles(q, vel);
    assertNear(alpha, 0, TOL, "alpha zero");
    assertNear(beta, 0, TOL, "beta zero");

    // Pitch up 10° → alpha ≈ +10° (nose up, velocity below body X → positive w component)
    Quat qpitch = qfromEulerZYX(0, 10.0 * M_PI / 180.0, 0);
    auto [a2, b2] = aeroAngles(qpitch, vel);
    assertNear(std::abs(a2), 10.0 * M_PI / 180.0, 1e-6, "alpha 10° pitch magnitude");
    assertNear(b2, 0, 1e-6, "beta zero with pitch");

    std::cout << "  Aero angles ✓ (α=" << a2 * 180/M_PI << "°)\n";
}

// ============================================================================
// Test 9: Applied Torque → Angular Acceleration
// ============================================================================

void testAppliedTorque() {
    InertiaTensor I = inertiaDiag(10.0, 10.0, 5.0);
    State s;
    s.quat = qidentity();
    s.omega = {0, 0, 0};
    s.mass = 50;

    // Apply constant torque around Z for 1 second
    double torque_z = 50.0; // N·m
    auto forceFn = [torque_z](const State&, double) -> ForcesTorques {
        ForcesTorques ft;
        ft.torque_body = {0, 0, torque_z};
        return ft;
    };

    double dt = 0.001;
    double t = 0;
    for (int i = 0; i < 1000; i++) {
        s = rk4Step(s, I, dt, t, forceFn);
        t += dt;
    }

    // Expected: ωz = τz * t / Izz = 50 * 1 / 5 = 10 rad/s
    assertNear(s.omega[2], 10.0, 1e-4, "torque-driven spin-up");

    // Expected angle: θ = ½ α t² = ½ * 10 * 1 = 5 rad
    Vec3 euler = qtoEulerZYX(s.quat);
    // Yaw angle wraps, so check modulo 2π
    double yaw = std::fmod(euler[2], 2 * M_PI);
    if (yaw < 0) yaw += 2 * M_PI;
    assertNear(yaw, 5.0, 0.01, "torque-driven rotation angle");

    std::cout << "  Applied torque ✓ (ωz=" << s.omega[2]
              << " rad/s, θ=" << yaw << " rad)\n";
}

// ============================================================================
// Test 10: Body-Frame Forces
// ============================================================================

void testBodyFrameForces() {
    InertiaTensor I = inertiaDiag(1, 1, 1);
    State s;
    s.pos = {0, 0, 0};
    s.vel = {0, 0, 0};
    // Body rotated 90° around Z → body X points along inertial Y
    s.quat = qfromAxisAngle({0, 0, 1}, M_PI / 2);
    s.mass = 1.0;

    // Thrust along body X (which is inertial Y after rotation)
    auto forceFn = [](const State&, double) -> ForcesTorques {
        ForcesTorques ft;
        ft.force_body = {10, 0, 0}; // 10N along body X
        return ft;
    };

    double dt = 0.001;
    double t = 0;
    for (int i = 0; i < 1000; i++) { // 1 second
        s = rk4Step(s, I, dt, t, forceFn);
        t += dt;
    }

    // After 1s: a = F/m = 10 m/s², v = 10 m/s along inertial Y
    // x = ½at² = 5m along inertial Y
    assertNear(s.pos[0], 0, 0.01, "body force: no X motion");
    assertNear(s.pos[1], 5.0, 0.01, "body force: Y = 5m");
    assertNear(s.vel[1], 10.0, 0.01, "body force: vy = 10 m/s");

    std::cout << "  Body-frame forces ✓ (pos=[" << s.pos[0] << ","
              << s.pos[1] << "," << s.pos[2] << "])\n";
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "=== sixdof_core tests ===\n";

    testQuaternionBasics();
    testEulerRoundTrip();
    testDCMRoundTrip();
    testInertiaTensor();
    testTorqueFreeSpinning();
    testTorqueFreePrecession();
    testFreeFallWithSpin();
    testAeroAngles();
    testAppliedTorque();
    testBodyFrameForces();

    std::cout << "All sixdof_core tests passed.\n";
    return 0;
}
