#include "doctest/doctest.h"
#include "sim/field.hpp"
#include "sim/physics.hpp"

using namespace sim;

TEST_CASE("Physics step_continuous: zero velocities leave state unchanged") {
    Field f;
    Physics phys(f);
    phys.set_state({10.0, -2.0, 45.0});

    const auto r = phys.step_continuous(0.0, 0.0);
    CHECK(r.delta.forward_in == doctest::Approx(0.0));
    CHECK(r.delta.rotation_deg == doctest::Approx(0.0));
    CHECK_FALSE(r.colliding);

    const auto s = phys.state();
    CHECK(s.x == doctest::Approx(10.0));
    CHECK(s.y == doctest::Approx(-2.0));
    CHECK(s.heading_deg == doctest::Approx(45.0));
}

TEST_CASE("Physics step_continuous: forward at heading zero moves +Y") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({0.0, 0.0, 0.0});

    const auto r = phys.step_continuous(36.0, 0.0);
    CHECK(r.delta.forward_in == doctest::Approx(cfg.max_velocity * cfg.dt).epsilon(1e-6));
    CHECK(r.delta.rotation_deg == doctest::Approx(0.0));
    CHECK(phys.state().x == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(phys.state().y == doctest::Approx(cfg.max_velocity * cfg.dt).epsilon(1e-6));
}

TEST_CASE("Physics step_continuous: combined translation and rotation") {
    Field f;
    PhysicsConfig cfg;
    Physics phys(f, cfg);
    phys.set_state({0.0, 0.0, 0.0});

    const auto r = phys.step_continuous(24.0, 120.0);
    CHECK(r.delta.rotation_deg == doctest::Approx(120.0 * cfg.dt).epsilon(1e-6));
    CHECK(r.delta.forward_in > 0.0);
    CHECK(phys.state().heading_deg == doctest::Approx(120.0 * cfg.dt).epsilon(1e-6));
}

TEST_CASE("Physics step_continuous: discrete forward parity") {
    Field f;
    PhysicsConfig cfg;
    Physics a(f, cfg);
    Physics b(f, cfg);
    a.set_state({4.0, -7.0, 135.0});
    b.set_state({4.0, -7.0, 135.0});

    const auto ra = a.step_continuous(cfg.max_velocity, 0.0);
    const auto rb = b.step(Action::FORWARD);
    CHECK(ra.delta.forward_in == doctest::Approx(rb.delta.forward_in).epsilon(1e-6));
    CHECK(ra.delta.rotation_deg == doctest::Approx(rb.delta.rotation_deg).epsilon(1e-6));
    CHECK(ra.colliding == rb.colliding);
    CHECK(a.state().x == doctest::Approx(b.state().x).epsilon(1e-6));
    CHECK(a.state().y == doctest::Approx(b.state().y).epsilon(1e-6));
    CHECK(a.state().heading_deg == doctest::Approx(b.state().heading_deg).epsilon(1e-6));
}

