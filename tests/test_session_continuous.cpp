#include "doctest/doctest.h"
#include "state/sim_session.hpp"

TEST_CASE("SimSession tick(linear, angular) advances robot") {
    state::SimSession session;
    const auto t0 = session.tick(24.0, 0.0);
    CHECK(t0.tick == 0);
    CHECK(t0.ground_truth.y > 0.0);
}

