#include <bms/charging.hh>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>

namespace {

using bms::charging::Run;
using bms::charging::State;

// Four cells at 3.8 V (100 uV resolution), one balance bitset.
struct Pack {
    std::array<std::uint16_t, 4> cells{38000, 38000, 38000, 38000};
    std::array<std::uint16_t, 1> balance{};
};

TEST(Charging, ChargesInConstantCurrent) {
    Pack pack;
    Run run;
    const auto control = run.update(pack.cells, pack.balance, 80, true);
    EXPECT_EQ(run.state(), State::ConstantCurrent);
    EXPECT_TRUE(control.enable);
    EXPECT_TRUE(bms::charging::is_active(run.state()));
}

TEST(Charging, StopWaitsForCurrentToDrop) {
    Pack pack;
    Run run;
    run.update(pack.cells, pack.balance, 80, true);
    run.request_stop();
    EXPECT_EQ(run.state(), State::Stopping);
    EXPECT_TRUE(bms::charging::is_active(run.state()));

    // Charger still putting out 8 A: stay in Stopping with the charger disabled.
    auto control = run.update(pack.cells, pack.balance, 80, true);
    EXPECT_EQ(run.state(), State::Stopping);
    EXPECT_FALSE(control.enable);

    // Current down to 0.5 A: stopped, contactors may open.
    control = run.update(pack.cells, pack.balance, 5, true);
    EXPECT_EQ(run.state(), State::Stopped);
    EXPECT_FALSE(control.enable);
    EXPECT_FALSE(bms::charging::is_active(run.state()));
}

TEST(Charging, StopTimesOutIfCurrentStaysHigh) {
    Pack pack;
    Run run;
    run.request_stop();
    for (int i = 0; i < 4; i++) {
        run.update(pack.cells, pack.balance, 80, true);
        EXPECT_EQ(run.state(), State::Stopping);
    }
    run.update(pack.cells, pack.balance, 80, true);
    EXPECT_EQ(run.state(), State::Stopped);
}

TEST(Charging, StopIsImmediateIfChargerGone) {
    Pack pack;
    Run run;
    run.request_stop();
    run.update(pack.cells, pack.balance, 80, false);
    EXPECT_EQ(run.state(), State::Stopped);
}

TEST(Charging, StopClearsBalancers) {
    Pack pack;
    pack.balance[0] = 0xf;
    Run run;
    run.request_stop();
    run.update(pack.cells, pack.balance, 0, true);
    EXPECT_EQ(pack.balance[0], 0u);
}

TEST(Charging, FaultEndsChargeAndStaysEnded) {
    Pack pack;
    Run run;
    run.update(pack.cells, pack.balance, 80, true);
    run.fault();
    EXPECT_EQ(run.state(), State::Faulted);
    EXPECT_FALSE(bms::charging::is_active(run.state()));

    const auto control = run.update(pack.cells, pack.balance, 80, true);
    EXPECT_EQ(run.state(), State::Faulted);
    EXPECT_FALSE(control.enable);

    // A stop after a fault doesn't change the reported reason.
    run.request_stop();
    EXPECT_EQ(run.state(), State::Faulted);
}

TEST(Charging, StoppedRunStaysStopped) {
    Pack pack;
    Run run(State::Stopped);
    const auto control = run.update(pack.cells, pack.balance, 0, true);
    EXPECT_EQ(run.state(), State::Stopped);
    EXPECT_FALSE(control.enable);
    run.fault();
    EXPECT_EQ(run.state(), State::Stopped);
}

} // namespace
