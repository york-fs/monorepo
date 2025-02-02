#include <util.hh>

#include <gtest/gtest.h>

namespace {

TEST(Util, Clamp) {
    EXPECT_EQ(util::clamp(1000, -1000, 2000), 1000);
    EXPECT_EQ(util::clamp(-50, 0, 1000), 0);
    EXPECT_EQ(util::clamp(10000, -2000, 2000), 2000);
}

} // namespace
