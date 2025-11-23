#include <bms.hh>

#include <gtest/gtest.h>

#include <cstdint>

namespace {

struct Params {
    bms::SegmentData segment_data;
    std::uint16_t expected_minimum_voltage;
    std::uint16_t expected_maximum_voltage;
    std::uint8_t expected_minimum_temperature;
    std::uint8_t expected_maximum_temperature;
    bms::ErrorFlags expected_error;
};

class BmsTest : public testing::TestWithParam<Params> {};

TEST_P(BmsTest, MinMaxVoltage) {
    const auto [minimum_voltage, maximum_voltage] = bms::min_max_voltage(GetParam().segment_data);
    EXPECT_EQ(minimum_voltage, GetParam().expected_minimum_voltage);
    EXPECT_EQ(maximum_voltage, GetParam().expected_maximum_voltage);
}

INSTANTIATE_TEST_SUITE_P(ValidData, BmsTest, testing::Values(Params{
            
        }));

} // namespace
